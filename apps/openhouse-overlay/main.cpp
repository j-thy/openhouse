// openhouse-overlay: the openhouse management app. It shows the running driver (devices,
// base stations, dongles, the room in 3D, its log) and changes its settings, over the
// driver's local telemetry (lh/telemetry.hpp), so it works under any runtime the driver
// runs in (SteamVR, Monado, WiVRn).
//
//   openhouse-overlay              a desktop window
//   openhouse-overlay --overlay [--driver DIR]
//                                  the SteamVR dashboard overlay, or exit if SteamVR isn't running
//                                  (what SteamVR starts with each session, from its manifest). With
//                                  --driver it also runs the session check (driver_check.hpp)
//   openhouse-overlay --check DIR  only the session check, then exit (Linux: the Monado / WiVRn
//                                  service). DIR: SteamVR/drivers/lighthouse/bin/<platform>
//
// The dashboard overlay renders the same ImGui UI into an OpenGL texture of a hidden
// window and hands SteamVR the texture; the laser pointer comes back as mouse events.
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl2.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <string>
#include <thread>
#include "common/crash_log.hpp"
#include "driver_check.hpp"
#include <cstdlib>
#include <vector>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "lh/telemetry.hpp"
#include "logo_rgba.h"
#include "openvr_runtime.hpp"
#include "ui.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

namespace {
using namespace openhouse::overlay;

// The overlay's texture, and its size in SteamVR's dashboard.
// SteamVR fits the dashboard overlay to its panel: a smaller texture shows the UI larger.
// 2128 x 1330 with the 2x font: the UI at 1.5x of a 1x desktop window, still pixel-sharp.
constexpr int kTextureWidth = 2128, kTextureHeight = 1330;
constexpr float kOverlayWidthMeters = 2.5f;
constexpr const char* kOverlayKey = "openhouse.overlay";
constexpr uint32_t kTrackerKeyboardMaxChars = 256;

int TelemetryPort() {
  const int port = lh::telemetry::PortFromEnvironment();
  return port > 0 ? port : lh::telemetry::kDefaultPort;
}

// OPENHOUSE_OVERLAY_SNAPSHOT=FILE.ppm (testing without a headset or a screen): the frame
// drawn after ~2 s is written to FILE, and the app exits.
bool SnapshotDue(int frame, int w, int h) {
  const char* path = std::getenv("OPENHOUSE_OVERLAY_SNAPSHOT");
  // OPENHOUSE_OVERLAY_SNAPSHOT_FRAMES: frames to wait (120 by default), for data that takes a while.
  static const int wait = std::getenv("OPENHOUSE_OVERLAY_SNAPSHOT_FRAMES") ? std::atoi(std::getenv("OPENHOUSE_OVERLAY_SNAPSHOT_FRAMES")) : 120;
  if (!path || frame < wait) return false;
  std::vector<unsigned char> rgb(size_t(w) * h * 3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
  if (FILE* f = std::fopen(path, "wb")) {
    std::fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (int y = h - 1; y >= 0; --y) std::fwrite(rgb.data() + size_t(y) * w * 3, 1, size_t(w) * 3, f);  // GL rows are bottom-up
    std::fclose(f);
  }
  return true;
}

GLuint UploadRgba(const uint8_t* rgba, int w, int h) {
  GLuint texture = 0;
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
  return texture;
}

// ---- the desktop window ----
int RunDesktop() {
  // OPENHOUSE_OVERLAY_SCALE=2 (testing): the window drawn as the VR overlay is, at its size.
  const char* scaleEnv = std::getenv("OPENHOUSE_OVERLAY_SCALE");
  const int scale = scaleEnv && std::atoi(scaleEnv) > 1 ? std::atoi(scaleEnv) : 1;
  GLFWwindow* window = scale > 1 ? glfwCreateWindow(kTextureWidth, kTextureHeight, "openhouse", nullptr, nullptr)
                                 : glfwCreateWindow(1280, 800, "openhouse", nullptr, nullptr);
  if (!window) { std::fprintf(stderr, "openhouse-overlay: can't open a window\n"); return 1; }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  GLFWimage icon{kLogoSize, kLogoSize, const_cast<unsigned char*>(kLogoRgba)};
  glfwSetWindowIcon(window, 1, &icon);
  ImGui::CreateContext();
  Ui::SetUpImGui(scale);
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL2_Init();
  Ui ui(TelemetryPort());
  ui.SetLogo(UploadRgba(kLogoRgba, kLogoSize, kLogoSize));
  double next = glfwGetTime();
  int frame = 0;
  while (!glfwWindowShouldClose(window) && !ui.QuitRequested()) {
    // 60 frames a second at most, 20 unfocused, none minimized (as the viewer).
    if (glfwGetWindowAttrib(window, GLFW_ICONIFIED)) {
      glfwWaitEventsTimeout(0.25);
      ui.Poll(glfwGetTime());
      continue;
    }
    const double interval = glfwGetWindowAttrib(window, GLFW_FOCUSED) ? 1.0 / 60 : 1.0 / 20;
    const double wait = next + interval - glfwGetTime();
    // Wait on the window's events, not a sleep: Windows hears a click (a drag starting) at once.
    if (wait > 0) glfwWaitEventsTimeout(wait);
    else glfwPollEvents();
    next = glfwGetTime();
    ui.Poll(glfwGetTime());
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    ui.Draw(ImGui::GetIO().DisplaySize.x, ImGui::GetIO().DisplaySize.y);
    ImGui::Render();
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(0.067f, 0.075f, 0.098f, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    if (SnapshotDue(++frame, w, h)) break;
    glfwSwapBuffers(window);
  }
  ui.SaveSettings();
  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  return 0;
}

// ---- the SteamVR dashboard overlay ----
// Framebuffer objects: OpenGL 3.0 / ARB_framebuffer_object, loaded at run time (the GL 1.1
// headers Windows has don't declare them).
#ifndef GL_FRAMEBUFFER
#define GL_FRAMEBUFFER 0x8D40
#define GL_COLOR_ATTACHMENT0 0x8CE0
#define GL_FRAMEBUFFER_COMPLETE 0x8CD5
#endif
#ifndef APIENTRY
#define APIENTRY
#endif
struct Fbo {
  void(APIENTRY* genFramebuffers)(GLsizei, GLuint*) = nullptr;
  void(APIENTRY* bindFramebuffer)(GLenum, GLuint) = nullptr;
  void(APIENTRY* framebufferTexture2D)(GLenum, GLenum, GLenum, GLuint, GLint) = nullptr;
  GLenum(APIENTRY* checkFramebufferStatus)(GLenum) = nullptr;
  GLuint framebuffer = 0, texture = 0;

  bool Create(int w, int h) {
    genFramebuffers = reinterpret_cast<decltype(genFramebuffers)>(glfwGetProcAddress("glGenFramebuffers"));
    bindFramebuffer = reinterpret_cast<decltype(bindFramebuffer)>(glfwGetProcAddress("glBindFramebuffer"));
    framebufferTexture2D = reinterpret_cast<decltype(framebufferTexture2D)>(glfwGetProcAddress("glFramebufferTexture2D"));
    checkFramebufferStatus = reinterpret_cast<decltype(checkFramebufferStatus)>(glfwGetProcAddress("glCheckFramebufferStatus"));
    if (!genFramebuffers || !bindFramebuffer || !framebufferTexture2D || !checkFramebufferStatus) return false;
    texture = UploadRgba(nullptr, w, h);
    genFramebuffers(1, &framebuffer);
    bindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    framebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);
    const bool ok = checkFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    bindFramebuffer(GL_FRAMEBUFFER, 0);
    return ok;
  }
};

// SteamVR on Linux gives the laser's mouse position from the bottom edge; on Windows from the top.
float MouseY(float y) {
#ifdef _WIN32
  return y;
#else
  return float(kTextureHeight) - y;
#endif
}

int RunOverlay() {
  vr::EVRInitError error = vr::VRInitError_None;
  vr::VR_Init(&error, vr::VRApplication_Overlay);
  if (error != vr::VRInitError_None) {
    std::fprintf(stderr, "openhouse-overlay: SteamVR: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(error));
    return 1;
  }
  vr::IVROverlay* overlay = vr::VROverlay();
  vr::VROverlayHandle_t main = vr::k_ulOverlayHandleInvalid, thumb = vr::k_ulOverlayHandleInvalid;
  if (!overlay || overlay->CreateDashboardOverlay(kOverlayKey, "openhouse", &main, &thumb) != vr::VROverlayError_None) {
    std::fprintf(stderr, "openhouse-overlay: can't create the dashboard overlay (another one running?)\n");
    vr::VR_Shutdown();
    return 1;
  }
  overlay->SetOverlayWidthInMeters(main, kOverlayWidthMeters);
  overlay->SetOverlayInputMethod(main, vr::VROverlayInputMethod_Mouse);
  overlay->SetOverlayFlag(main, vr::VROverlayFlags_SendVRSmoothScrollEvents, true);
  const vr::HmdVector2_t scale{float(kTextureWidth), float(kTextureHeight)};
  overlay->SetOverlayMouseScale(main, &scale);
  overlay->SetOverlayRaw(thumb, const_cast<uint8_t*>(kLogoRgba), kLogoSize, kLogoSize, 4);

  // A hidden window only for its OpenGL context.
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  GLFWwindow* window = glfwCreateWindow(64, 64, "openhouse overlay", nullptr, nullptr);
  if (!window) { vr::VR_Shutdown(); return 1; }
  glfwMakeContextCurrent(window);
  Fbo fbo;
  if (!fbo.Create(kTextureWidth, kTextureHeight)) {
    std::fprintf(stderr, "openhouse-overlay: no OpenGL framebuffer objects\n");
    vr::VR_Shutdown();
    return 1;
  }
  ImGui::CreateContext();
  Ui::SetUpImGui(2);
  ImGuiIO& io = ImGui::GetIO();
  io.DisplaySize = ImVec2(float(kTextureWidth), float(kTextureHeight));
  io.BackendPlatformName = "openhouse-openvr";
  ImGui_ImplOpenGL2_Init();
  Ui ui(TelemetryPort());
  ui.SetInVR(true);
  ui.SetNameKeyboard([&](const char* current, uint64_t edit) {
    return overlay->ShowKeyboardForOverlay(main, vr::k_EGamepadTextInputModeNormal, vr::k_EGamepadTextInputLineModeSingleLine,
                                           vr::KeyboardFlag_Modal, "Tracker name", kTrackerKeyboardMaxChars,
                                           current, edit) == vr::VROverlayError_None;
  });
  ui.SetLogo(UploadRgba(kLogoRgba, kLogoSize, kLogoSize));

  auto last = std::chrono::steady_clock::now();
  bool running = true, notified = false;
  while (running) {
    // In the headset too: the desktop dialog isn't seen with it on.
    // SteamVR's own notification in the headset: a replaced driver stays until dismissed, a new
    // release is a passing one.
    if (!notified && !openhouse::overlay::NoticeText().empty()) {
      notified = true;
      if (vr::IVRNotifications* n = vr::VRNotifications()) {
        const bool replaced = openhouse::overlay::Notice().kind == openhouse::overlay::NoticeKind::Replaced;
        const std::string text = openhouse::overlay::NoticeText() + (replaced ? ". Open openhouse in the dashboard to reinstall" : "");
        vr::NotificationBitmap_t logo;
        logo.m_pImageData = const_cast<uint8_t*>(kLogoRgba);
        logo.m_nWidth = kLogoSize;
        logo.m_nHeight = kLogoSize;
        logo.m_nBytesPerPixel = 4;
        vr::VRNotificationId id = 0;
        n->CreateNotification(main, 0, replaced ? vr::EVRNotificationType_Persistent : vr::EVRNotificationType_Transient,
                              text.c_str(), vr::EVRNotificationStyle_Application, &logo, &id);
      }
    }
    vr::VREvent_t event;
    while (vr::VRSystem() && vr::VRSystem()->PollNextEvent(&event, sizeof event))
      if (event.eventType == vr::VREvent_Quit) {
        vr::VRSystem()->AcknowledgeQuit_Exiting();
        running = false;
      }
    while (overlay->PollNextOverlayEvent(main, &event, sizeof event)) {
      switch (event.eventType) {
        case vr::VREvent_MouseMove: io.AddMousePosEvent(event.data.mouse.x, MouseY(event.data.mouse.y)); break;
        case vr::VREvent_MouseButtonDown:
        case vr::VREvent_MouseButtonUp: {
          const int button = event.data.mouse.button == vr::VRMouseButton_Right ? 1 : event.data.mouse.button == vr::VRMouseButton_Middle ? 2 : 0;
          io.AddMousePosEvent(event.data.mouse.x, MouseY(event.data.mouse.y));
          io.AddMouseButtonEvent(button, event.eventType == vr::VREvent_MouseButtonDown);
          break;
        }
        case vr::VREvent_KeyboardDone: {
          // Allow four UTF-8 bytes per character before checking the storage limit.
          std::array<char, 4 * kTrackerKeyboardMaxChars + 1> name{};
          const uint32_t size = overlay->GetKeyboardText(name.data(), static_cast<uint32_t>(name.size()));
          const auto end = size > name.size() ? name.end() : std::find(name.begin(), name.end(), '\0');
          ui.CommitTrackerName(std::string_view(name.data(), end - name.begin()), event.data.keyboard.uUserValue);
          break;
        }
        case vr::VREvent_KeyboardClosed:
          ui.CancelTrackerName(event.data.keyboard.uUserValue);
          break;
        case vr::VREvent_ScrollSmooth: io.AddMouseWheelEvent(event.data.scroll.xdelta, event.data.scroll.ydelta); break;
        case vr::VREvent_FocusLeave: io.AddMousePosEvent(-FLT_MAX, -FLT_MAX); break;
        default: break;
      }
    }
    const auto now = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(now.time_since_epoch()).count();
    ui.Poll(seconds);
    // Drawn only while the dashboard shows it; otherwise it keeps reading the driver, slowly.
    if (!overlay->IsOverlayVisible(main)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      last = std::chrono::steady_clock::now();
      continue;
    }
    io.DeltaTime = std::max(1e-4f, std::chrono::duration<float>(now - last).count());
    last = now;
    ImGui_ImplOpenGL2_NewFrame();
    ImGui::NewFrame();
    ui.Draw(float(kTextureWidth), float(kTextureHeight));
    ImGui::Render();
    fbo.bindFramebuffer(GL_FRAMEBUFFER, fbo.framebuffer);
    glViewport(0, 0, kTextureWidth, kTextureHeight);
    glClearColor(0.067f, 0.075f, 0.098f, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    glFlush();
    fbo.bindFramebuffer(GL_FRAMEBUFFER, 0);
    vr::Texture_t texture{reinterpret_cast<void*>(uintptr_t(fbo.texture)), vr::TextureType_OpenGL, vr::ColorSpace_Auto};
    overlay->SetOverlayTexture(main, &texture);
    std::this_thread::sleep_for(std::chrono::milliseconds(16));  // ~60 frames a second
  }
  ui.SaveSettings();
  ImGui_ImplOpenGL2_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  vr::VR_Shutdown();
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  lh::crash::Install("openhouse-overlay");
  bool overlay = false;
  std::string driverDir;
  for (int i = 1; i < argc; ++i) {
    // --check and --driver take the rest of the line: the path may have come in pieces.
    if (!std::strcmp(argv[i], "--check")) return openhouse::overlay::RunDriverCheck(openhouse::overlay::JoinedPath(argc, argv, i + 1));
    if (!std::strcmp(argv[i], "--driver")) { driverDir = openhouse::overlay::JoinedPath(argc, argv, i + 1); break; }
    if (!std::strcmp(argv[i], "--overlay")) overlay = true;
  }
  if (!glfwInit()) { std::fprintf(stderr, "openhouse-overlay: no display\n"); return 1; }
  int rc = 0;
  if (!overlay) rc = RunDesktop();
  else if (SteamVRRunning()) {
    // Its dialogs wait for the user: on their own thread, the overlay goes on meanwhile.
    if (!driverDir.empty()) std::thread([driverDir] { openhouse::overlay::RunDriverCheck(driverDir, true); }).detach();
    rc = RunOverlay();
  }
  // --overlay without SteamVR: nothing to do
  glfwTerminate();
  return rc;
}
