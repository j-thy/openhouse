// doorman-dongle-manager: the Doorman Dongle Manager. The dongles plugged in, what's paired to
// each of their eight slots, pairing into a slot and unpairing, the radio's live performance,
// its channels and the dongle firmware. A desktop window (Windows, Linux), in the look of
// openhouse-overlay; it works with or without the openhouse driver running (with it, device
// names come from the driver's telemetry).
//
//   OPENHOUSE_DONGLE_SNAPSHOT=FILE.ppm  (testing) the frame after ~3 s goes to FILE, then exit
//   OPENHOUSE_DONGLE_PAGE=slots|radio|channels|firmware|about  (testing) the page shown first
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl2.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

#include "common/crash_log.hpp"
#include "dongle_logo_rgba.h"
#include "logo_rgba.h"
#include "ui.hpp"

namespace {
using openhouse::overlay::kLogoRgba;
using openhouse::overlay::kLogoSize;

bool SnapshotDue(int frame, int w, int h) {
  const char* path = std::getenv("OPENHOUSE_DONGLE_SNAPSHOT");
  // OPENHOUSE_DONGLE_SNAPSHOT_FRAMES: frames to wait (180 by default), for the radio stats to settle.
  static const int wait = std::getenv("OPENHOUSE_DONGLE_SNAPSHOT_FRAMES") ? std::atoi(std::getenv("OPENHOUSE_DONGLE_SNAPSHOT_FRAMES")) : 180;
  if (!path || frame < wait) return false;
  std::vector<unsigned char> rgb(size_t(w) * h * 3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
  if (FILE* f = std::fopen(path, "wb")) {
    std::fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (int y = h - 1; y >= 0; --y) std::fwrite(rgb.data() + size_t(y) * w * 3, 1, size_t(w) * 3, f);
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
}  // namespace

int main() {
  lh::crash::Install("doorman-dongle-manager");
  if (!glfwInit()) { std::fprintf(stderr, "doorman-dongle-manager: no display\n"); return 1; }
  // OPENHOUSE_DONGLE_SCALE=1|2|3: the UI size (default: 2 on screens over 1080 lines, else 1)
  int scale = 1;
  if (const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor()); mode && mode->height > 1300) scale = 2;
  if (const char* s = std::getenv("OPENHOUSE_DONGLE_SCALE"); s && std::atoi(s) >= 1) scale = std::atoi(s);
  GLFWwindow* window = glfwCreateWindow(1180 * scale, 760 * scale, "Doorman Dongle Manager", nullptr, nullptr);
  if (!window) { std::fprintf(stderr, "doorman-dongle-manager: can't open a window\n"); glfwTerminate(); return 1; }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  // The logo with a dongle badge (tools/make-app-icons.py).
  GLFWimage icon{openhouse::dongle::kDongleLogoSize, openhouse::dongle::kDongleLogoSize,
                 const_cast<unsigned char*>(openhouse::dongle::kDongleLogoRgba)};
  glfwSetWindowIcon(window, 1, &icon);
  ImGui::CreateContext();
  openhouse::dongle::Ui::SetUpImGui(scale);
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL2_Init();
  int rc = 0;
  {
    openhouse::dongle::Ui ui;
    ui.SetLogo(UploadRgba(kLogoRgba, kLogoSize, kLogoSize));
    double next = glfwGetTime();
    int frame = 0;
    while (!glfwWindowShouldClose(window)) {
      // 60 frames a second at most, 20 unfocused, none minimized (as openhouse-overlay)
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
  }
  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return rc;
}
