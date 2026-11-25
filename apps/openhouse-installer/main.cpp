// openhouse-installer: installs and uninstalls the openhouse driver, and flashes openhouse's
// firmware (1.0 base stations, doorman dongles, trackers' and controllers' radios), with a guided
// window for each. Windows and Linux; everything it needs is linked in, and the firmware files
// come from the package's firmwares/ folder.
#include <string>
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
#include "logo_rgba.h"
#include "ui.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

namespace {
using namespace openhouse::installer;

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

int Run(const std::string& mode) {
  if (!glfwInit()) {
#ifdef _WIN32
    MessageBoxW(nullptr, L"openhouse-installer couldn't open its window.", L"openhouse", MB_ICONERROR);
#else
    std::fprintf(stderr, "openhouse-installer: no display\n");
#endif
    return 1;
  }
  GLFWwindow* window = glfwCreateWindow(1000, 720, "openhouse installer", nullptr, nullptr);
  if (!window) { glfwTerminate(); return 1; }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  GLFWimage icon{openhouse::overlay::kLogoSize, openhouse::overlay::kLogoSize, const_cast<unsigned char*>(openhouse::overlay::kLogoRgba)};
  glfwSetWindowIcon(window, 1, &icon);
  ImGui::CreateContext();
  Ui::SetUpImGui();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL2_Init();
  {
    Ui ui;
    ui.SetLogo(UploadRgba(openhouse::overlay::kLogoRgba, openhouse::overlay::kLogoSize, openhouse::overlay::kLogoSize));
    // Testing without a screen: OPENHOUSE_INSTALLER_PAGE picks the page, OPENHOUSE_INSTALLER_SNAPSHOT=FILE.ppm
    // writes the frame drawn after ~2 s and exits.
    if (const char* page = std::getenv("OPENHOUSE_INSTALLER_PAGE")) ui.ShowForTest(page);
    if (mode == "--update") ui.StartSelfUpdate();
    else if (mode == "--install" || mode == "--repair") ui.StartAutoInstall();
    else if (mode == "--uninstall") ui.StartAutoUninstall();
    const char* snapshot = std::getenv("OPENHOUSE_INSTALLER_SNAPSHOT");
    int frame = 0;
    while (!glfwWindowShouldClose(window) && !ui.QuitRequested()) {
      glfwWaitEventsTimeout(1.0 / 30);  // keeps redrawing for progress bars and device lists
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
      static const int kSnapFrame = std::getenv("OPENHOUSE_INSTALLER_SNAPSHOT_FRAMES") ? std::atoi(std::getenv("OPENHOUSE_INSTALLER_SNAPSHOT_FRAMES")) : 60;
      if (snapshot && ++frame == kSnapFrame) {
        std::vector<unsigned char> rgb(size_t(w) * h * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
        if (FILE* f = std::fopen(snapshot, "wb")) {
          std::fprintf(f, "P6\n%d %d\n255\n", w, h);
          for (int y = h - 1; y >= 0; --y) std::fwrite(rgb.data() + size_t(y) * w * 3, 1, size_t(w) * 3, f);   // GL rows are bottom-up
          std::fclose(f);
        }
        break;
      }
      glfwSwapBuffers(window);
    }
  }
  // Scans finish on their own threads: let the last one end before the process does (bounded).
  openhouse::installer::g_closing = true;
  std::unique_lock devices(openhouse::installer::g_deviceMutex, std::defer_lock);
  for (int i = 0; i < 100 && !devices.try_lock(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
}  // namespace

// --update (the overlay's Update / Reinstall): download the latest release and hand over to its
// installer; --install (that installer) and --repair (the overlay's Fix, from the package kept at
// install): install right away; --uninstall (its Uninstall openhouse): uninstall right away.
int main(int argc, char** argv) {
  lh::crash::Install("openhouse-installer");
  return Run(argc > 1 ? argv[1] : "");
}
