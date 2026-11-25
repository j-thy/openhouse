#pragma once

#include <filesystem>

#include <imgui.h>

#include "platform/platform.hpp"

namespace lh::ui {

inline void MergeTrackerNameFonts(ImFontAtlas& atlas) {
#ifdef _WIN32
  const auto dir = platform::FsPath(platform::Env("WINDIR")) / "Fonts";
  const std::filesystem::path candidates[] = {
      dir / "segoeui.ttf", dir / "msyh.ttc", dir / "mingliub.ttc", dir / "msgothic.ttc", dir / "malgun.ttf"};
#else
  const std::filesystem::path candidates[] = {
      "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
      "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans.ttf",
      "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
      "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
      "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
      "/usr/share/fonts/google-noto-cjk/NotoSansCJK-Regular.ttc",
      "/usr/share/fonts/google-noto-sans-cjk-vf-fonts/NotoSansCJK-VF.ttc"};
#endif
  ImFontConfig merge;
  merge.MergeMode = true;
  merge.Flags = ImFontFlags_NoLoadError;
  // The OpenGL2 backend builds the atlas on demand from the installed fonts.
  for (const auto& path : candidates) {
    std::error_code error;
    if (!std::filesystem::is_regular_file(path, error)) continue;
    const auto utf8 = path.u8string();
    atlas.AddFontFromFileTTF(reinterpret_cast<const char*>(utf8.c_str()), 0.0f, &merge);
  }
}

}  // namespace lh::ui
