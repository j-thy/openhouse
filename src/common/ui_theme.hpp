// The look shared by openhouse's ImGui apps (installer, overlay, dongle manager): colours, drawn
// icons, sidebar items and the snow behind everything. Sizes follow the current font, so the
// overlay's 2x rendering inside SteamVR scales the same way as a 1x window.
#pragma once

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace lh::ui {

inline constexpr ImU32 kAccent = IM_COL32(110, 168, 255, 255);
inline constexpr ImU32 kRed = IM_COL32(240, 104, 100, 255);
inline constexpr ImU32 kGreen = IM_COL32(98, 206, 120, 255);
inline constexpr ImU32 kYellow = IM_COL32(242, 196, 82, 255);
inline constexpr ImU32 kMutedText = IM_COL32(158, 163, 174, 255);

// 1 at ImGui's 13 px font, 2 at 26 px, ...
inline float Scale() { return ImGui::GetFontSize() / 13.f; }

// Colours and rounding. k: the app's size multiple (call after ScaleAllSizes).
inline void ApplyTheme(float k = 1) {
  ImGuiStyle& style = ImGui::GetStyle();
  style.FrameRounding = 6 * k;
  style.ChildRounding = 8 * k;
  style.GrabRounding = 6 * k;
  style.PopupRounding = 8 * k;
  style.ScrollbarRounding = 8 * k;
  style.TabRounding = 6 * k;
  ImVec4* c = style.Colors;
  c[ImGuiCol_WindowBg] = ImVec4(0.067f, 0.075f, 0.098f, 1);
  c[ImGuiCol_ChildBg] = ImVec4(0, 0, 0, 0);
  c[ImGuiCol_PopupBg] = ImVec4(0.10f, 0.11f, 0.14f, 0.98f);
  c[ImGuiCol_ModalWindowDimBg] = ImVec4(0.02f, 0.02f, 0.04f, 0.6f);
  c[ImGuiCol_Border] = ImVec4(0.25f, 0.27f, 0.33f, 0.6f);
  c[ImGuiCol_Separator] = ImVec4(0.22f, 0.24f, 0.30f, 1);
  c[ImGuiCol_FrameBg] = ImVec4(0.14f, 0.16f, 0.20f, 1);
  c[ImGuiCol_FrameBgHovered] = ImVec4(0.19f, 0.22f, 0.28f, 1);
  c[ImGuiCol_FrameBgActive] = ImVec4(0.22f, 0.26f, 0.34f, 1);
  c[ImGuiCol_Button] = ImVec4(0.20f, 0.33f, 0.55f, 1);
  c[ImGuiCol_ButtonHovered] = ImVec4(0.26f, 0.42f, 0.68f, 1);
  c[ImGuiCol_ButtonActive] = ImVec4(0.17f, 0.28f, 0.48f, 1);
  c[ImGuiCol_Header] = ImVec4(0.20f, 0.33f, 0.55f, 0.55f);
  c[ImGuiCol_HeaderHovered] = ImVec4(0.26f, 0.42f, 0.68f, 0.7f);
  c[ImGuiCol_HeaderActive] = ImVec4(0.26f, 0.42f, 0.68f, 0.9f);
  c[ImGuiCol_Tab] = ImVec4(0.14f, 0.16f, 0.20f, 1);
  c[ImGuiCol_TabHovered] = ImVec4(0.26f, 0.42f, 0.68f, 1);
  c[ImGuiCol_TabSelected] = ImVec4(0.20f, 0.33f, 0.55f, 1);
  c[ImGuiCol_CheckMark] = ImVec4(0.45f, 0.68f, 1, 1);
  c[ImGuiCol_SliderGrab] = ImVec4(0.36f, 0.58f, 0.95f, 1);
  c[ImGuiCol_SliderGrabActive] = ImVec4(0.45f, 0.68f, 1, 1);
  c[ImGuiCol_PlotHistogram] = ImVec4(0.36f, 0.62f, 1, 1);
  c[ImGuiCol_PlotLines] = ImVec4(0.45f, 0.68f, 1, 1);
  c[ImGuiCol_TableHeaderBg] = ImVec4(0.12f, 0.14f, 0.18f, 1);
  c[ImGuiCol_TableBorderStrong] = ImVec4(0.25f, 0.27f, 0.33f, 0.8f);
  c[ImGuiCol_TableBorderLight] = ImVec4(0.20f, 0.22f, 0.27f, 0.8f);
  c[ImGuiCol_TableRowBgAlt] = ImVec4(1, 1, 1, 0.025f);
  c[ImGuiCol_ScrollbarBg] = ImVec4(0, 0, 0, 0);
}

// A card: a rounded panel a bit lighter than the page.
inline ImVec4 CardColor() { return ImVec4(0.11f, 0.122f, 0.157f, 0.92f); }

// ---- icons ----

enum class Icon {
  Download, Refresh, Trash, Chip, Restore, BaseStation, Wireless, Dongle, Tracker, Wire,
  Overview, Map, Settings, Debug, About, Slots, Channels, Record, Folder, Wrench
};

inline void Arc(ImDrawList* dl, ImVec2 c, float r, float a0, float a1, ImU32 col, float th) {
  dl->PathArcTo(c, r, a0, a1, 32);
  dl->PathStroke(col, 0, th);
}

// A ring with an arrowhead on its end (refresh style). clockwise false = mirror image.
// The ring ends at the top with a gap beside it; the arrowhead's base sits across the ring's end
// and its tip points sideways into the gap.
inline void RingArrow(ImDrawList* dl, ImVec2 c, float r, float th, bool clockwise, ImU32 col) {
  const float pi = 3.14159265f;
  // clockwise in screen space (y down) = increasing angle
  const float end = 1.5f * pi, start = end - 1.6f * pi;  // ends at the top
  const auto mirror = [&](float a) { return clockwise ? a : pi - a; };
  const float a0 = mirror(start), a1 = mirror(end);
  dl->PathArcTo(c, r, a0, a1, 40);
  dl->PathStroke(col, 0, th);
  const ImVec2 e(c.x + std::cos(a1) * r, c.y + std::sin(a1) * r);
  const ImVec2 radial(std::cos(a1), std::sin(a1));
  const float sgn = clockwise ? 1.f : -1.f;  // direction of travel at the end
  const ImVec2 d(-radial.y * sgn, radial.x * sgn);
  const float half = th * 2.3f, len = th * 2.8f;
  dl->AddTriangleFilled(ImVec2(e.x + radial.x * half, e.y + radial.y * half),
                        ImVec2(e.x - radial.x * half, e.y - radial.y * half),
                        ImVec2(e.x + d.x * len, e.y + d.y * len), col);
}

// An icon in a box of side s around c.
inline void DrawIcon(ImDrawList* dl, Icon icon, ImVec2 c, float s, ImU32 col) {
  const float th = std::max(1.5f, s * 0.08f), h = s / 2;
  const auto P = [&](float x, float y) { return ImVec2(c.x + x * h, c.y + y * h); };
  const float pi = 3.14159265f;
  switch (icon) {
    case Icon::Download:
      dl->AddLine(P(0, -0.85f), P(0, 0.0f), col, th);
      dl->AddTriangleFilled(P(-0.36f, -0.05f), P(0.36f, -0.05f), P(0, 0.38f), col);
      dl->PathLineTo(P(-0.8f, 0.35f)); dl->PathLineTo(P(-0.8f, 0.8f)); dl->PathLineTo(P(0.8f, 0.8f)); dl->PathLineTo(P(0.8f, 0.35f));
      dl->PathStroke(col, 0, th);
      break;
    case Icon::Refresh:
      RingArrow(dl, c, h * 0.62f, th, true, col);
      break;
    case Icon::Wrench:
      // A handle down to the lower left, an open jaw up to the upper right.
      dl->AddLine(P(-0.72f, 0.72f), P(0.12f, -0.12f), col, th * 1.7f);
      dl->PathArcTo(P(0.38f, -0.38f), h * 0.34f, 0.0f, 3.14159265f * 1.5f, 16);
      dl->PathStroke(col, ImDrawFlags_None, th * 1.5f);
      break;
    case Icon::Restore:
      RingArrow(dl, c, h * 0.62f, th, false, col);
      dl->AddLine(ImVec2(c.x, c.y + th * 0.3f), P(0, -0.24f), col, th);
      dl->AddLine(ImVec2(c.x - th * 0.3f, c.y), P(0.26f, 0.14f), col, th);
      break;
    case Icon::Trash:
      dl->AddLine(P(-0.8f, -0.55f), P(0.8f, -0.55f), col, th);
      dl->AddRect(P(-0.25f, -0.82f), P(0.25f, -0.55f), col, th * 0.5f, 0, th);
      dl->AddRect(P(-0.6f, -0.4f), P(0.6f, 0.85f), col, th * 1.2f, 0, th);
      dl->AddLine(P(-0.2f, -0.15f), P(-0.2f, 0.6f), col, th);
      dl->AddLine(P(0.2f, -0.15f), P(0.2f, 0.6f), col, th);
      break;
    case Icon::Chip:
      dl->AddRect(P(-0.55f, -0.55f), P(0.55f, 0.55f), col, th * 1.5f, 0, th);
      dl->AddRectFilled(P(-0.22f, -0.22f), P(0.22f, 0.22f), col, th * 0.6f);
      for (int i = -1; i <= 1; ++i) {
        const float k = float(i) * 0.3f;
        dl->AddLine(P(k, -0.55f), P(k, -0.85f), col, th);
        dl->AddLine(P(k, 0.55f), P(k, 0.85f), col, th);
        dl->AddLine(P(-0.55f, k), P(-0.85f, k), col, th);
        dl->AddLine(P(0.55f, k), P(0.85f, k), col, th);
      }
      break;
    case Icon::BaseStation:
      dl->AddRect(P(-0.75f, -0.75f), P(0.75f, 0.75f), col, h * 0.35f, 0, th);
      dl->AddCircleFilled(P(0, -0.45f), h * 0.1f, col, 16);  // status light
      break;
    case Icon::Wireless: {
      const ImVec2 o = P(0, 0.62f);
      dl->AddCircleFilled(o, h * 0.13f, col);
      for (int i = 1; i <= 3; ++i) Arc(dl, o, h * 0.42f * float(i), -pi * 0.78f, -pi * 0.22f, col, th);
      break;
    }
    case Icon::Dongle:
      dl->AddRect(P(-0.42f, -0.2f), P(0.42f, 0.9f), col, th * 1.5f, 0, th);
      dl->AddRect(P(-0.3f, -0.85f), P(0.3f, -0.2f), col, 0, 0, th);
      dl->AddRectFilled(P(-0.15f, -0.65f), P(-0.04f, -0.5f), col);
      dl->AddRectFilled(P(0.04f, -0.65f), P(0.15f, -0.5f), col);
      dl->AddCircleFilled(P(0, 0.45f), h * 0.1f, col);
      break;
    case Icon::Tracker:
      dl->AddCircle(P(0, 0.1f), h * 0.62f, col, 40, th);
      dl->AddCircleFilled(P(0, 0.1f), h * 0.16f, col);
      for (int i = 0; i < 3; ++i) {
        const float a = -pi / 2 + float(i) * 2 * pi / 3;
        dl->AddLine(ImVec2(c.x + std::cos(a) * h * 0.62f, c.y + h * 0.1f + std::sin(a) * h * 0.62f),
                    ImVec2(c.x + std::cos(a) * h * 0.9f, c.y + h * 0.1f + std::sin(a) * h * 0.9f), col, th * 1.4f);
      }
      break;
    case Icon::Wire:
      dl->AddRect(P(-0.35f, -0.45f), P(0.35f, 0.1f), col, th, 0, th);
      dl->AddLine(P(-0.15f, -0.45f), P(-0.15f, -0.85f), col, th);
      dl->AddLine(P(0.15f, -0.45f), P(0.15f, -0.85f), col, th);
      dl->AddBezierCubic(P(0, 0.1f), P(0, 0.6f), P(-0.7f, 0.4f), P(-0.6f, 0.9f), col, th);
      break;
    case Icon::Overview:  // four tiles
      for (int i = 0; i < 4; ++i) {
        const float x = (i % 2) ? 0.12f : -0.82f, y = (i / 2) ? 0.12f : -0.82f;
        dl->AddRect(P(x, y), P(x + 0.7f, y + 0.7f), col, th * 1.2f, 0, th);
      }
      break;
    case Icon::Map: {  // folded map
      const ImVec2 pts[] = {P(-0.85f, -0.6f), P(-0.3f, -0.8f), P(0.3f, -0.6f), P(0.85f, -0.8f),
                            P(0.85f, 0.6f), P(0.3f, 0.8f), P(-0.3f, 0.6f), P(-0.85f, 0.8f)};
      dl->AddPolyline(pts, 8, col, ImDrawFlags_Closed, th);
      dl->AddLine(P(-0.3f, -0.8f), P(-0.3f, 0.6f), col, th);
      dl->AddLine(P(0.3f, -0.6f), P(0.3f, 0.8f), col, th);
      break;
    }
    case Icon::Settings: {  // gear
      const int teeth = 8;
      for (int i = 0; i < teeth; ++i) {
        const float a = float(i) * 2 * pi / float(teeth);
        dl->AddLine(ImVec2(c.x + std::cos(a) * h * 0.5f, c.y + std::sin(a) * h * 0.5f),
                    ImVec2(c.x + std::cos(a) * h * 0.85f, c.y + std::sin(a) * h * 0.85f), col, th * 2.0f);
      }
      dl->AddCircleFilled(c, h * 0.6f, col, 32);
      dl->AddCircleFilled(c, h * 0.26f, IM_COL32(20, 23, 30, 255), 24);
      break;
    }
    case Icon::Debug:  // bug: outlined body and head, two legs a side
      dl->AddEllipse(P(0, 0.2f), ImVec2(h * 0.45f, h * 0.6f), col, 0, 32, th);
      dl->PathArcTo(P(0, -0.4f), h * 0.3f, pi, 2 * pi, 16);
      dl->PathStroke(col, 0, th);
      dl->AddLine(P(0, -0.15f), P(0, 0.8f), col, th);
      for (float y : {0.0f, 0.45f}) {
        dl->AddLine(P(-0.45f, y), P(-0.85f, y), col, th);
        dl->AddLine(P(0.45f, y), P(0.85f, y), col, th);
      }
      break;
    case Icon::About:  // i in a ring
      dl->AddCircle(c, h * 0.82f, col, 32, th);
      dl->AddLine(P(0, -0.1f), P(0, 0.48f), col, th * 1.3f);
      dl->AddCircleFilled(P(0, -0.38f), th * 0.85f, col, 12);
      break;
    case Icon::Slots:  // a list
      for (int i = 0; i < 3; ++i) {
        const float y = -0.55f + float(i) * 0.55f;
        dl->AddCircleFilled(P(-0.68f, y), th * 0.9f, col, 12);
        dl->AddLine(P(-0.35f, y), P(0.82f, y), col, th);
      }
      break;
    case Icon::Channels:  // bars
      for (int i = 0; i < 4; ++i) {
        const float x = -0.66f + float(i) * 0.44f, top = 0.75f - (0.45f + 0.35f * float((i * 7 + 3) % 4)) ;
        dl->AddRectFilled(P(x - 0.13f, top), P(x + 0.13f, 0.75f), col, th * 0.5f);
      }
      dl->AddLine(P(-0.9f, 0.85f), P(0.9f, 0.85f), col, th);
      break;
    case Icon::Record:
      dl->AddCircle(c, h * 0.8f, col, 32, th);
      dl->AddCircleFilled(c, h * 0.42f, col, 24);
      break;
    case Icon::Folder: {
      const ImVec2 pts[] = {P(-0.85f, -0.6f), P(-0.25f, -0.6f), P(-0.1f, -0.4f), P(0.85f, -0.4f), P(0.85f, 0.7f), P(-0.85f, 0.7f)};
      dl->AddPolyline(pts, 6, col, ImDrawFlags_Closed, th);
      break;
    }
  }
}

// An icon inline with text, at the cursor (one text line high), followed by SameLine.
inline void InlineIcon(Icon icon, ImU32 col = kAccent) {
  const float s = ImGui::GetTextLineHeight();
  const ImVec2 p = ImGui::GetCursorScreenPos();
  DrawIcon(ImGui::GetWindowDrawList(), icon, ImVec2(p.x + s / 2, p.y + s / 2), s * 0.9f, col);
  ImGui::Dummy(ImVec2(s, s));
  ImGui::SameLine();
}

// A sidebar entry: icon and label, a soft accent fill and bar when it's the page shown.
inline bool NavItem(const char* label, Icon icon, bool selected, float height) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const float w = ImGui::GetContentRegionAvail().x, fs = ImGui::GetFontSize();
  const ImVec2 a = ImGui::GetCursorScreenPos(), b(a.x + w, a.y + height);
  ImGui::PushID(label);
  const bool pressed = ImGui::InvisibleButton("##nav", ImVec2(w, height));
  ImGui::PopID();
  const bool hover = ImGui::IsItemHovered(), held = ImGui::IsItemActive();
  const float r = fs * 0.5f;
  if (selected) {
    dl->AddRectFilled(a, b, IM_COL32(110, 168, 255, 48), r);
    dl->AddRectFilled(ImVec2(a.x, a.y + height * 0.22f), ImVec2(a.x + fs * 0.22f, b.y - height * 0.22f), kAccent, fs * 0.11f);
  } else if (hover || held) {
    dl->AddRectFilled(a, b, held ? IM_COL32(255, 255, 255, 22) : IM_COL32(255, 255, 255, 14), r);
  }
  if (hover) ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
  const float side = std::min(height * 0.5f, fs * 1.35f);
  const ImVec2 ic(a.x + fs * 0.9f + side / 2, a.y + height / 2);
  DrawIcon(dl, icon, ic, side, selected ? kAccent : hover ? IM_COL32(205, 212, 226, 255) : kMutedText);
  dl->AddText(ImVec2(ic.x + side / 2 + fs * 0.75f, a.y + (height - fs) / 2),
              selected ? IM_COL32(240, 243, 248, 255) : hover ? IM_COL32(225, 229, 236, 255) : IM_COL32(190, 195, 205, 255), label);
  return pressed;
}

// ---- snow ----

// Slow, faint snow over [origin, origin + size]; draw it first so everything sits on top.
inline void Snow(ImDrawList* dl, ImVec2 origin, ImVec2 size) {
  struct Flake { float x, y, r, v, phase; };
  static std::vector<Flake> flakes;
  static unsigned seed = 12345;
  const auto rnd = [] { seed = seed * 1103515245u + 12345u; return float((seed >> 8) & 0xFFFF) / 65535.f; };
  if (flakes.empty())
    for (int i = 0; i < 70; ++i) flakes.push_back({rnd(), rnd(), 0.8f + rnd() * 1.6f, 0.012f + rnd() * 0.02f, rnd() * 6.28f});
  const float k = Scale();
  const float dt = std::min(0.1f, ImGui::GetIO().DeltaTime);
  const float t = float(ImGui::GetTime());
  for (auto& f : flakes) {
    f.y += f.v * dt * (0.6f + f.r * 0.25f);
    if (f.y > 1.02f) { f.y = -0.02f; f.x = rnd(); }
    const float x = origin.x + f.x * size.x + std::sin(t * 0.5f + f.phase) * 14.f * k;
    dl->AddCircleFilled(ImVec2(x, origin.y + f.y * size.y), f.r * k, IM_COL32(220, 230, 255, int(40 + f.r * 30)), 8);
  }
}

}  // namespace lh::ui
