#include "ui.hpp"
#include "self_update.hpp"
#include "lh/build_time.h"

#include <imgui.h>
#include <imgui_internal.h>

#include <map>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <cfloat>
#include <cstdio>
#include <fstream>
#include <cstdlib>

#include "common/dongle_bank.hpp"
#include "common/nordic_dfu.hpp"
#include "common/ui_theme.hpp"
#include "lh/github.hpp"
#include "lh/version.hpp"

namespace openhouse::installer {
namespace {

ImFont* g_title = nullptr;
ImFont* g_huge = nullptr;
const ImVec4 kGood{0.36f, 0.80f, 0.45f, 1}, kWarn{1.00f, 0.78f, 0.30f, 1}, kBad{1.00f, 0.42f, 0.40f, 1},
    kMuted{0.62f, 0.64f, 0.68f, 1};

double Now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

bool BigButton(const char* label, float width = 0, float height = 0) {
  return ImGui::Button(label, ImVec2(width, height > 0 ? height : ImGui::GetFrameHeight() * 1.6f));
}

void Title(const char* text) {
  ImGui::PushFont(g_title);
  ImGui::TextUnformatted(text);
  ImGui::PopFont();
  ImGui::Spacing();
}

void Wrapped(const char* text) {
  ImGui::PushTextWrapPos(0);
  ImGui::TextUnformatted(text);
  ImGui::PopTextWrapPos();
}

void Step(int n, const char* text) {
  ImGui::TextColored(kWarn, "%d", n);
  ImGui::SameLine();
  Wrapped(text);
}

// The ProMicro nRF52840 from above, USB port at the top: the board's own antenna runs along the
// bottom edge, and a 31.2 mm wire (a quarter wave at 2.4 GHz) soldered to its end reaches much
// further. Laid out as SlimeVR's build guide shows it.
void ProMicroAntenna() {
  const float fs = ImGui::GetFontSize(), u = fs * 1.7f;    // drawing unit: 3 mm
  const float bw = u * 6, bh = u * 11;                      // the board: 18 x 33 mm
  const float wire = bw * 31.2f / 18.0f;                    // to scale
  ImDrawList* dl = ImGui::GetWindowDrawList();
  ImFont* font = ImGui::GetFont();
  const ImU32 pcb = IM_COL32(24, 26, 32, 255), edge = IM_COL32(90, 96, 110, 255), silk = IM_COL32(150, 155, 165, 255);
  const ImU32 gold = IM_COL32(214, 176, 86, 255), steel = IM_COL32(178, 183, 192, 255), steelDark = IM_COL32(120, 125, 135, 255);
  const ImU32 ceramic = IM_COL32(226, 150, 140, 255), copper = IM_COL32(205, 120, 70, 255);
  const ImU32 hot = ImGui::GetColorU32(kWarn), text = ImGui::GetColorU32(ImGuiCol_Text), muted = ImGui::GetColorU32(kMuted);
  const ImVec2 o = ImGui::GetCursorScreenPos();
  // The antenna's end sits at 0.8 of the board's width: the wire runs left from below it, past the
  // board's left edge, so the board moves right by what sticks out.
  const float endX = 0.8f, left = std::max(0.f, wire - bw * endX) + u * 0.3f;
  const ImVec2 b0(o.x + left, o.y + u * 0.7f), b1(b0.x + bw, b0.y + bh);
  const auto X = [&](float f) { return b0.x + bw * f; };
  const auto Y = [&](float f) { return b0.y + bh * f; };
  const auto label = [&](float size, ImVec2 at, ImU32 col, const char* str, float alignX = 0) {
    const ImVec2 ts = font->CalcTextSizeA(size, FLT_MAX, 0, str);
    dl->AddText(font, size, ImVec2(at.x - ts.x * alignX, at.y - ts.y / 2), col, str);
  };

  // the board
  dl->AddRectFilled(b0, b1, pcb, u * 0.25f);
  dl->AddRect(b0, b1, edge, u * 0.25f, 0, 1.5f);
  // USB-C at the top, over the edge
  const ImVec2 c0(X(0.29f), b0.y - u * 0.45f), c1(X(0.71f), Y(0.13f));
  dl->AddRectFilled(c0, c1, steel, u * 0.28f);
  dl->AddRectFilled(ImVec2(c0.x + u * 0.3f, c0.y + u * 0.3f), ImVec2(c1.x - u * 0.3f, c0.y + u * 0.75f), steelDark, u * 0.2f);
  label(fs * 0.8f, ImVec2(X(0.5f), (c0.y + c1.y) / 2 + u * 0.35f), pcb, "USB", 0.5f);
  // castellated pins: 12 down each side, 3 along the bottom
  const auto pin = [&](ImVec2 c) {
    dl->AddCircleFilled(c, u * 0.26f, gold, 20);
    dl->AddCircleFilled(c, u * 0.11f, pcb, 16);
  };
  for (int i = 0; i < 12; ++i)
    for (const float x : {X(0.075f), X(0.925f)}) pin(ImVec2(x, Y(0.075f + 0.066f * float(i))));
  for (int i = 0; i < 3; ++i) pin(ImVec2(X(0.3f + 0.2f * float(i)), Y(0.83f)));
  // GND and RST, third and fourth down on the right: shorted twice for DFU mode
  {
    const ImVec2 gnd(X(0.925f), Y(0.075f + 0.066f * 2)), rst(X(0.925f), Y(0.075f + 0.066f * 3));
    for (const ImVec2& c : {gnd, rst}) dl->AddCircle(c, u * 0.36f, hot, 20, 2.0f);
    label(fs * 0.85f, ImVec2(gnd.x - u * 0.45f, gnd.y), silk, "GND", 1.0f);
    label(fs * 0.85f, ImVec2(rst.x - u * 0.45f, rst.y), silk, "RST", 1.0f);
    // an arrow from the note to the pair
    const ImVec2 tipAt(gnd.x + u * 0.45f, (gnd.y + rst.y) / 2), from(b1.x + u * 1.8f, tipAt.y);
    dl->AddLine(from, ImVec2(tipAt.x + u * 0.3f, tipAt.y), hot, 2.0f);
    dl->AddTriangleFilled(ImVec2(tipAt.x + u * 0.35f, tipAt.y - u * 0.22f), ImVec2(tipAt.x + u * 0.35f, tipAt.y + u * 0.22f), tipAt, hot);
    label(fs, ImVec2(from.x + u * 0.25f, tipAt.y - fs * 0.6f), hot, "Double tap these pins");
    label(fs, ImVec2(from.x + u * 0.25f, tipAt.y + fs * 0.6f), hot, "together for DFU mode");
  }
  // the nRF52840: a square chip turned 45 degrees, its pins along each side
  {
    const ImVec2 c(X(0.5f), Y(0.45f));
    const float r = bw * 0.27f;   // centre to corner
    const ImVec2 q[] = {ImVec2(c.x, c.y - r), ImVec2(c.x + r, c.y), ImVec2(c.x, c.y + r), ImVec2(c.x - r, c.y)};
    for (int side = 0; side < 4; ++side) {   // pins: short lines out from each side
      const ImVec2 a0 = q[side], a1 = q[(side + 1) % 4];
      const ImVec2 n((a1.y - a0.y) / r * 0.7071f, -(a1.x - a0.x) / r * 0.7071f);   // outward normal
      for (int i = 1; i < 8; ++i) {
        const float t = float(i) / 8;
        const ImVec2 p(a0.x + (a1.x - a0.x) * t, a0.y + (a1.y - a0.y) * t);
        dl->AddLine(p, ImVec2(p.x + n.x * u * 0.16f, p.y + n.y * u * 0.16f), steel, 1.5f);
      }
    }
    dl->AddConvexPolyFilled(q, 4, IM_COL32(18, 19, 23, 255));
    dl->AddPolyline(q, 4, steelDark, ImDrawFlags_Closed, 1.2f);
    dl->AddCircleFilled(ImVec2(c.x, c.y - r * 0.62f), u * 0.07f, steelDark);   // pin 1
    label(fs * 0.7f, c, silk, "nRF52840", 0.5f);
  }
  // the ceramic chip antenna along the bottom edge; its right end is where the wire goes
  const ImVec2 a0(X(0.22f), Y(0.915f)), a1(X(endX), Y(0.97f));
  dl->AddRectFilled(a0, a1, ceramic, u * 0.06f);
  dl->AddRect(a0, a1, IM_COL32(170, 100, 95, 255), u * 0.06f);
  const ImVec2 end(a1.x, (a0.y + a1.y) / 2);
  // the wire: down from the end, then left, to scale
  const float wy = b1.y + u * 1.2f, th = u * 0.16f;
  dl->AddLine(end, ImVec2(end.x, wy), copper, th);
  dl->AddLine(ImVec2(end.x + th / 2, wy), ImVec2(end.x - wire, wy), copper, th);
  dl->AddCircleFilled(end, u * 0.24f, hot, 20);
  // labels: the solder point, the ceramic antenna, the wire's length with a dimension line
  label(fs, ImVec2(b1.x + u * 0.4f, end.y), hot, "Solder here");
  dl->AddLine(ImVec2(b1.x + u * 0.3f, end.y), ImVec2(end.x + u * 0.35f, end.y), hot, 1.5f);
  label(fs * 0.85f, ImVec2(b1.x + u * 0.4f, Y(0.83f)), muted, "Ceramic antenna");
  dl->AddLine(ImVec2(b1.x + u * 0.3f, Y(0.83f)), ImVec2(X(0.66f), a0.y), muted, 1.0f);
  const float dy = wy + u * 0.7f;
  dl->AddLine(ImVec2(end.x - wire, dy), ImVec2(end.x, dy), silk, 1.0f);
  for (const float x : {end.x - wire, end.x}) dl->AddLine(ImVec2(x, dy - u * 0.2f), ImVec2(x, dy + u * 0.2f), silk, 1.0f);
  label(fs, ImVec2(end.x - wire / 2, dy + u * 0.55f), text, "31.2 mm", 0.5f);
  ImGui::Dummy(ImVec2(left + bw + fs * 15, dy + u * 1.0f - o.y));
}

// The Holyiot 21017 standing up: the USB plug at the top, the antenna out to the left from the round
// hinge at the bottom. The thin clear strip near the plug is the LED window: a magnet brought close
// there starts the bootloader (a reed switch inside). Drawn lying down (plug to the right, antenna
// up, proportions from a photo) and turned a quarter left.
void HolyiotMagnet() {
  const float fs = ImGui::GetFontSize(), H = fs * 3.8f;   // H: the case's width
  ImDrawList* dl = ImGui::GetWindowDrawList();
  ImFont* font = ImGui::GetFont();
  const ImU32 black = IM_COL32(34, 36, 41, 255), shade = IM_COL32(24, 25, 29, 255), hi = IM_COL32(78, 83, 93, 255);
  const ImU32 edge = IM_COL32(88, 93, 104, 255), steel = IM_COL32(198, 202, 209, 255), steelHi = IM_COL32(225, 228, 233, 255);
  const ImU32 steelDark = IM_COL32(112, 117, 127, 255), red = IM_COL32(210, 70, 60, 255), grey = IM_COL32(170, 175, 185, 255);
  const ImU32 window = IM_COL32(170, 215, 245, 235), glow = IM_COL32(120, 190, 255, 60);
  const ImU32 hot = ImGui::GetColorU32(kWarn), text = ImGui::GetColorU32(ImGuiCol_Text);
  const ImVec2 o = ImGui::GetCursorScreenPos();
  const auto label = [&](float size, ImVec2 at, ImU32 col, const char* str, float alignX = 0) {
    const ImVec2 ts = font->CalcTextSizeA(size, FLT_MAX, 0, str);
    dl->AddText(font, size, ImVec2(at.x - ts.x * alignX, at.y - ts.y / 2), col, str);
  };
  // Lying-down coordinates (x toward the plug, y down, 0 at the case's lower left) to the screen,
  // turned a quarter left: x goes up, y goes right.
  const float antennaLen = H * 4.4f;
  const ImVec2 P(o.x + antennaLen + H * 0.6f, o.y + H * 3.3f);
  const auto S = [&](float x, float y) { return ImVec2(P.x + y, P.y - x); };
  const auto rect = [&](float x0, float y0, float x1, float y1, ImU32 col, float round, bool outline = false) {
    const ImVec2 a = S(x0, y0), b = S(x1, y1);
    const ImVec2 lo(std::min(a.x, b.x), std::min(a.y, b.y)), hi2(std::max(a.x, b.x), std::max(a.y, b.y));
    if (outline) dl->AddRect(lo, hi2, col, round, 0, 1.2f);
    else dl->AddRectFilled(lo, hi2, col, round);
  };
  // the antenna, up from the hinge (left on screen), tapering to a rounded tip
  const ImVec2 hinge(H * 0.2f, -H * 0.5f);
  {
    const float wb = H * 0.27f, wt = H * 0.21f, from = hinge.y - H * 0.45f, to = hinge.y - antennaLen;
    ImVec2 q[] = {S(hinge.x - wt, to), S(hinge.x + wt, to), S(hinge.x + wb, from), S(hinge.x - wb, from)};
    dl->AddConvexPolyFilled(q, 4, black);
    dl->AddCircleFilled(S(hinge.x, to), wt, black, 24);
    dl->AddLine(S(hinge.x + wt * 0.45f, to + H * 0.2f), S(hinge.x + wb * 0.45f, from - H * 0.1f), hi, 2.0f);
  }
  // the case
  rect(0, -H, H * 2.4f, 0, black, H * 0.14f);
  dl->AddLine(S(H * 0.6f, -H * 0.9f), S(H * 2.3f, -H * 0.9f), hi, 2.0f);
  rect(0, -H, H * 2.4f, 0, edge, H * 0.14f, true);
  // the hinge: round, a little wider than the case
  dl->AddCircleFilled(S(hinge.x, hinge.y), H * 0.42f, black, 32);
  dl->AddCircle(S(hinge.x, hinge.y), H * 0.42f, edge, 32, 1.2f);
  dl->AddCircleFilled(S(hinge.x, hinge.y), H * 0.16f, shade, 24);
  // the LED window near the plug: short, across the middle
  const float wx = H * 2.0f;
  rect(wx - H * 0.12f, -H * 0.68f, wx + H * 0.12f, -H * 0.32f, glow, H * 0.06f);
  rect(wx - H * 0.035f, -H * 0.66f, wx + H * 0.035f, -H * 0.34f, window, H * 0.03f);
  // the USB-A plug, short, with its two holes
  rect(H * 2.4f, -H * 0.86f, H * 3.15f, -H * 0.14f, steel, H * 0.04f);
  rect(H * 2.4f, -H * 0.86f, H * 3.15f, -H * 0.14f, steelDark, H * 0.04f, true);
  dl->AddLine(S(H * 2.45f, -H * 0.8f), S(H * 3.1f, -H * 0.8f), steelHi, 2.0f);
  for (const float sy : {-0.67f, -0.33f}) rect(H * 2.7f, H * sy - H * 0.08f, H * 2.95f, H * sy + H * 0.08f, steelDark, H * 0.02f);
  // the magnet to the right, an arrow left to the window
  const ImVec2 w = S(wx, 0);   // the window at the case's right edge
  const ImVec2 m0(w.x + H * 1.3f, w.y - H * 0.28f), m1(m0.x + H * 1.2f, w.y + H * 0.28f);
  dl->AddRectFilled(m0, ImVec2((m0.x + m1.x) / 2, m1.y), red, H * 0.06f, ImDrawFlags_RoundCornersLeft);
  dl->AddRectFilled(ImVec2((m0.x + m1.x) / 2, m0.y), m1, grey, H * 0.06f, ImDrawFlags_RoundCornersRight);
  label(fs * 0.85f, ImVec2(m0.x + H * 0.3f, w.y), IM_COL32(255, 255, 255, 255), "N", 0.5f);
  label(fs * 0.85f, ImVec2(m1.x - H * 0.3f, w.y), black, "S", 0.5f);
  label(fs, ImVec2(m1.x + H * 0.25f, w.y), text, "Bring a magnet close");
  const float ax0 = m0.x - H * 0.15f, ax1 = w.x + H * 0.15f;
  dl->AddLine(ImVec2(ax0, w.y), ImVec2(ax1 + H * 0.12f, w.y), hot, 2.0f);
  dl->AddTriangleFilled(ImVec2(ax1 + H * 0.16f, w.y - H * 0.13f), ImVec2(ax1 + H * 0.16f, w.y + H * 0.13f), ImVec2(ax1, w.y), hot);
  ImGui::Dummy(ImVec2(m1.x + fs * 12 - o.x, S(-H * 0.42f, 0).y + H * 0.2f - o.y));
}

// Nordic's PCA10059 from above, USB contacts to the left: the RESET button is the square one at the
// right end, next to the round white one (SW1).
void NordicReset() {
  const float fs = ImGui::GetFontSize(), u = fs * 1.6f;
  ImDrawList* dl = ImGui::GetWindowDrawList();
  ImFont* font = ImGui::GetFont();
  const ImU32 pcb = IM_COL32(30, 105, 175, 255), pcbEdge = IM_COL32(90, 150, 210, 255), gold = IM_COL32(214, 176, 86, 255);
  const ImU32 chip = IM_COL32(28, 30, 36, 255), white = IM_COL32(235, 235, 235, 255), steel = IM_COL32(170, 175, 185, 255);
  const ImU32 hot = ImGui::GetColorU32(kWarn), silk = IM_COL32(225, 235, 245, 255);
  const ImVec2 o = ImGui::GetCursorScreenPos();
  const auto label = [&](float size, ImVec2 at, ImU32 col, const char* str, float alignX = 0) {
    const ImVec2 ts = font->CalcTextSizeA(size, FLT_MAX, 0, str);
    dl->AddText(font, size, ImVec2(at.x - ts.x * alignX, at.y - ts.y / 2), col, str);
  };
  const float bw = u * 12, bh = u * 4.4f, plug = u * 3.2f;
  const ImVec2 b0(o.x, o.y + u * 1.6f), b1(b0.x + bw, b0.y + bh);
  // the narrower USB tongue, then the board
  const ImVec2 t0(b0.x, b0.y + bh * 0.12f), t1(b0.x + plug, b1.y - bh * 0.12f);
  dl->AddRectFilled(t0, t1, pcb, u * 0.15f);
  dl->AddRectFilled(ImVec2(b0.x + plug - u * 0.2f, b0.y), b1, pcb, u * 0.2f);
  dl->AddRect(ImVec2(b0.x + plug - u * 0.2f, b0.y), b1, pcbEdge, u * 0.2f, 0, 1.5f);
  for (int i = 0; i < 4; ++i) {   // USB contacts
    const float y = t0.y + (t1.y - t0.y) * (0.2f + 0.2f * float(i));
    dl->AddRectFilled(ImVec2(t0.x + u * 0.3f, y - u * 0.17f), ImVec2(t0.x + (i == 0 || i == 3 ? u * 2.5f : u * 2.1f), y + u * 0.17f), gold, u * 0.05f);
  }
  // castellations along both long edges
  for (int i = 0; i < 9; ++i) {
    const float x = b0.x + plug + u * 0.6f + float(i) * u * 0.95f;
    dl->AddRectFilled(ImVec2(x - u * 0.25f, b0.y), ImVec2(x + u * 0.25f, b0.y + u * 0.45f), gold, u * 0.05f);
    dl->AddRectFilled(ImVec2(x - u * 0.25f, b1.y - u * 0.45f), ImVec2(x + u * 0.25f, b1.y), gold, u * 0.05f);
  }
  // the chip
  const ImVec2 cc(b0.x + plug + u * 4.0f, (b0.y + b1.y) / 2);
  dl->AddRectFilled(ImVec2(cc.x - u * 1.3f, cc.y - u * 1.3f), ImVec2(cc.x + u * 1.3f, cc.y + u * 1.3f), chip, u * 0.08f);
  label(fs * 0.75f, cc, IM_COL32(150, 155, 165, 255), "nRF52840", 0.5f);
  // SW1 (round, white) and RESET (square)
  const ImVec2 sw1(b1.x - u * 2.6f, cc.y + u * 0.6f), rst(b1.x - u * 1.1f, cc.y + u * 0.4f);
  dl->AddRectFilled(ImVec2(sw1.x - u * 0.5f, sw1.y - u * 0.5f), ImVec2(sw1.x + u * 0.5f, sw1.y + u * 0.5f), steel, u * 0.08f);
  dl->AddCircleFilled(sw1, u * 0.33f, white, 20);
  dl->AddRectFilled(ImVec2(rst.x - u * 0.5f, rst.y - u * 0.6f), ImVec2(rst.x + u * 0.5f, rst.y + u * 0.6f), steel, u * 0.08f);
  dl->AddRectFilled(ImVec2(rst.x - u * 0.25f, rst.y - u * 0.35f), ImVec2(rst.x + u * 0.25f, rst.y + u * 0.35f), IM_COL32(110, 115, 125, 255), u * 0.05f);
  label(fs * 0.75f, ImVec2(rst.x, rst.y - u * 1.0f), silk, "RESET", 0.5f);
  dl->AddCircle(rst, u * 0.95f, hot, 28, 2.0f);
  // a callout below the board
  dl->AddLine(ImVec2(rst.x, rst.y + u * 0.95f), ImVec2(rst.x, b1.y + u * 0.9f), hot, 1.5f);
  label(fs, ImVec2(rst.x, b1.y + u * 1.3f), hot, "Press RESET", 0.5f);
  ImGui::Dummy(ImVec2(bw + u * 0.5f, b1.y - o.y + u * 1.9f));
}

// The Kuroshin dongle's board as it looks with the case open, chip side up and the USB plug at the bottom:
// the row of five pads at the top right, above the chip; the first and the last (RESET and GND) are
// shorted for DFU mode. Proportions measured from a photo of the board.
void KuroshinPads() {
  const float fs = ImGui::GetFontSize(), u = fs * 1.6f;   // drawing unit
  const float bw = u * 6, bh = u * 12.4f;                  // the board (photo: 2.07 tall per wide)
  const float top = fs * 1.6f;                             // room for "Short"
  const float w = bw + u * 0.4f, h = top + bh + u * 2.4f;
  const ImVec2 o = ImGui::GetCursorScreenPos();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  ImFont* font = ImGui::GetFont();
  const ImU32 board = IM_COL32(52, 58, 72, 255), edge = IM_COL32(120, 128, 145, 255), metal = IM_COL32(170, 175, 185, 255);
  const ImU32 pad = IM_COL32(150, 155, 165, 255), chip = IM_COL32(22, 24, 30, 255), part = IM_COL32(95, 100, 112, 255);
  const ImU32 hot = ImGui::GetColorU32(kWarn), text = ImGui::GetColorU32(ImGuiCol_Text), muted = ImGui::GetColorU32(kMuted);
  const auto centred = [&](float size, float x, float y, ImU32 col, const char* str) {
    const ImVec2 ts = font->CalcTextSizeA(size, FLT_MAX, 0, str);
    dl->AddText(font, size, ImVec2(x - ts.x / 2, y), col, str);
  };
  const ImVec2 b0(o.x + u * 0.2f, o.y + top), b1(b0.x + bw, b0.y + bh);
  const auto X = [&](float f) { return b0.x + bw * f; };   // fractions of the board, as in the photo
  const auto Y = [&](float f) { return b0.y + bh * f; };

  // USB plug, then the board (rounded top like the case)
  const float cx = X(0.5f);
  dl->AddRectFilled(ImVec2(cx - u * 1.7f, b1.y - u * 0.3f), ImVec2(cx + u * 1.7f, o.y + h), metal, u * 0.12f);
  dl->AddRect(ImVec2(cx - u * 1.7f, b1.y - u * 0.3f), ImVec2(cx + u * 1.7f, o.y + h), edge, u * 0.12f);
  dl->AddRectFilled(b0, b1, board, u * 0.9f, ImDrawFlags_RoundCornersTop);
  dl->AddRect(b0, b1, edge, u * 0.9f, ImDrawFlags_RoundCornersTop, 1.5f);

  // the four larger holes below the chip, on the left, and a few parts around the chip
  for (int i = 0; i < 4; ++i) {
    dl->AddCircleFilled(ImVec2(X(0.17f + 0.15f * float(i)), Y(0.74f)), u * 0.32f, metal);
    dl->AddCircleFilled(ImVec2(X(0.17f + 0.15f * float(i)), Y(0.74f)), u * 0.14f, board);
  }
  for (const auto& p : {ImVec2(0.12f, 0.36f), ImVec2(0.12f, 0.45f), ImVec2(0.12f, 0.55f), ImVec2(0.88f, 0.42f),
                        ImVec2(0.88f, 0.55f), ImVec2(0.35f, 0.66f), ImVec2(0.5f, 0.66f), ImVec2(0.8f, 0.66f)})
    dl->AddRectFilled(ImVec2(X(p.x) - u * 0.22f, Y(p.y) - u * 0.12f), ImVec2(X(p.x) + u * 0.22f, Y(p.y) + u * 0.12f), part, u * 0.04f);

  // the chip
  const float cs = bw * 0.56f, cy = Y(0.51f);
  dl->AddRectFilled(ImVec2(cx - cs / 2, cy - cs / 2), ImVec2(cx + cs / 2, cy + cs / 2), chip, u * 0.12f);
  for (int i = 0; i < 7; ++i) {   // pins on each side
    const float t = -cs / 2 + cs * (float(i) + 0.5f) / 7, pw = u * 0.05f, pl = u * 0.16f;
    dl->AddRectFilled(ImVec2(cx - cs / 2 - pl, cy + t - pw), ImVec2(cx - cs / 2, cy + t + pw), metal);
    dl->AddRectFilled(ImVec2(cx + cs / 2, cy + t - pw), ImVec2(cx + cs / 2 + pl, cy + t + pw), metal);
    dl->AddRectFilled(ImVec2(cx + t - pw, cy - cs / 2 - pl), ImVec2(cx + t + pw, cy - cs / 2), metal);
    dl->AddRectFilled(ImVec2(cx + t - pw, cy + cs / 2), ImVec2(cx + t + pw, cy + cs / 2 + pl), metal);
  }
  centred(fs * 0.85f, cx, cy - fs * 0.45f, muted, "nRF52840");

  // the pad row at the top right, numbers under it, and the wire bridging 1 and 5
  const float padY = Y(0.29f), step = bw * 0.1f, r = u * 0.2f;
  float padX[5];
  for (int i = 0; i < 5; ++i) padX[i] = X(0.73f) + (float(i) - 2) * step;
  dl->AddBezierCubic(ImVec2(padX[0], padY - r), ImVec2(padX[0], b0.y - fs * 0.2f), ImVec2(padX[4], b0.y - fs * 0.2f),
                     ImVec2(padX[4], padY - r), hot, u * 0.11f);
  for (int i = 0; i < 5; ++i) {
    const bool on = i == 0 || i == 4;
    dl->AddCircleFilled(ImVec2(padX[i], padY), r, on ? hot : pad);
    dl->AddCircleFilled(ImVec2(padX[i], padY), r * 0.4f, board);
    const char n[2] = {char('1' + i), 0};
    centred(fs * 0.95f, padX[i], padY + r + u * 0.08f, on ? hot : muted, n);
  }
  centred(fs * 1.1f, X(0.73f), o.y, text, "Short");
  ImGui::Dummy(ImVec2(w, h));
}

// Two to four big buttons side by side, centred; returns the one pressed (-1: none).
enum class Tint { None, Red, Green, Yellow };

// ---- tiles: big square choices with a drawn icon ----

using lh::ui::DrawIcon;
using lh::ui::Icon;
using lh::ui::kAccent;

ImU32 TintColor(Tint t) {
  switch (t) {
    case Tint::Red: return IM_COL32(240, 104, 100, 255);
    case Tint::Green: return IM_COL32(98, 206, 120, 255);
    case Tint::Yellow: return IM_COL32(242, 196, 82, 255);
    default: return kAccent;
  }
}

// Word-wrapped text in the current font, centred (or left-aligned); returns the number of lines.
// dl == nullptr only measures.
int WrapText(ImDrawList* dl, ImVec2 topLeft, float width, ImU32 col, const char* text, bool centred = true) {
  ImFont* f = ImGui::GetFont();
  const float size = ImGui::GetFontSize();
  const std::string all = text;
  std::string line;
  float y = topLeft.y;
  int lines = 0;
  const auto flush = [&] {
    if (dl) {
      const float w = ImGui::CalcTextSize(line.c_str()).x;
      dl->AddText(f, size, ImVec2(topLeft.x + (centred ? (width - w) / 2 : 0), y), col, line.c_str());
    }
    y += size * 1.25f;
    ++lines;
    line.clear();
  };
  size_t i = 0;
  while (i <= all.size()) {
    size_t j = all.find_first_of(" \n", i);
    if (j == std::string::npos) j = all.size();
    const std::string word = all.substr(i, j - i);
    const std::string next = line.empty() ? word : line + " " + word;
    if (!line.empty() && ImGui::CalcTextSize(next.c_str()).x > width) { flush(); line = word; }
    else line = next;
    if (j < all.size() && all[j] == '\n') flush();
    i = j + 1;
  }
  if (!line.empty()) flush();
  return lines;
}

struct TileSpec {
  Icon icon;
  const char* title;
  const char* subtitle = nullptr;
  Tint tint = Tint::None;
  const char* badge = nullptr;   // small tag in the corner, e.g. "Recommended"
};

// One tile at the cursor (w x h). wide: icon on the left, text beside it; else icon on top.
bool Tile(const TileSpec& t, float w, float h, bool wide) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 a = ImGui::GetCursorScreenPos(), b(a.x + w, a.y + h);
  const bool disabled = ImGui::GetCurrentContext()->CurrentItemFlags & ImGuiItemFlags_Disabled;   // inside BeginDisabled
  const bool pressed = ImGui::InvisibleButton(t.title, ImVec2(w, h));
  const bool hover = ImGui::IsItemHovered() && !disabled, held = ImGui::IsItemActive() && !disabled;
  const ImU32 accent = disabled ? IM_COL32(88, 92, 102, 255) : TintColor(t.tint);
  const ImVec4 av = ImGui::ColorConvertU32ToFloat4(accent);
  const float fs = ImGui::GetFontSize(), r = fs * 1.1f;
  dl->AddRectFilled(a, b, disabled ? IM_COL32(22, 24, 30, 200) : held ? IM_COL32(40, 44, 54, 235) : hover ? IM_COL32(44, 50, 64, 235) : IM_COL32(28, 31, 40, 225), r);
  dl->AddRect(a, b, hover ? accent : disabled ? IM_COL32(44, 47, 56, 255) : IM_COL32(62, 68, 82, 255), r, 0, hover ? 2.f : 1.f);
  if (hover) ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
  const ImU32 disc = ImGui::GetColorU32(ImVec4(av.x, av.y, av.z, hover ? 0.22f : 0.13f));
  const ImU32 titleCol = disabled ? IM_COL32(104, 108, 118, 255) : IM_COL32(236, 238, 242, 255);
  const ImU32 subCol = disabled ? IM_COL32(80, 84, 94, 255) : IM_COL32(158, 163, 174, 255);
  if (wide) {
    const float side = fs * 3.2f, pad = fs * 1.4f;
    const ImVec2 ic(a.x + pad + side * 0.85f, a.y + h / 2);
    dl->AddCircleFilled(ic, side * 0.85f, disc, 48);
    DrawIcon(dl, t.icon, ic, side, accent);
    const float tx = ic.x + side * 0.85f + pad, tw = b.x - tx - pad;
    ImGui::PushFont(g_title);
    const int tl = WrapText(nullptr, {}, tw, 0, t.title, false);
    const float th = ImGui::GetFontSize() * 1.25f * float(tl);
    ImGui::PopFont();
    const int sl = t.subtitle ? WrapText(nullptr, {}, tw, 0, t.subtitle, false) : 0;
    const float total = th + (sl ? 4 + fs * 1.25f * float(sl) : 0);
    float y = a.y + (h - total) / 2;
    ImGui::PushFont(g_title);
    WrapText(dl, ImVec2(tx, y), tw, titleCol, t.title, false);
    ImGui::PopFont();
    if (t.subtitle) WrapText(dl, ImVec2(tx, y + th + 4), tw, subCol, t.subtitle, false);
  } else {
    const float side = std::min(w * 0.30f, fs * 4.4f);
    const ImVec2 ic(a.x + w / 2, a.y + fs * 1.4f + side * 0.85f);
    dl->AddCircleFilled(ic, side * 0.85f, disc, 48);
    DrawIcon(dl, t.icon, ic, side, accent);
    float y = ic.y + side * 0.85f + fs * 1.0f;
    ImGui::PushFont(g_title);
    y += ImGui::GetFontSize() * 1.25f * float(WrapText(dl, ImVec2(a.x + 12, y), w - 24, titleCol, t.title)) + 4;
    ImGui::PopFont();
    if (t.subtitle) WrapText(dl, ImVec2(a.x + 16, y), w - 32, subCol, t.subtitle);
  }
  if (t.badge) {
    const ImVec2 bs = ImGui::CalcTextSize(t.badge);
    const ImVec2 p0(b.x - bs.x - 24, a.y + 10), p1(b.x - 10, a.y + 10 + bs.y + 8);
    dl->AddRectFilled(p0, p1, ImGui::GetColorU32(ImVec4(av.x, av.y, av.z, 0.2f)), bs.y);
    dl->AddText(ImVec2(p0.x + 7, p0.y + 4), accent, t.badge);
  }
  return pressed;
}

// Height a tall tile needs for its text at width w.
float TallTileHeight(const TileSpec& t, float w) {
  const float fs = ImGui::GetFontSize(), side = std::min(w * 0.30f, fs * 4.4f);
  float h = fs * 1.4f + side * 1.7f + fs * 1.0f;
  ImGui::PushFont(g_title);
  h += ImGui::GetFontSize() * 1.25f * float(WrapText(nullptr, {}, w - 24, 0, t.title)) + 4;
  ImGui::PopFont();
  if (t.subtitle) h += fs * 1.25f * float(WrapText(nullptr, {}, w - 32, 0, t.subtitle));
  return h + fs * 1.4f;
}

// A centred row of tall tiles, all as high as the tallest; returns the one pressed (-1: none).
int Tiles(const std::vector<TileSpec>& tiles, float maxSide = 0) {
  const float gap = ImGui::GetFontSize() * 1.5f;
  const float avail = ImGui::GetContentRegionAvail().x;
  const float n = float(tiles.size());
  if (maxSide <= 0) maxSide = ImGui::GetFontSize() * 20;
  const float w = std::min(maxSide, (avail - gap * (n - 1)) / n);
  float h = w * 0.8f;
  for (const auto& t : tiles) h = std::max(h, TallTileHeight(t, w));
  const float total = w * n + gap * (n - 1);
  const float x0 = ImGui::GetCursorPosX() + std::max(0.f, (avail - total) / 2);
  int pressed = -1;
  for (size_t i = 0; i < tiles.size(); ++i) {
    if (i) ImGui::SameLine(0, gap);
    else ImGui::SetCursorPosX(x0);
    ImGui::PushID(int(i));
    if (Tile(tiles[i], w, h, false)) pressed = int(i);
    ImGui::PopID();
  }
  return pressed;
}

// A centred grid of wide tiles (icon on the left), cols per row; returns the one pressed (-1: none).
// rowWidth: the whole row's width (0: as wide as two-column tiles allow), so rows of 2 and 3 line up.
int TileGrid(const std::vector<TileSpec>& tiles, int cols, float rowWidth = 0) {
  const float fs = ImGui::GetFontSize(), gap = fs * 1.2f;
  const float avail = ImGui::GetContentRegionAvail().x;
  const float w = rowWidth > 0 ? (std::min(rowWidth, avail) - gap * float(cols - 1)) / float(cols)
                               : std::min(fs * 34, (avail - gap * float(cols - 1)) / float(cols));
  // text column width as Tile lays it out (wide)
  const float side = fs * 3.2f, pad = fs * 1.4f, tw = w - (pad + side * 1.7f + pad) - pad;
  float h = fs * 8;
  for (const auto& t : tiles) {
    ImGui::PushFont(g_title);
    float th = ImGui::GetFontSize() * 1.25f * float(WrapText(nullptr, {}, tw, 0, t.title, false));
    ImGui::PopFont();
    if (t.subtitle) th += 4 + fs * 1.25f * float(WrapText(nullptr, {}, tw, 0, t.subtitle, false));
    h = std::max(h, th + fs * 2.4f);
  }
  const int used = std::min<int>(cols, int(tiles.size()));
  const float x0 = ImGui::GetCursorPosX() + std::max(0.f, (avail - (w * float(used) + gap * float(used - 1))) / 2);
  int pressed = -1;
  for (size_t i = 0; i < tiles.size(); ++i) {
    if (i % size_t(cols)) ImGui::SameLine(0, gap);
    else {
      if (i) ImGui::Dummy(ImVec2(0, gap - ImGui::GetStyle().ItemSpacing.y * 2));
      ImGui::SetCursorPosX(x0);
    }
    ImGui::PushID(int(i));
    if (Tile(tiles[i], w, h, true)) pressed = int(i);
    ImGui::PopID();
  }
  return pressed;
}

// A centred heading over a row of tiles.
void Prompt(const char* text) {
  ImGui::PushFont(g_title);
  const float avail = ImGui::GetContentRegionAvail().x;
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.f, (avail - ImGui::CalcTextSize(text).x) / 2));
  ImGui::TextUnformatted(text);
  ImGui::PopFont();
  ImGui::Dummy(ImVec2(0, ImGui::GetFontSize() * 0.6f));
}

// A soft blue box with a hint.
void Note(const char* text) {
  const float avail = ImGui::GetContentRegionAvail().x, w = std::min(avail, ImGui::GetFontSize() * 52);
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (avail - w) / 2);
  ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.16f, 0.24f, 0.38f, 0.75f));
  ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, ImGui::GetFontSize() * 0.6f);
  ImGui::BeginChild(text, ImVec2(w, 0), ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysUseWindowPadding);
  const float fs = ImGui::GetFontSize();
  const ImVec2 p = ImGui::GetCursorScreenPos();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 c(p.x + fs * 0.7f, p.y + fs * 0.6f);
  dl->AddCircle(c, fs * 0.62f, kAccent, 24, 1.6f);
  dl->AddLine(ImVec2(c.x, c.y - fs * 0.05f), ImVec2(c.x, c.y + fs * 0.32f), kAccent, 2);
  dl->AddCircleFilled(ImVec2(c.x, c.y - fs * 0.3f), 1.4f, kAccent);
  ImGui::Indent(fs * 2);
  Wrapped(text);
  ImGui::Unindent(fs * 2);
  ImGui::EndChild();
  ImGui::PopStyleVar();
  ImGui::PopStyleColor();
}

void Warning(const char* heading, const char* body) {
  ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.45f, 0.10f, 0.10f, 1));
  ImGui::BeginChild(heading, ImVec2(0, 0), ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysUseWindowPadding);
  ImGui::PushFont(g_title);
  ImGui::TextUnformatted(heading);
  ImGui::PopFont();
  Wrapped(body);
  ImGui::EndChild();
  ImGui::PopStyleColor();
}

void JobLog(const Job& job) {
  for (const auto& line : job.LogLines()) ImGui::BulletText("%s", line.c_str());
}

void ProgressBar(const Job& job) {
  const std::string text = job.StatusText();
  ImGui::TextUnformatted(text.empty() ? "Working..." : text.c_str());
  char pct[16];
  std::snprintf(pct, sizeof pct, "%.0f%%", job.Progress() * 100);
  ImGui::ProgressBar(job.Progress(), ImVec2(-1, 0), pct);
}

// A device openhouse doesn't know: its id, its likely radio, a link to report it, and the
// user's go-ahead to flash it anyway. True once accepted.
bool UnknownDevice(uint32_t hwid, RadioFamily family, const std::string& key, std::set<std::string>* accepted) {
  ImGui::Indent();
  ImGui::TextColored(kWarn, "Unknown device (ID %08X). Device type: %s", hwid,
                     family == RadioFamily::Tracker10 ? "older hardware (VIVE 1.0, neo)" : "newer hardware (wm3)");
  ImGui::TextColored(kWarn, "It might not work. Report the ID on GitHub");
  if (ImGui::SmallButton(("Report on GitHub##" + key).c_str())) {
    char url[160];
    std::snprintf(url, sizeof url, "https://github.com/YusufOzmen01/openhouse/issues/new?title=New%%20device%%20ID%%3A%%20%08X", hwid);
    OpenInFileManager(Path(url));
  }
  ImGui::SameLine();
  bool ok = accepted->count(key) > 0;
  if (ImGui::Checkbox(("I accept##" + key).c_str(), &ok)) { if (ok) accepted->insert(key); else accepted->erase(key); }
  ImGui::Unindent();
  return ok;
}


// SteamVR's own 1.0 base station firmware: the newest numbered lighthouse_tx_htc_2_0 image
// (436), never the rescue ones.
fs::path StockBaseStationImage(const fs::path& steamvr) {
  fs::path best;
  int bestVersion = -1;
  std::error_code ec;
  for (const auto& e : fs::directory_iterator(steamvr / "tools" / "lighthouse" / "firmware" / "lighthouse_tx" / "archive" / "htc_2.0", ec)) {
    const std::string name = Utf8(e.path().filename());
    const std::string prefix = "lighthouse_tx_htc_2_0-";
    if (name.rfind(prefix, 0) != 0 || e.path().extension() != ".bin") continue;
    const int version = std::atoi(name.c_str() + prefix.size());
    if (version > bestVersion) { bestVersion = version; best = e.path(); }
  }
  return best;
}

// The lhbs image in the package's firmwares/basestation folder.
fs::path OurBaseStationImage() {
  std::error_code ec;
  for (const auto& e : fs::directory_iterator(FirmwareFolder() / "basestation", ec))
    if (e.path().extension() == ".bin") return e.path();
  return {};
}

// A device row's radio: its colour (stock: none; Doorman: green when current, yellow when older)
// and what to show next to it.
struct RadioState {
  const ImVec4* color = nullptr;
  std::string text;
};
RadioState DescribeRadio(uint32_t version, RadioFamily family) {
  static std::map<RadioFamily, uint32_t> ours;
  if (!ours.count(family)) ours[family] = OurRadioVersion(family);
  char text[64];
  if (!version) return {nullptr, "Radio: unknown"};
  if (!IsDoormanRadio(version)) {
    std::snprintf(text, sizeof text, "Radio: stock %u", version);
    return {nullptr, text};
  }
  const bool current = ours[family] && version >= ours[family];
  std::snprintf(text, sizeof text, current ? "Radio: doorman %08X" : "Radio: doorman %08X (older)", version);
  return {current ? &kGood : &kWarn, text};
}

// Device lists fill the page down to what's under them: a warning line, Flash and Back.
float DeviceListReserve() {
  const ImGuiStyle& st = ImGui::GetStyle();
  return ImGui::GetTextLineHeightWithSpacing() + ImGui::GetFrameHeight() * 1.6f + st.ItemSpacing.y +
         ImGui::GetFrameHeightWithSpacing() + st.ItemSpacing.y * 2;
}

// The image for a board on Nordic's DFU bootloader (DoormanDongleInfo::board).
fs::path NordicImage(int board) {
  return FirmwareFolder() / "dongle" / (board == 3 ? "doorman-kuroshin.hex" : board == 2 ? "doorman-pca10059.hex" : "doorman-holyiot.hex");
}

// The dongle firmware this package carries (firmwares/dongle/bank/version.txt), 0 if it has none.
uint16_t PackagedDongleVersion() {
  std::ifstream in(FirmwareFolder() / "dongle" / "bank" / "version.txt");
  std::string text;
  if (!(in >> text)) return 0;
  try { return uint16_t(std::stoul(text, nullptr, 16)); } catch (...) { return 0; }
}

// A device's radio is ours but not the packaged one.
bool RadioNeedsUpdate(const DongleSlot& s) {
  if (!s.answered || !s.device.model.known || !IsDoormanRadio(s.device.radioVersion)) return false;
  static std::map<RadioFamily, uint32_t> ours;
  if (!ours.count(s.device.model.family)) ours[s.device.model.family] = OurRadioVersion(s.device.model.family);
  const uint32_t v = ours[s.device.model.family];
  return v && s.device.radioVersion != v;
}

fs::path DongleImage(bool softDevice) {
  return FirmwareFolder() / "dongle" / (softDevice ? "doorman-dongle-sd.uf2" : "doorman-dongle-nosd.uf2");
}

}  // namespace

void Job::Start(std::function<bool(Job&)> work, bool devices) {
  Reset();
  running_ = true;
  thread_ = std::thread([this, work, devices] {
    bool ok = false;
    std::unique_lock lock(g_deviceMutex, std::defer_lock);
    if (devices) lock.lock();  // a stopped scan may still be finishing
    try {
      ok = work(*this);
    } catch (const std::exception& e) {
      Fail(e.what());
    }
    ok_ = ok;
    finished_ = true;
    running_ = false;
  });
}

void Ui::SetUpImGui() {
  ImGuiIO& io = ImGui::GetIO();
  io.IniFilename = nullptr;
  io.Fonts->Clear();
  // ImGui's own pixel font: sharp only at whole multiples of its 13 px.
  ImFontConfig font;
  font.OversampleH = font.OversampleV = 1;
  font.PixelSnapH = true;
  font.SizePixels = 13.f;
  io.Fonts->AddFontDefault(&font);
  font.SizePixels = 26.f;
  g_title = io.Fonts->AddFontDefault(&font);
  font.SizePixels = 39.f;
  g_huge = io.Fonts->AddFontDefault(&font);
  ImGui::StyleColorsDark();
  ImGuiStyle& style = ImGui::GetStyle();
  style.WindowRounding = 0;
  style.FrameRounding = 2;
  style.FramePadding = ImVec2(10, 7);
  style.ItemSpacing = ImVec2(10, 9);
  style.WindowPadding = ImVec2(18, 14);
  lh::ui::ApplyTheme();
}

Ui::Ui() {
  steamvr_ = FindSteamVR();
  askSteamVR_ = steamvr_.root.empty();
  RefreshStatus();
}

bool Ui::PackageNewer() {
  if (!status_.openhouse) return false;
  // Installed before the version marker (1.0.3 or older): anything here is newer.
  if (status_.version.empty()) return true;
  if (!order_ || orderFor_ != status_.version) {
    orderFor_ = status_.version;
    order_ = std::make_shared<std::atomic<int>>(-1);
    const std::string installed = status_.version;
    // Numbers alone answer at once; a GitHub compare (two builds of one version) on a thread.
    // Anything that isn't a clear "newer" (older, the same, no answer) stays a reinstall.
    static const char* const kOrder[] = {"older", "the same", "newer", "unknown"};
    const auto decided = [installed](lh::BuildOrder o, const char* how) {
      std::fprintf(stderr, "openhouse-installer: installed v%s, this package v%s is %s (%s): %s\n", installed.c_str(),
                   lh::kVersion, kOrder[int(o)], how, o == lh::BuildOrder::Newer ? "update" : "reinstall");
    };
    lh::BuildOrder quick = lh::CompareBuilds(installed, lh::kVersion, [](const std::string&) { return std::string(); });
    // Two builds of one version: whichever was built later, when both say (no GitHub needed).
    if (quick == lh::BuildOrder::Unknown && status_.buildTime > 0)
      quick = LH_BUILD_TIME > status_.buildTime ? lh::BuildOrder::Newer
            : LH_BUILD_TIME < status_.buildTime ? lh::BuildOrder::Older : lh::BuildOrder::Same;
    if (quick != lh::BuildOrder::Unknown) { order_->store(int(quick)); decided(quick, "by version"); }
    else std::thread([order = order_, installed, decided] {
      const lh::BuildOrder o = lh::CompareBuilds(installed, lh::kVersion, lh::GithubGet);
      order->store(int(o));
      decided(o, "GitHub compare");
    }).detach();
  }
  return order_->load() == int(lh::BuildOrder::Newer);
}

bool Ui::Replaced() const {
  if (status_.openhouse || !status_.driverFound) return false;
  std::error_code ec;
  const fs::path data = DataDirectory();
  return status_.backup || fs::exists(data / "package", ec) || fs::exists(data / "openhouse-overlay.vrmanifest", ec);
}

int Ui::InstallAction() { return Replaced() ? 3 : !status_.openhouse ? 0 : PackageNewer() ? 1 : 2; }

void Ui::RefreshStatus() { status_ = steamvr_.root.empty() ? DriverStatus{} : ReadStatus(steamvr_.root); }

void Ui::ShowForTest(const std::string& page) {
  if (page == "kuroshin") { Go(Page::Dongle); dongleBoard_ = 3; dongleStep_ = DongleStep::Detect; }
  else if (page == "dongle") Go(Page::Dongle);
  else if (page == "holyiot") { Go(Page::Dongle); dongleBoard_ = 1; dongleStep_ = DongleStep::Detect; }
  else if (page == "nordic") { Go(Page::Dongle); dongleBoard_ = 2; dongleStep_ = DongleStep::Detect; }
  else if (page == "promicro") { Go(Page::Dongle); dongleBoard_ = 0; dongleStep_ = DongleStep::Detect; }
  else if (page == "home") Go(Page::Home);
  else if (page == "pick") Go(Page::DongleUpdatePick);
  else if (page == "tools") Go(Page::Tools);
  else if (page == "flash") Go(Page::FlashMenu);
  else if (page == "doorman") Go(Page::DoormanMenu);
  else if (page == "radio") Go(Page::RadioMenu);
  else if (page == "wireless") Go(Page::RadioWireless);
  else if (page == "wired") Go(Page::RadioWired);
  else if (page == "base") Go(Page::BaseStation);
  else if (page == "restore") Go(Page::RestoreMenu);
  else if (page == "update") {
    testOnly_ = true;   // a snapshot never updates anything
    const auto dongles = FindDoormanDongles();
    if (!dongles.empty()) updateDongle_ = dongles.front();
    Go(Page::DongleUpdate);
  }
}

void Ui::Go(Page page) {
  baseDrives_.Stop();
  dongleSeen_.Stop();
  wired_.Stop();
  wireless_.Stop();
  homeDongles_.Stop();
  homeWatching_ = false;
  updateSlots_.Stop();
  if (!job_.Running()) job_.Reset();
  page_ = page;
  checked_.clear();
  RefreshStatus();
  installAction_ = InstallAction();
  switch (page) {
    case Page::BaseStation:
    case Page::RestoreBase:
      baseStep_ = BaseStep::Warning;
      confirmed10_ = false;
      break;
    case Page::Dongle: dongleStep_ = DongleStep::Board; break;
    case Page::RadioWired:
    case Page::RestoreRadio:
      radioStep_ = RadioStep::Choose;
      wired_.Start(FindWiredDevices, 2.0);
      break;
    case Page::DongleUpdate: {
      std::lock_guard l(updateMutex_);
      updateResults_.clear();
      updateSnapshot_.clear();
      const DoormanDongleInfo dongle = updateDongle_;
      updateSlots_.Start([dongle] { return ScanDongleSlots(dongle); }, 2.0);
      break;
    }
    case Page::DeviceType: {
      std::lock_guard l(kindsMutex_);
      kinds_.clear();  // read again: a device may have been switched elsewhere meanwhile
      kindTarget_.clear();
      wired_.Start(FindWiredDevices, 2.0);
      break;
    }
    case Page::RadioWireless:
    case Page::RestoreRadioWireless:
      radioStep_ = RadioStep::Choose;
      stockChoice_.clear();
      stockModels_ = steamvr_.root.empty() ? std::vector<StockModel>{} : StockModels(steamvr_.root);
      wireless_.Start(FindWirelessDevices, 3.0);
      break;
    default: break;
  }
}

void Ui::Back(Page to) {
  if (BackButton(to == Page::Home ? "< Home" : "< Back")) Go(to);
}

bool Ui::BackButton(const char* label) {
  ImGui::Spacing();
  ImGui::BeginDisabled(job_.Running());
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(ImGui::GetFontSize() * 1.2f, ImGui::GetFontSize() * 0.7f));
  const bool pressed = ImGui::Button(label);
  ImGui::PopStyleVar();
  ImGui::EndDisabled();
  return pressed;
}

void Ui::Draw(float width, float height) {
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(ImVec2(width, height));
  ImGui::Begin("openhouse installer", nullptr,
               ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);
  lh::ui::Snow(ImGui::GetWindowDrawList(), ImVec2(0, 0), ImVec2(width, height));
  Header();
  ImGui::Separator();
  ImGui::Spacing();
  ImGui::BeginChild("page", ImVec2(0, -(ImGui::GetTextLineHeightWithSpacing() + ImGui::GetStyle().ItemSpacing.y * 2)));
  switch (page_) {
    case Page::Home: Home(); break;
    case Page::SelfUpdate: SelfUpdatePage(); break;
    case Page::Install: InstallPage(true); break;
    case Page::Uninstall: InstallPage(false); break;
    case Page::FlashMenu: FlashMenu(); break;
    case Page::BaseStation: BaseStationPage(false); break;
    case Page::DoormanMenu: DoormanMenu(); break;
    case Page::Dongle: DonglePage(); break;
    case Page::RadioMenu: RadioMenu(); break;
    case Page::RadioWired: RadioWiredPage(false); break;
    case Page::RadioWireless: RadioWirelessPage(false); break;
    case Page::RestoreRadioWireless: RadioWirelessPage(true); break;
    case Page::RestoreRadioMenu: RestoreRadioMenu(); break;
    case Page::RestoreMenu: RestoreMenu(); break;
    case Page::RestoreBase: BaseStationPage(true); break;
    case Page::RestoreRadio: RadioWiredPage(true); break;
    case Page::DeviceType: DeviceTypePage(); break;
    case Page::DongleUpdatePick: DongleUpdatePick(); break;
    case Page::DongleUpdate: DongleUpdatePage(); break;
    case Page::Tools: ToolsPage(); break;
  }
  ImGui::EndChild();
  Footer();
  SteamVRPrompt();
  ImGui::End();
}

void Ui::Header() {
  const float side = ImGui::GetFontSize() * 4;
  if (logo_) {
    ImGui::Image(static_cast<ImTextureID>(logo_), ImVec2(side, side));
    ImGui::SameLine();
  }
  ImGui::BeginGroup();
  ImGui::PushFont(g_huge);
  ImGui::TextUnformatted("openhouse");
  ImGui::PopFont();
  ImGui::TextColored(kMuted, "Installer v%s", lh::kVersion);
  ImGui::EndGroup();
}

// Bottom: what's installed now, and where SteamVR is.
void Ui::Footer() {
  ImGui::Separator();
  if (steamvr_.root.empty()) {
    ImGui::TextColored(kBad, "SteamVR not found");
  } else if (!status_.driverFound) {
    ImGui::TextColored(kBad, "SteamVR's lighthouse driver not found");
  } else if (status_.openhouse) {
    if (status_.version.empty()) ImGui::TextColored(kGood, "Installed: openhouse (1.0.3 or older)");
    else ImGui::TextColored(kGood, "Installed: openhouse v%s", status_.version.c_str());
  } else {
    ImGui::TextColored(kMuted, "Installed: SteamVR's own driver");
  }
  const char* label = steamvr_.root.empty() ? "Choose SteamVR folder" : "Change SteamVR folder";
  if (!steamvr_.root.empty()) {
    ImGui::SameLine(0, ImGui::GetFontSize() * 2);
    // Long paths lose their start, so the button always fits.
    const float room = ImGui::GetWindowWidth() - ImGui::GetCursorPosX() - ImGui::GetStyle().WindowPadding.x -
                       ImGui::CalcTextSize(label).x - ImGui::GetFontSize() * 3;
    if (FlatpakSteam(steamvr_.root)) {
      // The path says little here; that it's Flatpak's matters.
      ImGui::TextColored(kWarn, "SteamVR: Flatpak (not fully tested)");
    } else {
      std::string where = "SteamVR: " + Utf8(steamvr_.root);
      while (where.size() > 12 && ImGui::CalcTextSize(where.c_str()).x > room) where = "SteamVR: ..." + where.substr(13);
      ImGui::TextColored(kMuted, "%s", where.c_str());
    }
  }
  ImGui::SameLine(std::max(ImGui::GetCursorPosX(), ImGui::GetWindowWidth() - ImGui::GetStyle().WindowPadding.x -
                                                         ImGui::CalcTextSize(label).x - ImGui::GetStyle().FramePadding.x * 2));
  ImGui::BeginDisabled(job_.Running());
  if (ImGui::SmallButton(label)) askSteamVR_ = true;
  ImGui::EndDisabled();
}

void Ui::SteamVRPrompt() {
  if (askSteamVR_) {
    ImGui::OpenPopup("SteamVR folder");
    askSteamVR_ = false;
    std::snprintf(steamvrInput_, sizeof steamvrInput_, "%s", Utf8(steamvr_.root).c_str());
    steamvrError_.clear();
  }
  ImGui::SetNextWindowSize(ImVec2(ImGui::GetFontSize() * 44, 0));
  if (!ImGui::BeginPopupModal("SteamVR folder", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;
  if (steamvr_.root.empty()) {
    ImGui::TextColored(kWarn, "SteamVR not found. Pick its folder");
  }
  ImGui::TextColored(kMuted, "Usually Steam/steamapps/common/SteamVR");
#ifndef _WIN32
  ImGui::TextColored(kMuted, "Flatpak Steam isn't fully tested");
#endif
  ImGui::Spacing();
  ImGui::SetNextItemWidth(-ImGui::GetFontSize() * 7);
  ImGui::InputText("##steamvr", steamvrInput_, sizeof steamvrInput_);
  ImGui::SameLine();
  if (ImGui::Button("Browse...", ImVec2(-1, 0))) {
    const std::string picked = PickFolder("Choose the SteamVR folder");
    if (!picked.empty()) std::snprintf(steamvrInput_, sizeof steamvrInput_, "%s", picked.c_str());
  }
  if (!steamvrError_.empty()) ImGui::TextColored(kBad, "%s", steamvrError_.c_str());
  ImGui::Spacing();
  if (BigButton("Use this folder", ImGui::GetFontSize() * 14)) {
    fs::path chosen = Path(steamvrInput_);
    // A Steam library or Steam itself picked by mistake: the SteamVR folder in it.
    for (const fs::path& c : {chosen, chosen / "SteamVR", chosen / "common" / "SteamVR", chosen / "steamapps" / "common" / "SteamVR"})
      if (IsSteamVRFolder(c)) { chosen = c; break; }
    if (IsSteamVRFolder(chosen)) {
      steamvr_.root = chosen;
      steamvr_.saved = true;
      RememberSteamVR(chosen);
      RefreshStatus();
      ImGui::CloseCurrentPopup();
    } else {
      steamvrError_ = "That isn't a SteamVR folder";
    }
  }
  ImGui::SameLine();
  if (BigButton(steamvr_.root.empty() ? "Not now" : "Cancel", ImGui::GetFontSize() * 10)) ImGui::CloseCurrentPopup();
  ImGui::EndPopup();
}

// Linux, before talking to devices: the udev rule that lets this user open them.
bool Ui::DeviceAccess() {
  if (Now() - accessChecked_ > 2.0) {
    haveAccess_ = DeviceAccessInstalled();
    accessChecked_ = Now();
  }
  if (haveAccess_) return true;
  ImGui::TextColored(kWarn, "Device access isn't set up");
  ImGui::BeginDisabled(closeJob_.Running());
  if (BigButton(closeJob_.Running() ? "Setting up..." : "Set up device access")) {
    closeJob_.Start([](Job& j) {
      std::string error;
      if (!InstallDeviceAccess(&error)) { j.Fail(error); return false; }
      return true;
    }, false);
    accessChecked_ = -1;
  }
  ImGui::EndDisabled();
  if (closeJob_.Finished()) {
    if (!closeJob_.Ok()) ImGui::TextColored(kBad, "Failed: %s", closeJob_.Error().c_str());
    accessChecked_ = -1;
  }
  return false;
}

bool Ui::RuntimesClosed() {
  if (Now() - runningChecked_ > 1.0) {
    running_ = RunningRuntimes();
    runningChecked_ = Now();
  }
  if (running_.empty()) return true;
  // --install: the user already chose to update from the overlay.
  if (autoInstall_ && !autoClosed_ && !closeJob_.Running()) {
    autoClosed_ = true;
    closeJob_.Start([](Job& j) {
      if (!CloseRuntimes()) { j.Fail("Couldn't close them, close them manually"); return false; }
      return true;
    }, false);
  }
  std::string list;
  for (const auto& n : running_) list += (list.empty() ? "" : ", ") + n;
  ImGui::TextColored(kWarn, "Close these first: %s", list.c_str());
  ImGui::BeginDisabled(closeJob_.Running());
  if (BigButton(closeJob_.Running() ? "Closing..." : "Close them")) {
    closeJob_.Start([](Job& j) {
      if (!CloseRuntimes()) { j.Fail("Couldn't close them, close them manually"); return false; }
      return true;
    }, false);
  }
  ImGui::EndDisabled();
  if (closeJob_.Finished() && !closeJob_.Ok()) ImGui::TextColored(kBad, "%s", closeJob_.Error().c_str());
  return false;
}

// ---- home ----

// Two rows of three wide tiles, all as high as the tallest; disabled[i] greys tile i out. Returns
// the one pressed (-1: none).
int TileRows(const std::vector<TileSpec>& tiles, const std::vector<bool>& disabled) {
  const float fs = ImGui::GetFontSize(), gap = fs * 1.2f, avail = ImGui::GetContentRegionAvail().x;
  const float rowWidth = std::min(fs * 34, (avail - gap) / 2) * 2 + gap;
  const float w = (std::min(rowWidth, avail) - gap * 2) / 3;
  const float side = fs * 3.2f, pad = fs * 1.4f, tw = w - (pad + side * 1.7f + pad) - pad;
  float h = fs * 8;
  for (const auto& t : tiles) {
    ImGui::PushFont(g_title);
    float th = ImGui::GetFontSize() * 1.25f * float(WrapText(nullptr, {}, tw, 0, t.title, false));
    ImGui::PopFont();
    if (t.subtitle) th += 4 + fs * 1.25f * float(WrapText(nullptr, {}, tw, 0, t.subtitle, false));
    h = std::max(h, th + fs * 2.4f);
  }
  const float x0 = ImGui::GetCursorPosX() + std::max(0.f, (avail - (w * 3 + gap * 2)) / 2);
  int pressed = -1;
  for (size_t i = 0; i < tiles.size(); ++i) {
    if (i % 3) ImGui::SameLine(0, gap);
    else {
      if (i) ImGui::Dummy(ImVec2(0, gap - ImGui::GetStyle().ItemSpacing.y * 2));
      ImGui::SetCursorPosX(x0);
    }
    ImGui::PushID(int(i));
    ImGui::BeginDisabled(i < disabled.size() && disabled[i]);
    if (Tile(tiles[i], w, h, true)) pressed = int(i);
    ImGui::EndDisabled();
    ImGui::PopID();
  }
  return pressed;
}

void Ui::Home() {
  ImGui::Dummy(ImVec2(0, ImGui::GetFontSize() * 2));
  Prompt("What would you like to do?");
  // Doorman Dongles on other firmware than this package's.
  if (!homeWatching_) {
    homeDongles_.Start(FindDoormanDongles, 3.0);
    homeWatching_ = true;
  }
  const uint16_t packaged = PackagedDongleVersion();
  int outdated = 0;
  for (const auto& d : homeDongles_.Value()) outdated += packaged && d.firmware != packaged;
  static std::string installSub;
  static char dongleSub[64];
  const bool installed = status_.openhouse;
  const bool replaced = Replaced();
  const bool newer = installed && PackageNewer();
  if (replaced) installSub = "SteamVR replaced the openhouse driver";
  else if (!installed) installSub = std::string("Installs openhouse v") + lh::kVersion + " to SteamVR";
  else installSub = std::string(newer ? "Updates openhouse to v" : "Installs openhouse v") + lh::kVersion + (newer ? "" : " again");
  if (outdated) std::snprintf(dongleSub, sizeof dongleSub, outdated == 1 ? "A dongle has an update" : "%d dongles have an update", outdated);
  else std::snprintf(dongleSub, sizeof dongleSub, "No update available");
  const std::vector<TileSpec> tiles{
      replaced ? TileSpec{Icon::Wrench, "Repair Driver", installSub.c_str(), Tint::Yellow}
               : TileSpec{installed ? (newer ? Icon::Download : Icon::Refresh) : Icon::Download,
                          installed ? (newer ? "Update Driver" : "Reinstall Driver") : "Install Driver", installSub.c_str(),
                          installed && !newer ? Tint::Yellow : Tint::Green},
      {Icon::Trash, "Uninstall Driver",
       replaced ? "Cleans up what's left of openhouse" : installed ? "Puts the SteamVR Lighthouse driver back" : "openhouse isn't installed", Tint::Red},
      {Icon::Settings, "Tools", "Dongle manager and device converter"},
      {Icon::Chip, "Install or Update Firmware", "For base stations, dongles, trackers and controllers"},
      {Icon::Restore, "Restore Firmware", "Puts the original firmware back"},
      {Icon::Dongle, "Update Dongles", dongleSub, outdated ? Tint::Yellow : Tint::None}};
  switch (TileRows(tiles, {false, !installed && !replaced, false, false, false, outdated == 0})) {
    case 0: Go(Page::Install); break;
    case 1: Go(Page::Uninstall); break;
    case 2: Go(Page::Tools); break;
    case 3: Go(Page::FlashMenu); break;
    case 4: Go(Page::RestoreMenu); break;
    case 5: Go(Page::DongleUpdatePick); break;
    default: break;
  }
}

// ---- tools ----

void Ui::ToolsPage() {
  Title("Tools");
  ImGui::Dummy(ImVec2(0, ImGui::GetFontSize()));
#ifdef _WIN32
  static const fs::path manager = ExeDirectory() / "apps" / "doorman-dongle-manager.exe";
#else
  static const fs::path manager = ExeDirectory() / "apps" / "doorman-dongle-manager";
#endif
  switch (Tiles({{Icon::Dongle, "Dongle Manager", "Pair devices and see your Doorman dongles"},
                 {Icon::Tracker, "Device Type", "Convert your devices into trackers and vice versa"}})) {
    case 0: StartApp(manager); break;
    case 1: Go(Page::DeviceType); break;
    default: break;
  }
  Back(Page::Home);
}

// ---- install / uninstall ----

void Ui::InstallPage(bool install) {
  // What Install does here, decided when the page opened (the status changes once it's done).
  static const char* const kActions[] = {"Install", "Update", "Reinstall", "Repair"};
  static const char* const kDoing[] = {"Installing", "Updating", "Reinstalling", "Repairing"};
  static const char* const kDone[] = {"installed", "updated", "reinstalled", "repaired"};
  // Until it starts, a GitHub compare still going may turn a reinstall into an update.
  if (install && !job_.Running() && !job_.Finished()) installAction_ = InstallAction();
  const int a = install ? installAction_ : 0;
  Title(install ? (std::string(kActions[a]) + " Driver").c_str() : "Uninstall Driver");
  if (job_.Finished() && !statusFresh_) {
    RefreshStatus();
    statusFresh_ = true;
  }
  if (job_.Finished()) {
    if (job_.Ok()) {
      ImGui::PushFont(g_title);
      ImGui::TextColored(kGood, "%s", install ? (std::string("openhouse has been ") + kDone[a]).c_str() : "openhouse has been uninstalled");
      ImGui::PopFont();
      // Started from the overlay (Update, Fix): VR was closed for this.
#ifdef _WIN32
      if (autoInstall_) ImGui::TextUnformatted("You can start SteamVR again");
#else
      if (autoInstall_) ImGui::TextUnformatted("You can start SteamVR or Monado again");
#endif
      if (install) DongleUpdateOffer();
    } else {
      ImGui::PushFont(g_title);
      ImGui::TextColored(kBad, "%s", install ? (std::string(kActions[a]) + " failed").c_str() : "Uninstall failed");
      ImGui::PopFont();
      ImGui::TextUnformatted(job_.Error().c_str());
    }
    Back(Page::Home);
    return;
  }
  if (job_.Running()) {
    ImGui::Text("%s openhouse...", install ? kDoing[a] : "Uninstalling");
    return;
  }
  if (steamvr_.root.empty()) {
    ImGui::TextColored(kBad, "SteamVR not found");
    if (BigButton("Choose SteamVR folder")) askSteamVR_ = true;
    Back(Page::Home);
    return;
  }
  if (install) {
    if (!PackageHasDriver()) {
      ImGui::TextColored(kBad, "Driver not found next to the installer. Extract the whole package first");
      Back(Page::Home);
      return;
    }
    const std::string installed = status_.version.empty() ? "1.0.3 or older" : "v" + status_.version;
    if (a == 3) ImGui::Text("SteamVR replaced the openhouse driver. Puts openhouse v%s back", lh::kVersion);
    else if (a == 1) ImGui::Text("Updates openhouse %s to v%s", installed.c_str(), lh::kVersion);
    else if (a == 2) ImGui::Text("openhouse %s is installed. Reinstalls v%s", installed.c_str(), lh::kVersion);
    else ImGui::Text("Installs openhouse v%s", lh::kVersion);
#ifndef _WIN32
    if (FlatpakSteam(steamvr_.root)) ImGui::TextColored(kWarn, "Flatpak Steam isn't fully tested and may cause problems");
    ImGui::Spacing();
    ImGui::Checkbox("Update check for Monado / WiVRn", &monadoCheck_);
    ImGui::SameLine();
    ImGui::TextColored(kMuted, "Needs systemd");
#endif
  } else {
    if (Replaced()) ImGui::TextUnformatted("SteamVR replaced the openhouse driver. Cleans up what's left of openhouse");
    else if (!status_.openhouse) ImGui::TextColored(kMuted, "openhouse isn't installed");
    ImGui::TextUnformatted("Your room and settings are kept");
  }
  ImGui::Spacing();
  if (!RuntimesClosed()) { Back(Page::Home); return; }
  if (!install) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.62f, 0.24f, 0.24f, 1));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.74f, 0.30f, 0.30f, 1));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.52f, 0.18f, 0.18f, 1));
  }
  if (install && (a == 1 || a == 3)) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.50f, 0.28f, 1));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.27f, 0.60f, 0.34f, 1));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.19f, 0.43f, 0.24f, 1));
  } else if (install && a == 2) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.62f, 0.50f, 0.16f, 1));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.74f, 0.60f, 0.20f, 1));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.53f, 0.43f, 0.14f, 1));
  }
  bool start = BigButton(install ? kActions[a] : "Uninstall", ImGui::GetFontSize() * 14);
  if (autoInstall_ && !autoStarted_) start = autoStarted_ = true;
  if (install && a != 0) ImGui::PopStyleColor(3);
  if (!install) ImGui::PopStyleColor(3);
  if (start) {
    statusFresh_ = false;
    const fs::path root = steamvr_.root;
    InstallOptions options;
    options.monadoCheck = monadoCheck_;
    job_.Start([this, root, options, install](Job& j) {
      auto log = [&j](const std::string& line) { j.Log(line); };
      if (install) InstallDriver(root, options, log);
      else UninstallDriver(root, log);
      return true;
    }, false);
  }
  Back(Page::Home);
}

void Ui::StartSelfUpdate() { Go(Page::SelfUpdate); }

void Ui::StartAutoInstall() {
  autoInstall_ = true;
  Go(Page::Install);
}

void Ui::StartAutoUninstall() {
  autoInstall_ = true;
  Go(Page::Uninstall);
}

// After installing: Doorman Dongles on older firmware than this package's.
void Ui::DongleUpdateOffer() {
  if (!homeWatching_) {
    homeDongles_.Start(FindDoormanDongles, 3.0);
    homeWatching_ = true;
  }
  const uint16_t packaged = PackagedDongleVersion();
  int outdated = 0;
  for (const auto& d : homeDongles_.Value()) outdated += packaged && d.firmware != packaged;
  // OPENHOUSE_TEST_DONGLE_OUTDATED (testing): as if a dongle had older firmware.
  if (std::getenv("OPENHOUSE_TEST_DONGLE_OUTDATED")) outdated = std::max(outdated, 1);
  if (!outdated) return;
  // In the middle of the page, hard to miss: the dongle and its devices need the new firmware.
  const float fs = ImGui::GetFontSize(), w = ImGui::GetContentRegionAvail().x;
  ImGui::Dummy(ImVec2(0, fs * 2));
  const char* text = outdated == 1 ? "Your Doorman Dongle needs to be updated!" : "Your Doorman Dongles need to be updated!";
  ImGui::PushFont(g_title);
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.f, (w - ImGui::CalcTextSize(text).x) / 2));
  ImGui::TextColored(kWarn, "%s", text);
  ImGui::PopFont();
  ImGui::Spacing();
  const float bw = fs * 14;
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.f, (w - bw) / 2));
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.62f, 0.50f, 0.16f, 1));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.74f, 0.60f, 0.20f, 1));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.53f, 0.43f, 0.14f, 1));
  if (BigButton("Update", bw)) Go(Page::DongleUpdatePick);
  ImGui::PopStyleColor(3);
  ImGui::Dummy(ImVec2(0, fs));
}

void Ui::SelfUpdatePage() {
  Title("Update openhouse");
  if (!job_.Running() && !job_.Finished()) {
    job_.Start([this](Job& j) {
      std::string tag;
      const fs::path installer = DownloadLatestPackage([&j](float p, const std::string& s) { j.Status(p, s); }, &tag);
      j.Log("Downloaded " + tag);
      if (!StartDetached(installer, "--install")) { j.Fail("Couldn't start the new installer"); return false; }
      quit_ = true;  // the new one takes over
      return true;
    });
  }
  if (job_.Finished() && !job_.Ok()) {
    ImGui::PushFont(g_title);
    ImGui::TextColored(kBad, "Update failed");
    ImGui::PopFont();
    Wrapped(job_.Error().c_str());
    if (BigButton("Try again")) job_.Reset();
    ImGui::SameLine();
    if (BigButton("Open releases page")) OpenInFileManager(Path("https://github.com/YusufOzmen01/openhouse/releases/latest"));
    Back(Page::Home);
    return;
  }
  const std::string status = job_.StatusText();
  ImGui::Text("%s...", status.empty() ? "Starting" : status.c_str());
  ImGui::ProgressBar(job_.Progress(), ImVec2(ImGui::GetFontSize() * 30, 0), "");
}

// ---- menus ----

namespace {
constexpr const char* kWirelessHint = "Only if your devices already run Doorman radio firmware and are paired to a Doorman dongle";
}

void Ui::FlashMenu() {
  Title("Install or Update Firmware");
  ImGui::Dummy(ImVec2(0, ImGui::GetFontSize()));
  Prompt("What do you want to flash?");
  switch (Tiles({{Icon::BaseStation, "1.0 Base Stations", "Flashes lhbs to your 1.0 base stations"},
                 {Icon::Wireless, "Doorman", "Flashes your dongle or your devices"}})) {
    case 0: Go(Page::BaseStation); break;
    case 1: Go(Page::DoormanMenu); break;
  }
  Back(Page::Home);
}

// ---- device type ----

void Ui::DeviceTypePage() {
  Title("Device Type");
  Wrapped("Plug the device in with a USB cable.");
  if (steamvr_.root.empty()) {
    ImGui::TextColored(kBad, "SteamVR not found");
    if (BigButton("Choose SteamVR folder")) askSteamVR_ = true;
    Back(Page::Tools);
    return;
  }
  if (!RuntimesClosed() || !DeviceAccess()) { Back(Page::Tools); return; }
  const fs::path root = steamvr_.root;
  const auto devices = wired_.Value();

  // Each new device's type, read from its config once.
  std::vector<std::string> unread;
  {
    std::lock_guard l(kindsMutex_);
    for (const auto& d : devices)
      if (!kinds_.count(d.serial) && d.serial.rfind("LHR-", 0) == 0) unread.push_back(d.serial);
    if (!unread.empty() && !kindJob_.Running() && !job_.Running())
      for (const auto& s : unread) kinds_[s] = "";
    else
      unread.clear();
  }
  if (!unread.empty()) {
    kindJob_.Start([this, unread, root](Job&) {
      for (const auto& serial : unread) {
        std::string text, error, kind;
        if (ReadDeviceConfig(root, serial, &text, &error)) {
          kind = KindName(KindOf(nlohmann::json::parse(text, nullptr, false)));
          if (kind == "unknown") kind = "can't tell from its config";
        } else {
          kind = "couldn't read its config: " + error;
        }
        std::lock_guard l(kindsMutex_);
        kinds_[serial] = kind;
      }
      return true;
    });
  }

  ImGui::Spacing();
  ImGui::BeginChild("types", ImVec2(0, -DeviceListReserve()), ImGuiChildFlags_Borders);
  if (devices.empty()) ImGui::TextColored(kMuted, "Waiting for devices...");
  for (const auto& d : devices) {
    if (d.serial.rfind("LHR-", 0) != 0) continue;  // a wand in its bootloader: no config to read there
    std::string kind;
    {
      std::lock_guard l(kindsMutex_);
      kind = kinds_[d.serial];
    }
    ImGui::PushID(d.serial.c_str());
    ImGui::TextUnformatted((d.model.name + "  " + d.serial).c_str());
    ImGui::SameLine();
    if (kind.empty()) {
      ImGui::TextColored(kMuted, "reading...");
    } else if (kind == "controller" || kind == "tracker") {
      ImGui::TextColored(kGood, "%s", kind == "controller" ? "Controller" : "Tracker");
      ImGui::SameLine();
      const bool toTracker = kind == "controller";
      ImGui::BeginDisabled(job_.Running() || kindJob_.Running());
      if (ImGui::Button(toTracker ? "Make it a tracker" : "Make it a controller")) {
        kindTarget_ = d.serial;
        const std::string serial = d.serial;
        const uint16_t pid = d.pid;
        const DeviceKind to = toTracker ? DeviceKind::Tracker : DeviceKind::Controller;
        job_.Start([this, root, serial, pid, to](Job& j) {
          std::string error;
          const bool ok = SetDeviceKind(root, serial, pid, to, [&](const std::string& s) { j.Log(s); }, &error);
          if (!ok) j.Fail(error);
          std::lock_guard l(kindsMutex_);
          if (ok) kinds_[serial] = KindName(to);
          else kinds_.erase(serial);  // read it again
          return ok;
        });
      }
      ImGui::EndDisabled();
    } else if (kind == "headset") {
      ImGui::TextColored(kMuted, "(headset, not changed here)");
    } else {
      ImGui::TextColored(kWarn, "%s", kind.c_str());
    }
    ImGui::PopID();
  }
  ImGui::EndChild();
  if (!kindTarget_.empty()) {
    if (job_.Running()) {
      const auto log = job_.LogLines();
      ImGui::TextColored(kWarn, "%s...", log.empty() ? "Working" : log.back().c_str());
    } else if (job_.Finished()) {
      const auto log = job_.LogLines();
      if (job_.Ok()) ImGui::TextColored(kGood, "%s. Unplug it and turn it on.", log.empty() ? "Done" : log.back().c_str());
      else ImGui::TextColored(kBad, "Failed: %s", job_.Error().c_str());
    }
  } else {
    ImGui::TextColored(kMuted, "Configs are backed up to %s before they're changed.", Utf8(DeviceConfigBackups()).c_str());
  }
  Back(Page::Tools);
}

void Ui::DoormanMenu() {
  Title("Doorman Firmware");
  ImGui::Dummy(ImVec2(0, ImGui::GetFontSize() * 0.5f));
  Note("No Doorman dongle yet? Flash the dongle first, then come back here and flash your trackers and controllers.");
  ImGui::Dummy(ImVec2(0, ImGui::GetFontSize() * 0.8f));
  Prompt("What do you want to flash?");
  switch (Tiles({{Icon::Dongle, "Dongle", "Flashes the Doorman firmware to your dongle"},
                 {Icon::Tracker, "Trackers & Controllers", "Flashes the Doorman radio firmware to your devices"}})) {
    case 0: Go(Page::Dongle); break;
    case 1: Go(Page::RadioMenu); break;
  }
  Back(Page::FlashMenu);
}

void Ui::RadioMenu() {
  Title("Trackers & Controllers");
  ImGui::Dummy(ImVec2(0, ImGui::GetFontSize()));
  Prompt("How are your devices connected?");
  switch (Tiles({{Icon::Wire, "Wired", "Plug the device in with a USB cable", Tint::None, "Recommended"},
                 {Icon::Wireless, "Wireless", kWirelessHint}})) {
    case 0: Go(Page::RadioWired); break;
    case 1: Go(Page::RadioWireless); break;
  }
  Back(Page::DoormanMenu);
}

void Ui::RestoreRadioMenu() {
  Title("Restore Tracker/Controller Radio Firmware");
  ImGui::Dummy(ImVec2(0, ImGui::GetFontSize()));
  Prompt("How are your devices connected?");
  switch (Tiles({{Icon::Wire, "Wired", "Plug the device in with a USB cable", Tint::None, "Recommended"},
                 {Icon::Wireless, "Wireless", kWirelessHint}})) {
    case 0: Go(Page::RestoreRadio); break;
    case 1: Go(Page::RestoreRadioWireless); break;
  }
  Back(Page::RestoreMenu);
}

void Ui::RestoreMenu() {
  Title("Restore Firmware");
  ImGui::Dummy(ImVec2(0, ImGui::GetFontSize()));
  Prompt("What do you want to restore?");
  switch (Tiles({{Icon::BaseStation, "1.0 Base Stations", "Puts Valve's base station firmware back"},
                 {Icon::Tracker, "Trackers & Controllers", "Puts the original radio firmware back"}})) {
    case 0: Go(Page::RestoreBase); break;
    case 1: Go(Page::RestoreRadioMenu); break;
  }
  Back(Page::Home);
}

// ---- 1.0 base station ----

void Ui::BaseStationPage(bool restore) {
  Title(restore ? "Restore 1.0 Base Station Firmware" : "Flash 1.0 Lighthouse (lhbs) Firmware");
  const Page back = restore ? Page::RestoreMenu : Page::FlashMenu;
  auto flashAgain = [&] {
    if (BigButton("Flash Again", ImGui::GetFontSize() * 12)) Go(restore ? Page::RestoreBase : Page::BaseStation);
    ImGui::SameLine();
  };
  switch (baseStep_) {
    case BaseStep::Warning: {
      Warning("ONLY for Lighthouse 1.0 base stations!",
              "This firmware is only for 1.0 base stations (the ones that came with the original HTC Vive).\n"
              "It WILL BRICK 2.0 base stations!");
      ImGui::Spacing();
      if (restore && (steamvr_.root.empty() || StockBaseStationImage(steamvr_.root).empty())) {
        ImGui::TextColored(kBad, steamvr_.root.empty() ? "SteamVR not found" : "SteamVR's base station firmware not found");
        if (steamvr_.root.empty() && BigButton("Choose SteamVR folder")) askSteamVR_ = true;
        Back(back);
        return;
      } else if (!restore && OurBaseStationImage().empty()) {
        ImGui::TextColored(kBad, "lhbs firmware not found in the firmwares folder");
        Back(back);
        return;
      }
      ImGui::Checkbox("This is a Lighthouse 1.0 base station", &confirmed10_);
      ImGui::BeginDisabled(!confirmed10_);
      if (BigButton("Continue", ImGui::GetFontSize() * 12)) {
        const fs::path image = restore ? StockBaseStationImage(steamvr_.root) : OurBaseStationImage();
        std::string error;
        if (!ReadFile(image, &baseImage_)) error = "couldn't read " + Utf8(image);
        else error = CheckBaseStationImage(baseImage_);
        baseImageName_ = Utf8(image.filename());
        if (error.empty()) {
          baseStep_ = BaseStep::Waiting;
          baseDrives_.Start(FindBaseStations, 1.0);
        } else {
          job_.Reset();
          job_.Fail(baseImageName_ + ": " + error);
          baseStep_ = BaseStep::Done;
        }
      }
      ImGui::EndDisabled();
      Back(back);
      return;
    }
    case BaseStep::Waiting:
    case BaseStep::Ready: {
      const auto drives = baseDrives_.Value();
      if (drives.empty()) {
        baseStep_ = BaseStep::Waiting;
        Step(1, "Unplug the base station's power cable");
        Step(2, "Hold the button on the back");
        Step(3, "Plug the micro-USB cable to your PC, then plug the power cable and let go of the button");
        ImGui::Spacing();
        ImGui::TextColored(kMuted, "Waiting for the base station...");
      } else {
        baseStep_ = BaseStep::Ready;
        baseDrive_ = drives.front();
        ImGui::TextColored(kGood, "Base station found (%s)", baseDrive_.name.c_str());
        ImGui::Text("Firmware: %s", baseImageName_.c_str());
        ImGui::Spacing();
        if (BigButton("Flash", ImGui::GetFontSize() * 12)) {
          baseDrives_.Stop();
          baseStep_ = BaseStep::Writing;
          const Drive drive = baseDrive_;
          const std::string image = baseImage_;
          job_.Start([this, drive, image](Job& j) {
            alreadyInstalled_ = false;
            std::string current;
            if (ReadBaseStation(drive, &current) && current.compare(0, image.size(), image) == 0) {
              alreadyInstalled_ = true;
              Eject(drive);
              return true;
            }
            std::string error;
            if (!WriteBaseStation(drive, image, &error, [&](float f) { j.Status(f, "Writing"); })) { j.Fail(error); return false; }
            Eject(drive);
            return true;
          });
        }
      }
      if (BackButton()) { baseDrives_.Stop(); baseStep_ = BaseStep::Warning; }
      return;
    }
    case BaseStep::Writing: {
      JobLog(job_);
      if (job_.Running()) { ProgressBar(job_); return; }
      if (!job_.Ok() || alreadyInstalled_) { baseStep_ = BaseStep::Done; return; }
      baseStep_ = BaseStep::Replug;
      sawUnplug_ = false;
      baseDrives_.Start(FindBaseStations, 1.0);
      return;
    }
    case BaseStep::Replug: {
      JobLog(job_);
      ImGui::Spacing();
      ImGui::TextColored(kWarn, "Verifying:");
      Step(1, "Unplug the base station's power cable");
      Step(2, "Hold the button again and plug the power cable back in");
      const auto drives = baseDrives_.Value();
      if (drives.empty()) sawUnplug_ = true;
      if (sawUnplug_ && !drives.empty() && baseDrives_.Have()) {
        baseDrives_.Stop();
        baseStep_ = BaseStep::Verifying;
        const Drive drive = drives.front();
        const std::string image = baseImage_;
        job_.Start([drive, image](Job& j) {
          j.Status(0.5f, "Verifying");
          std::string check;
          const bool read = ReadBaseStation(drive, &check);
          Eject(drive);
          if (!read || check.compare(0, image.size(), image) != 0) {
            j.Fail("verification failed, try again");
            return false;
          }
          return true;
        });
      } else {
        ImGui::TextColored(kMuted, sawUnplug_ ? "Waiting for the base station..." : "Waiting for it to be unplugged...");
      }
      return;
    }
    case BaseStep::Verifying:
      if (job_.Running()) { ProgressBar(job_); return; }
      baseStep_ = BaseStep::Done;
      return;
    case BaseStep::Done: {
      JobLog(job_);
      ImGui::Spacing();
      if (job_.Ok()) {
        ImGui::PushFont(g_title);
        ImGui::TextColored(kGood, alreadyInstalled_ ? "This firmware is already installed" : restore ? "Stock firmware has been restored" : "lhbs has been installed");
        ImGui::PopFont();
        ImGui::TextUnformatted("Unplug the USB cable and power cycle the base station");
      } else {
        ImGui::TextColored(kBad, "Failed: %s", job_.Error().c_str());
      }
      ImGui::Spacing();
      flashAgain();
      if (ImGui::Button("< Home")) Go(Page::Home);
      return;
    }
  }
}

// ---- dongle ----

void Ui::DonglePage() {
  Title("Dongle Firmware");
  switch (dongleStep_) {
    case DongleStep::Board: {
      ImGui::Dummy(ImVec2(0, ImGui::GetFontSize()));
      Prompt("Which dongle do you have?");
      const auto watch = [this](int board) {
        dongleBoard_ = board;
        dongleStep_ = DongleStep::Detect;
        dongleSeen_.Start([board] {
          DongleSeen s;
          if (board) s.ports = lh::dfu::FindBootloaderPorts();
          else s.drives = FindUf2Drives();
          for (auto& d : FindDoormanDongles())
            if (d.board == board) s.dongles.push_back(d);
          return s;
        }, 1.0);
      };
      // Board ids (DoormanDongleInfo::board) in tile order.
      static constexpr int kBoards[] = {1, 2, 0, 3};
      const int tile = Tiles({{Icon::Dongle, "Holyiot 21017", "Nordic bootloader", Tint::None, "Recommended"},
                              {Icon::Dongle, "Nordic PCA10059 Dongle", "Nordic bootloader"},
                              {Icon::Dongle, "ProMicro nRF52840", "Adafruit bootloader. Soldering an antenna to the board is highly recommended"},
                              {Icon::Dongle, "Kuroshin", "It needs to be put into DFU mode by opening the dongle and shorting two pins"}},
                             ImGui::GetFontSize() * 15);
      if (tile >= 0) watch(kBoards[tile]);
      Back(Page::DoormanMenu);
      return;
    }
    case DongleStep::Detect: {
      const DongleSeen seen = dongleSeen_.Value();
      // Back: to the board picker.
      const auto back = [this] {
        if (BackButton()) { dongleSeen_.Stop(); dongleStep_ = DongleStep::Board; }
      };
      if (!DeviceAccess()) { back(); return; }
      Step(1, "Plug the dongle in");
      Step(2, dongleBoard_ == 3 ? "If it doesn't show up, open the dongle and briefly short pads 1 and 5 to put it in DFU mode"
              : dongleBoard_ == 1 ? "If it doesn't show up, bring a magnet close to the dongle's LED window to put it in DFU mode"
              : dongleBoard_ ? "If it doesn't show up, press its RESET button to put it in DFU mode"
                             : "If it doesn't show up, short the RST and GND pins twice to put it in DFU mode");
      if (dongleBoard_ == 1 || dongleBoard_ == 2) {
        ImGui::Spacing();
        if (dongleBoard_ == 1) HolyiotMagnet();
        else NordicReset();
      }
      if (dongleBoard_ == 0) {
        ImGui::Spacing();
        ImGui::BeginGroup();
        ProMicroAntenna();
        ImGui::EndGroup();
        ImGui::SameLine(0, ImGui::GetFontSize() * 1.5f);
        ImGui::BeginGroup();
        ImGui::Dummy(ImVec2(0, ImGui::GetFontSize() * 2));
        ImGui::TextColored(kWarn, "Antenna: optional, but highly recommended");
        Wrapped("Solder a 31.2 mm wire to the end of the board's antenna, at the bottom right");
        ImGui::EndGroup();
      }
      if (dongleBoard_ == 3) {
        ImGui::Spacing();
        ImGui::BeginGroup();
        KuroshinPads();
        ImGui::EndGroup();
        ImGui::SameLine(0, ImGui::GetFontSize() * 1.5f);
        ImGui::BeginGroup();
        ImGui::Dummy(ImVec2(0, ImGui::GetFontSize() * 2));
        Wrapped("Chip side up, USB plug at the bottom");
        Wrapped("Pads 1 and 5 are the first and the last of the row above the chip");
        Wrapped("Touch them together for a moment with tweezers or a wire");
        ImGui::Spacing();
        ImGui::TextColored(kWarn, "Only the version with the anime character on the board is tested");
        ImGui::EndGroup();
      }
      ImGui::Spacing();
      if (seen.drives.empty() && seen.ports.empty() && seen.dongles.empty()) ImGui::TextColored(kMuted, "Waiting for the dongle...");
      // It restarts as a Doorman Dongle: check it answers.
      // A wall-clock deadline, not a pass count: a dongle that shows up but doesn't answer makes each
      // write block ~5 s in the kernel (Linux hidraw), so 200 passes took ~17 minutes.
      static const auto verify = [](Job& j) {
        const auto start = std::chrono::steady_clock::now();
        const auto limit = std::chrono::seconds(20);
        bool seen = false;
        for (;;) {
          const auto elapsed = std::chrono::steady_clock::now() - start;
          if (elapsed >= limit) break;
          j.Status(0.8f + 0.2f * std::chrono::duration<float>(elapsed) / std::chrono::duration<float>(limit), "Verifying");
          for (const auto& d : FindDoormanDongles()) {
            seen = true;
            if (d.firmware) {
              char text[96];
              std::snprintf(text, sizeof text, "Dongle %s is working (firmware %04X)", d.serial.c_str(), d.firmware);
              j.Log(text);
              j.Status(1, "Done");
              return true;
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        j.Fail(seen ? "the dongle showed up but doesn't answer, replug it and try again"
                    : "the dongle didn't show up after flashing, replug it and try again");
        return false;
      };
      static const auto flashUf2 = [](Job& j, const Uf2Drive& drive) {
        const fs::path image = DongleImage(drive.softDevice);
        j.Log("Firmware: " + Utf8(image.filename()));
        std::string error;
        if (!CopyUf2(drive, image, &error, [&](float f) { j.Status(f * 0.8f, "Flashing"); })) { j.Fail(error); return false; }
        return verify(j);
      };
      static const auto flashDfu = [](Job& j, const std::string& port, int board) {
        const fs::path file = NordicImage(board);
        j.Log("Firmware: " + Utf8(file.filename()));
        lh::dfu::Image image;
        std::string error;
        // App space of Nordic's open bootloader: after the MBR, below the pairing page and the bootloader.
        if (!lh::dfu::LoadHex(Utf8(file), 0x1000, 0xDA000, &image, &error)) { j.Fail(error); return false; }
        if (image.base != 0x1000) { j.Fail("this firmware isn't built for this bootloader"); return false; }
        j.Log("Bootloader on " + port);
        if (!lh::dfu::Flash(port, image, [&](float f) { j.Status(f * 0.8f, "Flashing"); }, &error)) { j.Fail(error); return false; }
        return verify(j);
      };
      const auto start = [this](std::function<bool(Job&)> work) {
        dongleSeen_.Stop();
        dongleStep_ = DongleStep::Flashing;
        job_.Start(std::move(work));
      };
      for (const auto& d : seen.drives) {
        const std::string label = d.name + (d.board.empty() ? "" : "  (" + d.board + ")");
        if (BigButton(label.c_str(), ImGui::GetFontSize() * 34)) start([d](Job& j) { return flashUf2(j, d); });
      }
      for (const auto& port : seen.ports) {
        const std::string label = "Nordic DFU bootloader (" + port + ")";
        if (BigButton(label.c_str(), ImGui::GetFontSize() * 34)) start([port, board = dongleBoard_](Job& j) { return flashDfu(j, port, board); });
      }
      for (const auto& d : seen.dongles) {
        char label[128];
        std::snprintf(label, sizeof label, "Doorman Dongle %s", d.serial.c_str());
        if (!BigButton(label, ImGui::GetFontSize() * 34)) continue;
        const DoormanDongleInfo dongle = d;
        if (dongle.Banked()) {   // updates itself: the firmware for the bank it isn't running
          start([dongle](Job& j) {
            const fs::path file = FirmwareFolder() / "dongle" / "bank" /
                                  lh::dongle_bank::ImageName(dongle.board, dongle.softDeviceBuild, dongle.bank);
            j.Log("Firmware: " + Utf8(file.filename()));
            std::ifstream in(file, std::ios::binary);
            std::vector<uint8_t> image((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
            if (image.size() < 1024) { j.Fail("firmware not found in the firmwares folder"); return false; }
            std::string error;
            if (!BankUpdate(dongle, std::move(image), [&](float f) { j.Status(f * 0.8f, "Flashing"); }, &error)) { j.Fail(error); return false; }
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));   // it restarts into the new bank
            return verify(j);
          });
          continue;
        }
        // Older firmware: restarts it in its update mode, then flashes what appears.
        start([dongle](Job& j) {
          const auto drivesBefore = FindUf2Drives();
          const auto portsBefore = lh::dfu::FindBootloaderPorts();
          j.Status(0, "Rebooting into DFU mode");
          if (!RebootToBootloader(dongle)) { j.Fail("couldn't open the dongle, close SteamVR and doorman-dongle-manager"); return false; }
          for (int i = 0; i < 150; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (dongle.NordicDfu()) {
              for (const auto& port : lh::dfu::FindBootloaderPorts())
                if (std::find(portsBefore.begin(), portsBefore.end(), port) == portsBefore.end()) {
                  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // let the port settle
                  return flashDfu(j, port, dongle.board);
                }
            } else {
              for (const auto& drive : FindUf2Drives())
                if (std::none_of(drivesBefore.begin(), drivesBefore.end(), [&](const Uf2Drive& b) { return b.mount == drive.mount; }))
                  return flashUf2(j, drive);
            }
          }
          j.Fail(dongle.NordicDfu() ? "the dongle didn't enter DFU mode, press its RESET button and try again"
                                : "the dongle didn't enter DFU mode, short RST and GND twice and try again");
          return false;
        });
      }
      back();
      return;
    }
    case DongleStep::Flashing:
      JobLog(job_);
      if (job_.Running()) { ProgressBar(job_); return; }
      dongleStep_ = DongleStep::Done;
      return;
    case DongleStep::Done:
      JobLog(job_);
      ImGui::Spacing();
      if (job_.Ok()) {
        ImGui::PushFont(g_title);
        ImGui::TextColored(kGood, "Flashed");
        ImGui::PopFont();
      } else {
        ImGui::TextColored(kBad, "Failed: %s", job_.Error().c_str());
      }
      ImGui::Spacing();
      if (BigButton("Flash Again", ImGui::GetFontSize() * 12)) Go(Page::Dongle);
      ImGui::SameLine();
      if (ImGui::Button("< Home")) Go(Page::Home);
      return;
  }
}

// ---- radios, wired ----

void Ui::RadioWiredPage(bool restore) {
  Title(restore ? "Restore Tracker/Controller Radio Firmware" : "Flash Radio Firmware: Wired");
  const Page back = restore ? Page::RestoreRadioMenu : Page::RadioMenu;
  if (steamvr_.root.empty()) {
    ImGui::TextColored(kBad, "SteamVR not found");
    if (BigButton("Choose SteamVR folder")) askSteamVR_ = true;
    Back(back);
    return;
  }
  if (radioStep_ != RadioStep::Choose) {
    JobLog(job_);
    if (job_.Running()) {
      char text[64];
      std::snprintf(text, sizeof text, "Device %d of %d", current_.load(), total_);
      ImGui::TextColored(kWarn, "%s", text);
      ProgressBar(job_);
      return;
    }
    ImGui::Spacing();
    if (job_.Ok()) {
      ImGui::PushFont(g_title);
      ImGui::TextColored(kGood, "Flashed");
      ImGui::PopFont();
      ImGui::TextUnformatted("Unplug the devices and turn them on");
    } else {
      ImGui::TextColored(kBad, "Failed: %s", job_.Error().c_str());
    }
    ImGui::Spacing();
    if (BigButton("Flash Again", ImGui::GetFontSize() * 12)) Go(restore ? Page::RestoreRadio : Page::RadioWired);
    ImGui::SameLine();
    if (ImGui::Button("< Home")) Go(Page::Home);
    return;
  }
  ImGui::TextUnformatted("Plug your devices in with a USB cable and pick the ones to flash");
  if (!RuntimesClosed() || !DeviceAccess()) { Back(back); return; }
  const auto devices = wired_.Value();
  ImGui::Spacing();
  ImGui::BeginChild("wired", ImVec2(0, -DeviceListReserve()), ImGuiChildFlags_Borders);
  if (devices.empty()) ImGui::TextColored(kMuted, "Waiting for devices...");
  int trackers10 = 0;
  for (const auto& d : devices) {
    trackers10 += d.model.family == RadioFamily::Tracker10;
    const bool supported = d.model.family != RadioFamily::Unsupported;
    const fs::path image = restore ? StockRadioImage(steamvr_.root, d.model.hwid) : OurRadioImage(d.model.family);
    const bool haveImage = supported && !image.empty();
    const bool allowed = d.model.known || accepted_.count(d.serial);
    if (!allowed) checked_.erase(d.serial);
    bool on = checked_.count(d.serial) > 0;
    ImGui::BeginDisabled(!haveImage || !allowed);
    const std::string label = d.model.name + "  " + d.serial;
    const RadioState radio = DescribeRadio(d.radioVersion, d.model.family);
    if (radio.color) ImGui::PushStyleColor(ImGuiCol_Text, *radio.color);
    if (ImGui::Checkbox(label.c_str(), &on)) { if (on) checked_.insert(d.serial); else checked_.erase(d.serial); }
    if (radio.color) ImGui::PopStyleColor();
    if (d.model.known && supported) { ImGui::SameLine(); ImGui::TextColored(radio.color ? *radio.color : kMuted, "%s", radio.text.c_str()); }
    ImGui::EndDisabled();
    if (!d.model.known && haveImage) UnknownDevice(d.model.hwid, d.model.family, d.serial, &accepted_);
    else if (!d.model.known) { ImGui::SameLine(); ImGui::TextColored(kMuted, "(unknown device %08X, no firmware to restore)", d.model.hwid); }
    else if (!supported) { ImGui::SameLine(); ImGui::TextColored(kMuted, "(not supported)"); }
    else if (!haveImage) { ImGui::SameLine(); ImGui::TextColored(kMuted, "(firmware not found)"); }
  }
  ImGui::EndChild();
  std::vector<WiredDevice> chosen;
  for (const auto& d : devices) if (checked_.count(d.serial)) chosen.push_back(d);
  const int chosen10 = int(std::count_if(chosen.begin(), chosen.end(), [](const WiredDevice& d) { return d.model.family == RadioFamily::Tracker10; }));
  if (trackers10 > 1 && chosen10 > 0)
    ImGui::TextColored(kWarn, "Only one VIVE Tracker 1.0 or VIVE Controller can be plugged in while flashing");
  ImGui::BeginDisabled(chosen.empty() || (trackers10 > 1 && chosen10 > 0));
  if (BigButton("Flash", ImGui::GetFontSize() * 12)) {
    wired_.Stop();
    radioStep_ = RadioStep::Flashing;
    total_ = int(chosen.size());
    current_ = 0;
    const fs::path root = steamvr_.root;
    job_.Start([this, chosen, root, restore](Job& j) {
      bool all = true;
      for (const auto& d : chosen) {
        ++current_;
        const fs::path image = restore ? StockRadioImage(root, d.model.hwid) : OurRadioImage(d.model.family);
        std::string error;
        const bool ok = FlashWired(root, d, image, [&](float f, const std::string& s) { j.Status(f, d.serial + ": " + s); }, &error);
        j.Log(d.model.name + " " + d.serial + (ok ? ": done" : ": failed (" + error + ")"));
        all = all && ok;
        if (!ok) j.Fail(d.serial + ": " + error);
      }
      return all;
    });
  }
  ImGui::EndDisabled();
  Back(back);
}

// ---- radios, wireless ----

void Ui::RadioWirelessPage(bool restore) {
  Title(restore ? "Restore Radio Firmware: Wireless" : "Flash Radio Firmware: Wireless");
  const Page back = restore ? Page::RestoreRadioMenu : Page::RadioMenu;
  if (restore && steamvr_.root.empty()) {
    ImGui::TextColored(kBad, "SteamVR not found");
    if (BigButton("Choose SteamVR folder")) askSteamVR_ = true;
    Back(back);
    return;
  }
  // The stock image for a device: by its hardware id. Behind a Doorman Dongle the user picks it
  // only when the device didn't say which it is.
  auto stockFor = [this](const WirelessDevice& d) -> fs::path {
    if (d.route == Route::Watchman || !IsDoormanRadio(d.model.hwid)) return StockRadioImage(steamvr_.root, d.model.hwid);
    const auto c = stockChoice_.find(d.Key());
    if (c != stockChoice_.end()) return stockModels_[size_t(c->second)].image;
    std::vector<int> fits;
    for (size_t i = 0; i < stockModels_.size(); ++i) if (stockModels_[i].family == d.model.family) fits.push_back(int(i));
    return fits.size() == 1 ? stockModels_[size_t(fits[0])].image : fs::path();
  };
  if (radioStep_ != RadioStep::Choose) {
    JobLog(job_);
    if (job_.Running()) {
      char text[64];
      std::snprintf(text, sizeof text, "Device %d of %d", current_.load(), total_);
      ImGui::TextColored(kWarn, "%s", text);
      ProgressBar(job_);
      return;
    }
    ImGui::Spacing();
    if (job_.Ok()) {
      ImGui::PushFont(g_title);
      ImGui::TextColored(kGood, "Flashed");
      ImGui::PopFont();
      ImGui::TextUnformatted(restore ? "Turn the devices back on. Pair them to their original dongle again" : "Turn the devices back on. You can pair them to a Doorman Dongle now");
    } else {
      ImGui::TextColored(kBad, "Failed: %s", job_.Error().c_str());
    }
    ImGui::Spacing();
    if (BigButton("Flash Again", ImGui::GetFontSize() * 12)) Go(restore ? Page::RestoreRadioWireless : Page::RadioWireless);
    ImGui::SameLine();
    if (ImGui::Button("< Home")) Go(Page::Home);
    return;
  }
  ImGui::TextUnformatted("Turn your devices on. They need to be paired to a dongle");
  if (!RuntimesClosed() || !DeviceAccess()) { Back(back); return; }
  const auto devices = wireless_.Value();
  ImGui::Spacing();
  ImGui::BeginChild("wireless", ImVec2(0, -DeviceListReserve()), ImGuiChildFlags_Borders);
  if (devices.empty()) ImGui::TextColored(kMuted, "Waiting for devices...");
  for (const auto& d : devices) {
    const bool supported = d.model.family != RadioFamily::Unsupported && !OurRadioImage(d.model.family).empty();
    const bool needsPick = restore && d.route == Route::Doorman && stockFor(d).empty();
    const bool unknownOk = !d.model.known && !restore && supported;
    const bool allowed = d.model.known || (unknownOk && accepted_.count(d.Key()));
    if (!allowed) checked_.erase(d.Key());
    bool on = checked_.count(d.Key()) > 0;
    ImGui::BeginDisabled(!supported || !allowed);
    const std::string label = d.model.name + "  " + d.serial + "  (" + (d.route == Route::Doorman ? "Doorman Dongle " : "dongle ") + d.dongle + ")";
    const RadioState radio = DescribeRadio(d.radioVersion, d.model.family);
    if (radio.color) ImGui::PushStyleColor(ImGuiCol_Text, *radio.color);
    if (ImGui::Checkbox(label.c_str(), &on)) { if (on) checked_.insert(d.Key()); else checked_.erase(d.Key()); }
    if (radio.color) ImGui::PopStyleColor();
    if (d.model.known && supported) { ImGui::SameLine(); ImGui::TextColored(radio.color ? *radio.color : kMuted, "%s", radio.text.c_str()); }
    ImGui::EndDisabled();
    if (unknownOk) UnknownDevice(d.model.hwid, d.model.family, d.Key(), &accepted_);
    else if (!d.model.known) { ImGui::SameLine(); ImGui::TextColored(kMuted, "(unknown device %08X)", d.model.hwid); }
    else if (!supported) { ImGui::SameLine(); ImGui::TextColored(kMuted, "(not supported)"); }
    if (restore && supported && d.model.known && d.route == Route::Doorman && IsDoormanRadio(d.model.hwid)) {
      ImGui::SameLine();
      const auto c = stockChoice_.find(d.Key());
      const std::string current = c != stockChoice_.end() ? stockModels_[size_t(c->second)].name : needsPick ? "Device type..." : "";
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 16);
      if (ImGui::BeginCombo(("##type" + d.Key()).c_str(), current.c_str())) {
        for (size_t i = 0; i < stockModels_.size(); ++i)
          if (stockModels_[i].family == d.model.family && ImGui::Selectable(stockModels_[i].name.c_str())) stockChoice_[d.Key()] = int(i);
        ImGui::EndCombo();
      }
    } else if (restore && supported && stockFor(d).empty()) {
      ImGui::SameLine();
      ImGui::TextColored(kMuted, "(firmware not found)");
    }
  }
  ImGui::EndChild();
  std::vector<std::pair<WirelessDevice, fs::path>> chosen;
  for (const auto& d : devices)
    if (checked_.count(d.Key())) chosen.emplace_back(d, restore ? stockFor(d) : OurRadioImage(d.model.family));
  const bool missing = std::any_of(chosen.begin(), chosen.end(), [](const auto& c) { return c.second.empty(); });
  if (missing) ImGui::TextColored(kWarn, "Pick the device type for each selected device");
  ImGui::BeginDisabled(chosen.empty() || missing);
  if (BigButton("Flash", ImGui::GetFontSize() * 12)) {
    wireless_.Stop();
    radioStep_ = RadioStep::Flashing;
    total_ = int(chosen.size());
    current_ = 0;
    job_.Start([this, chosen](Job& j) {
      bool all = true;
      for (const auto& [d, file] : chosen) {
        ++current_;
        RadioImage image;
        std::string error;
        bool ok = LoadRadioImage(file, &image, &error) &&
                  FlashWireless(d, image, [&](float f, const std::string& s) { j.Status(f, d.serial + ": " + s); }, &error);
        j.Log(d.model.name + " " + d.serial + (ok ? ": done" : ": failed (" + error + ")"));
        all = all && ok;
        if (!ok) j.Fail(d.serial + ": " + error);
      }
      return all;
    });
  }
  ImGui::EndDisabled();
  Back(back);
}

// ---- dongle update ----

void Ui::DongleUpdatePick() {
  Title("Update Dongles");
  if (!homeWatching_) {
    homeDongles_.Start(FindDoormanDongles, 2.0);
    homeWatching_ = true;
  }
  const uint16_t packaged = PackagedDongleVersion();
  Prompt("Select the dongle");
  ImGui::Spacing();
  int shown = 0;
  for (const auto& d : homeDongles_.Value()) {
    if (!packaged || d.firmware == packaged) continue;
    ++shown;
    char sub[96];
    std::snprintf(sub, sizeof sub, "Firmware %04X, this package has %04X", d.firmware, packaged);
    const std::string title = "Doorman Dongle " + d.serial;
    ImGui::PushID(d.serial.c_str());
    if (TileGrid({{Icon::Dongle, title.c_str(), sub, Tint::Yellow}}, 1) == 0) {   // sized to its text
      updateDongle_ = d;
      Go(Page::DongleUpdate);
    }
    ImGui::PopID();
  }
  if (!shown) ImGui::TextColored(kMuted, homeDongles_.Have() ? "Every dongle is up to date" : "Looking for dongles...");
  Back(Page::Home);
}

void Ui::StartDongleUpdate(const std::vector<DongleSlot>& slots) {
  if (testOnly_) return;
  updateSlots_.Stop();
  {
    std::lock_guard l(updateMutex_);
    updateSnapshot_ = slots;
    updateResults_.clear();
  }
  const DoormanDongleInfo dongle = updateDongle_;
  const uint16_t packaged = PackagedDongleVersion();
  job_.Start([this, dongle, slots, packaged](Job& j) {
    auto set = [this](int slot, int state, const std::string& text) {
      std::lock_guard l(updateMutex_);
      updateResults_[slot] = {state, text};
    };
    // The devices first: the dongle restarting can't cut one off halfway.
    std::vector<const DongleSlot*> todo;
    for (const auto& s : slots)
      if (RadioNeedsUpdate(s)) todo.push_back(&s);
    int step = 0;
    const int steps = int(todo.size()) + 1;
    for (const DongleSlot* s : todo) {
      RadioImage image;
      std::string error;
      if (!LoadRadioImage(OurRadioImage(s->device.model.family), &image, &error)) {
        set(s->slot, 3, "Firmware not found in the firmwares folder");
        ++step;
        continue;
      }
      bool ok = false;
      constexpr int kTries = 3;
      for (int attempt = 1; attempt <= kTries && !ok; ++attempt) {
        set(s->slot, 1, attempt == 1 ? "Updating" : "Updating, try " + std::to_string(attempt) + " of " + std::to_string(kTries));
        error.clear();
        ok = FlashWireless(s->device, image, [&](float f, const std::string&) { j.Status((float(step) + f) / float(steps), "Updating slot " + std::to_string(s->slot + 1)); }, &error);
        if (!ok) {
          j.Log("Slot " + std::to_string(s->slot + 1) + ": " + error);
          std::this_thread::sleep_for(std::chrono::seconds(3));   // let it reconnect
        }
      }
      if (ok) set(s->slot, 2, "Updated, turned off");
      else set(s->slot, 3, "Failed. Connect it with a cable and flash it from Install or Update Firmware");
      ++step;
    }
    // Then the dongle.
    if (packaged && dongle.firmware == packaged) { set(-1, 2, "Up to date"); return true; }
    if (!dongle.Banked()) {
      set(-1, 3, "Its firmware is too old to update here: use Install or Update Firmware > Doorman");
      j.Fail("the dongle wasn't updated");
      return false;
    }
    const fs::path file = FirmwareFolder() / "dongle" / "bank" / lh::dongle_bank::ImageName(dongle.board, dongle.softDeviceBuild, dongle.bank);
    std::ifstream in(file, std::ios::binary);
    const std::vector<uint8_t> image((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (image.size() < 1024) { set(-1, 3, "Firmware not found in the firmwares folder"); j.Fail("the dongle wasn't updated"); return false; }
    for (int attempt = 1; attempt <= 3; ++attempt) {
      set(-1, 1, attempt == 1 ? "Updating" : "Updating, try " + std::to_string(attempt) + " of 3");
      // Its bank changes with every update: find it again for the current one.
      DoormanDongleInfo now = dongle;
      for (const auto& d : FindDoormanDongles())
        if (d.serial == dongle.serial) now = d;
      if (now.firmware == packaged) { set(-1, 2, "Updated"); return true; }
      std::string error;
      const fs::path f = FirmwareFolder() / "dongle" / "bank" / lh::dongle_bank::ImageName(now.board, now.softDeviceBuild, now.bank);
      std::ifstream fin(f, std::ios::binary);
      std::vector<uint8_t> img((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());
      if (BankUpdate(now, std::move(img), [&](float p) { j.Status((float(step) + p) / float(steps), "Updating the dongle"); }, &error)) {
        for (int i = 0; i < 100; ++i) {   // it restarts into the new bank
          std::this_thread::sleep_for(std::chrono::milliseconds(200));
          for (const auto& d : FindDoormanDongles())
            if (d.serial == dongle.serial && d.firmware == packaged) { set(-1, 2, "Updated"); j.Status(1, "Done"); return true; }
        }
        error = "it didn't come back with the new firmware";
      }
      j.Log("Dongle: " + error);
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    set(-1, 3, "Failed. Unplug it, plug it back in and try again");
    j.Fail("the dongle wasn't updated");
    return false;
  });
}

void Ui::DongleUpdatePage() {
  Title(("Update Doorman Dongle " + updateDongle_.serial).c_str());
  const bool started = job_.Running() || job_.Finished();
  std::vector<DongleSlot> slots;
  std::map<int, SlotResult> results;
  {
    std::lock_guard l(updateMutex_);
    results = updateResults_;
    if (started) slots = updateSnapshot_;
  }
  if (!started) slots = updateSlots_.Value();
  const bool have = started || updateSlots_.Have();
  int paired = 0, off = 0, outdated = 0;
  for (const auto& s : slots) {
    paired += s.paired;
    off += s.paired && !s.answered;
    outdated += RadioNeedsUpdate(s);
  }
  if (!started) {
    ImGui::TextUnformatted("Turn on all the devices paired to this dongle, so their radios are updated too.");
    ImGui::TextColored(kMuted, "Updated devices are turned off. The dongle is updated last.");
  } else if (job_.Running()) {
    ImGui::TextUnformatted(job_.StatusText().c_str());
    ImGui::ProgressBar(job_.Progress(), ImVec2(-1, 0), "");
  }
  ImGui::Spacing();

  // The slots: a dot (green up to date, yellow needs an update, grey off or empty), the device, its state.
  const float fs = ImGui::GetFontSize();
  if (ImGui::BeginTable("slots", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_PadOuterX)) {
    ImGui::TableSetupColumn("Slot", ImGuiTableColumnFlags_WidthFixed, fs * 5);
    ImGui::TableSetupColumn("Device", ImGuiTableColumnFlags_WidthStretch, 1.f);
    ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthStretch, 1.4f);
    auto row = [&](const std::string& label, const ImVec4& dot, const std::string& device, const ImVec4& stateColor, const std::string& state) {
      ImGui::TableNextRow(0, fs * 2.1f);
      ImGui::TableNextColumn();
      const ImVec2 p = ImGui::GetCursorScreenPos();
      ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(p.x + fs * 0.45f, p.y + fs * 0.95f), fs * 0.32f, ImGui::GetColorU32(dot), 20);
      ImGui::SetCursorScreenPos(ImVec2(p.x + fs * 1.2f, p.y + fs * 0.35f));
      ImGui::TextUnformatted(label.c_str());
      ImGui::TableNextColumn();
      ImGui::SetCursorPosY(ImGui::GetCursorPosY() + fs * 0.35f);
      ImGui::TextUnformatted(device.c_str());
      ImGui::TableNextColumn();
      ImGui::SetCursorPosY(ImGui::GetCursorPosY() + fs * 0.35f);
      ImGui::PushTextWrapPos(0);
      ImGui::TextColored(stateColor, "%s", state.c_str());
      ImGui::PopTextWrapPos();
    };
    const ImVec4 grey = kMuted;
    for (int i = 0; i < updateDongle_.slots; ++i) {
      const DongleSlot* s = i < int(slots.size()) ? &slots[size_t(i)] : nullptr;
      const std::string label = "Slot " + std::to_string(i + 1);
      if (!have || !s) { row(label, grey, "", grey, "..."); continue; }
      if (!s->paired) { row(label, grey, "Empty", grey, ""); continue; }
      const std::string name = s->answered ? s->device.model.name : "Paired device";
      const bool needs = RadioNeedsUpdate(*s);
      const auto r = results.find(i);
      if (r != results.end()) {
        const ImVec4 c = r->second.state == 2 ? kGood : r->second.state == 3 ? kBad : kWarn;
        row(label, c, name, c, r->second.text);
      } else if (!s->answered) {
        row(label, grey, name, grey, started ? "Was off: not updated" : "Off: turn it on");
      } else if (needs) {
        char text[64];
        std::snprintf(text, sizeof text, "Needs an update (radio %08X)", s->device.radioVersion);
        row(label, kWarn, name, kWarn, started ? std::string("Waiting") : std::string(text));
      } else {
        row(label, kGood, name, kGood, "Up to date");
      }
    }
    {
      const auto r = results.find(-1);
      char fw[64];
      std::snprintf(fw, sizeof fw, "Firmware %04X, updates to %04X", updateDongle_.firmware, PackagedDongleVersion());
      const bool current = updateDongle_.firmware == PackagedDongleVersion();
      const ImVec4 c = r == results.end() ? (current ? kGood : kWarn) : r->second.state == 2 ? kGood : r->second.state == 3 ? kBad : kWarn;
      row("Dongle", c, "Doorman Dongle", c, r != results.end() ? r->second.text : current ? std::string("Up to date") : std::string(fw));
    }
    ImGui::EndTable();
  }
  ImGui::Spacing();

  if (job_.Finished()) {
    ImGui::PushFont(g_title);
    if (job_.Ok()) ImGui::TextColored(kGood, "Update finished");
    else ImGui::TextColored(kBad, "Update failed");
    ImGui::PopFont();
    if (!job_.Ok()) ImGui::TextUnformatted(job_.Error().c_str());
    Back(Page::Home);
    return;
  }
  if (job_.Running()) return;

  if (!RuntimesClosed()) { Back(Page::DongleUpdatePick); return; }
  if (!updateDongle_.Banked()) {
    ImGui::TextColored(kWarn, "This dongle's firmware is too old to update here: use Install or Update Firmware > Doorman");
    Back(Page::DongleUpdatePick);
    return;
  }
  // Every paired device on and up to date: only the dongle is left, so it starts on its own.
  if (have && off == 0 && outdated == 0) {
    StartDongleUpdate(slots);
    return;
  }
  const float w = std::min(fs * 34, ImGui::GetContentRegionAvail().x);
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.f, (ImGui::GetContentRegionAvail().x - w) / 2));
  char sub[96];
  if (off) std::snprintf(sub, sizeof sub, "%d of %d devices are off", off, paired);
  else std::snprintf(sub, sizeof sub, "%d device%s, then the dongle", outdated, outdated == 1 ? "" : "s");
  ImGui::BeginDisabled(!have);
  const bool go = Tile({Icon::Download, "Update", sub, off ? Tint::Yellow : Tint::Green}, w, fs * 5, true);
  ImGui::EndDisabled();
  if (off && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    ImGui::SetTooltip("Turning on all the devices is recommended! If a device does not automatically connect after updating, "
                      "you can update that device via a cable through the flashing menu!");
  if (go) StartDongleUpdate(slots);
  Back(Page::DongleUpdatePick);
}

}  // namespace openhouse::installer
