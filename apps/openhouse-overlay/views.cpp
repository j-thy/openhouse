#include "views.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <imgui.h>
#include <string>

namespace openhouse::viewer {
namespace {
const ImVec4 kGood{0.36f, 0.80f, 0.45f, 1}, kWarn{1.00f, 0.78f, 0.30f, 1}, kBad{1.00f, 0.42f, 0.40f, 1},
    kInfo{0.45f, 0.70f, 1.00f, 1}, kOff{0.55f, 0.58f, 0.62f, 1};

ImU32 U32(const ImVec4& c) { return ImGui::ColorConvertFloat4ToU32(c); }

// "LHB-D7B1AD76" -> "D7B1AD76".
std::string Short(const std::string& serial) {
  const auto dash = serial.find('-');
  return dash == std::string::npos ? serial : serial.substr(dash + 1);
}

std::string BaseLabel(const State& s, const std::string& serial) {
  for (const auto& b : s.bases)
    if (b.serial == serial && !b.channel.empty()) return "ch " + b.channel + " (" + Short(serial) + ")";
  return Short(serial);
}

std::string DeviceName(const Device& d) {
  if (!d.name.empty()) return d.name;
  std::string kind = d.role == "hmd" ? "Headset" : d.role == "controller" ? "Controller" : "Tracker";
  if (d.hand == "left_hand") kind += " (left)";
  else if (d.hand == "right_hand") kind += " (right)";
  return kind;
}

// What the user should read for a device's tracking state.
void StateText(const Device& d) {
  if (!d.connected) { ImGui::TextColored(kOff, "Off"); return; }
  if (!d.enabled) { ImGui::TextColored(kOff, "Disabled"); return; }
  if (d.state == "tracking") ImGui::TextColored(kGood, "Tracking");
  else if (d.state == "coasting") ImGui::TextColored(kWarn, "Briefly hidden");
  else if (d.state == "lost") ImGui::TextColored(kBad, "Lost - no base sees it");
  else if (d.state == "acquiring") ImGui::TextColored(kInfo, "Finding its place");
  else ImGui::TextColored(kOff, "Waiting for light");
}

int SeenBy(const Device& d) {
  int n = 0;
  for (const auto& c : d.coverage) n += c.sensors >= 3;
  return n;
}

// A base's placement in plain words.
void PlacementText(const Base& b, double settleSigma) {
  if (b.origin) ImGui::TextColored(kGood, "Reference point");
  else if (b.locked) ImGui::TextColored(kGood, "Locked");
  else if (!b.placed) ImGui::TextColored(kWarn, "Not placed yet");
  else if (b.learned) ImGui::TextColored(kGood, "Placed (%.1f mm)", b.sigma * 1000);
  else ImGui::TextColored(kWarn, "Learning (%.0f mm, needs %.0f)", b.sigma * 1000, settleSigma * 1000);
}

}  // namespace

void DrawConnection(const TelemetryClient& client) {
  if (client.Connected()) {
    ImGui::TextColored(kGood, "Connected");
    ImGui::SameLine();
    ImGui::TextDisabled("to openhouse %s in %s (pid %d)", client.Version().empty() ? "(older than 1.0)" : client.Version().c_str(),
                        client.Host().empty() ? "?" : client.Host().c_str(), client.Pid());
  } else {
    ImGui::TextColored(kBad, "Not connected");
    ImGui::SameLine();
    ImGui::TextDisabled("Waiting for the driver (port %d)", client.Port());
  }
}

void DrawRoom(const State& s) {
  int visible = 0;
  std::string waiting;
  const Base* worst = nullptr;
  for (const auto& b : s.bases) {
    if (!b.seen) continue;
    ++visible;
    if (b.origin || (b.placed && b.learned)) continue;  // as the driver settles
    if (!worst || b.sigma > worst->sigma) worst = &b;
  }
  if (visible == 0) {
    ImGui::TextColored(kWarn, "No base stations seen yet");
    ImGui::TextDisabled("No base stations seen yet");
    return;
  }
  if (s.room.settled) {
    ImGui::TextColored(kGood, "Room ready");
    ImGui::SameLine();
    ImGui::TextDisabled("- all %d visible base stations are placed", visible);
  } else {
    ImGui::TextColored(kWarn, "Room still learning");
    if (worst) {
      ImGui::SameLine();
      ImGui::TextDisabled("- waiting for %s", BaseLabel(s, worst->serial).c_str());
      ImGui::TextWrapped("Move a device slowly where it and another base station both see it");
    }
  }
}

void DrawDevices(const State& s, Detail detail) {
  const bool adv = detail == Detail::Advanced;
  const int columns = adv ? 8 : 5;
  if (!ImGui::BeginTable("devices", columns, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp))
    return;
  ImGui::TableSetupColumn("Device");
  ImGui::TableSetupColumn("Serial");
  ImGui::TableSetupColumn("Status");
  ImGui::TableSetupColumn("Battery");
  ImGui::TableSetupColumn("Seen by");
  if (adv) {
    ImGui::TableSetupColumn("Position (m)");
    ImGui::TableSetupColumn("Speed");
    ImGui::TableSetupColumn("Tracking from");
  }
  ImGui::TableHeadersRow();
  for (const auto& d : s.devices) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(DeviceName(d).c_str());
    ImGui::TableNextColumn();
    ImGui::TextDisabled("%s", d.serial.c_str());
    ImGui::TableNextColumn();
    StateText(d);
    ImGui::TableNextColumn();
    if (d.battery) ImGui::TextColored(*d.battery <= 15 ? kBad : *d.battery <= 30 ? kWarn : kGood, "%d%%%s", *d.battery, d.charging ? " +" : "");
    else ImGui::TextDisabled("-");
    ImGui::TableNextColumn();
    const int seen = SeenBy(d);
    if (!d.connected) ImGui::TextDisabled("-");
    else ImGui::TextColored(seen >= 2 ? kGood : seen == 1 ? kWarn : kBad, "%d base%s", seen, seen == 1 ? "" : "s");
    if (ImGui::IsItemHovered() && !d.coverage.empty()) {
      ImGui::BeginTooltip();
      for (const auto& c : d.coverage) ImGui::Text("%s: %.1f sensors lit", BaseLabel(s, c.base).c_str(), c.sensors);
      ImGui::EndTooltip();
    }
    if (adv) {
      ImGui::TableNextColumn();
      if (d.valid) ImGui::Text("%.2f %.2f %.2f", d.p[0], d.p[1], d.p[2]);
      else ImGui::TextDisabled("-");
      ImGui::TableNextColumn();
      ImGui::Text("%.2f m/s", std::sqrt(d.v[0] * d.v[0] + d.v[1] * d.v[1] + d.v[2] * d.v[2]));
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(d.primary.empty() ? "-" : BaseLabel(s, d.primary).c_str());
    }
  }
  ImGui::EndTable();

  if (!adv) return;
  // Per device: how much of each base's light it gets right now.
  for (const auto& d : s.devices) {
    if (!d.connected || d.coverage.empty()) continue;
    if (!ImGui::TreeNode(d.serial.c_str(), "%s %s: light per base", DeviceName(d).c_str(), d.serial.c_str())) continue;
    for (const auto& c : d.coverage) {
      const bool inFilter = std::find(d.bases.begin(), d.bases.end(), c.base) != d.bases.end();
      char overlay[64];
      std::snprintf(overlay, sizeof overlay, "%.1f sensors, %d sweeps", c.sensors, c.frames);
      ImGui::ProgressBar(float(std::min(c.sensors / 12.0, 1.0)), ImVec2(220, 0), overlay);
      ImGui::SameLine();
      ImGui::Text("%s%s", BaseLabel(s, c.base).c_str(), inFilter ? "" : "  (not used yet)");
    }
    ImGui::TreePop();
  }
}

void DrawBases(const State& s, Detail detail) {
  const bool adv = detail == Detail::Advanced;
  if (!ImGui::BeginTable("bases", adv ? 7 : 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp))
    return;
  ImGui::TableSetupColumn("Channel");
  ImGui::TableSetupColumn("Serial");
  ImGui::TableSetupColumn("Seen");
  ImGui::TableSetupColumn("Placement");
  if (adv) {
    ImGui::TableSetupColumn("Evidence");
    ImGui::TableSetupColumn("Position (m)");
    ImGui::TableSetupColumn("Faults");
  }
  ImGui::TableHeadersRow();
  for (const auto& b : s.bases) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(b.channel.empty() ? "?" : b.channel.c_str());
    ImGui::TableNextColumn();
    ImGui::TextDisabled("%s", b.serial.c_str());
    ImGui::TableNextColumn();
    if (b.seen) ImGui::TextColored(kGood, "Yes");
    else ImGui::TextColored(kOff, "Not this session");
    ImGui::TableNextColumn();
    PlacementText(b, s.room.settleSigma);
    if (adv) {
      ImGui::TableNextColumn();
      ImGui::Text("%d keyframes", b.observations);
      ImGui::TableNextColumn();
      if (b.placed) ImGui::Text("%.2f %.2f %.2f", b.p[0], b.p[1], b.p[2]);
      else ImGui::TextDisabled("-");
      ImGui::TableNextColumn();
      if (b.faults) ImGui::TextColored(kBad, "0x%02x", b.faults);
      else ImGui::TextDisabled("none");
    }
  }
  ImGui::EndTable();
}

void DrawReceivers(const State& s) {
  if (s.receivers.empty()) { ImGui::TextDisabled("No dongles plugged in"); return; }
  for (const auto& r : s.receivers)
    ImGui::BulletText("%s: %s", r.serial.c_str(), r.connected.empty() ? "-" : r.connected.c_str());
}

namespace {
using V3 = std::array<double, 3>;
V3 Rotate(const std::array<double, 4>& q, const V3& v) {  // q: w x y z
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  const V3 u{x, y, z};
  const V3 t{2 * (u[1] * v[2] - u[2] * v[1]), 2 * (u[2] * v[0] - u[0] * v[2]), 2 * (u[0] * v[1] - u[1] * v[0])};
  return {v[0] + w * t[0] + (u[1] * t[2] - u[2] * t[1]), v[1] + w * t[1] + (u[2] * t[0] - u[0] * t[2]),
          v[2] + w * t[2] + (u[0] * t[1] - u[1] * t[0])};
}
V3 Add(const V3& a, const V3& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }

// An orbit camera around a target, perspective-projected onto a rectangle.
struct Camera {
  double yaw = 0.6, pitch = 0.55, distance = 7;
  V3 target{0, 1, 0};
  ImVec2 origin, size;
  // false when behind the camera.
  bool Project(const V3& p, ImVec2* out) const {
    const double cy = std::cos(yaw), sy = std::sin(yaw), cp = std::cos(pitch), sp = std::sin(pitch);
    const V3 eye{target[0] + distance * cp * sy, target[1] + distance * sp, target[2] + distance * cp * cy};
    // camera basis: forward toward target
    V3 f{target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]};
    const double fn = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    for (auto& c : f) c /= fn;
    V3 r{f[2], 0, -f[0]};  // forward x up(0,1,0), normalized
    const double rn = std::hypot(r[0], r[2]);
    r = {-r[0] / rn, 0, -r[2] / rn};
    const V3 u{r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]};
    const V3 d{p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]};
    const double z = d[0] * f[0] + d[1] * f[1] + d[2] * f[2];
    if (z < 0.05) return false;
    const double x = d[0] * r[0] + d[1] * r[1] + d[2] * r[2], y = d[0] * u[0] + d[1] * u[1] + d[2] * u[2];
    const double focal = std::min(size.x, size.y) * 1.1;
    *out = ImVec2(float(origin.x + size.x / 2 + focal * x / z), float(origin.y + size.y / 2 - focal * y / z));
    return true;
  }
};

void Line3(ImDrawList* draw, const Camera& cam, const V3& a, const V3& b, ImU32 color, float width = 1) {
  ImVec2 pa, pb;
  if (cam.Project(a, &pa) && cam.Project(b, &pb)) draw->AddLine(pa, pb, color, width);
}
}  // namespace

void DrawMap(const State& s) {
  static Camera cam;
  static bool framed = false;
  ImDrawList* draw = ImGui::GetWindowDrawList();
  cam.origin = ImGui::GetCursorScreenPos();
  ImVec2 size = ImGui::GetContentRegionAvail();
  cam.size = ImVec2(std::max(size.x, 120.f), std::max(size.y, 120.f));
  ImGui::InvisibleButton("map", cam.size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
  const ImGuiIO& io = ImGui::GetIO();
  // Frame the room once there is something to frame (and on double-click).
  auto frame = [&] {
    double lo[3] = {1e9, 1e9, 1e9}, hi[3] = {-1e9, -1e9, -1e9};
    int n = 0;
    auto take = [&](const V3& p) { for (int k = 0; k < 3; ++k) { lo[k] = std::min(lo[k], p[k]); hi[k] = std::max(hi[k], p[k]); } ++n; };
    for (const auto& b : s.bases) if (b.placed) take(b.p);
    for (const auto& d : s.devices) if (d.valid) take(d.p);
    if (!n) return false;
    cam.target = {(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2};
    cam.distance = std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2], 3.0}) * 1.6;
    return true;
  };
  if (!framed) framed = frame();
  if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) frame();
  if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
    cam.yaw -= io.MouseDelta.x * 0.008;
    cam.pitch = std::clamp(cam.pitch + io.MouseDelta.y * 0.008, -1.4, 1.5);
  }
  if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
    const double k = cam.distance * 0.0015;
    cam.target[0] -= (std::cos(cam.yaw) * io.MouseDelta.x) * k;
    cam.target[2] += (std::sin(cam.yaw) * io.MouseDelta.x) * k;
    cam.target[1] += io.MouseDelta.y * k;
  }
  if (ImGui::IsItemHovered() && io.MouseWheel != 0) cam.distance = std::clamp(cam.distance * std::pow(0.88, io.MouseWheel), 0.5, 60.0);

  const ImVec2 end(cam.origin.x + cam.size.x, cam.origin.y + cam.size.y);
  draw->AddRectFilled(cam.origin, end, IM_COL32(18, 21, 26, 255), 4);
  draw->PushClipRect(cam.origin, end, true);
  // Floor grid at the lowest device/base height minus a little: the room's own floor
  // isn't known here, so it is only a reference for direction and scale (1 m).
  double floorY = cam.target[1] - 1.5;
  for (const auto& d : s.devices) if (d.valid) floorY = std::min(floorY, d.p[1] - 0.2);
  const int half = 6;
  const double gx = std::round(cam.target[0]), gz = std::round(cam.target[2]);
  for (int k = -half; k <= half; ++k) {
    const ImU32 c = k == 0 ? IM_COL32(70, 78, 90, 255) : IM_COL32(40, 45, 52, 255);
    Line3(draw, cam, {gx + k, floorY, gz - half}, {gx + k, floorY, gz + half}, c);
    Line3(draw, cam, {gx - half, floorY, gz + k}, {gx + half, floorY, gz + k}, c);
  }
  // Base stations: a small pyramid along the direction they sweep (-z of the base).
  for (const auto& b : s.bases) {
    if (!b.placed) continue;
    const ImU32 col = U32(!b.seen ? kOff : (b.origin || b.learned) ? kGood : kWarn);
    const V3 tip = b.p;
    const double l = 0.35, w = 0.18;
    const V3 corners[4] = {Add(tip, Rotate(b.q, {-w, -w, -l})), Add(tip, Rotate(b.q, {w, -w, -l})),
                           Add(tip, Rotate(b.q, {w, w, -l})), Add(tip, Rotate(b.q, {-w, w, -l}))};
    for (int k = 0; k < 4; ++k) {
      Line3(draw, cam, tip, corners[k], col, 1.5f);
      Line3(draw, cam, corners[k], corners[(k + 1) % 4], col, 1.5f);
    }
    Line3(draw, cam, tip, {tip[0], floorY, tip[2]}, IM_COL32(60, 66, 76, 255));  // drop line: height at a glance
    ImVec2 p;
    if (cam.Project(tip, &p)) {
      const std::string label = b.channel.empty() ? Short(b.serial) : "ch " + b.channel;
      draw->AddText({p.x + 8, p.y - 16}, IM_COL32(220, 222, 226, 255), label.c_str());
    }
  }
  // Devices: a dot and their own axes (red x, green y, blue z), 10 cm.
  for (const auto& d : s.devices) {
    if (!d.valid || !d.connected) continue;
    const ImVec4 col = d.state == "tracking" ? kGood : d.state == "coasting" ? kWarn : kBad;
    Line3(draw, cam, d.p, Add(d.p, Rotate(d.q, {0.1, 0, 0})), IM_COL32(230, 80, 80, 255), 2);
    Line3(draw, cam, d.p, Add(d.p, Rotate(d.q, {0, 0.1, 0})), IM_COL32(80, 220, 100, 255), 2);
    Line3(draw, cam, d.p, Add(d.p, Rotate(d.q, {0, 0, 0.1})), IM_COL32(90, 140, 255, 255), 2);
    Line3(draw, cam, d.p, {d.p[0], floorY, d.p[2]}, IM_COL32(60, 66, 76, 255));
    ImVec2 p;
    if (cam.Project(d.p, &p)) {
      draw->AddCircleFilled(p, d.role == "hmd" ? 6.f : 4.5f, U32(col));
      draw->AddText({p.x + 8, p.y - 6}, IM_COL32(200, 204, 210, 255), DeviceName(d).c_str());
    }
  }
  draw->PopClipRect();
  draw->AddText({cam.origin.x + 8, cam.origin.y + 6}, IM_COL32(140, 146, 154, 255),
                "Drag: orbit   Right-drag: pan   Scroll: zoom   Double-click: fit   (1 m grid)");
}

}  // namespace openhouse::viewer
