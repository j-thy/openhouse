// lhviz — 3D visualizer for the lh runtime. Uses only the public liblh session API.
//
//   lhviz                       live USB devices (SteamVR must not be running)
//   lhviz --replay CAPTURE      replay a lhcapture recording (real time)
//
// Right-drag orbit, middle-drag pan, wheel zoom, double-click a device to follow it.
#include <GLFW/glfw3.h>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl2.h"
#include "imgui.h"
#include "lh/session.hpp"

namespace {

struct Camera {
  double yaw = 35, pitch = 25, distance = 7;
  Eigen::Vector3d target = Eigen::Vector3d(0, -1.5, -2.5);
  Eigen::Matrix4d view = Eigen::Matrix4d::Identity(), projection = Eigen::Matrix4d::Identity();
  int width = 1, height = 1;

  Eigen::Vector3d Eye() const {
    const double y = yaw * M_PI / 180, p = pitch * M_PI / 180;
    return target + distance * Eigen::Vector3d(std::cos(p) * std::sin(y), std::sin(p), std::cos(p) * std::cos(y));
  }
  void Update(int w, int h) {
    width = std::max(w, 1); height = std::max(h, 1);
    const Eigen::Vector3d eye = Eye(), f = (target - eye).normalized();
    const Eigen::Vector3d s = f.cross(Eigen::Vector3d::UnitY()).normalized(), u = s.cross(f);
    view.setIdentity();
    view.block<1, 3>(0, 0) = s.transpose(); view.block<1, 3>(1, 0) = u.transpose(); view.block<1, 3>(2, 0) = -f.transpose();
    view(0, 3) = -s.dot(eye); view(1, 3) = -u.dot(eye); view(2, 3) = f.dot(eye);
    const double n = 0.02, fa = 200, t = n * std::tan(25 * M_PI / 180), r = t * width / height;
    projection.setZero();
    projection(0, 0) = n / r; projection(1, 1) = n / t;
    projection(2, 2) = -(fa + n) / (fa - n); projection(2, 3) = -2 * fa * n / (fa - n); projection(3, 2) = -1;
  }
  std::optional<ImVec2> Project(const Eigen::Vector3d& p) const {
    const Eigen::Vector4d c = projection * view * p.homogeneous();
    if (c.w() <= 1e-6) return std::nullopt;
    const Eigen::Vector3d n = c.head<3>() / c.w();
    if (std::abs(n.x()) > 1.2 || std::abs(n.y()) > 1.2) return std::nullopt;
    return ImVec2(float((n.x() * 0.5 + 0.5) * width), float((0.5 - n.y() * 0.5) * height));
  }
};

Eigen::Isometry3d ToIso(const lh::Pose& p) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = Eigen::Quaterniond(p.orientation[0], p.orientation[1], p.orientation[2], p.orientation[3])
                   .normalized().toRotationMatrix();
  t.translation() = Eigen::Vector3d(p.position[0], p.position[1], p.position[2]);
  return t;
}

void Vertex(const Eigen::Vector3d& v) { glVertex3d(v.x(), v.y(), v.z()); }

void Axes(const Eigen::Isometry3d& t, double size, float alpha = 1) {
  glBegin(GL_LINES);
  const Eigen::Vector3d o = t.translation();
  glColor4f(1, .25f, .25f, alpha); Vertex(o); Vertex(t * Eigen::Vector3d(size, 0, 0));
  glColor4f(.25f, 1, .25f, alpha); Vertex(o); Vertex(t * Eigen::Vector3d(0, size, 0));
  glColor4f(.3f, .5f, 1, alpha); Vertex(o); Vertex(t * Eigen::Vector3d(0, 0, size));
  glEnd();
}

ImVec4 StateColor(lh::TrackingState s) {
  switch (s) {
    case lh::TrackingState::Tracking: return {0.3f, 0.95f, 0.4f, 1};
    case lh::TrackingState::Coasting: return {1.0f, 0.8f, 0.2f, 1};
    case lh::TrackingState::Acquiring: return {0.4f, 0.7f, 1.0f, 1};
    case lh::TrackingState::Lost: return {1.0f, 0.35f, 0.3f, 1};
    case lh::TrackingState::Uninitialized: return {0.7f, 0.7f, 0.7f, 1};
    default: return {0.45f, 0.45f, 0.45f, 1};
  }
}

ImVec4 RoleColor(lh::DeviceRole r) {
  switch (r) {
    case lh::DeviceRole::Hmd: return {0.95f, 0.6f, 0.25f, 1};
    case lh::DeviceRole::Controller: return {0.35f, 0.75f, 1.0f, 1};
    case lh::DeviceRole::Tracker: return {0.8f, 0.5f, 1.0f, 1};
    default: return {0.9f, 0.9f, 0.9f, 1};
  }
}

const char* ResultName(lh::TrackingResult r) {
  switch (r) {
    case lh::TrackingResult::RunningOk: return "Running_OK";
    case lh::TrackingResult::RunningOutOfRange: return "Running_OutOfRange";
    case lh::TrackingResult::CalibratingInProgress: return "Calibrating_InProgress";
    case lh::TrackingResult::CalibratingOutOfRange: return "Calibrating_OutOfRange";
    default: return "Uninitialized";
  }
}

void Bar(const char* label, double value) {
  ImGui::ProgressBar(float(std::clamp(value, 0.0, 1.0)), ImVec2(140, 0), "");
  ImGui::SameLine();
  ImGui::Text("%s %.2f", label, value);
}

void Light(const char* label, bool on, bool touched = false) {
  const ImVec4 c = on ? ImVec4(0.3f, 0.95f, 0.4f, 1) : touched ? ImVec4(0.35f, 0.6f, 1, 1) : ImVec4(0.3f, 0.3f, 0.3f, 1);
  ImGui::PushStyleColor(ImGuiCol_Button, c);
  ImGui::Button(label, ImVec2(64, 0));
  ImGui::PopStyleColor();
}

void Stick(const char* label, double x, double y, bool active) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 p = ImGui::GetCursorScreenPos();
  const float r = 32;
  const ImVec2 c(p.x + r, p.y + r);
  dl->AddCircle(c, r, IM_COL32(150, 150, 150, 255), 32, 1.5f);
  dl->AddCircleFilled(ImVec2(c.x + float(x) * r, c.y - float(y) * r), 5,
                      active ? IM_COL32(80, 240, 110, 255) : IM_COL32(200, 200, 200, 255));
  ImGui::Dummy(ImVec2(2 * r, 2 * r));
  ImGui::SameLine();
  ImGui::Text("%s\nx %+.2f\ny %+.2f", label, x, y);
}

struct Trail {
  std::deque<Eigen::Vector3d> points;
};

// Receivers panel: one row per Watchman dongle and per Doorman Dongle slot, with live link stats.
struct ReceiverHistory {
  std::deque<float> kbps;   // one sample per refresh, for the sparkline
};

void Badge(const char* text, ImVec4 color) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 sz = ImGui::CalcTextSize(text);
  const ImVec2 p = ImGui::GetCursorScreenPos();
  const ImVec2 q(p.x + sz.x + 10, p.y + sz.y + 2);
  dl->AddRectFilled(p, q, ImGui::ColorConvertFloat4ToU32(ImVec4(color.x, color.y, color.z, 0.25f)), 4);
  dl->AddRect(p, q, ImGui::ColorConvertFloat4ToU32(color), 4);
  dl->AddText(ImVec2(p.x + 5, p.y + 1), ImGui::ColorConvertFloat4ToU32(color), text);
  ImGui::Dummy(ImVec2(sz.x + 10, sz.y + 2));
}

void Dot(bool on, bool warn = false) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 p = ImGui::GetCursorScreenPos();
  const float h = ImGui::GetTextLineHeight();
  const ImU32 c = !on ? IM_COL32(110, 110, 110, 255) : warn ? IM_COL32(240, 190, 60, 255) : IM_COL32(80, 230, 110, 255);
  dl->AddCircleFilled(ImVec2(p.x + h * 0.5f, p.y + h * 0.5f + 1), h * 0.3f, c);
  ImGui::Dummy(ImVec2(h, h));
}

ImVec4 LossColor(double loss) {
  if (loss < 0) return ImVec4(0.6f, 0.6f, 0.6f, 1);
  return loss < 3 ? ImVec4(0.4f, 0.9f, 0.5f, 1) : loss < 10 ? ImVec4(0.95f, 0.8f, 0.3f, 1) : ImVec4(1, 0.4f, 0.35f, 1);
}

ImVec4 RssiColor(int rssi) {
  if (!rssi) return ImVec4(0.6f, 0.6f, 0.6f, 1);
  return rssi > -65 ? ImVec4(0.4f, 0.9f, 0.5f, 1) : rssi > -78 ? ImVec4(0.95f, 0.8f, 0.3f, 1) : ImVec4(1, 0.4f, 0.35f, 1);
}

}  // namespace

int main(int argc, char** argv) {
  lh::SessionOptions options;
  std::string screenshotPath;   // --screenshot FILE.ppm SECONDS: save one frame, then exit
  double screenshotAfter = 0;
  std::string initialSelection;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) { fprintf(stderr, "missing value for %s\n", a.c_str()); std::exit(2); }
      return argv[++i];
    };
    if (a == "--replay") options.replayPath = next();
    else if (a == "--fast") options.replayRealtime = false;
    else if (a == "--room") options.roomMapPath = next();
    else if (a == "--steam-db") options.steamDatabasePath = next();
    else if (a == "--select") initialSelection = next();
    else if (a == "--disable") options.disabledDevices.push_back(next());
    else if (a == "--screenshot") { screenshotPath = next(); screenshotAfter = std::stod(next()); }
    else if (a == "--help" || a == "-h") {
      printf("usage: lhviz [--replay CAPTURE.lhraw [--fast]] [--room roommap.json] [--steam-db lighthousedb.json|-]\n"
             "             [--disable SERIAL]... [--select SERIAL] [--screenshot FILE.ppm SECONDS]\n"
             "Devices can also be enabled/disabled in the device table (saved for live sessions).\n");
      return 0;
    } else { fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
  }

  auto session = lh::Session::Create(options);
  std::string error;
  if (!session->Start(&error)) { fprintf(stderr, "lhviz: %s\n", error.c_str()); return 1; }

  glfwSetErrorCallback([](int, const char* text) { fprintf(stderr, "GLFW: %s\n", text); });
  if (!glfwInit()) return 1;
  glfwWindowHint(GLFW_SAMPLES, 4);
  GLFWwindow* window = glfwCreateWindow(1500, 900, "lhviz — lh runtime", nullptr, nullptr);
  if (!window) { glfwTerminate(); return 1; }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::GetIO().IniFilename = nullptr;
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL2_Init();

  Camera camera;
  std::map<std::string, Trail> trails;
  std::deque<lh::SessionEvent> log;
  std::string selected = initialSelection, follow;
  bool showTrails = true, showSensors = true, showHead = false, showFrustums = true, autoScroll = true;
  float floorHeight = -2.6f;
  bool floorAuto = true;
  double lastTrailTime = 0;
  std::vector<lh::Session::Receiver> receivers;
  std::map<std::string, ReceiverHistory> receiverHistory;
  double lastReceiverPoll = -1;

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    for (auto& e : session->PollEvents()) {
      log.push_back(std::move(e));
      if (log.size() > 500) log.pop_front();
    }
    const lh::SessionSnapshot snap = session->Snapshot();

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    ImGuiIO& io = ImGui::GetIO();

    // ---- camera input (when the mouse is over the 3D view) ----------------------
    if (!io.WantCaptureMouse) {
      if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
        camera.yaw -= io.MouseDelta.x * 0.3;
        camera.pitch = std::clamp(camera.pitch + io.MouseDelta.y * 0.3, -89.0, 89.0);
      }
      if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
        const Eigen::Vector3d eye = camera.Eye(), f = (camera.target - eye).normalized();
        const Eigen::Vector3d s = f.cross(Eigen::Vector3d::UnitY()).normalized(), u = s.cross(f);
        const double k = camera.distance * 0.0015;
        camera.target += (-s * io.MouseDelta.x + u * io.MouseDelta.y) * k;
        follow.clear();
      }
      if (io.MouseWheel != 0) camera.distance = std::clamp(camera.distance * std::exp(-io.MouseWheel * 0.12), 0.3, 80.0);
    }

    // ---- follow / floor ------------------------------------------------------
    double lowest = 1e9;
    for (const auto& d : snap.devices) {
      if (!d.pose.valid) continue;
      // Only well-tracked poses: one flight out of the room would drag the floor down for good.
      if (d.pose.state == lh::TrackingState::Tracking) lowest = std::min(lowest, d.pose.worldFromImu.position[1]);
      if (d.info.serial == follow) {
        const auto& p = d.pose.worldFromImu.position;
        camera.target = camera.target * 0.9 + Eigen::Vector3d(p[0], p[1], p[2]) * 0.1;
      }
    }
    // The floor: just under the 2nd percentile of tracked heights (one sample per frame,
    // last 4000), not the minimum: a device thrown metres down by a bad fix isn't the floor.
    if (floorAuto && lowest < 1e8) {
      static std::deque<double> heights;
      heights.push_back(lowest);
      if (heights.size() > 4000) heights.pop_front();
      std::vector<double> sorted(heights.begin(), heights.end());
      const size_t k = sorted.size() / 50;
      std::nth_element(sorted.begin(), sorted.begin() + long(k), sorted.end());
      floorHeight = float(sorted[k] - 0.05);
    }

    // ---- trails ------------------------------------------------------------------
    if (snap.hostTime - lastTrailTime > 0.02 || snap.hostTime < lastTrailTime) {
      lastTrailTime = snap.hostTime;
      for (const auto& d : snap.devices) {
        auto& t = trails[d.info.serial];
        if (!d.pose.valid) continue;
        const auto& p = d.pose.worldFromImu.position;
        t.points.emplace_back(p[0], p[1], p[2]);
        if (t.points.size() > 600) t.points.pop_front();
      }
    }

    // ---- 3D scene -------------------------------------------------------------------
    int fw, fh;
    glfwGetFramebufferSize(window, &fw, &fh);
    camera.Update(fw, fh);
    glViewport(0, 0, fw, fh);
    glClearColor(0.07f, 0.08f, 0.1f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(camera.projection.data());
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd(camera.view.data());

    // Floor grid (world is Y up; height is a display choice, see the panel).
    glLineWidth(1);
    glBegin(GL_LINES);
    for (int i = -10; i <= 10; ++i) {
      const float a = i == 0 ? 0.35f : 0.15f;
      glColor4f(0.6f, 0.65f, 0.7f, a);
      glVertex3f(float(i) * 0.5f, floorHeight, -5); glVertex3f(float(i) * 0.5f, floorHeight, 5);
      glVertex3f(-5, floorHeight, float(i) * 0.5f); glVertex3f(5, floorHeight, float(i) * 0.5f);
    }
    glEnd();
    glLineWidth(2);
    Axes(Eigen::Isometry3d::Identity(), 0.3, 0.6f);

    // Base stations: box + view frustum (LH1 looks down its -Z axis, ~120 deg).
    for (const auto& b : snap.bases) {
      if (!b.known) continue;
      const Eigen::Isometry3d t = ToIso(b.worldFromBase);
      const float c = b.seen ? 1.0f : 0.45f;
      glColor4f(0.9f * c, 0.9f * c, 0.95f * c, 1);
      glBegin(GL_LINES);
      const double s = 0.07;
      const Eigen::Vector3d corners[8] = {{-s, -s, -s}, {s, -s, -s}, {s, s, -s}, {-s, s, -s},
                                          {-s, -s, s},  {s, -s, s},  {s, s, s},  {-s, s, s}};
      const int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
      for (const auto& e : edges) { Vertex(t * corners[e[0]]); Vertex(t * corners[e[1]]); }
      if (showFrustums) {
        glColor4f(0.9f * c, 0.85f * c, 0.4f * c, 0.35f);
        const double d = 1.2, h = d * std::tan(60 * M_PI / 180);
        const Eigen::Vector3d f[4] = {{-h, -h, -d}, {h, -h, -d}, {h, h, -d}, {-h, h, -d}};
        for (int i = 0; i < 4; ++i) {
          Vertex(t.translation()); Vertex(t * f[i]);
          Vertex(t * f[i]); Vertex(t * f[(i + 1) % 4]);
        }
      }
      glEnd();
      Axes(t, 0.15, 0.8f);
    }

    // Devices.
    for (const auto& d : snap.devices) {
      const auto& p = d.pose;
      const ImVec4 rc = RoleColor(d.info.role), sc = StateColor(p.state);
      if (showTrails) {
        const auto& t = trails[d.info.serial];
        glLineWidth(1.5f);
        glBegin(GL_LINE_STRIP);
        for (size_t i = 0; i < t.points.size(); ++i) {
          glColor4f(rc.x, rc.y, rc.z, 0.08f + 0.6f * float(i) / float(t.points.size()));
          Vertex(t.points[i]);
        }
        glEnd();
      }
      if (!p.valid) continue;
      const Eigen::Isometry3d imu = ToIso(p.worldFromImu);
      glLineWidth(d.info.serial == selected ? 4.f : 2.5f);
      Axes(imu, d.info.role == lh::DeviceRole::Hmd ? 0.18 : 0.12);
      if (showHead) Axes(ToIso(p.worldFromHead), 0.08, 0.6f);
      if (showSensors) {
        glPointSize(d.info.serial == selected ? 6.f : 4.f);
        glBegin(GL_POINTS);
        glColor4f(sc.x, sc.y, sc.z, 1);
        for (const auto& s : d.info.sensors) Vertex(imu * Eigen::Vector3d(s[0], s[1], s[2]));
        glEnd();
      }
      // Rays to the bases currently in the filter.
      glLineWidth(1);
      glBegin(GL_LINES);
      for (uint32_t id : p.bases) {
        for (const auto& b : snap.bases) {
          if (b.id != id || !b.known) continue;
          glColor4f(sc.x, sc.y, sc.z, id == p.primaryBase ? 0.35f : 0.15f);
          Vertex(imu.translation());
          Vertex(ToIso(b.worldFromBase).translation());
        }
      }
      glEnd();
    }
    glDisable(GL_DEPTH_TEST);

    // ---- 3D labels ---------------------------------------------------------------------
    ImDrawList* fg = ImGui::GetBackgroundDrawList();
    for (const auto& b : snap.bases) {
      if (!b.known) continue;
      if (auto s = camera.Project(ToIso(b.worldFromBase).translation()))
        fg->AddText(ImVec2(s->x + 8, s->y - 6), b.seen ? IM_COL32(230, 230, 240, 255) : IM_COL32(140, 140, 150, 255),
                    (b.serial + (b.origin ? " (origin)" : "")).c_str());
    }
    for (const auto& d : snap.devices) {
      if (!d.pose.valid) continue;
      const auto& q = d.pose.worldFromImu.position;
      if (auto s = camera.Project(Eigen::Vector3d(q[0], q[1], q[2]))) {
        const ImVec4 c = StateColor(d.pose.state);
        fg->AddText(ImVec2(s->x + 10, s->y - 18), ImGui::ColorConvertFloat4ToU32(c), d.info.serial.c_str());
        if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse &&
            std::hypot(io.MousePos.x - s->x, io.MousePos.y - s->y) < 25) {
          follow = d.info.serial;
          selected = d.info.serial;
        }
      }
    }

    // ---- panels -------------------------------------------------------------------------------
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(560, 330), ImGuiCond_FirstUseEver);
    ImGui::Begin("Session");
    ImGui::Text("%s   room %s (rev %llu)   t=%.2f", options.replayPath.empty() ? "live" : "replay",
                snap.roomId.empty() ? "-" : snap.roomId.c_str(), static_cast<unsigned long long>(snap.roomRevision),
                snap.hostTime);
    if (session->Finished()) { ImGui::SameLine(); ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "  replay finished"); }
    if (ImGui::Button("Reset room")) session->ResetRoom();
    ImGui::SameLine();
    if (ImGui::Button("Save room")) {
      std::string err;
      if (!session->SaveRoom(&err)) log.push_back({snap.hostTime, lh::SessionEventType::Log, "", err});
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear trails")) trails.clear();
    ImGui::SameLine();
    if (ImGui::Button("Unfollow")) follow.clear();
    ImGui::Checkbox("trails", &showTrails); ImGui::SameLine();
    ImGui::Checkbox("sensors", &showSensors); ImGui::SameLine();
    ImGui::Checkbox("head frame", &showHead); ImGui::SameLine();
    ImGui::Checkbox("frustums", &showFrustums);
    ImGui::Checkbox("auto floor", &floorAuto); ImGui::SameLine();
    ImGui::SetNextItemWidth(160);
    ImGui::SliderFloat("floor height", &floorHeight, -4, 1, "%.2f m");
    if (ImGui::BeginTable("devices", 7, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_SizingStretchProp)) {
      ImGui::TableSetupColumn("on", ImGuiTableColumnFlags_WidthFixed, 24); ImGui::TableSetupColumn("device"); ImGui::TableSetupColumn("role"); ImGui::TableSetupColumn("state");
      ImGui::TableSetupColumn("bases"); ImGui::TableSetupColumn("updates"); ImGui::TableSetupColumn("battery");
      ImGui::TableHeadersRow();
      for (const auto& d : snap.devices) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        bool enabled = d.info.enabled;
        ImGui::PushID(d.info.serial.c_str());
        if (ImGui::Checkbox("##on", &enabled)) session->SetDeviceEnabled(d.info.serial, enabled);
        ImGui::PopID();
        ImGui::TableNextColumn();
        if (ImGui::Selectable(d.info.serial.c_str(), selected == d.info.serial, ImGuiSelectableFlags_SpanAllColumns))
          selected = d.info.serial;
        if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) follow = d.info.serial;
        ImGui::TableNextColumn();
        ImGui::TextColored(RoleColor(d.info.role), "%s", lh::ToString(d.info.role));
        ImGui::TableNextColumn();
        if (d.info.enabled) ImGui::TextColored(StateColor(d.pose.state), "%s", lh::ToString(d.pose.state));
        else ImGui::TextDisabled("disabled");
        ImGui::TableNextColumn();
        ImGui::Text("%zu", d.pose.bases.size());
        ImGui::TableNextColumn();
        ImGui::Text("%llu", static_cast<unsigned long long>(d.pose.updates));
        ImGui::TableNextColumn();
        if (d.info.batteryPercent) ImGui::Text("%d%%%s", *d.info.batteryPercent, d.info.charging ? " +" : "");
        else ImGui::TextDisabled("-");
      }
      ImGui::EndTable();
    }
    ImGui::End();

    // Selected device.
    const lh::DeviceState* sel = nullptr;
    for (const auto& d : snap.devices) if (d.info.serial == selected) sel = &d;
    if (!sel && !snap.devices.empty()) { selected = snap.devices.front().info.serial; sel = &snap.devices.front(); }
    if (sel) {
      ImGui::SetNextWindowPos(ImVec2(float(fw) - 430, 10), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(420, 560), ImGuiCond_FirstUseEver);
      ImGui::Begin("Device");
      const auto& i = sel->info;
      const auto& p = sel->pose;
      ImGui::TextColored(RoleColor(i.role), "%s  %s", i.serial.c_str(), lh::ToString(i.role));
      ImGui::Text("%s %s  class=%s  %s", i.manufacturer.c_str(), i.model.c_str(), i.deviceClass.c_str(),
                  i.controllerRole.c_str());
      ImGui::TextColored(StateColor(p.state), "%s  (%s)%s", lh::ToString(p.state), ResultName(p.result),
                         p.valid ? "" : "  pose invalid");
      ImGui::Text("primary base %08X   filter bases:", p.primaryBase);
      for (uint32_t b : p.bases) { ImGui::SameLine(); ImGui::Text("%08X", b); }
      const auto& pos = p.worldFromImu.position;
      const auto& o = p.worldFromImu.orientation;
      ImGui::Text("position  %+.3f %+.3f %+.3f m", pos[0], pos[1], pos[2]);
      ImGui::Text("rotation  w%+.3f x%+.3f y%+.3f z%+.3f", o[0], o[1], o[2], o[3]);
      ImGui::Text("velocity  %+.2f %+.2f %+.2f m/s", p.velocity[0], p.velocity[1], p.velocity[2]);
      ImGui::Text("ang. vel  %+.2f %+.2f %+.2f rad/s", p.angularVelocity[0], p.angularVelocity[1], p.angularVelocity[2]);
      if (i.batteryPercent) ImGui::Text("battery %d%%%s", *i.batteryPercent, i.charging ? " (charging)" : "");
      if (sel->input && (i.role == lh::DeviceRole::Tracker || i.controllerType == lh::ControllerType::ViveWand)) {
        // Vive wands and HTC trackers: the wand's buttons. On a tracker System is the power
        // button and Pad the thumb input (pogo pin 5); grip, trigger and menu are pins 3, 4, 6.
        const auto& in = *sel->input;
        const bool tracker = i.role == lh::DeviceRole::Tracker;
        ImGui::SeparatorText(tracker ? "Tracker input (pogo pins)" : "Vive wand input");
        Light(tracker ? "Power" : "System", in.systemClick); ImGui::SameLine();
        Light("Menu", in.menuClick); ImGui::SameLine();
        Light("Grip", in.gripClick); ImGui::SameLine();
        Light("Trigger", in.triggerClick);
        Light(tracker ? "Thumb" : "Pad click", in.trackpadClick); ImGui::SameLine();
        Light("Pad touch", in.trackpadTouch);
        Bar("trigger", in.trigger);
        if (!tracker) Stick("trackpad", in.trackpadX, in.trackpadY, in.trackpadTouch);
      } else if (sel->input) {
        const auto& in = *sel->input;
        ImGui::SeparatorText("Index controller input");
        Light("System", in.systemClick, in.systemTouch); ImGui::SameLine();
        Light("A", in.aClick, in.aTouch); ImGui::SameLine();
        Light("B", in.bClick, in.bTouch); ImGui::SameLine();
        Light("Trigger", in.triggerClick, in.triggerTouch);
        Light("Stick", in.thumbstickClick, in.thumbstickTouch); ImGui::SameLine();
        Light("Pad", in.trackpadTouch);
        Bar("trigger", in.trigger);
        Bar("grip force", in.gripForce);
        Bar("pad force", in.trackpadForce);
        Stick("thumbstick", in.thumbstickX, in.thumbstickY, in.thumbstickTouch);
        Stick("trackpad", in.trackpadX, in.trackpadY, in.trackpadTouch);
        ImGui::Text("fingers");
        Bar("index", in.fingerIndex);
        Bar("middle", in.fingerMiddle);
        Bar("ring", in.fingerRing);
        Bar("pinky", in.fingerPinky);
      } else if (i.role != lh::DeviceRole::Controller) {
        ImGui::TextDisabled("no inputs decoded for this device");
      }
      ImGui::End();
    }

    // Base stations.
    ImGui::SetNextWindowPos(ImVec2(10, 350), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(560, 150), ImGuiCond_FirstUseEver);
    ImGui::Begin("Base stations");
    if (ImGui::BeginTable("bases", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders)) {
      ImGui::TableSetupColumn("base"); ImGui::TableSetupColumn("seen"); ImGui::TableSetupColumn("located");
      ImGui::TableSetupColumn("position"); ImGui::TableSetupColumn("sigma");
      ImGui::TableHeadersRow();
      for (const auto& b : snap.bases) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn(); ImGui::Text("%s%s", b.serial.c_str(), b.origin ? " *" : "");
        ImGui::TableNextColumn();
        if (b.seen) ImGui::Text("yes (channel %s)", b.channel.empty() ? "?" : b.channel.c_str()); else ImGui::Text("no");
        ImGui::TableNextColumn(); ImGui::Text("%s", b.known ? "yes" : "no");
        ImGui::TableNextColumn();
        if (b.known) ImGui::Text("%+.2f %+.2f %+.2f", b.worldFromBase.position[0], b.worldFromBase.position[1], b.worldFromBase.position[2]);
        ImGui::TableNextColumn();
        if (b.known) ImGui::Text("%.1f mm", std::sqrt(b.positionVariance / 3) * 1000);
      }
      ImGui::EndTable();
    }
    ImGui::End();

    // Receivers: Watchman dongles and each Doorman Dongle slot, alike.
    if (options.replayPath.empty() && glfwGetTime() - lastReceiverPoll >= 0.5) {
      lastReceiverPoll = glfwGetTime();
      receivers = session->Receivers();
      std::stable_sort(receivers.begin(), receivers.end(), [](const auto& a, const auto& b) {
        return a.kind != b.kind ? a.kind == "openhouse" : a.serial < b.serial;
      });
      for (const auto& r : receivers) {
        auto& h = receiverHistory[r.serial].kbps;
        h.push_back(float(r.kBytesPerSec));
        while (h.size() > 60) h.pop_front();
      }
    }
    ImGui::SetNextWindowPos(ImVec2(float(fw) - 640, 580), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(630, float(fh) - 590), ImGuiCond_FirstUseEver);
    ImGui::Begin("Receivers");
    {
      int oh = 0, wm = 0, ohJoined = 0; double total = 0;
      for (const auto& r : receivers) {
        (r.kind == "openhouse" ? oh : wm)++;
        if (r.kind == "openhouse") ohJoined = r.devicesOnDongle;
        total += r.kBytesPerSec;
      }
      if (!options.replayPath.empty()) ImGui::TextDisabled("replay: no receivers");
      else if (receivers.empty()) ImGui::TextDisabled("no dongles plugged in");
      else {
        ImGui::Text("%d Watchman", wm);
        ImGui::SameLine(); ImGui::TextDisabled("|");
        ImGui::SameLine(); ImGui::Text("openhouse: %d slot%s, %d joined", oh, oh == 1 ? "" : "s", ohJoined);
        ImGui::SameLine(); ImGui::TextDisabled("|");
        ImGui::SameLine(); ImGui::Text("%.1f kB/s total", total);
      }
    }
    if (!receivers.empty() &&
        ImGui::BeginTable("receivers", 6, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingFixedFit |
                                          ImGuiTableFlags_PadOuterX)) {
      ImGui::TableSetupColumn("receiver"); ImGui::TableSetupColumn("device");
      ImGui::TableSetupColumn("throughput", ImGuiTableColumnFlags_WidthStretch);
      ImGui::TableSetupColumn("frames/s"); ImGui::TableSetupColumn("loss"); ImGui::TableSetupColumn("rssi d/t");
      ImGui::TableHeadersRow();
      for (const auto& r : receivers) {
        const bool ohd = r.kind == "openhouse";
        ImGui::TableNextRow(ImGuiTableRowFlags_None, 30);
        ImGui::TableNextColumn();
        Dot(r.linked, ohd && (r.lossPercent >= 10 || r.dropsPerSec > 0));
        ImGui::SameLine();
        Badge(ohd ? "OH" : "WM", ohd ? ImVec4(0.45f, 0.75f, 1, 1) : ImVec4(0.8f, 0.6f, 1, 1));
        ImGui::SameLine();
        if (ohd) {
          const auto cut = r.serial.find('/');
          ImGui::Text("%s", r.serial.substr(0, cut).c_str());
          ImGui::SameLine(0, 2); ImGui::TextDisabled("slot %d", r.slot);
        } else {
          ImGui::Text("%s", r.serial.c_str());
        }
        ImGui::TableNextColumn();
        if (!r.connected.empty()) ImGui::Text("%s", r.connected.c_str());
        else ImGui::TextDisabled(!r.paired ? "empty slot" : r.linked ? "identifying..." : "off");
        ImGui::TableNextColumn();
        {
          const auto& h = receiverHistory[r.serial].kbps;
          std::vector<float> v(h.begin(), h.end());
          float peak = 4;
          for (float x : v) peak = std::max(peak, x * 1.2f);
          char label[32];
          std::snprintf(label, sizeof label, "%.1f kB/s", r.kBytesPerSec);
          ImGui::PushStyleColor(ImGuiCol_PlotLines, ohd ? ImVec4(0.45f, 0.75f, 1, 1) : ImVec4(0.8f, 0.6f, 1, 1));
          ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.1f, 0.1f, 0.13f, 1));
          ImGui::PlotLines("##kbps", v.empty() ? nullptr : v.data(), int(v.size()), 0, label, 0, peak,
                           ImVec2(std::max(120.0f, ImGui::GetContentRegionAvail().x), 26));
          ImGui::PopStyleColor(2);
        }
        ImGui::TableNextColumn();
        if (r.linked) ImGui::Text("%.0f", r.framesPerSec); else ImGui::TextDisabled("-");
        ImGui::TableNextColumn();
        if (r.linked && r.lossPercent >= 0) {
          ImGui::TextColored(LossColor(r.lossPercent), "%.1f%%", r.lossPercent);
          if (r.dropsPerSec > 0) { ImGui::SameLine(); ImGui::TextColored(ImVec4(1, 0.4f, 0.35f, 1), "drop %.0f/s", r.dropsPerSec); }
        } else ImGui::TextDisabled("-");
        ImGui::TableNextColumn();
        if (r.linked && (r.rssiDongle || r.rssiDevice)) {
          ImGui::TextColored(RssiColor(r.rssiDongle), "%d", r.rssiDongle);
          ImGui::SameLine(0, 3); ImGui::TextDisabled("/");
          ImGui::SameLine(0, 3); ImGui::TextColored(RssiColor(r.rssiDevice), "%d", r.rssiDevice);
        } else ImGui::TextDisabled("-");
      }
      ImGui::EndTable();
    }
    ImGui::End();

    // Events.
    ImGui::SetNextWindowPos(ImVec2(10, float(fh) - 250), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(std::min(900.0f, float(fw) - 670), 240), ImGuiCond_FirstUseEver);
    ImGui::Begin("Events");
    ImGui::Checkbox("auto-scroll", &autoScroll);
    ImGui::SameLine();
    if (ImGui::Button("clear")) log.clear();
    ImGui::BeginChild("log");
    for (const auto& e : log) {
      ImVec4 c(0.85f, 0.85f, 0.85f, 1);
      if (e.type == lh::SessionEventType::TrackingStateChanged) c = ImVec4(0.6f, 0.8f, 1, 1);
      else if (e.type == lh::SessionEventType::BaseStationLocated || e.type == lh::SessionEventType::RoomChanged)
        c = ImVec4(1, 0.85f, 0.4f, 1);
      else if (e.text.find("Resetting") != std::string::npos || e.text.find("failed") != std::string::npos)
        c = ImVec4(1, 0.5f, 0.45f, 1);
      ImGui::TextColored(c, "%10.3f  %s", e.hostTime, e.text.c_str());
    }
    if (autoScroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 20) ImGui::SetScrollHereY(1.0f);
    ImGui::EndChild();
    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    if (!screenshotPath.empty() && glfwGetTime() > screenshotAfter) {
      std::vector<unsigned char> pixels(size_t(fw) * size_t(fh) * 3);
      glPixelStorei(GL_PACK_ALIGNMENT, 1);
      glReadPixels(0, 0, fw, fh, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
      if (FILE* f = std::fopen(screenshotPath.c_str(), "wb")) {
        std::fprintf(f, "P6\n%d %d\n255\n", fw, fh);
        for (int y = fh - 1; y >= 0; --y) std::fwrite(&pixels[size_t(y) * size_t(fw) * 3], 1, size_t(fw) * 3, f);
        std::fclose(f);
      }
      glfwSetWindowShouldClose(window, 1);
    }
    glfwSwapBuffers(window);
  }

  session->Stop();
  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
