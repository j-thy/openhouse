#include "platform/platform.hpp"
#include "ui.hpp"

#include <imgui.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>

#include "common/dongle_bank.hpp"
#include "common/nordic_dfu.hpp"
#include "common/ui_theme.hpp"
#include "lh/telemetry.hpp"
#include "lh/version.hpp"
#include "update_check.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

namespace openhouse::dongle {
namespace {
using json = nlohmann::json;
namespace fs = std::filesystem;

ImFont* g_title = nullptr;
ImFont* g_huge = nullptr;

// openhouse-overlay's colours
const ImVec4 kGood{0.36f, 0.80f, 0.45f, 1}, kWarn{1.00f, 0.78f, 0.30f, 1}, kBad{1.00f, 0.42f, 0.40f, 1},
    kInfo{0.45f, 0.70f, 1.00f, 1}, kOff{0.55f, 0.58f, 0.62f, 1}, kAccent{1.00f, 0.86f, 0.55f, 1};

fs::path SettingsPath() { return lh::platform::FsPath(lh::platform::ConfigDirectory()) / "dongle.json"; }

ImU32 U32(const ImVec4& c, float alpha = 1) { return ImGui::ColorConvertFloat4ToU32(ImVec4(c.x, c.y, c.z, c.w * alpha)); }

// A status dot; `frame`: centred on a frame-height row (after AlignTextToFramePadding).
void Dot(const ImVec4& color, float radius = 5, bool frame = false) {
  const ImVec2 p = ImGui::GetCursorScreenPos();
  const float h = frame ? ImGui::GetFrameHeight() : ImGui::GetTextLineHeight();
  ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(p.x + radius, p.y + h / 2), radius, U32(color));
  ImGui::Dummy(ImVec2(radius * 2 + 4, frame ? ImGui::GetTextLineHeight() : h));
  ImGui::SameLine();
}

// A page's title: large, with its icon before it.
void Title(const char* text, lh::ui::Icon icon) {
  ImGui::PushFont(g_title);
  const float s = ImGui::GetTextLineHeight();
  const ImVec2 p = ImGui::GetCursorScreenPos();
  lh::ui::DrawIcon(ImGui::GetWindowDrawList(), icon, ImVec2(p.x + s / 2, p.y + s / 2), s * 0.85f, lh::ui::kAccent);
  ImGui::Dummy(ImVec2(s, s));
  ImGui::SameLine(0, s * 0.45f);
  ImGui::TextUnformatted(text);
  ImGui::PopFont();
  ImGui::Dummy(ImVec2(0, ImGui::GetStyle().ItemSpacing.y));
}

void Title(const char* text) { Title(text, lh::ui::Icon::Dongle); }

void Tip(const char* text) {
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled | ImGuiHoveredFlags_DelayShort)) ImGui::SetTooltip("%s", text);
}

bool BigButton(const char* label, float width = 0) { return ImGui::Button(label, ImVec2(width, ImGui::GetFrameHeight() * 1.25f)); }

// A button tinted with a colour: the main action, or a dangerous one.
bool TintButton(const char* label, const ImVec4& c, float width = 0) {
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(c.x * 0.45f, c.y * 0.45f, c.z * 0.45f, 1));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(c.x * 0.6f, c.y * 0.6f, c.z * 0.6f, 1));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(c.x * 0.75f, c.y * 0.75f, c.z * 0.75f, 1));
  const bool r = BigButton(label, width);
  ImGui::PopStyleColor(3);
  return r;
}

bool Confirm(const char* id, const char* text) {
  bool yes = false;
  if (ImGui::BeginPopupModal(id, nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28);
    ImGui::TextWrapped("%s", text);
    ImGui::PopTextWrapPos();
    ImGui::Spacing();
    const float w = ImGui::GetFontSize() * 7, h = ImGui::GetFrameHeight() * 1.5f;
    if (ImGui::Button("Yes", ImVec2(w, h))) { yes = true; ImGui::CloseCurrentPopup(); }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(w, h))) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
  }
  return yes;
}

// A summary tile: a big value and what it is.
void Tile(const char* id, const char* value, const char* label, const ImVec4& color, float width) {
  const ImGuiStyle& st = ImGui::GetStyle();
  ImGui::PushFont(g_huge);
  const float big = ImGui::GetTextLineHeight();
  ImGui::PopFont();
  const float h = big + ImGui::GetTextLineHeight() + st.ItemSpacing.y + st.WindowPadding.y * 2 + 2;
  ImGui::BeginChild(id, ImVec2(width, h), ImGuiChildFlags_Borders, ImGuiWindowFlags_NoScrollbar);
  ImGui::PushFont(g_huge);
  ImGui::TextColored(color, "%s", value);
  ImGui::PopFont();
  ImGui::TextDisabled("%s", label);
  ImGui::EndChild();
}

ImVec4 LossColor(double loss) {
  if (loss < 0) return kOff;
  return loss < 3 ? kGood : loss < 10 ? kWarn : kBad;
}

ImVec4 RssiColor(int rssi) {
  if (!rssi) return kOff;
  return rssi > -65 ? kGood : rssi > -78 ? kWarn : kBad;
}

const char* SignalWord(int rssi) {
  if (!rssi) return "-";
  return rssi > -55 ? "excellent" : rssi > -65 ? "good" : rssi > -78 ? "fair" : "weak";
}

// Signal bars (4), coloured by strength.
void SignalBars(int rssi) {
  const int bars = !rssi ? 0 : rssi > -55 ? 4 : rssi > -65 ? 3 : rssi > -78 ? 2 : 1;
  ImDrawList* dl = ImGui::GetWindowDrawList();
  ImVec2 p = ImGui::GetCursorScreenPos();
  const float h = ImGui::GetTextLineHeight(), w = h * 0.22f, gap = h * 0.12f;
  p.y += h * 0.3f;    // sit on the text's baseline, not the line's top
  for (int i = 0; i < 4; ++i) {
    const float bh = h * (0.35f + 0.65f * (i + 1) / 4);
    const ImVec2 a(p.x + i * (w + gap), p.y + h - bh), b(a.x + w, p.y + h);
    dl->AddRectFilled(a, b, i < bars ? U32(RssiColor(rssi)) : U32(kOff, 0.3f), 1);
  }
  ImGui::Dummy(ImVec2(4 * (w + gap), h));
}

// A small graph with its current value written on it.
void Graph(const char* id, const std::deque<float>& h, float lo, float hi, const ImVec4& color, const char* overlay,
           float width, float height) {
  std::vector<float> v(h.begin(), h.end());
  if (v.empty()) v.push_back(lo);
  ImGui::PushStyleColor(ImGuiCol_PlotLines, color);
  ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.06f, 0.07f, 0.09f, 1));
  ImGui::PlotLines(id, v.data(), int(v.size()), 0, overlay, lo, hi, ImVec2(width, height));
  ImGui::PopStyleColor(2);
}

std::string Uptime(uint32_t ms) {
  const uint32_t s = ms / 1000;
  char t[48];
  if (s >= 3600) std::snprintf(t, sizeof t, "%uh %02um", s / 3600, s / 60 % 60);
  else std::snprintf(t, sizeof t, "%um %02us", s / 60, s % 60);
  return t;
}

// Folders of mounted UF2 drives (the dongle's update mode shows up as one).
std::vector<fs::path> Uf2Drives() {
  std::vector<fs::path> out;
  std::error_code ec;
#ifdef _WIN32
  const DWORD mask = GetLogicalDrives();
  for (int i = 0; i < 26; ++i)
    if (mask & (1u << i)) {
      const fs::path root = std::string(1, char('A' + i)) + ":\\";
      if (fs::exists(root / "INFO_UF2.TXT", ec)) out.push_back(root);
    }
#else
  const char* user = std::getenv("USER");
  std::vector<fs::path> bases = {"/media", "/mnt"};
  if (user) { bases.push_back(fs::path("/run/media") / user); bases.push_back(fs::path("/media") / user); }
  for (const auto& base : bases) {
    if (!fs::is_directory(base, ec)) continue;
    for (const auto& e : fs::directory_iterator(base, ec))
      if (fs::exists(e.path() / "INFO_UF2.TXT", ec)) out.push_back(e.path());
  }
#endif
  return out;
}
}  // namespace

// ---------------------------------------------------------------------------------------------
Ui::Ui() : driver_(lh::telemetry::kDefaultPort) {
  std::ifstream in(SettingsPath());
  const json j = json::parse(in, nullptr, false);
  // Kept in a variable: items() refers to the object it came from.
  const json devices = j.is_object() ? j.value("devices", json::object()) : json::object();
  if (devices.is_object())
    for (const auto& [id, v] : devices.items())
      known_[id] = {v.value("serial", ""), v.value("model", ""), v.value("nickname", "")};
  if (const char* page = std::getenv("OPENHOUSE_DONGLE_PAGE")) {   // testing, with the snapshot
    const std::string p = page;
    page_ = p == "slots" ? Page::Slots : p == "radio" ? Page::Radio : p == "channels" ? Page::Channels
          : p == "firmware" ? Page::Firmware : p == "about" ? Page::About : Page::Overview;
  }
}

Ui::~Ui() {
  if (updater_.joinable()) updater_.join();
}

void Ui::SaveSettings() {
  json devices = json::object();
  for (const auto& [id, k] : known_) devices[id] = {{"serial", k.serial}, {"model", k.model}, {"nickname", k.nickname}};
  std::error_code ec;
  fs::create_directories(SettingsPath().parent_path(), ec);
  std::ofstream(SettingsPath()) << json{{"devices", devices}}.dump(2) << "\n";
  knownDirty_ = false;
}

void Ui::SetUpImGui(int k) {
  ImGuiIO& io = ImGui::GetIO();
  io.IniFilename = nullptr;
  io.Fonts->Clear();
  ImFontConfig font;
  font.OversampleH = font.OversampleV = 1;
  font.PixelSnapH = true;
  font.SizePixels = 13.f * k;
  io.Fonts->AddFontDefault(&font);
  font.SizePixels = 13.f * (k + 1);
  g_title = io.Fonts->AddFontDefault(&font);
  font.SizePixels = 13.f * (k + 2);
  g_huge = io.Fonts->AddFontDefault(&font);
  ImGui::StyleColorsDark();
  ImGuiStyle& style = ImGui::GetStyle();
  style.WindowRounding = 0;
  style.FramePadding = ImVec2(8, 6);
  style.ItemSpacing = ImVec2(10, 8);
  style.WindowPadding = ImVec2(12, 12);
  style.ScaleAllSizes(float(k));
  lh::ui::ApplyTheme(float(k));
  style.Colors[ImGuiCol_ChildBg] = lh::ui::CardColor();   // bordered panels are cards over the snow
}

std::shared_ptr<Dongle> Ui::Current() {
  const auto& all = manager_.Dongles();
  for (const auto& d : all)
    if (d->Serial() == dongle_) return d;
  if (all.empty()) return nullptr;
  dongle_ = all.front()->Serial();
  return all.front();
}

void Ui::Toast(const std::string& text, bool error) {
  toast_ = text;
  toastError_ = error;
  toastUntil_ = now_ + 6;
}

Dongle::Done Ui::Report(const std::string& what) {
  return [this, what](bool ok, const std::string& message) {
    std::lock_guard lock(resultsMutex_);
    results_.emplace_back(ok ? message : what + " failed: " + message, ok);
  };
}

std::string Ui::DeviceLabel(const SlotState& s) const {
  if (!s.nickname.empty()) return s.nickname;
  if (!s.id.model.empty() && !s.id.serial.empty()) return s.id.model + "  " + s.id.serial;
  if (!s.id.serial.empty()) return s.id.serial;
  return s.used ? "Unknown device" : "Empty";
}

// What we learn about a slot's device is remembered by its pairing id, and given back to
// every dongle (a device keeps its id when paired elsewhere).
void Ui::Remember(const DongleState& d) {
  for (const auto& s : d.slots) {
    if (s.pairId.empty() || s.id.serial.empty()) continue;
    auto& k = known_[s.pairId];
    if (k.serial != s.id.serial || (!s.id.model.empty() && k.model != s.id.model)) {
      k.serial = s.id.serial;
      if (!s.id.model.empty()) k.model = s.id.model;
      knownDirty_ = true;
    }
  }
}

void Ui::Poll(double now) {
  now_ = now;
  manager_.Poll(Now());
  driver_.Poll(now);
  {
    std::lock_guard lock(resultsMutex_);
    for (auto& [text, ok] : results_) Toast(text, !ok);
    results_.clear();
  }
  // Names: the driver's (it knows which device is on which slot), then what we remember.
  std::map<std::string, std::string> driverModel;
  if (driver_.Connected())
    for (const auto& dev : driver_.Current().devices) driverModel[dev.serial] = dev.model;
  for (const auto& d : manager_.Dongles()) {
    d->SetAutoIdentify(!driver_.Connected());
    const DongleState st = d->Snapshot();
    Remember(st);
    for (int i = 0; i < st.slotCount; ++i) {
      const auto& s = st.slots[i];
      if (s.pairId.empty()) continue;
      Identity id;
      if (driver_.Connected())
        for (const auto& r : driver_.Current().receivers)
          if (r.serial == st.serial + "/" + std::to_string(i) && !r.connected.empty()) {
            id = {r.connected, driverModel.count(r.connected) ? driverModel[r.connected] : "", "driver"};
          }
      const auto k = known_.find(s.pairId);
      if (id.serial.empty() && k != known_.end() && !k->second.serial.empty())
        id = {k->second.serial, k->second.model, "remembered"};
      if (!id.serial.empty() && id.serial != s.id.serial) d->SetKnown(s.pairId, id);
      if (!st.names) {   // older firmware: the name lives in our settings only
        if (k != known_.end() && k->second.nickname != s.nickname) d->SetNickname(i, k->second.nickname);
      } else if (s.nameRead && s.nickname.empty() && k != known_.end() && !k->second.nickname.empty() &&
                 movedNames_.insert(st.serial + "/" + s.pairId).second) {
        d->SetName(i, k->second.nickname, nullptr);   // a name from before the dongle kept them: moved onto it
      }
    }
  }
  if (knownDirty_ && now - lastSave_ > 2.0) {  // a file write: not every frame
    SaveSettings();
    lastSave_ = now;
  }
}

// ---- layout ----

void Ui::Draw(float width, float height) {
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(ImVec2(width, height));
  ImGui::Begin("Doorman Dongle", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                                ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollbar);
  lh::ui::Snow(ImGui::GetWindowDrawList(), ImVec2(0, 0), ImVec2(width, height));
  const float statusHeight = ImGui::GetFrameHeightWithSpacing() + 4;
  const float body = ImGui::GetContentRegionAvail().y - statusHeight;
  Sidebar(body);
  ImGui::SameLine();
  ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0, 0, 0, 0));
  ImGui::BeginChild("page", ImVec2(0, body), ImGuiChildFlags_None);
  ImGui::PopStyleColor();
  const auto dongle = Current();
  if (page_ == Page::About) About();
  else if (!dongle) NoDongle();
  else {
    const DongleState d = dongle->Snapshot();
    if (!d.open && !d.error.empty()) {
      Title(d.serial.c_str());
      ImGui::TextColored(kBad, "Can't use this dongle: %s", d.error.c_str());
    } else {
      switch (page_) {
        case Page::Overview: Overview(d); break;
        case Page::Slots: Slots(d); break;
        case Page::Radio: Radio(d); break;
        case Page::Channels: Channels(d); break;
        case Page::Firmware: Firmware(d); break;
        case Page::About: break;
      }
    }
  }
  ImGui::EndChild();
  StatusBar();
  ImGui::End();
}

void Ui::Sidebar(float height) {
  ImGui::PushFont(g_title);
  const float name = ImGui::CalcTextSize("openhouse").x;
  ImGui::PopFont();
  const float logoSide = ImGui::GetFontSize() * 2.6f;
  const float w = std::max(ImGui::GetFontSize() * 11, logoSide + name + ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().WindowPadding.x * 2 + 4);
  ImGui::BeginChild("sidebar", ImVec2(w, height), ImGuiChildFlags_Borders, ImGuiWindowFlags_NoScrollbar);
  if (logo_) {
    ImGui::Image(static_cast<ImTextureID>(logo_), ImVec2(logoSide, logoSide));
    ImGui::SameLine();
  }
  ImGui::BeginGroup();
  ImGui::PushFont(g_title);
  ImGui::TextUnformatted("openhouse");
  ImGui::PopFont();
  ImGui::TextDisabled("dongle manager");
  ImGui::EndGroup();
  ImGui::Spacing();
  // which dongle (more than one can be plugged in)
  const auto& all = manager_.Dongles();
  if (all.size() > 1) {
    ImGui::SetNextItemWidth(-1);
    if (ImGui::BeginCombo("##dongle", dongle_.c_str())) {
      for (const auto& d : all)
        if (ImGui::Selectable(d->Serial().c_str(), d->Serial() == dongle_)) dongle_ = d->Serial();
      ImGui::EndCombo();
    }
  } else if (all.size() == 1) {
    ImGui::TextDisabled("%s", all.front()->Serial().c_str());
  }
  ImGui::Separator();
  ImGui::Spacing();
  using lh::ui::Icon;
  struct Item { Page page; const char* label; Icon icon; };
  static const Item items[] = {{Page::Overview, "Overview", Icon::Overview}, {Page::Slots, "Slots", Icon::Slots},
                               {Page::Radio, "Radio", Icon::Wireless},       {Page::Channels, "Channels", Icon::Channels},
                               {Page::Firmware, "Firmware", Icon::Chip},     {Page::About, "About", Icon::About}};
  const float rowHeight = ImGui::GetFrameHeight() * 1.3f;
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(ImGui::GetStyle().ItemSpacing.x, 4 * lh::ui::Scale()));
  for (const auto& item : items)
    if (lh::ui::NavItem(item.label, item.icon, page_ == item.page, rowHeight)) page_ = item.page;
  ImGui::PopStyleVar();
  // the dongle and the driver, at the bottom
  const float line = ImGui::GetTextLineHeightWithSpacing();
  ImGui::SetCursorPosY(std::max(ImGui::GetCursorPosY(), ImGui::GetWindowHeight() - line * 3.4f));
  ImGui::Separator();
  const auto d = Current();
  const bool open = d && d->Snapshot().open;
  Dot(open ? kGood : kBad);
  ImGui::TextUnformatted(open ? "Dongle connected" : "No dongle");
  Dot(driver_.Connected() ? kGood : kOff);
  ImGui::TextUnformatted(driver_.Connected() ? "Driver running" : "Driver not running");
  ImGui::EndChild();
}

void Ui::StatusBar() {
  ImGui::Separator();
  ImGui::AlignTextToFramePadding();
  if (!toast_.empty() && now_ < toastUntil_) ImGui::TextColored(toastError_ ? kBad : kGood, "%s", toast_.c_str());
  else ImGui::TextDisabled("openhouse %s", lh::kVersion);
}

void Ui::NoDongle() {
  Title("No Doorman Dongle");
  ImGui::TextWrapped("Plug in a Doorman Dongle");
#ifndef _WIN32
  ImGui::Spacing();
  ImGui::TextDisabled("Needs the openhouse udev rule (see the README)");
#endif
}

// The pairing banner: a countdown while the dongle is pairing, else the button.
void Ui::PairBar(const DongleState& d) {
  const auto dongle = Current();
  if (d.pairing) {
    const double left = d.pairUntil > 0 ? std::max(0.0, d.pairUntil - Now()) : -1;
    char text[128];
    std::snprintf(text, sizeof text, "Pairing into %s%s",
                  d.pairTarget >= 0 ? ("slot " + std::to_string(d.pairTarget)).c_str() : "the first free slot",
                  left >= 0 ? ("  -  " + std::to_string(int(left)) + " s left").c_str() : "");
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.18f, 0.28f, 1));
    // One row: the title and the button, centred on each other; the height fits it at any scale.
    ImGui::PushFont(g_title);
    const float titleH = ImGui::GetTextLineHeight();
    ImGui::PopFont();
    const float rowH = std::max(titleH, ImGui::GetFrameHeight());
    const float padY = ImGui::GetStyle().WindowPadding.y;
    ImGui::BeginChild("pairing", ImVec2(0, rowH + 2 * padY), ImGuiChildFlags_Borders,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    const float y0 = ImGui::GetCursorPosY();
    ImGui::SetCursorPosY(y0 + (rowH - titleH) * 0.5f);
    ImGui::PushFont(g_title);   // first: the dot centres on the title's line
    Dot(kInfo, 6);
    ImGui::TextColored(kInfo, "%s", text);
    ImGui::PopFont();
    const float bw = ImGui::GetFontSize() * 9;
    ImGui::SameLine(ImGui::GetContentRegionMax().x - bw);
    ImGui::SetCursorPosY(y0 + (rowH - ImGui::GetFrameHeight()) * 0.5f);
    if (ImGui::Button("Stop pairing", ImVec2(bw, 0))) dongle->StopPairing(Report("Stop pairing"));
    ImGui::EndChild();
    ImGui::PopStyleColor();
    return;
  }
  int firstFree = -1;
  for (int i = 0; i < d.slotCount && firstFree < 0; ++i)
    if (!d.slots[i].used) firstFree = i;
  if (firstFree >= 0) {
    if (TintButton("Pair a device", kInfo, ImGui::GetFontSize() * 12)) dongle->Pair(60, -1, Report("Pairing"));
    Tip("60 s of pairing into the first free slot");
  } else {
    ImGui::BeginDisabled();
    BigButton("All slots paired", ImGui::GetFontSize() * 12);
    ImGui::EndDisabled();
    Tip("Pairing from a slot's card replaces its device");
  }
}

// ---- pages ----

void Ui::Overview(const DongleState& d) {
  Title("Overview", lh::ui::Icon::Overview);
  PairBar(d);
  ImGui::Spacing();
  const float gap = ImGui::GetStyle().ItemSpacing.x;
  const float tw = (ImGui::GetContentRegionAvail().x - gap * 3) / 4;
  double loss = 0, corrupted = 0, unanswered = 0;
  int n = 0, worstRssi = 0;
  for (const auto& s : d.slots)
    if (s.active) {
      if (s.lossPercent >= 0) { loss += s.lossPercent; n++; }
      if (s.rssiDongle && (!worstRssi || s.rssiDongle < worstRssi)) worstRssi = s.rssiDongle;
      corrupted += s.crcPerSec; unanswered += s.timeoutsPerSec;
    }
  char v[32];
  std::snprintf(v, sizeof v, "%d / %d", d.joined, d.slotCount);
  Tile("t1", v, "devices connected", d.joined ? kGood : kOff, tw);
  ImGui::SameLine();
  std::snprintf(v, sizeof v, "%d", d.paired);
  Tile("t2", v, "slots paired", kInfo, tw);
  ImGui::SameLine();
  std::snprintf(v, sizeof v, "%.1f kB/s", d.kBytesPerSec);
  Tile("t3", v, "data from devices", kAccent, tw);
  ImGui::SameLine();
  if (n) std::snprintf(v, sizeof v, "%.1f %%", loss / n); else std::snprintf(v, sizeof v, "-");
  Tile("t4", v, "radio loss (resent)", n ? LossColor(loss / n) : kOff, tw);
  // The link full rather than the radio bad: a lot of data, loss everywhere, and answers missing rather
  // than corrupted. The channels can't fix that; fewer devices on this dongle can.
  if (d.haveTiming) {
    ImGui::Spacing();
    char line[160];
    int at = std::snprintf(line, sizeof line, "Answers %d B", d.answerBytes);
    if (d.largeAnswers && d.normalAnswerBytes) at += std::snprintf(line + at, sizeof line - at, " (large, normally %d)", d.normalAnswerBytes);
    at += std::snprintf(line + at, sizeof line - at, "  |  cycle %.1f ms", d.cycleUs / 1000.0);
    if (d.saturationPercent >= 0) std::snprintf(line + at, sizeof line - at, "  |  saturation %d %%", d.saturationPercent);
    ImGui::TextDisabled("%s", line);
  }
  // Saturated even with large answers (or a dongle that can't tell), loss everywhere, answers missing rather
  // than corrupted: the link is full. The channels can't fix that; fewer devices on this dongle can.
  const bool full = d.saturationPercent >= 0 ? d.largeAnswers && d.saturationPercent >= 50
                                             : d.kBytesPerSec >= 110;
  if (n && full && loss / n >= 20 && corrupted < 0.1 * unanswered) {
    ImGui::Spacing();
    ImGui::TextColored(kWarn, "Link full: move some devices to a second dongle");
  }

  ImGui::Spacing();
  ImGui::SeparatorText("Slots");
  if (ImGui::BeginTable("slots", 6, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp)) {
    ImGui::TableSetupColumn("Slot", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 4);
    ImGui::TableSetupColumn("Device");
    ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 7);
    ImGui::TableSetupColumn("Signal", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 8);
    ImGui::TableSetupColumn("Loss", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 5);
    ImGui::TableSetupColumn("Data", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 6);
    ImGui::TableHeadersRow();
    for (int i = 0; i < d.slotCount; ++i) {
      const auto& s = d.slots[i];
      ImGui::TableNextRow(ImGuiTableRowFlags_None, ImGui::GetFrameHeight());
      ImGui::TableNextColumn();
      ImGui::AlignTextToFramePadding();
      ImGui::Text("%d", i);
      ImGui::TableNextColumn();
      ImGui::AlignTextToFramePadding();
      if (s.used) ImGui::TextUnformatted(DeviceLabel(s).c_str()); else ImGui::TextDisabled("Empty");
      ImGui::TableNextColumn();
      ImGui::AlignTextToFramePadding();
      if (!s.used) ImGui::TextDisabled("-");
      else { Dot(s.active ? kGood : kOff, 5, true); ImGui::TextColored(s.active ? kGood : kOff, s.active ? "Connected" : "Off"); }
      ImGui::TableNextColumn();
      ImGui::AlignTextToFramePadding();
      if (s.active) { SignalBars(s.rssiDongle); ImGui::SameLine(); ImGui::TextColored(RssiColor(s.rssiDongle), "%d dBm", s.rssiDongle); }
      else ImGui::TextDisabled("-");
      ImGui::TableNextColumn();
      ImGui::AlignTextToFramePadding();
      if (s.active && s.lossPercent >= 0) ImGui::TextColored(LossColor(s.lossPercent), "%.1f %%", s.lossPercent); else ImGui::TextDisabled("-");
      ImGui::TableNextColumn();
      ImGui::AlignTextToFramePadding();
      if (s.active) ImGui::Text("%.1f kB/s", s.kBytesPerSec); else ImGui::TextDisabled("-");
    }
    ImGui::EndTable();
  }
  ImGui::Spacing();
  int on = 0;
  for (int k = 0; k < kHops; ++k) on += (d.channelMask >> k) & 1;
  ImGui::TextDisabled("Radio on %d of %d channels, now at %d MHz  |  firmware %d  |  up %s", on, kHops, 2400 + d.currentChannel,
                      d.firmware, Uptime(d.uptimeMs).c_str());
}

void Ui::SlotCard(const DongleState& d, int i, float width) {
  const auto dongle = Current();
  const auto& s = d.slots[i];
  const ImVec4 edge = !s.used ? kOff : s.active ? kGood : kWarn;
  ImGui::PushID(i);
  ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(edge.x, edge.y, edge.z, s.used ? 0.7f : 0.25f));
  // header (title font) + separator + four lines + the buttons
  ImGui::PushFont(g_title);
  const float titleH = ImGui::GetTextLineHeight();
  ImGui::PopFont();
  const ImGuiStyle& st = ImGui::GetStyle();
  const float cardH = st.WindowPadding.y * 2 + titleH + st.ItemSpacing.y * 7 + ImGui::GetTextLineHeight() * 4 +
                      ImGui::GetFrameHeight() * 1.25f + 2;
  ImGui::BeginChild("card", ImVec2(width, cardH), ImGuiChildFlags_Borders, ImGuiWindowFlags_NoScrollbar);
  ImGui::PopStyleColor();
  // header: slot number and state
  ImGui::PushFont(g_title);
  ImGui::Text("Slot %d", i);
  ImGui::PopFont();
  ImGui::SameLine();
  ImGui::SetCursorPosX(ImGui::GetContentRegionMax().x - ImGui::GetFontSize() * 7);
  Dot(edge);
  ImGui::TextColored(edge, "%s", !s.used ? "Empty" : s.active ? "Connected" : "Off");
  ImGui::Separator();
  if (!s.used) {
    ImGui::TextDisabled("Nothing paired");
    ImGui::Spacing();
    ImGui::BeginDisabled(d.pairing);
    if (TintButton("Pair to this slot", kInfo, -1)) dongle->Pair(60, i, Report("Pairing"));
    ImGui::EndDisabled();
    ImGui::EndChild();
    ImGui::PopID();
    return;
  }
  ImGui::TextUnformatted(DeviceLabel(s).c_str());
  if (!s.nickname.empty() && !s.id.serial.empty()) ImGui::TextDisabled("%s %s", s.id.model.c_str(), s.id.serial.c_str());
  else ImGui::TextDisabled("radio id %s%s", s.pairId.c_str(), s.id.source.empty() ? "" : ("  (" + s.id.source + ")").c_str());
  if (s.active) {
    SignalBars(s.rssiDongle);
    ImGui::SameLine();
    ImGui::TextColored(RssiColor(s.rssiDongle), "%d dBm %s", s.rssiDongle, SignalWord(s.rssiDongle));
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    ImGui::TextColored(LossColor(s.lossPercent), "%.1f %% loss", std::max(0.0, s.lossPercent));
    ImGui::Text("%.1f kB/s   IMU %.0f Hz   light %.0f/s", s.kBytesPerSec, s.imuHz, s.lightPerSec);
  } else {
    ImGui::TextDisabled("Turned off or out of range");
    ImGui::TextDisabled(" ");
  }
  // actions
  ImGui::SetCursorPosY(ImGui::GetWindowHeight() - ImGui::GetFrameHeight() * 1.25f - ImGui::GetStyle().WindowPadding.y);
  const float bw = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x * 2) / 3;
  ImGui::BeginDisabled(d.pairing);
  if (BigButton("Replace", bw)) { pendingSlot_ = i; ImGui::OpenPopup("Replace?"); }
  ImGui::EndDisabled();
  Tip("Replaces the device in this slot");
  ImGui::SameLine();
  if (TintButton("Unpair", kBad, bw)) { pendingSlot_ = i; ImGui::OpenPopup("Unpair?"); }
  
  ImGui::SameLine();
  if (BigButton("More...", bw)) {
    pendingSlot_ = i;
    std::snprintf(rename_, sizeof rename_, "%s", s.nickname.c_str());
    ImGui::OpenPopup("slot menu");
  }
  if (ImGui::BeginPopup("slot menu")) {
    ImGui::BeginDisabled(!s.active);
    if (ImGui::MenuItem("Turn off")) dongle->PowerOff(i, Report("Turn off"));
    if (ImGui::MenuItem("Identify (read its name)")) dongle->Identify(i, Report("Identify"));
    ImGui::EndDisabled();
    ImGui::Separator();
    ImGui::TextDisabled("Name");
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 14);
    if (ImGui::InputText("##name", rename_, sizeof rename_, ImGuiInputTextFlags_EnterReturnsTrue)) {
      known_[s.pairId].nickname = rename_;
      knownDirty_ = true;
      if (dongle->Snapshot().names) dongle->SetName(i, rename_, Report("Rename"));
      else dongle->SetNickname(i, rename_);
      ImGui::CloseCurrentPopup();
    }
    ImGui::TextDisabled("Enter to save");
    ImGui::EndPopup();
  }
  char text[160];
  std::snprintf(text, sizeof text, "Pair a new device into slot %d? %s will be forgotten.", i, DeviceLabel(s).c_str());
  if (Confirm("Replace?", text) && pendingSlot_ == i) dongle->Pair(60, i, Report("Pairing"));
  std::snprintf(text, sizeof text, "Unpair %s from slot %d? It can't connect until it's paired again.", DeviceLabel(s).c_str(), i);
  if (Confirm("Unpair?", text) && pendingSlot_ == i) dongle->Forget(i, Report("Unpair"));
  ImGui::EndChild();
  ImGui::PopID();
}

void Ui::Slots(const DongleState& d) {
  Title("Slots", lh::ui::Icon::Slots);
  PairBar(d);
  ImGui::SameLine();
  ImGui::SetCursorPosX(ImGui::GetContentRegionMax().x - ImGui::GetFontSize() * 10);
  if (!d.pairing) {
    ImGui::BeginDisabled(d.paired == 0);
    if (TintButton("Unpair all", kBad, ImGui::GetFontSize() * 10)) ImGui::OpenPopup("Unpair all?");
    ImGui::EndDisabled();
  }
  if (Confirm("Unpair all?", "Every device will need to be paired again"))
    Current()->Forget(-1, Report("Unpair all"));
  ImGui::Spacing();
  const float gap = ImGui::GetStyle().ItemSpacing.x;
  const int columns = ImGui::GetContentRegionAvail().x > ImGui::GetFontSize() * 60 ? 3 : 2;
  const float w = (ImGui::GetContentRegionAvail().x - gap * (columns - 1)) / columns;
  for (int i = 0; i < d.slotCount; ++i) {
    if (i % columns) ImGui::SameLine();
    SlotCard(d, i, w);
  }
}

void Ui::Radio(const DongleState& d) {
  Title("Radio", lh::ui::Icon::Wireless);
  // every slot, one row each
  if (ImGui::BeginTable("radio", 9, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp)) {
    ImGui::TableSetupColumn("Slot", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 3);
    ImGui::TableSetupColumn("Device");
    for (const char* c : {"Polls/s", "Loss", "Resent/s", "Frames/s", "Data", "On air", "Signal d / t"})
      ImGui::TableSetupColumn(c, ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * (c[0] == 'S' ? 7.5f : 5.2f));
    ImGui::TableHeadersRow();
    for (int i = 0; i < d.slotCount; ++i) {
      const auto& s = d.slots[i];
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      char label[8];
      std::snprintf(label, sizeof label, "%d", i);
      if (ImGui::Selectable(label, selectedSlot_ == i, ImGuiSelectableFlags_SpanAllColumns)) selectedSlot_ = i;
      ImGui::TableNextColumn();
      if (!s.used) { ImGui::TextDisabled("Empty"); continue; }
      ImGui::TextColored(s.active ? ImGui::GetStyleColorVec4(ImGuiCol_Text) : kOff, "%s", DeviceLabel(s).c_str());
      if (!s.active) continue;
      ImGui::TableNextColumn(); ImGui::Text("%.0f", s.pollsPerSec);
      ImGui::TableNextColumn(); ImGui::TextColored(LossColor(s.lossPercent), "%.1f %%", std::max(0.0, s.lossPercent));
      ImGui::TableNextColumn(); ImGui::Text("%.0f", s.resendsPerSec + s.dupPerSec);
      ImGui::TableNextColumn(); ImGui::Text("%.0f", s.framesPerSec);
      ImGui::TableNextColumn(); ImGui::Text("%.1f kB/s", s.kBytesPerSec);
      ImGui::TableNextColumn();
      if (s.airKBytesPerSec > 0) ImGui::Text("%.1f kB/s", s.airKBytesPerSec); else ImGui::TextDisabled("-");
      ImGui::TableNextColumn();
      ImGui::TextColored(RssiColor(s.rssiDongle), "%d", s.rssiDongle);
      ImGui::SameLine(0, 4); ImGui::TextDisabled("/"); ImGui::SameLine(0, 4);
      ImGui::TextColored(RssiColor(s.rssiDevice), "%d dBm", s.rssiDevice);
    }
    ImGui::EndTable();
  }
  // the selected slot in detail: graphs over the last two minutes
  const auto& s = d.slots[selectedSlot_];
  ImGui::Spacing();
  char head[160];
  std::snprintf(head, sizeof head, "Slot %d: %s", selectedSlot_, s.used ? DeviceLabel(s).c_str() : "empty");
  ImGui::SeparatorText(head);
  if (!s.active) { ImGui::TextDisabled(s.used ? "Not connected" : "Nothing paired"); return; }
  const float gap = ImGui::GetStyle().ItemSpacing.x;
  const float gw = (ImGui::GetContentRegionAvail().x - gap * 2) / 3, gh = ImGui::GetFontSize() * 6;
  char o[64];
  std::snprintf(o, sizeof o, "loss %.1f %%", std::max(0.0, s.lossPercent));
  Graph("##loss", s.lossHistory, 0, 25, LossColor(s.lossPercent), o, gw, gh);
  ImGui::SameLine();
  std::snprintf(o, sizeof o, "signal %d dBm", s.rssiDongle);
  Graph("##rssi", s.rssiHistory, -95, -35, RssiColor(s.rssiDongle), o, gw, gh);
  ImGui::SameLine();
  float peak = 4;
  for (float x : s.kbpsHistory) peak = std::max(peak, x * 1.2f);
  std::snprintf(o, sizeof o, "%.1f kB/s", s.kBytesPerSec);
  Graph("##kbps", s.kbpsHistory, 0, peak, kInfo, o, gw, gh);
  ImGui::TextDisabled("last two minutes");
  ImGui::Spacing();
  if (ImGui::BeginTable("detail", 4, ImGuiTableFlags_SizingStretchProp)) {
    const auto row = [](const char* a, const std::string& b, const char* c, const std::string& e) {
      ImGui::TableNextRow();
      ImGui::TableNextColumn(); ImGui::TextDisabled("%s", a);
      ImGui::TableNextColumn(); ImGui::TextUnformatted(b.c_str());
      ImGui::TableNextColumn(); ImGui::TextDisabled("%s", c);
      ImGui::TableNextColumn(); ImGui::TextUnformatted(e.c_str());
    };
    char a[64], b[64];
    std::snprintf(a, sizeof a, "%.0f Hz", s.imuHz); std::snprintf(b, sizeof b, "%.0f / s", s.lightPerSec);
    row("IMU", a, "Light frames", b);
    std::snprintf(a, sizeof a, "%.1f / s", s.crcPerSec); std::snprintf(b, sizeof b, "%.1f / s", s.timeoutsPerSec);
    row("Corrupted answers", a, "No answer", b);
    std::snprintf(a, sizeof a, "%d", s.deviceMissed); std::snprintf(b, sizeof b, "%d", s.deviceCrc);
    row("Polls the device missed", a, "Corrupted polls (device)", b);
    std::snprintf(a, sizeof a, "%.0f / s", s.dropsPerSec);
    std::snprintf(b, sizeof b, "%d .. %d dBm", s.rssiMin, s.rssiMax);
    row("Dropped on the dongle", a, "Signal range", b);
    if (s.saturationPercent >= 0) {
      std::snprintf(a, sizeof a, "%d %%", s.saturationPercent);
      row("Saturation", a, "", "");
    }
    std::snprintf(a, sizeof a, "%.0f / s", s.deviceDropsPerSec); std::snprintf(b, sizeof b, "%.1f / s", s.deviceReplyDropsPerSec);
    row("Dropped on the device", a, "Replies dropped", b);
    if (s.airKBytesPerSec > 0 && s.kBytesPerSec > 0) {
      std::snprintf(a, sizeof a, "%.0f %% smaller", 100.0 * (1 - s.airKBytesPerSec / s.kBytesPerSec));
      std::snprintf(b, sizeof b, "%llu", static_cast<unsigned long long>(s.frames));
      row("Compression", a, "Frames since start", b);
    }
    ImGui::EndTable();
  }
}

void Ui::Channels(const DongleState& d) {
  Title("Channels", lh::ui::Icon::Channels);
  ImGui::TextWrapped("Loss per channel. Grey ones are skipped for now");
  ImGui::Spacing();
  // hop channels: loss bars
  {
    const float w = ImGui::GetContentRegionAvail().x, h = ImGui::GetFontSize() * 9;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), U32(ImVec4(0.06f, 0.07f, 0.09f, 1)), 4);
    const float bw = w / kHops, label = ImGui::GetTextLineHeight() * 2.2f, top = ImGui::GetTextLineHeight();
    for (int k = 0; k < kHops; ++k) {
      const bool on = (d.channelMask >> k) & 1;
      const bool idle = d.joined == 0;   // nothing connected: no traffic, so no loss to measure
      const float loss = idle ? 0.f : d.hopLoss[k];
      const float frac = std::min(1.f, loss / 30.f);
      const float x0 = p.x + k * bw + bw * 0.18f, x1 = p.x + (k + 1) * bw - bw * 0.18f;
      const float base = p.y + h - label;
      const ImVec4 c = !on || idle ? kOff : LossColor(loss);
      dl->AddRectFilled(ImVec2(x0, base - std::max(2.f, frac * (h - label - top))), ImVec2(x1, base), U32(c, on ? 0.9f : 0.35f), 2);
      char t[16];
      std::snprintf(t, sizeof t, "%d", 2400 + d.hopChannel[k]);
      const ImVec2 ts = ImGui::CalcTextSize(t);
      dl->AddText(ImVec2((x0 + x1 - ts.x) / 2, base + 2), U32(on ? ImGui::GetStyleColorVec4(ImGuiCol_Text) : kOff), t);
      if (idle) std::snprintf(t, sizeof t, on ? "-" : "off");
      else std::snprintf(t, sizeof t, on ? "%d%%" : "off", int(loss));
      const ImVec2 ls = ImGui::CalcTextSize(t);
      dl->AddText(ImVec2((x0 + x1 - ls.x) / 2, base + 2 + ts.y), U32(kOff), t);
      if (d.hopChannel[k] == d.currentChannel && on) dl->AddRect(ImVec2(x0 - 2, p.y + 2), ImVec2(x1 + 2, base), U32(kInfo, 0.6f), 2);
    }
    ImGui::Dummy(ImVec2(w, h));
  }
  int on = 0;
  for (int k = 0; k < kHops; ++k) on += (d.channelMask >> k) & 1;
  if (d.joined == 0) ImGui::TextDisabled("%d of %d channels in use  |  no device connected: loss shows once one is", on, kHops);
  else ImGui::TextDisabled("%d of %d channels in use  |  MHz, loss over the last couple of seconds", on, kHops);
  ImGui::Spacing();
  ImGui::SeparatorText("2.4 GHz band");
  if (BigButton("Scan the band", ImGui::GetFontSize() * 12)) Current()->Scan(Report("Scan"));
  Tip("Devices pause for a moment");
  ImGui::SameLine();
  ImGui::AlignTextToFramePadding();
  if (d.scan.empty()) ImGui::TextDisabled("Not scanned yet");
  else ImGui::TextDisabled("Scanned %.0f s ago. Wi-Fi channels 1, 6 and 11 are marked", Now() - d.scanTime);
  if (!d.scan.empty()) {
    const float w = ImGui::GetContentRegionAvail().x, h = ImGui::GetFontSize() * 9;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), U32(ImVec4(0.06f, 0.07f, 0.09f, 1)), 4);
    const int n = int(d.scan.size());
    const float bw = w / n, label = ImGui::GetTextLineHeight() * 1.3f;
    const float base = p.y + h - label;
    // Wi-Fi channels 1/6/11 (22 MHz wide around 2412/2437/2462)
    for (int wifi : {12, 37, 62}) {
      const float x0 = p.x + (wifi - 11) * bw, x1 = p.x + (wifi + 11) * bw;
      dl->AddRectFilled(ImVec2(x0, p.y + 2), ImVec2(x1, base), U32(kAccent, 0.06f));
      char t[16];
      std::snprintf(t, sizeof t, "Wi-Fi %d", wifi == 12 ? 1 : wifi == 37 ? 6 : 11);
      dl->AddText(ImVec2(x0 + 4, p.y + 4), U32(kAccent, 0.6f), t);
    }
    for (int i = 0; i < n; ++i) {
      if (d.scan[i] == 0xFF) continue;
      const float dbm = -float(d.scan[i]);  // -100 (quiet) .. -40 (busy)
      const float frac = std::clamp((dbm + 100.f) / 60.f, 0.f, 1.f);
      const ImVec4 c = dbm > -60 ? kBad : dbm > -75 ? kWarn : kGood;
      dl->AddRectFilled(ImVec2(p.x + i * bw + 1, base - std::max(2.f, frac * (h - label - 4))), ImVec2(p.x + (i + 1) * bw - 1, base), U32(c, 0.85f));
    }
    for (int k = 0; k < kHops; ++k) {   // our hop channels
      const int ch = d.hopChannel[k];
      if (ch >= n) continue;
      const float x = p.x + (ch + 0.5f) * bw;
      dl->AddTriangleFilled(ImVec2(x, base + 2), ImVec2(x - 4, base + 9), ImVec2(x + 4, base + 9),
                            U32((d.channelMask >> k) & 1 ? kInfo : kOff));
    }
    for (int mhz = 0; mhz <= 80; mhz += 10) {
      char t[8];
      std::snprintf(t, sizeof t, "%d", 2400 + mhz);
      dl->AddText(ImVec2(p.x + mhz * bw, base + 10), U32(kOff), t);
    }
    ImGui::Dummy(ImVec2(w, h + ImGui::GetTextLineHeight()));
    ImGui::TextDisabled("Blue: channels in use");
  }
}

void Ui::Firmware(const DongleState& d) {
  Title("Firmware", lh::ui::Icon::Chip);
  const auto dongle = Current();
  if (ImGui::BeginTable("fw", 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
    const auto row = [](const char* a, const std::string& b) {
      ImGui::TableNextRow();
      ImGui::TableNextColumn(); ImGui::TextDisabled("%s", a);
      ImGui::TableNextColumn(); ImGui::TextUnformatted(b.c_str());
    };
    char t[96];
    row("Dongle", d.serial);
    std::snprintf(t, sizeof t, "%d (%s build%s)", d.firmware,
                  d.board == 1 ? "Holyiot 21017" : d.board == 2 ? "PCA10059" : d.board == 3 ? (d.bank == 2 ? "Kuroshin, bank B" : "Kuroshin, bank A") : d.softdeviceVariant ? "Nordic bootloader" : "bare",
                  d.watchdog ? ", watchdog" : "");
    row("Firmware", t);
    row("Up for", Uptime(d.uptimeMs));
    std::snprintf(t, sizeof t, "0x%X", d.resetReason);
    row("Last reset", d.resetReason == 0 ? "power on" : d.resetReason & 1 ? "reset pin" : d.resetReason & 4 ? "software" : d.resetReason & 2 ? "watchdog" : t);
    std::snprintf(t, sizeof t, "%u%s", d.joinsHeard, "");
    row("Join requests heard", t);
    if (d.crashCount) {
      std::snprintf(t, sizeof t, "%u (last: type %u at 0x%08X)", d.crashCount, d.crashType, d.crashPc);
      row("Crashes", t);
    } else {
      row("Crashes", "none");
    }
    ImGui::EndTable();
  }
  ImGui::Spacing();
  if (BigButton("Restart dongle", ImGui::GetFontSize() * 12)) ImGui::OpenPopup("Restart?");
  if (d.crashCount) {
    ImGui::SameLine();
    if (BigButton("Clear crash record", ImGui::GetFontSize() * 12)) dongle->ClearCrash(Report("Clear crash record"));
  }
  if (Confirm("Restart?", "Every device drops for a moment")) dongle->Reset(Report("Restart"));

  ImGui::Spacing();
  ImGui::SeparatorText("Update");
  if (d.Banked()) {
    // Updates itself: the packaged firmware for the bank that isn't running.
    const fs::path image = PackagedBankImage(lh::dongle_bank::ImageName(d.board, d.softdeviceVariant, d.bank));
    if (image.empty()) ImGui::TextColored(kWarn, "Firmware not found next to the app");
    else ImGui::TextDisabled("%s", lh::platform::PathUtf8(image.filename()).c_str());
    ImGui::BeginDisabled(image.empty() || updating_);
    if (TintButton(updating_ ? "Updating..." : "Update", kAccent, ImGui::GetFontSize() * 12)) ImGui::OpenPopup("Update?");
    ImGui::EndDisabled();
    if (Confirm("Update?", "Every device drops for a few seconds")) StartBankUpdate(dongle, image);
  } else {
    // Older firmware: through its bootloader, from a file.
    if (d.NordicDfu())
      ImGui::TextWrapped("Pick %s", d.board == 2 ? "doorman-pca10059.hex" : "doorman-holyiot.hex");
    else
      ImGui::TextWrapped("Pick %s", d.softdeviceVariant ? "doorman-dongle-sd.uf2" : "doorman-dongle-nosd.uf2");
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - ImGui::GetFontSize() * 13);
    ImGui::InputTextWithHint("##uf2", d.NordicDfu() ? "path to the .hex file" : "path to the .uf2 file", firmwarePath_, sizeof firmwarePath_);
    ImGui::SameLine();
    const bool exists = firmwarePath_[0] && fs::exists(fs::u8path(firmwarePath_));
    ImGui::BeginDisabled(!exists || updating_);
    if (TintButton(updating_ ? "Updating..." : "Update", kAccent, -1)) ImGui::OpenPopup("Update?");
    ImGui::EndDisabled();
    if (Confirm("Update?", "Every device drops for about 10 s")) StartUpdate(dongle, firmwarePath_, d.NordicDfu());
  }
  {
    std::lock_guard lock(updateMutex_);
    if (!updateStatus_.empty()) ImGui::TextColored(updateError_ ? kBad : updating_ ? kInfo : kGood, "%s", updateStatus_.c_str());
  }
}

// The package's firmwares/dongle/bank/<name>: this app runs from the package's apps/, or its root.
fs::path Ui::PackagedBankImage(const std::string& name) {
  std::error_code ec;
#ifdef _WIN32
  wchar_t self[MAX_PATH * 2]{};
  GetModuleFileNameW(nullptr, self, DWORD(std::size(self)));
  const fs::path dir = fs::path(self).parent_path();
#else
  fs::path dir = fs::read_symlink("/proc/self/exe", ec).parent_path();
  if (ec) dir = fs::current_path();
#endif
  for (const fs::path& base : {dir, dir.parent_path()}) {
    const fs::path p = base / "firmwares" / "dongle" / "bank" / name;
    if (fs::exists(p, ec)) return p;
  }
  return {};
}

// Updates itself: the image for the bank that isn't running goes over USB (lh/dongle_bank.hpp).
void Ui::StartBankUpdate(std::shared_ptr<Dongle> dongle, const fs::path& file) {
  if (updating_) return;
  if (updater_.joinable()) updater_.join();
  const auto set = [this](const std::string& s, bool error = false) {
    std::lock_guard lock(updateMutex_);
    updateStatus_ = s;
    updateError_ = error;
  };
  std::ifstream in(file, std::ios::binary);
  std::vector<uint8_t> image((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (image.size() < 1024) { set("Can't read " + lh::platform::PathUtf8(file), true); return; }
  updating_ = true;
  set("Writing the firmware...");
  dongle->BankUpdate(std::move(image),
                     [set](float f) { char t[48]; std::snprintf(t, sizeof t, "Writing the firmware... %d%%", int(f * 100)); set(t); },
                     [this, set](bool ok, const std::string& message) { set(ok ? message : "Update failed: " + message, !ok); updating_ = false; });
}

void Ui::StartUpdate(std::shared_ptr<Dongle> dongle, const std::string& file, bool nordicDfu) {
  if (updating_) return;
  if (updater_.joinable()) updater_.join();
  updating_ = true;
  const auto set = [this](const std::string& s, bool error = false) {
    std::lock_guard lock(updateMutex_);
    updateStatus_ = s;
    updateError_ = error;
  };
  set("Putting the dongle in update mode...");
  if (nordicDfu) {
    updater_ = std::thread([this, dongle, file, set] {
      lh::dfu::Image image;
      std::string error;
      if (!lh::dfu::LoadHex(file, 0x1000, 0xDA000, &image, &error) || image.base != 0x1000) {
        set(error.empty() ? "This file isn't built for this dongle." : "Can't use this file: " + error, true);
        updating_ = false;
        return;
      }
      const auto before = lh::dfu::FindBootloaderPorts();
      dongle->EnterBootloader(nullptr);
      std::string port;
      for (const double until = Now() + 20; Now() < until && port.empty();) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        for (const auto& p : lh::dfu::FindBootloaderPorts())
          if (std::find(before.begin(), before.end(), p) == before.end()) port = p;
      }
      if (port.empty()) { set("The DFU bootloader didn't show up. Press the dongle's RESET button and try again.", true); updating_ = false; return; }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));  // let the port settle
      const bool ok = lh::dfu::Flash(port, image, [&](float f) {
        char text[64];
        std::snprintf(text, sizeof text, "Sending firmware... %d%%", int(f * 100));
        set(text);
      }, &error);
      set(ok ? "Done: the dongle restarts with the new firmware." : "Update failed: " + error, !ok);
      updating_ = false;
    });
    return;
  }
  updater_ = std::thread([this, dongle, file, set] {
    const auto before = Uf2Drives();
    dongle->EnterBootloader(nullptr);
    const double until = Now() + 20;
    fs::path drive;
    while (Now() < until && drive.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      for (const auto& d : Uf2Drives())
        if (std::find(before.begin(), before.end(), d) == before.end() || before.size() == 0) drive = d;
    }
    if (drive.empty()) {
      set("The update drive didn't show up. If it appeared but isn't mounted, mount it and copy the file onto it yourself.", true);
      updating_ = false;
      return;
    }
    set("Copying onto " + lh::platform::PathUtf8(drive) + "...");
    std::error_code ec;
    fs::copy_file(fs::u8path(file), drive / fs::u8path(file).filename(), fs::copy_options::overwrite_existing, ec);
    std::error_code gone;
    if (ec && fs::exists(drive, gone)) {   // the drive leaves as the dongle restarts: that's success
      set("Copy failed: " + ec.message(), true);
    } else {
      set("Done: the dongle restarts with the new firmware.");
    }
    updating_ = false;
  });
}

void Ui::About() {
  Title("About", lh::ui::Icon::About);
  ImGui::TextWrapped("Doorman Dongle Manager");
  ImGui::Spacing();
  ImGui::TextWrapped("Eight slots, one device each. SteamVR's pairing page works too while the openhouse driver runs");
  ImGui::Spacing();
  ImGui::TextDisabled("openhouse %s", lh::kVersion);
  if (ImGui::Button("Release page")) overlay::UpdateCheck::OpenReleasesPage();
}

}  // namespace openhouse::dongle
