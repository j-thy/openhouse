#include "log_view.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <imgui.h>
#include <vector>

namespace openhouse::viewer {
namespace {
ImVec4 LevelColor(lh::LogLevel level) {
  switch (level) {
    case lh::LogLevel::Debug: return {0.55f, 0.58f, 0.62f, 1};
    case lh::LogLevel::Info: return {0.86f, 0.88f, 0.90f, 1};
    case lh::LogLevel::Warn: return {1.00f, 0.78f, 0.30f, 1};
    case lh::LogLevel::Error: return {1.00f, 0.42f, 0.40f, 1};
  }
  return {1, 1, 1, 1};
}

std::string Lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return char(std::tolower(c)); });
  return s;
}
}  // namespace

std::string LogFilter::Shape(const std::string& text) {
  std::string out;
  for (size_t i = 0; i < text.size();) {
    const unsigned char c = static_cast<unsigned char>(text[i]);
    // A token of hex digits that has a digit in it (serials, numbers, times): '#'.
    size_t j = i;
    bool digit = false;
    while (j < text.size() && (std::isxdigit(static_cast<unsigned char>(text[j])) || text[j] == '.')) {
      digit |= std::isdigit(static_cast<unsigned char>(text[j])) != 0;
      ++j;
    }
    const bool wordStart = i == 0 || !std::isalnum(static_cast<unsigned char>(text[i - 1]));
    const bool wordEnd = j == text.size() || !std::isalnum(static_cast<unsigned char>(text[j]));
    if (j > i && digit && wordStart && wordEnd) { out += '#'; i = j; continue; }
    out += char(c);
    ++i;
  }
  return out;
}

bool LogFilter::Shows(const LogLine& line) const {
  switch (line.level) {
    case lh::LogLevel::Debug: if (!debug) return false; break;
    case lh::LogLevel::Info: if (!info) return false; break;
    case lh::LogLevel::Warn: if (!warn) return false; break;
    case lh::LogLevel::Error: if (!error) return false; break;
  }
  if (hideMulti && line.text.find("[multi]") != std::string::npos) return false;
  if (hideLh2 && line.text.find("[lh2]") != std::string::npos) return false;
  if (!hidden.empty() && hidden.count(Shape(line.text))) return false;
  if (!text.empty() && Lower(line.text).find(text) == std::string::npos && Lower(line.serial).find(text) == std::string::npos)
    return false;
  return true;
}

void LogView::Draw(TelemetryClient& client) {
  int counts[4] = {};
  for (const auto& l : client.Logs()) ++counts[int(l.level)];
  char label[48];
  std::snprintf(label, sizeof label, "ERR (%d)", counts[3]);
  ImGui::Checkbox(label, &filter_.error);
  ImGui::SameLine();
  std::snprintf(label, sizeof label, "WARN (%d)", counts[2]);
  ImGui::Checkbox(label, &filter_.warn);
  ImGui::SameLine();
  std::snprintf(label, sizeof label, "INFO (%d)", counts[1]);
  ImGui::Checkbox(label, &filter_.info);
  ImGui::SameLine();
  std::snprintf(label, sizeof label, "DEBUG (%d)", counts[0]);
  ImGui::Checkbox(label, &filter_.debug);
  ImGui::SameLine();
  ImGui::TextDisabled("|");
  ImGui::SameLine();
  ImGui::Checkbox("Hide multi-station chatter", &filter_.hideMulti);
  ImGui::SameLine();
  ImGui::Checkbox("Hide 2.0 light chatter", &filter_.hideLh2);

  ImGui::SetNextItemWidth(240);
  if (ImGui::InputTextWithHint("##search", "Search messages or serials", search_, sizeof search_)) filter_.text = Lower(search_);
  ImGui::SameLine();
  ImGui::Checkbox("Auto-scroll", &autoScroll_);
  ImGui::SameLine();
  if (ImGui::Checkbox("Pause", &paused_) && paused_) pausedAt_ = client.LogsReceived();
  ImGui::SameLine();
  if (ImGui::Button("Clear")) { client.ClearLogs(); pausedAt_ = client.LogsReceived(); }
  ImGui::SameLine();
  if (ImGui::Button("Copy shown")) {
    std::string all;
    for (const auto& l : client.Logs())
      if (filter_.Shows(l)) all += std::string("[") + lh::LogLevelName(l.level) + "] " + l.text + "\n";
    ImGui::SetClipboardText(all.c_str());
  }
  if (!filter_.hidden.empty()) {
    ImGui::SameLine();
    std::snprintf(label, sizeof label, "Hidden kinds (%zu)", filter_.hidden.size());
    if (ImGui::Button(label)) ImGui::OpenPopup("hidden");
    if (ImGui::BeginPopup("hidden")) {
      ImGui::TextDisabled("Click one to show it again");
      for (auto it = filter_.hidden.begin(); it != filter_.hidden.end();) {
        if (ImGui::Selectable(it->c_str())) it = filter_.hidden.erase(it);
        else ++it;
      }
      if (ImGui::Button("Show all again")) filter_.hidden.clear();
      ImGui::EndPopup();
    }
  }

  ImGui::BeginChild("lines", ImVec2(0, 0), ImGuiChildFlags_Borders, ImGuiWindowFlags_HorizontalScrollbar);
  const auto& logs = client.Logs();
  // Paused: the lines received up to then that are still stored. An index would slide to newer
  // lines once the log is full and drops its oldest.
  size_t end = logs.size();
  if (paused_) {
    const uint64_t first = client.LogsReceived() - logs.size();  // running number of logs[0]
    end = pausedAt_ <= first ? 0 : size_t(std::min<uint64_t>(pausedAt_ - first, logs.size()));
  }
  std::vector<const LogLine*> shown;
  shown.reserve(end);
  for (size_t i = 0; i < end; ++i)
    if (filter_.Shows(logs[i])) shown.push_back(&logs[i]);
  ImGuiListClipper clipper;
  clipper.Begin(int(shown.size()));
  while (clipper.Step()) {
    for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
      const LogLine& l = *shown[size_t(i)];
      ImGui::PushID(i);
      ImGui::TextColored(LevelColor(l.level), "%-5s", lh::LogLevelName(l.level));
      ImGui::SameLine();
      ImGui::Selectable(l.text.c_str(), false, ImGuiSelectableFlags_AllowOverlap);
      if (ImGui::BeginPopupContextItem("line")) {
        if (ImGui::MenuItem("Hide messages like this")) filter_.hidden.insert(LogFilter::Shape(l.text));
        if (ImGui::MenuItem("Copy")) ImGui::SetClipboardText(l.text.c_str());
        ImGui::EndPopup();
      }
      ImGui::PopID();
    }
  }
  if (autoScroll_ && !paused_ && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 40) ImGui::SetScrollHereY(1.0f);
  ImGui::EndChild();
}

}  // namespace openhouse::viewer
