// The log panel: level sections, the repeating chatter switched off with one click,
// any other repeating message hidden from its context menu.
#pragma once
#include <set>
#include <string>

#include "telemetry_client.hpp"

namespace openhouse::viewer {

struct LogFilter {
  bool info = true, warn = true, error = true, debug = false;  // DEBUG: off unless asked for
  bool hideMulti = true;  // "[multi]": multi-station decoding, a few lines a second
  bool hideLh2 = true;    // "[lh2]": 2.0 light decoding
  std::set<std::string> hidden;  // Shape() of messages hidden from the context menu
  std::string text;              // substring, any case

  bool Shows(const LogLine& line) const;
  // The message with its numbers, serials and times blanked: what "messages like this"
  // means (a 4 Hz "slot 2 swept axis 1 ... (300 so far)" is one shape).
  static std::string Shape(const std::string& text);
};

class LogView {
 public:
  // The panel's contents (inside a window or child the caller owns).
  void Draw(TelemetryClient& client);
  LogFilter& Filter() { return filter_; }

 private:
  LogFilter filter_;
  bool autoScroll_ = true, paused_ = false;
  uint64_t pausedAt_ = 0;  // lines received when paused (a running total: the log drops old lines)
  char search_[128] = "";
};

}  // namespace openhouse::viewer
