// One zip with everything a bug report needs, saved to the user's Documents folder:
// a recording (SteamVR settings > openhouse > Record Session), the room and settings
// files, SteamVR's server logs, and a summary (version, system, devices). The driver
// saves one when Record Session is turned off; openhouse-overlay has Save bug report.
#pragma once
#include <string>

namespace lh {

struct BugReportOptions {
  // Added to info.txt after the version and system lines: devices, bases, room.
  std::string summary;
  // Optional log text (the viewer's received driver log), saved as openhouse-log.txt.
  std::string log;
  // The recording to include; empty = the newest one in the recordings folder.
  std::string recording;
};

// <Documents>/openhouse reports (Windows: the Documents known folder; Linux: XDG
// DOCUMENTS, else ~/Documents).
std::string BugReportDirectory();

// Writes openhouse-report-YYYYMMDD-HHMMSS.zip there; returns its path, or "" with
// *error set. Missing files are skipped (and listed in info.txt). Can take a few
// seconds with a long recording: call it off any real-time thread.
std::string SaveBugReport(const BugReportOptions& options, std::string* error = nullptr);

}  // namespace lh
