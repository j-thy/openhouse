// Menu entries for the installed apps (openhouse-overlay's desktop window, the dongle manager):
// Windows: Start menu > openhouse (.lnk); Linux: ~/.local/share/applications (.desktop). On Linux the
// overlay's entry is also an Envision plugin that runs the update check when the XR service starts.
#pragma once
#include <string>
#include <vector>

#include "system.hpp"

namespace openhouse::installer {

struct Shortcut {
  std::string id;       // file name without extension
  std::string name;     // what the menu shows
  std::string comment;
  fs::path exe;
  fs::path icon;             // Linux: a PNG ("" for a generic one); Windows uses the program's own
  std::vector<std::string> xrPluginArgs;  // Linux: non-empty makes it an Envision plugin run with these arguments
};

// Writes them and removes ones a previous install made that aren't in the list. Returns what failed ("" if none).
std::string InstallShortcuts(const std::vector<Shortcut>& shortcuts);
void RemoveShortcuts();

}  // namespace openhouse::installer
