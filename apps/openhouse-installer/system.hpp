// openhouse-installer's platform layer: paths, files, processes, external tools. Every path
// a std::string holds is UTF-8; Path() turns it into a filesystem path.
#pragma once
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace openhouse::installer {

namespace fs = std::filesystem;

fs::path Path(const std::string& utf8);
std::string Utf8(const fs::path& p);

fs::path ExeDirectory();               // where openhouse-installer is: the package
fs::path DataDirectory();              // %LOCALAPPDATA%\openhouse, ~/.local/share/openhouse
fs::path ConfigDirectory();            // %APPDATA%\lh, ~/.config/lh (openhouse's own config)
fs::path HomeDirectory();

bool ReadFile(const fs::path& p, std::string* out);
bool WriteFileAtomic(const fs::path& p, const std::string& data);  // via a temporary + rename

// Copies over `to` (retrying a few seconds while it's locked, on Windows). Throws with a
// message a user can act on.
void CopyOver(const fs::path& from, const fs::path& to);
void RemoveFile(const fs::path& p);

// The installer's own settings (installer.json in openhouse's config folder).
std::string LoadSetting(const std::string& key);
void SaveSetting(const std::string& key, const std::string& value);

// Processes that hold the driver: SteamVR, Steam (Windows), Monado, WiVRn.
std::vector<std::string> RunningRuntimes();
bool CloseRuntimes();  // asks them to close, then kills; true when none are left

// Runs a program, giving each line of its output (stdout and stderr) to `line`. Returns its
// exit code, -1 if it couldn't start.
int RunProcess(const std::vector<std::string>& argv, const std::function<void(const std::string&)>& line);
bool HaveProgram(const char* name);  // on PATH (Linux)

// A folder picker; "" when cancelled or none is available.
std::string PickFolder(const std::string& title);
void OpenInFileManager(const fs::path& p);
void StartApp(const fs::path& exe);  // starts a program and doesn't wait for it

}  // namespace openhouse::installer
