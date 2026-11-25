// openhouse-installer --update (what the overlay's Update and Reinstall start): downloads the
// latest release's package for this platform from GitHub into a temporary folder, unpacks it and
// starts its installer with --install, which installs right away.
#pragma once
#include <functional>
#include <string>

#include "system.hpp"

namespace openhouse::installer {

// The new package's installer; throws with a message a user can act on. OPENHOUSE_UPDATE_PACKAGE
// (testing): a local package instead of the download.
fs::path DownloadLatestPackage(const std::function<void(float, const std::string&)>& status, std::string* tag);
// Starts it with `argument`, on its own (out of this process's job / session); false if it couldn't.
bool StartDetached(const fs::path& exe, const std::string& argument);

}  // namespace openhouse::installer
