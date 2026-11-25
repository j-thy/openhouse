// Generic raw-data recorder. Deliberately does not run a tracking estimator.
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#ifdef _WIN32
#include <conio.h>
#else
#include <poll.h>
#include <unistd.h>
#endif
#include "platform/platform.hpp"

#include "lh/input_sink.hpp"
#include "usb_transport.hpp"
#include "sync_monitor.hpp"

namespace {
namespace fs = std::filesystem;
using Json = nlohmann::json;
static_assert(std::atomic<bool>::is_always_lock_free);
std::atomic<bool> stopped{false};
void Stop(int) { stopped.store(true, std::memory_order_relaxed); }

double Now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
double UnixNow() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}
std::string Timestamp() {
  const std::time_t now = std::time(nullptr);
  std::ostringstream s;
  s << std::put_time(std::gmtime(&now), "%Y%m%d-%H%M%SZ") << "-" << lh::platform::ProcessId();
  return s.str();
}
void Save(const fs::path& path, const Json& value) {
  const fs::path temporary = lh::platform::FsPath(lh::platform::PathUtf8(path) + ".tmp");
  std::ofstream file(temporary);
  file.exceptions(std::ios::badbit | std::ios::failbit);
  file << value.dump(2) << '\n';
  file.close();
  lh::platform::ReplaceFile(temporary, path);
}
bool SteamVrRunning() { return lh::platform::VrserverRunning(); }

// >0 bytes available now, 0 no input yet, -1 EOF. Never block capture on stdin.
long ReadNotes(char* buffer, size_t capacity) {
#ifdef _WIN32
  HANDLE input = GetStdHandle(STD_INPUT_HANDLE);
  const DWORD type = GetFileType(input);
  if (type == FILE_TYPE_CHAR) {
    size_t n = 0;
    while (n < capacity && _kbhit()) {
      const int c = _getch();
      if (c == 0 || c == 224) { _getch(); continue; }
      buffer[n++] = c == '\r' ? '\n' : char(c);
    }
    return long(n);
  }
  if (type == FILE_TYPE_PIPE) {
    DWORD available = 0;
    if (!PeekNamedPipe(input, nullptr, 0, nullptr, &available, nullptr)) return -1;
    if (!available) return 0;
  } else if (type != FILE_TYPE_DISK) return -1;
  DWORD count = 0;
  return ReadFile(input, buffer, DWORD(capacity), &count, nullptr) && count ? long(count) : -1;
#else
  pollfd input{STDIN_FILENO, POLLIN, 0};
  if (poll(&input, 1, 0) <= 0 || !(input.revents & (POLLIN | POLLHUP))) return 0;
  const auto n = read(STDIN_FILENO, buffer, capacity);
  return n > 0 ? long(n) : -1;
#endif
}

struct Counters : lh::InputSinkBase {
  struct Device {
    uint64_t imu = 0, pulses = 0, lh2 = 0;
    bool attached = false;
    std::string config;
    double offset = 0;
  };
  mutable std::mutex mutex;
  std::map<lh::DeviceId, Device> devices;
  lhcap::SyncMonitor sync;
  void OnDeviceAttached(const lh::DeviceId& id, const std::string& json) override {
    std::lock_guard lock(mutex);
    auto& d = devices[id]; d.attached = true;
    if (!json.empty()) d.config = json;
  }
  void OnDeviceDetached(const lh::DeviceId& id) override {
    std::lock_guard lock(mutex); devices[id].attached = false;
  }
  void OnClockOffset(const lh::DeviceId& id, double offset) override {
    std::lock_guard lock(mutex); devices[id].offset = offset;
  }
  void OnImu(const lh::DeviceId& id, const lh::ImuSample&) override {
    std::lock_guard lock(mutex); ++devices[id].imu;
  }
  void OnLightPulse(const lh::DeviceId& id, const lh::LightPulse& p) override {
    { std::lock_guard lock(mutex); ++devices[id].pulses; }
    sync.OnPulse(id, p);
  }
  void OnLh2BeamWord(const lh::DeviceId& id, const lh::Lh2BeamWord&) override {
    std::lock_guard lock(mutex); ++devices[id].lh2;
  }
  void OnLh2SensorWord(const lh::DeviceId& id, const lh::Lh2SensorWord&) override {
    std::lock_guard lock(mutex); ++devices[id].lh2;
  }
  std::vector<lh::DeviceId> Attached() const {
    std::lock_guard lock(mutex);
    std::vector<lh::DeviceId> result;
    for (const auto& [id, d] : devices) if (d.attached) result.push_back(id);
    return result;
  }
  Json Snapshot() const {
    std::lock_guard lock(mutex);
    Json result = Json::array();
    for (const auto& [id, d] : devices)
      result.push_back({{"serial", id}, {"attached", d.attached}, {"imu_samples", d.imu},
        {"light_pulses", d.pulses}, {"lh2_words", d.lh2}, {"has_config", !d.config.empty()},
        {"last_clock_offset", d.offset}});
    return result;
  }
  void SaveConfigs(const fs::path& directory) const {
    std::lock_guard lock(mutex);
    fs::create_directory(directory);
    unsigned index = 0;
    for (const auto& [id, d] : devices) {
      if (d.config.empty()) continue;
      // Numbered files avoid interpreting USB-provided serials as file paths.
      const auto stem = "device-" + std::to_string(index++);
      std::ofstream file(directory / (stem + ".json"));
      file.exceptions(std::ios::badbit | std::ios::failbit);
      file << d.config; file.close();
      Save(directory / (stem + ".identity.json"), {{"serial", id}});
    }
  }
};

// Guided recordings for the multi-station work (ai/driver/multistation_plan.md, step 7).
struct Phase { double seconds; const char* name; const char* instruction; };
const std::vector<Phase> kScenarioPhases = {
  {5, "settle", "Hold the controller still in view of every station."},
  {40, "moving", "Move the controller(s) all around: high, low, turn them, walk the play area."},
  {15, "still_all", "Put the controller down, still, where every station sees it."},
  {15, "still_block_master", "Leave it still. Block the MASTER station (slot 0) with your body or a cloth."},
  {15, "still_block_other", "Unblock the master. Now block the OTHER station(s)."},
  {25, "moving_fast", "Unblock. Pick it up: fast swings, then slow motion, turn around once."},
};

// --mode-test: does a device in LH2 light mode still report LH1 bases (and both at once),
// and how fast does a mode switch take effect? Each phase sets every wireless device's
// optical sub-protocol: 0 = LH1, 1 = LH2, -1 = alternate every second. For someone with
// both kinds of base station; ai/driver/lh1_as_lh2_research.md.
struct ModePhase { double seconds; const char* name; int mode; const char* instruction; };
const std::vector<ModePhase> kModeTestPhases = {
  {25, "lh1only_setup_mode0", 0, "Turn ON only the 1.0 base stations (2.0 OFF). Controllers on, in view, still."},
  {20, "lh1only_mode1", 1, "Keep only the 1.0 stations on. Keep the controllers still in view."},
  {10, "lh1only_mode0", 0, "Same: only 1.0 stations on, controllers still."},
  {12, "lh1only_toggle", -1, "Same: only 1.0 stations on, controllers still."},
  {30, "lh2only_setup_mode1", 1, "Turn OFF the 1.0 stations and turn ON the 2.0 stations. Controllers still in view."},
  {20, "lh2only_mode0", 0, "Keep only the 2.0 stations on. Controllers still."},
  {10, "lh2only_mode1", 1, "Same: only 2.0 stations on, controllers still."},
  {12, "lh2only_toggle", -1, "Same: only 2.0 stations on, controllers still."},
  {30, "both_setup_mode1", 1, "Turn the 1.0 stations back ON too (all stations on). Controllers still in view."},
  {20, "both_mode0", 0, "All stations on. Controllers still."},
  {20, "both_mode1", 1, "All stations on. Controllers still."},
  {12, "both_toggle", -1, "All stations on. Controllers still."},
  {20, "both_moving_mode1", 1, "All stations on. Now move the controllers around slowly."},
};

// --lh2: a guided recording for owners of 2.0 base stations, to build LH2 support from.
// Every device is kept in LH2 light mode (they start in LH1 mode until a host switches
// them). Still phases give reference poses, blocked phases each base alone.
const std::vector<ModePhase> kLh2Phases = {
  {20, "setup", 1, "Turn on all 2.0 base stations. Controllers/trackers on, in view of every station, held still."},
  {20, "still_all", 1, "Put every device down, still, where all stations see it."},
  {45, "moving", 1, "Pick them up and move around normally: high, low, turn them, walk the play area."},
  {15, "still_block_one", 1, "Put them down again, still. Block ONE station with your body or a cloth."},
  {15, "still_block_other", 1, "Unblock it. Block a DIFFERENT station."},
  {30, "moving_fast", 1, "Unblock. Pick them up: fast swings and spins, then slow motion."},
  {15, "still_end", 1, "Put every device down, still, in view of all stations."},
};

int Usage() {
  std::cout << "usage: lhcapture [OUTPUT_DIRECTORY] [--seconds N]\n"
               "       lhcapture --check [CAPTURE.lhraw] live (or recorded) sync-flash layout, records nothing\n"
               "       lhcapture OUTPUT_DIRECTORY --scenario NAME [--config TEXT]\n"
               "                                         guided ~2 min recording with timed phases\n\n"
               "--check / --scenario print, per device, each flash position in the index-event\n"
               "group, how often each sees skip=0 (sweeps), and the inferred mode.\n"
               "--config stores your firmware settings (e.g. \"multi N=4 slots 0,2 MASTER+ACTIVE\").\n"
               "       lhcapture OUTPUT_DIRECTORY --mode-test\n"
               "                                         guided ~4 min test for owners of both 1.0 and\n"
               "                                         2.0 base stations (switches controllers' light mode)\n"
               "       lhcapture OUTPUT_DIRECTORY --lh2\n"
               "                                         guided ~2.5 min recording with 2.0 base stations\n\n"
               "Records all Lighthouse devices; no tracking estimator runs.\n"
               "Default output: captures/session-<UTC>-<pid>/\n"
               "Default duration: until Ctrl-C. Close SteamVR and turn the controller on.\n"
               "Optional: type any note and Enter to timestamp it during recording.\n";
  return 0;
}

int Run(int argc, char** argv) {
  fs::path output;
  double seconds = 0;
  bool check = false, modeTest = false, lh2 = false;
  std::string scenario, config;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") return Usage();
    if (arg == "--seconds") {
      if (++i == argc) throw std::runtime_error("--seconds requires a number");
      char* end = nullptr;
      seconds = std::strtod(argv[i], &end);
      if (end == argv[i] || *end || !std::isfinite(seconds) || seconds < 0)
        throw std::runtime_error("invalid duration");
    } else if (arg == "--check") check = true;
    else if (arg == "--mode-test") modeTest = true;
    else if (arg == "--lh2") modeTest = lh2 = true;
    else if (arg == "--scenario" || arg == "--config") {
      if (++i == argc) throw std::runtime_error(arg + " requires a value");
      (arg == "--scenario" ? scenario : config) = argv[i];
    } else if (!arg.empty() && arg[0] != '-' && output.empty()) output = arg;
    else throw std::runtime_error("unknown argument: " + arg);
  }
  if (check && !output.empty()) {  // offline: --check CAPTURE.lhraw
    Counters counters;
    lh::usb::ReportDecoder decoder(&counters);
    lh::usb::RawCaptureReader reader;
    if (!reader.Open(lh::platform::PathUtf8(output))) throw std::runtime_error("cannot open " + lh::platform::PathUtf8(output));
    double next = -1;
    reader.ReadAll([&](const lh::usb::EndpointInfo& e) { decoder.AddEndpoint(e); },
                   [&](const lh::usb::RawReport& r) {
                     decoder.OnReport(r);
                     if (r.hostTime <= 0) return;
                     if (next < 0) next = r.hostTime + 5;
                     if (r.hostTime >= next) { next += 5; std::cout << "---\n" << counters.sync.Report(); }
                   });
    return 0;
  }
  if (check) {
    if (SteamVrRunning()) throw std::runtime_error("close SteamVR first (vrserver is running)");
    Counters counters;
    lh::usb::UsbTransport transport(&counters, {});
    if (!transport.Start()) { transport.Stop(); throw std::runtime_error("no devices opened"); }
    std::cout << "Checking sync layout. Ctrl-C to stop.\n" << std::flush;
    while (!stopped) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      std::cout << "---\n" << counters.sync.Report() << std::flush;
    }
    transport.Stop();
    return 0;
  }
  if (!scenario.empty() && modeTest) throw std::runtime_error("--scenario and --mode-test are separate recordings");
  const auto& modePhases = lh2 ? kLh2Phases : kModeTestPhases;
  if (!scenario.empty() || modeTest) {
    double total = 0;
    if (modeTest) for (const auto& p : modePhases) total += p.seconds;
    else for (const auto& p : kScenarioPhases) total += p.seconds;
    seconds = total;
  }
  if (output.empty()) output = fs::path("captures") / ("session-" + Timestamp());
  if (fs::exists(output)) throw std::runtime_error("output directory already exists (will not overwrite): " + lh::platform::PathUtf8(output));
  if (SteamVrRunning()) throw std::runtime_error("close SteamVR before recording (vrserver is running)");
  if (output.has_parent_path()) fs::create_directories(output.parent_path());
  if (!fs::create_directory(output)) throw std::runtime_error("cannot create output directory");
  const auto path = fs::absolute(output);
  Json session{{"schema", 1}, {"tool", "lhcapture"}, {"format", "LHRAW001"},
    {"status", "starting"}, {"raw_file", "input.lhraw"}, {"markers_file", "markers.jsonl"},
    {"has_reference_poses", false},
    {"start_host_time", Now()}, {"start_unix_time", UnixNow()},
    {"clock", "C++ steady_clock seconds; same host clock as raw reports"},
    {"requested_seconds", seconds}, {"devices", Json::array()}};
  if (!scenario.empty()) session["scenario"] = scenario;
  if (modeTest) session["scenario"] = lh2 ? "lh2" : "mode-test";
  if (!config.empty()) session["firmware_config"] = config;
  const fs::path cache = lh::platform::FsPath(lh::platform::SteamConfigDirectory());
  const auto database = cache / "lighthousedb.json";
  std::error_code copyError;
  if (fs::exists(database, copyError)) {
    fs::copy_file(database, path / "lighthousedb.json", fs::copy_options::none, copyError);
    if (!copyError) session["database_file"] = "lighthousedb.json";
    else session["database_snapshot_error"] = copyError.message();
  }
  // SteamVR's chaperone: its own solved base station poses, a reference for ours.
  const auto chaperone = cache / "chaperone_info.vrchap";
  if (fs::exists(chaperone, copyError)) {
    fs::copy_file(chaperone, path / "chaperone_info.vrchap", fs::copy_options::none, copyError);
    if (!copyError) session["chaperone_file"] = "chaperone_info.vrchap";
  }
  Save(path / "session.json", session);
  std::ofstream markers(path / "markers.jsonl");
  markers.exceptions(std::ios::badbit | std::ios::failbit);
  const auto mark = [&](const std::string& text) {
    markers << Json{{"host_time", Now()}, {"note", text}}.dump() << '\n';
    markers.flush();
  };
  Counters counters;
  lh::usb::UsbTransportOptions options;
  options.capturePath = lh::platform::PathUtf8(path / "input.lhraw");
  lh::usb::UsbTransport transport(&counters, options);
  std::cout << "Recording to " << path << "\nOpening devices..." << std::endl;
  if (!transport.Start()) {
    transport.Stop();
    session["status"] = "failed"; session["error"] = "no devices opened or capture file could not be opened";
    session["stop_host_time"] = Now(); Save(path / "session.json", session);
    throw std::runtime_error(session["error"].get<std::string>());
  }
  const auto endpoints = transport.Endpoints();
  session["endpoints"] = Json::array();
  for (const auto& ep : endpoints) {
    session["endpoints"].push_back({{"endpoint", ep.ep}, {"vid", ep.vid}, {"pid", ep.pid},
      {"interface", ep.iface}, {"kind", int(ep.kind)}, {"serial", ep.serial},
      {"gyro_range", int(ep.gyroRange)}, {"accel_range", int(ep.accelRange)}});
    std::cout << "  " << ep.serial << " endpoint " << ep.ep << (ep.configJson.empty() ? "" : " (config saved)") << '\n';
  }
  const double ready = Now();
  session["reader_ready_host_time"] = ready; session["status"] = "recording";
  Save(path / "session.json", session);
  mark("capture_ready");
  std::cout << "Recording. Ctrl-C to finish. Optional notes: type text and Enter.\n" << std::flush;
  double nextStatus = ready + 1;
  size_t phase = 0;
  double phaseEnd = ready;
  int modePhase = 0, toggleMode = 0;
  double nextToggle = 0;
  Json syncLog = Json::array();
  bool stdinOpen = true;
  std::string pending;
  bool healthy = true;
  while (!stopped && (seconds == 0 || Now() - ready < seconds)) {
    if (!transport.CaptureHealthy()) { healthy = false; break; }
    if (stdinOpen) {
      char buffer[1024];
      const auto count = ReadNotes(buffer, sizeof buffer);
      if (count != 0) {
        if (count < 0) stdinOpen = false;
        else {
          for (long i = 0; i < count; ++i) {
            if (buffer[i] == '\n') {
              if (!pending.empty()) { mark(pending); std::cout << "Marked: " << pending << '\n'; }
              pending.clear();
            } else if (pending.size() < 4096 && buffer[i] != '\r') pending += buffer[i];
          }
        }
      }
    }
    if (!scenario.empty() && phase < kScenarioPhases.size() && Now() >= phaseEnd) {
      const auto& p = kScenarioPhases[phase++];
      phaseEnd = Now() + p.seconds;
      mark(std::string("phase:") + p.name);
      std::cout << "\a\n>>> [" << phase << "/" << kScenarioPhases.size() << "] " << p.instruction
                << " (" << p.seconds << " s)\n\n" << std::flush;
    }
    if (modeTest) {
      const auto setMode = [&](int mode) {
        // Every attached device, now (controllers may turn on late); the transport
        // delivers it to wireless endpoints only.
        for (const auto& serial : counters.Attached()) transport.SetOpticalSubProtocol(serial, mode);
        mark("optical_mode:" + std::to_string(mode));
      };
      if (phase < modePhases.size() && Now() >= phaseEnd) {
        const auto& p = modePhases[phase++];
        phaseEnd = Now() + p.seconds;
        mark(std::string("phase:") + p.name);
        modePhase = p.mode;
        toggleMode = 0;
        nextToggle = Now() + 1;
        setMode(p.mode < 0 ? 0 : p.mode);
        std::cout << "\a\n>>> [" << phase << "/" << modePhases.size() << "] " << p.instruction << " ("
                  << p.seconds << " s)\n\n" << std::flush;
      } else if (modePhase < 0 && Now() >= nextToggle) {
        toggleMode ^= 1;
        setMode(toggleMode);
        nextToggle = Now() + 1;
      } else if (lh2 && Now() >= nextToggle) {  // devices turned on late join too
        setMode(1);
        nextToggle = Now() + 5;
      }
    }
    if (Now() >= nextStatus) {
      const auto report = counters.sync.Report();
      syncLog.push_back({{"host_time", Now()}, {"report", report}});
      std::cout << report;
      session["devices"] = counters.Snapshot(); session["last_update_host_time"] = Now();
      Save(path / "session.json", session);
      for (const auto& d : session["devices"]) {
        const auto imu = d["imu_samples"].get<uint64_t>(), light = d["light_pulses"].get<uint64_t>(),
                   lh2 = d["lh2_words"].get<uint64_t>();
        if (imu || light || lh2)
          std::cout << d["serial"].get<std::string>() << ": IMU " << imu << ", light " << light << ", LH2 words " << lh2
                    << '\n';
      }
      std::cout << std::flush;
      nextStatus = Now() + 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (modeTest) {  // leave the controllers in LH1 light mode, as they were
    for (const auto& serial : counters.Attached()) transport.SetOpticalSubProtocol(serial, 0);
    mark("optical_mode:0");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // let the USB thread send it
  }
  mark("capture_stop");
  transport.Stop();  // joins USB thread and flushes/closes the raw file
  healthy = healthy && transport.CaptureHealthy();
  counters.SaveConfigs(path / "device-configs");
  session["devices"] = counters.Snapshot();
  session["sync_reports"] = syncLog;
  session["stop_host_time"] = Now(); session["stop_unix_time"] = UnixNow();
  session["reader_seconds"] = Now() - ready;
  session["stop_reason"] = stopped ? "signal" : (healthy ? "duration" : "write_error");
  bool hasImu = false, hasLight = false, hasLh2 = false;
  for (const auto& d : session["devices"]) {
    hasImu = hasImu || d["imu_samples"].get<uint64_t>() > 0;
    hasLight = hasLight || d["light_pulses"].get<uint64_t>() > 0;
    hasLh2 = hasLh2 || d["lh2_words"].get<uint64_t>() > 0;
  }
  session["has_imu"] = hasImu; session["has_lh1_light"] = hasLight; session["has_lh2_light"] = hasLh2;
  if (lh2) hasLight = hasLh2;
  session["status"] = healthy ? "complete" : "failed";
  if (!healthy) session["error"] = "raw capture write/flush failed; file may be incomplete";
  Save(path / "session.json", session);
  std::cout << (healthy ? "Saved: " : "CAPTURE FAILED: ") << path << '\n';
  if (!hasImu || !hasLight) std::cerr << "Warning: missing IMU or " << (lh2 ? "LH2" : "LH1") << " light input; keep this session but check that the controller/bases were on.\n";
  if (!healthy) throw std::runtime_error("capture write/flush failure");
  return hasImu && hasLight ? 0 : 1;
}
}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, Stop); std::signal(SIGTERM, Stop);
  try { return Run(argc, argv); }
  catch (const std::exception& e) { std::cerr << "lhcapture: " << e.what() << '\n'; return 1; }
}
