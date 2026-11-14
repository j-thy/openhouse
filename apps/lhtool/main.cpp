// lhtool — command-line utility for the Lighthouse library.
//
//   lhtool capture OUT.lhraw [SECONDS]   record raw HID reports from all devices
//   lhtool stats [IN.lhraw | live SECONDS]  decode and print per-device statistics
//   lhtool inputs [IN.lhraw | live SECONDS] print each device's controller input as it changes
#include <chrono>
#include <csignal>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <nlohmann/json.hpp>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <thread>

#include "lh/input_sink.hpp"
#include "lh/session.hpp"
#include "lh/tracker.hpp"
#include "raw_capture.hpp"
#include "usb_transport.hpp"

namespace {

volatile std::sig_atomic_t interrupted = 0;
void Stop(int) { interrupted = 1; }

// Controller input as the transport frames it, each change per device: for checking a
// controller's layout (raw records of mapped-input firmware such as Shiftall FlipVR).
struct InputsSink : lh::InputSinkBase {
  struct Last {
    int format = -1;
    std::string text;
  };
  std::mutex mutex;
  std::map<std::string, Last> last;
  std::map<std::string, long> counts;
  void OnDeviceAttached(const lh::DeviceId&, const std::string&) override {}
  void OnDeviceDetached(const lh::DeviceId&) override {}
  void OnClockOffset(const lh::DeviceId&, double) override {}
  void OnImu(const lh::DeviceId&, const lh::ImuSample&) override {}
  void OnLightPulse(const lh::DeviceId&, const lh::LightPulse&) override {}
  void OnLh2BeamWord(const lh::DeviceId&, const lh::Lh2BeamWord&) override {}
  void OnLh2SensorWord(const lh::DeviceId&, const lh::Lh2SensorWord&) override {}
  void OnDemodCount(const lh::DeviceId&, lh::DeviceTime, int) override {}
  void OnEvent(const lh::DeviceId&, const lh::DeviceEvent&) override {}
  void OnAux(const lh::DeviceId&, const lh::AuxEvent&) override {}
  void OnInput(const lh::DeviceId& id, const lh::InputReport& r) override {
    std::string text;
    char buf[64];
    if (r.format == lh::InputFormat::Raw) {
      text = "raw[" + std::to_string(r.raw.size()) + "]";
      for (uint8_t b : r.raw) { std::snprintf(buf, sizeof buf, " %02x", b); text += buf; }
    } else {
      std::snprintf(buf, sizeof buf, "fmt%d", int(r.format));
      text = buf;
      if (r.hasButtons) { std::snprintf(buf, sizeof buf, " buttons=%02x", r.buttons); text += buf; }
      if (r.hasTrigger) { std::snprintf(buf, sizeof buf, " trigger=%u", r.trigger); text += buf; }
      if (r.hasAxes) { std::snprintf(buf, sizeof buf, " axes=%d,%d", r.axisX, r.axisY); text += buf; }
      if (r.format == lh::InputFormat::Compact) { std::snprintf(buf, sizeof buf, " extra=%02x", r.extra); text += buf; }
      if (r.hasExtended) {
        text += " ext";
        for (uint8_t b : r.extended) { std::snprintf(buf, sizeof buf, " %02x", b); text += buf; }
      }
    }
    std::lock_guard lock(mutex);
    ++counts[id];
    auto& l = last[id];
    if (l.text == text) return;
    l.text = text;
    std::printf("%12.4f %s %s\n", r.time, id.c_str(), text.c_str());
    std::fflush(stdout);
  }
};

struct StatsSink : lh::InputSinkBase {
  struct Dev {
    long imu = 0, pulses = 0, beam = 0, sensorWords = 0, demod = 0, aux = 0;
    std::map<int, long> events;
    std::map<int, long> sensors;
    std::map<int, long> channels;
    double firstT = 0, lastT = 0, offset = 0;
    bool hasConfig = false;
    void Time(double t) {
      if (firstT == 0) firstT = t;
      lastT = t;
    }
  };
  std::map<std::string, Dev> devs;
  void OnDeviceAttached(const lh::DeviceId& d, const std::string& cfg) override { devs[d].hasConfig |= !cfg.empty(); }
  void OnClockOffset(const lh::DeviceId& d, double o) override { devs[d].offset = o; }
  void OnImu(const lh::DeviceId& d, const lh::ImuSample& s) override {
    devs[d].imu++; devs[d].Time(s.time);
    // LH_IMU_DUMP=SERIAL: that device's raw IMU samples to stdout (time ax ay az gx gy gz saturated).
    static const char* dump = std::getenv("LH_IMU_DUMP");
    if (dump && d == dump)
      printf("imu %.6f %.4f %.4f %.4f %.5f %.5f %.5f %u\n", double(s.time), s.accel[0], s.accel[1], s.accel[2], s.gyro[0], s.gyro[1],
             s.gyro[2], s.saturated);
  }
  void OnLightPulse(const lh::DeviceId& d, const lh::LightPulse& p) override {
    devs[d].pulses++;
    devs[d].sensors[p.sensor]++;
  }
  void OnLh2BeamWord(const lh::DeviceId& d, const lh::Lh2BeamWord& w) override {
    devs[d].beam++;
    devs[d].channels[w.channel]++;
  }
  void OnLh2SensorWord(const lh::DeviceId& d, const lh::Lh2SensorWord& w) override {
    devs[d].sensorWords++;
    devs[d].sensors[w.value]++;
  }
  void OnDemodCount(const lh::DeviceId& d, lh::DeviceTime, int n) override { devs[d].demod += n; }
  void OnEvent(const lh::DeviceId& d, const lh::DeviceEvent& e) override { devs[d].events[e.type]++; }
  void OnAux(const lh::DeviceId& d, const lh::AuxEvent&) override { devs[d].aux++; }

  void Print() const {
    for (auto& [id, d] : devs) {
      const double span = d.lastT - d.firstT;
      printf("%s: config=%s imu=%ld (%.0f Hz) pulses=%ld lh2beam=%ld lh2sensor=%ld demod=%ld aux=%ld clockOffset=%.6f\n",
             id.c_str(), d.hasConfig ? "yes" : "no", d.imu, span > 0 ? d.imu / span : 0.0, d.pulses, d.beam,
             d.sensorWords, d.demod, d.aux, d.offset);
      if (!d.events.empty()) {
        printf("  events:");
        for (auto& [t, n] : d.events) printf(" 0x%x:%ld", t, n);
        printf("\n");
      }
      if (!d.sensors.empty()) {
        printf("  sensors/values:");
        for (auto& [s, n] : d.sensors) printf(" %d:%ld", s, n);
        printf("\n");
      }
      if (!d.channels.empty()) {
        printf("  lh2 channels:");
        for (auto& [c, n] : d.channels) printf(" %d:%ld", c, n);
        printf("\n");
      }
    }
  }
};

int Usage() {
  fprintf(stderr,
          "usage:\n"
          "  lhtool capture OUT.lhraw [SECONDS]\n"
          "  lhtool stats IN.lhraw\n"
          "  lhtool trim IN.lhraw OUT.lhraw --to SECONDS   the first SECONDS of a capture (test fixtures)\n"
          "  lhtool stats live [SECONDS]\n"
          "  lhtool track IN.lhraw [--db DATABASE.json]\n"
          "  lhtool track live [SECONDS] [--db DATABASE.json]\n"
          "  lhtool session IN.lhraw|live [--seconds N] [--rate HZ] [--room ROOM.json] [--realtime] [--disable SERIAL]... [--settings SETTINGS.json] [--steam-db DB.json|-] [--preinit DIR|-]\n"
          "      [--from S] [--to S] [--only SERIAL]... [--trace OUT.jsonl] [--smoothing S] [--no-acceleration] [--mixed-light]\n"
          "      [--mixed-no-rotor-reference] [--mixed-switch S]...\n"
          "      replay window (capture seconds), devices, and every pose at 100 Hz of capture time;\n"
          "      --mixed-light: 1.0 and 2.0 base stations together, as the SteamVR settings switch\n"
          "      --mixed-no-rotor-reference: with it, don't take each 2.0 rotor's wobble, as devices in LH2 mode\n"
          "        measure it, off the sorted hits (it's on by default)\n"
          "      --mixed-switch: flip that switch at S capture seconds, as it was live\n"
          "      liblh session: shared room map, roles from device JSON, Index input; JSON Lines\n"
          "  lhtool haptics SERIAL [--frequency HZ] [--amplitude 0..1] [--duration S] [--pulses N] [--capture OUT]\n"
          "      direct receiver vibration diagnostic; prints actual HID write results\n"
          "  lhtool configs OUT_DIR [SECONDS]\n"
          "      save the JSON config of every connected device as OUT_DIR/<serial>.json, waiting SECONDS\n"
          "      (default 10) for wireless devices; copy them to ~/.config/lh/preinit to preinitialize them\n");
  return 2;
}

void RunFor(double seconds) {
  const auto start = std::chrono::steady_clock::now();
  if (seconds <= 0) fprintf(stderr, "running, press Ctrl-C to stop\n");
  while (!interrupted && (seconds <= 0 || std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds))
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

const char* Status(lh::TrackingStatus s) {
  switch (s) {
    case lh::TrackingStatus::Optical: return "optical";
    case lh::TrackingStatus::Inertial: return "inertial";
    case lh::TrackingStatus::Lost: return "lost";
    default: return "uninitialized";
  }
}

int Track(int argc, char** argv) {
  if (argc < 3) return Usage();
  const bool live = std::string(argv[2]) == "live";
  double seconds = 5;
  std::string database;
  for (int i = 3; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--db" && i + 1 < argc) database = argv[++i];
    else if (live && i == 3) {
      char* end = nullptr;
      seconds = std::strtod(argv[i], &end);
      if (!end || *end || !std::isfinite(seconds) || seconds < 0) return Usage();
    } else return Usage();
  }
  lh::Tracker tracker;
  if (!database.empty()) {
    std::ifstream file(database);
    std::stringstream text; text << file.rdbuf();
    std::string error;
    if (!file || !tracker.LoadBasePoses(text.str(), &error)) {
      fprintf(stderr, "cannot load database %s: %s\n", database.c_str(), error.c_str()); return 1;
    }
  }
  size_t total = 0;
  auto drain = [&] {
    for (const auto& p : tracker.PollPoses()) {
      nlohmann::json j{{"device", p.device}, {"device_time", p.time}, {"host_time", p.hostTime},
        {"primary_base", p.primaryBase}, {"world_space", p.worldSpace},
        {"position", p.pose.position}, {"orientation_wxyz", p.pose.orientation},
        {"velocity", p.velocity}, {"angular_velocity_body", p.angularVelocity},
        {"status", Status(p.status)}, {"optical_updates", p.opticalUpdates}, {"flags", p.flags}};
      printf("%s\n", j.dump().c_str()); ++total;
    }
    for (const auto& log : tracker.PollLogs()) fprintf(stderr, "%s\n", log.c_str());
  };
  if (live) {
    lh::usb::UsbTransport transport(&tracker);
    if (!transport.Start()) { fprintf(stderr, "no devices\n"); return 1; }
    const auto start = std::chrono::steady_clock::now();
    while (!interrupted && (seconds == 0 || std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds)) {
      drain(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    transport.Stop();
  } else {
    lh::usb::RawCaptureReader reader;
    if (!reader.Open(argv[2])) { fprintf(stderr, "cannot open capture %s\n", argv[2]); return 1; }
    lh::usb::ReportDecoder decoder(&tracker);
    if (!reader.ReadAll([&](const lh::usb::EndpointInfo& e) { decoder.AddEndpoint(e); },
                        [&](const lh::usb::RawReport& r) { if (!interrupted) { decoder.OnReport(r); drain(); } })) {
      fprintf(stderr, "invalid/truncated capture\n"); return 1;
    }
  }
  drain();
  for (const auto& d : tracker.Devices())
    fprintf(stderr, "%s: configured=%d base=%08X updates=%llu bootstrap_attempts=%llu rejects=%llu %s\n",
            d.device.c_str(), d.configured, d.primaryBase, (unsigned long long)d.opticalUpdates,
            (unsigned long long)d.bootstrapAttempts, (unsigned long long)d.rejectedUpdates, d.error.c_str());
  fprintf(stderr, "%zu pose records (experimental orchestration; not end-to-end oracle verified)\n", total);
  return 0;
}

}  // namespace

// Every connected device's config, as the driver reads it, into OUT_DIR/<serial>.json.
int ConfigsCommand(int argc, char** argv) {
  const std::filesystem::path dir = argv[2];
  const double seconds = argc >= 4 ? atof(argv[3]) : 10;
  struct ConfigSink : lh::InputSinkBase {
    std::mutex mutex;
    std::map<std::string, std::string> configs;
    void OnDeviceAttached(const lh::DeviceId& id, const std::string& config) override {
      if (config.empty()) return;
      std::lock_guard lock(mutex);
      if (!configs.count(id)) fprintf(stderr, "%s: config read (%zu bytes)\n", id.c_str(), config.size());
      configs[id] = config;
    }
    void OnTransportLog(lh::LogLevel level, const std::string& text) override {
      if (level >= lh::LogLevel::Warn) fprintf(stderr, "%s\n", text.c_str());
    }
  } sink;
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) { fprintf(stderr, "lhtool: cannot create %s: %s\n", dir.string().c_str(), ec.message().c_str()); return 1; }
  lh::usb::UsbTransport transport(&sink, {});
  if (!transport.Start()) { fprintf(stderr, "lhtool: no receivers could be opened (close SteamVR/Monado first)\n"); return 1; }
  fprintf(stderr, "waiting %.0f s for devices (turn on the wireless ones now)\n", seconds);
  RunFor(seconds);
  transport.Stop();
  std::lock_guard lock(sink.mutex);
  int written = 0;
  for (const auto& [serial, config] : sink.configs) {
    const auto path = dir / (serial + ".json");
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << config;
    if (!out) { fprintf(stderr, "lhtool: cannot write %s\n", path.string().c_str()); continue; }
    printf("%s\n", path.string().c_str());
    ++written;
  }
  fprintf(stderr, "%d device config(s) saved to %s\n", written, dir.string().c_str());
  return written ? 0 : 1;
}

// liblh session front end: one JSON line per device per sample tick, plus events.
int HapticsCommand(int argc, char** argv) {
  if (argc < 3) return Usage();
  lh::InputSinkBase sink;
  lh::usb::UsbTransportOptions options;
  double frequency = 100, amplitude = .8, duration = .5;
  int pulses = 3;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    if (i + 1 >= argc) { fprintf(stderr, "missing value for %s\n", a.c_str()); return 2; }
    const std::string v = argv[++i];
    try {
      if (a == "--frequency") frequency = std::stod(v);
      else if (a == "--amplitude") amplitude = std::stod(v);
      else if (a == "--duration") duration = std::stod(v);
      else if (a == "--pulses") pulses = std::stoi(v);
      else if (a == "--capture") options.capturePath = v;
      else { fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
    } catch (const std::exception&) { fprintf(stderr, "invalid value for %s\n", a.c_str()); return 2; }
  }
  if (frequency < 0 || frequency > 3333 || amplitude <= 0 || amplitude > 1 || duration < 0 || duration > 2 ||
      pulses < 1 || pulses > 10) { fprintf(stderr, "haptic parameters out of bounds\n"); return 2; }
  options.onCommandResult = [](const lh::usb::DeviceCommandResult& r) {
    printf("%s %s: %s (HID bytes=%d, on=%d us, off=%d us, repeats=%d)\n", r.powerOff ? "power-off" : "haptic",
           r.serial.c_str(), r.status, r.bytes, r.report[4] | r.report[5] << 8,
           r.report[6] | r.report[7] << 8, r.report[8] | r.report[9] << 8);
  };
  lh::usb::UsbTransport transport(&sink, options);
  if (!transport.Start()) { fprintf(stderr, "lhtool: no receivers could be opened\n"); return 1; }
  RunFor(.5);  // the receiver may attach through the probe thread before any write
  for (int i = 0; i < pulses; ++i) {
    transport.Haptic(argv[2], frequency, amplitude, duration);
    std::this_thread::sleep_for(std::chrono::milliseconds(int((duration + .05) * 1000)));
    RunFor(.7);
  }
  transport.Stop();
  return 0;
}

int SessionCommand(int argc, char** argv) {
  lh::SessionOptions options;
  const bool live = std::string(argv[2]) == "live";
  if (!live) { options.replayPath = argv[2]; options.replayRealtime = false; }
  double seconds = 0, rate = 30, smoothing = 0.06;
  bool acceleration = true;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--seconds" && i + 1 < argc) seconds = std::atof(argv[++i]);
    else if (a == "--rate" && i + 1 < argc) rate = std::max(1.0, std::atof(argv[++i]));
    else if (a == "--room" && i + 1 < argc) options.roomMapPath = argv[++i];
    else if (a == "--smoothing" && i + 1 < argc) smoothing = std::atof(argv[++i]);
    else if (a == "--no-acceleration") acceleration = false;
    else if (a == "--steam-db" && i + 1 < argc) options.steamDatabasePath = argv[++i];  // "-": none
    else if (a == "--realtime") options.replayRealtime = true;
    else if (a == "--disable" && i + 1 < argc) options.disabledDevices.push_back(argv[++i]);
    else if (a == "--settings" && i + 1 < argc) options.settingsPath = argv[++i];
    else if (a == "--preinit" && i + 1 < argc) options.preinitDirectory = argv[++i];  // "-": none
    else if (a == "--from" && i + 1 < argc) options.replayFrom = std::atof(argv[++i]);
    else if (a == "--to" && i + 1 < argc) options.replayTo = std::atof(argv[++i]);
    else if (a == "--only" && i + 1 < argc) options.replayOnly.push_back(argv[++i]);
    else if (a == "--trace" && i + 1 < argc) options.replayTracePath = argv[++i];
    else if (a == "--mixed-light") options.mixedLight.enabled = true;
    else if (a == "--mixed-rotor-reference") options.mixedLight.rotorReference = true;  // the default
    else if (a == "--mixed-no-rotor-reference") options.mixedLight.rotorReference = false;
    else if (a == "--mixed-switch" && i + 1 < argc) options.replayMixedSwitch.push_back(std::atof(argv[++i]));
    else { fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
  }
  auto session = lh::Session::Create(options);
  std::string error;
  if (!session->Start(&error)) { fprintf(stderr, "lhtool: %s\n", error.c_str()); return 1; }
  session->SetOutputTuning(smoothing, smoothing, acceleration);  // what the SteamVR settings page sets
  const auto start = std::chrono::steady_clock::now();
  double lastSample = -1e9;
  auto emit = [&] {
    for (const auto& e : session->PollEvents()) {
      nlohmann::json j = {{"event", e.text}, {"t", e.hostTime}, {"serial", e.serial}, {"level", lh::LogLevelName(e.level)}};
      printf("%s\n", j.dump().c_str());
    }
    const auto snap = session->Snapshot();
    for (const auto& b : snap.bases) {  // a base's channel, when first known or changed
      static std::map<std::string, std::string> channels;
      if (b.channel.empty() || channels[b.serial] == b.channel) continue;
      channels[b.serial] = b.channel;
      printf("%s\n", nlohmann::json({{"t", snap.hostTime}, {"base", b.serial}, {"channel", b.channel}}).dump().c_str());
    }
    if (snap.hostTime - lastSample < 1.0 / rate) return;
    lastSample = snap.hostTime;
    for (const auto& d : snap.devices) {
      const auto& p = d.pose;
      if (!d.info.enabled) continue;
      nlohmann::json j = {{"t", p.hostTime}, {"serial", d.info.serial}, {"role", lh::ToString(d.info.role)},
                          {"state", lh::ToString(p.state)}, {"valid", p.valid}, {"result", int(p.result)},
                          {"position", p.worldFromImu.position}, {"orientation", p.worldFromImu.orientation},
                          {"velocity", p.velocity}, {"primaryBase", p.primaryBase}, {"bases", p.bases}};
      if (d.info.batteryPercent) j["battery"] = *d.info.batteryPercent;
      // Light per base over the last 0.5 s: [base, frames, sensors per frame].
      for (const auto& c : p.coverage) j["coverage"].push_back({c.base, c.frames, c.sensors});
      if (d.input) {
        const auto& in = *d.input;
        j["input"] = {{"trigger", in.trigger}, {"grip", in.gripForce}, {"stick", {in.thumbstickX, in.thumbstickY}},
                      {"buttons", {{"system", in.systemClick}, {"a", in.aClick}, {"b", in.bClick},
                                   {"trigger", in.triggerClick}, {"stick", in.thumbstickClick}}},
                      {"fingers", {in.fingerIndex, in.fingerMiddle, in.fingerRing, in.fingerPinky}}};
      }
      printf("%s\n", j.dump().c_str());
    }
  };
  while (true) {
    emit();
    if (!live && session->Finished()) break;
    if (seconds > 0 && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > seconds) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  emit();
  session->Stop();
  return 0;
}

// The first `seconds` of a capture: every endpoint record, and the reports up to that
// capture time (reports stamped 0 come from the header and are kept). Only the start
// of a capture can be cut this way: devices negotiate their protocol when they attach.
int TrimCommand(int argc, char** argv) {
  double seconds = -1;
  for (int i = 4; i + 1 < argc; ++i)
    if (std::string(argv[i]) == "--to") seconds = std::atof(argv[i + 1]);
  if (argc < 6 || seconds <= 0) return Usage();
  lh::usb::RawCaptureReader in;
  lh::usb::RawCaptureWriter out;
  if (!in.Open(argv[2])) { fprintf(stderr, "cannot open %s\n", argv[2]); return 1; }
  {
    // Opening the output truncates it: the same file as the input erased the recording.
    std::error_code ec;
    if (std::filesystem::equivalent(argv[2], argv[3], ec)) {
      fprintf(stderr, "the output is the input (%s): write the trimmed copy somewhere else\n", argv[3]);
      return 1;
    }
  }
  if (!out.Open(argv[3])) { fprintf(stderr, "cannot create %s\n", argv[3]); return 1; }
  double start = -1;
  size_t kept = 0, dropped = 0;
  const bool ok = in.ReadAll([&](const lh::usb::EndpointInfo& e) { out.WriteEndpoint(e); },
                             [&](const lh::usb::RawReport& r) {
                               if (r.hostTime > 0 && start < 0) start = r.hostTime;
                               if (r.hostTime > 0 && r.hostTime - start > seconds) { ++dropped; return; }
                               out.WriteReport(r.hostTime, r.ep, r.lastInBatch, r.data.data(), r.data.size());
                               ++kept;
                             });
  out.Close();
  if (!ok || !out.Healthy()) { fprintf(stderr, "trim failed\n"); return 1; }
  fprintf(stderr, "kept %zu reports (%.0f s), dropped %zu\n", kept, seconds, dropped);
  return 0;
}

int main(int argc, char** argv) {
  if (argc < 2) return Usage();
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);
  std::string cmd = argv[1];
  if (cmd == "track") return Track(argc, argv);
  if (cmd == "trim") return TrimCommand(argc, argv);
  if (cmd == "session" && argc >= 3) return SessionCommand(argc, argv);
  if (cmd == "haptics" && argc >= 3) return HapticsCommand(argc, argv);
  if (cmd == "configs" && argc >= 3) return ConfigsCommand(argc, argv);
  if (cmd == "capture" && argc >= 3) {
    StatsSink stats;
    lh::usb::UsbTransportOptions o;
    o.capturePath = argv[2];
    lh::usb::UsbTransport t(&stats, o);
    if (!t.Start()) {
      fprintf(stderr, "no devices\n");
      return 1;
    }
    for (auto& e : t.Endpoints())
      printf("ep%d %04x:%04x if%d kind=%d %s range g%d a%d config=%zuB\n", e.ep, e.vid, e.pid, e.iface, int(e.kind),
             e.serial.c_str(), e.gyroRange, e.accelRange, e.configJson.size());
    RunFor(argc >= 4 ? atof(argv[3]) : 0);
    t.Stop();
    stats.Print();
    return 0;
  }
  if (cmd == "inputs" && argc >= 3) {
    InputsSink sink;
    if (std::string(argv[2]) == "live") {
      lh::usb::UsbTransport t(&sink);
      if (!t.Start()) return 1;
      RunFor(argc >= 4 ? atof(argv[3]) : 10);
      t.Stop();
    } else {
      lh::usb::RawCaptureReader r;
      if (!r.Open(argv[2])) {
        fprintf(stderr, "cannot read %s\n", argv[2]);
        return 1;
      }
      lh::usb::ReportDecoder dec(&sink);
      r.ReadAll([&](const lh::usb::EndpointInfo& e) { dec.AddEndpoint(e); },
                [&](const lh::usb::RawReport& rep) { dec.OnReport(rep); });
    }
    for (const auto& [id, n] : sink.counts) printf("%s: %ld input reports\n", id.c_str(), n);
    return 0;
  }
  if (cmd == "stats" && argc >= 3) {
    StatsSink stats;
    if (std::string(argv[2]) == "live") {
      lh::usb::UsbTransport t(&stats);
      if (!t.Start()) return 1;
      RunFor(argc >= 4 ? atof(argv[3]) : 5);
      t.Stop();
    } else {
      lh::usb::RawCaptureReader r;
      if (!r.Open(argv[2])) {
        fprintf(stderr, "cannot read %s\n", argv[2]);
        return 1;
      }
      lh::usb::ReportDecoder dec(&stats);
      r.ReadAll([&](const lh::usb::EndpointInfo& e) { dec.AddEndpoint(e); },
                [&](const lh::usb::RawReport& rep) { dec.OnReport(rep); });
    }
    stats.Print();
    return 0;
  }
  return Usage();
}
