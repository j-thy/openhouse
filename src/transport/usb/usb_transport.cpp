#include <cstdarg>
#include <array>
#include "usb_transport.hpp"

#include <mutex>
#include "device_config.hpp"
#include "haptics.hpp"
#include "platform/platform.hpp"

#include <hidapi/hidapi.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <set>
#include <cstring>
#include <thread>

namespace lh::usb {

// A message for the sink (the session: its event log, the driver's log, the viewer).
void UsbTransport::Log(LogLevel level, const char* format, ...) {
  char text[512];
  va_list args;
  va_start(args, format);
  std::vsnprintf(text, sizeof text, format, args);
  va_end(args);
  std::string line = text;
  while (!line.empty() && line.back() == '\n') line.pop_back();
  sink_->OnTransportLog(level, line);
}
namespace {

// No wireless data for this long: a new radio epoch (the receiver may have missed the off notification).
constexpr double kRadioGap = 1.0;  // s
// #92: how long a receiver that doesn't say which device it has is asked again, and how often.
constexpr double kDeviceCheck = 3.0, kDeviceRecheck = 0.25;  // s

constexpr uint16_t kValveVid = 0x28de;

double HostNow() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

std::string Narrow(const wchar_t* w) {
  std::string s;
  if (!w) return s;
  for (; *w; ++w) s.push_back(char(*w));
  return s;
}

// Which endpoint kind a (pid, interface) pair is. Wired HMD/tracker: iface 0 = IMU,
// iface 1 = optical (verified on Index HMD 28de:2000, see ai/driver/usb_protocol.md).
bool Classify(uint16_t pid, int iface, EndpointKind& kind) {
  switch (pid) {
    // Wired receivers, as stock's device scan (bitmask 0x400040001 over 0x2000-0x2022, plus
    // 0x2300; the dongles 0x2101/0x2102 take the wireless path). Interface layout of the
    // others assumed as on 0x2000 (not yet verified on hardware).
    case 0x2000:  // Lighthouse FPGA RX (Index HMD, Vive Pro)
    case 0x2012:
    case 0x2022:
    case 0x2300:  // Bigscreen Beyond (SteamVR shows it as a Tundra device), Vive Tracker 2018/3.0
      if (iface == 0) { kind = EndpointKind::Imu; return true; }
      if (iface == 1) { kind = EndpointKind::Optical; return true; }
      // Buttons (libsurvive TRACKER0/1_BUTTONS, W_WATCHMAN1_BUTTONS): a tracker's power button
      // and pogo pins on USB. Not on 0x2000, whose iface 2 is the headset's own.
      if (iface == 2 && pid != 0x2000) { kind = EndpointKind::Buttons; return true; }
      return false;
    case 0x2101:  // Watchman dongle
    case 0x2102:  // Watchman radio / Kuroshin dongle
      if (iface == 0) { kind = EndpointKind::Wireless; return true; }
      return false;
    default:
      return false;
  }
}

}  // namespace

// ---------------------------------------------------------------- ReportDecoder

void ReportDecoder::AddEndpoint(const EndpointInfo& e) {
  auto& dev = devices_[e.serial];
  bool fresh = !dev;
  if (fresh) {
    dev = std::make_shared<DeviceState>();
    dev->id = e.serial;
  }
  if (e.gyroRange >= 0) dev->gyroRange = e.gyroRange;
  if (e.accelRange >= 0) dev->accelRange = e.accelRange;
  Ep ep;
  ep.info = e;
  if (e.kind == EndpointKind::Imu) ep.imu = std::make_unique<ImuReportParser>(dev, sink_);
  if (e.kind == EndpointKind::Optical) ep.optical = std::make_unique<OpticalReportParser>(dev, sink_);
  if (e.kind == EndpointKind::Wireless) ep.wireless = std::make_unique<WirelessReportParser>(dev, sink_);
  if (e.kind == EndpointKind::Buttons) ep.buttons = std::make_unique<ButtonsReportParser>(dev, sink_);
  eps_[e.ep] = std::move(ep);
  // The buttons interface carries no config: its device attaches with its IMU interface.
  // Announced when it's new, carries a config, or is the device's only endpoint again after
  // an unplug (a replugged device without a readable config was otherwise never re-attached, #49.3).
  const bool alone = std::none_of(eps_.begin(), eps_.end(), [&](const auto& other) {
    return other.first != e.ep && other.second.info.serial == e.serial;
  });
  if (e.kind != EndpointKind::Buttons && (fresh || alone || !e.configJson.empty())) sink_->OnDeviceAttached(e.serial, e.configJson);
}

void ReportDecoder::ResetWireless(Ep& ep) {
  if (ep.connected) sink_->OnDeviceDetached(ep.info.serial);
  auto dev = std::make_shared<DeviceState>();
  dev->id = ep.info.serial;
  dev->gyroRange = ep.info.gyroRange;
  dev->accelRange = ep.info.accelRange;
  devices_[ep.info.serial] = dev;
  ep.wireless = std::make_unique<WirelessReportParser>(dev, sink_);
  if (ep.versioning) ep.wireless->SetPendingVersionRequest();
  ep.connected = false;
  ep.lastWirelessReport = 0;
  ++ep.generation;
}

void ReportDecoder::OnReport(const RawReport& r) { OnReport(r.hostTime, r.ep, r.lastInBatch, r.data.data(), r.data.size()); }

void ReportDecoder::OnReport(double hostTime, uint16_t epId, bool last, const uint8_t* data, size_t len) {
  auto it = eps_.find(epId);
  if (it == eps_.end()) return;
  Ep& ep = it->second;
  if (ep.imu) ep.imu->Parse(hostTime, data, len, last);
  else if (ep.optical) ep.optical->Parse(hostTime, data, len);
  else if (ep.buttons) ep.buttons->Parse(hostTime, data, len);
  else if (ep.wireless && len) {
    if (data[0] == 0x26) { ResetWireless(ep); return; }
    if (data[0] != 0x23 && data[0] != 0x24) return;
    // Some receivers miss the off notification. A radio data gap is an epoch
    // boundary: discard protocol substate, IMU dedup timestamps and clock fit.
    if (ep.lastWirelessReport && hostTime - ep.lastWirelessReport > kRadioGap) ResetWireless(ep);
    if (!ep.connected) {
      sink_->OnDeviceAttached(ep.info.serial, ep.info.configJson);
      ep.connected = true;
    }
    ep.lastWirelessReport = hostTime;
    ep.wireless->Parse(hostTime, data, len, last);
  }
}

// ---------------------------------------------------------------- UsbTransport

UsbTransport::UsbTransport(IInputSink* sink, UsbTransportOptions opts)
    : sink_(sink), opts_(std::move(opts)), decoder_(sink) {
  if (opts_.steamLighthouseConfigDir.empty()) {
    opts_.steamLighthouseConfigDir = platform::SteamConfigDirectory();
  }
}

UsbTransport::~UsbTransport() { Stop(); }

std::string UsbTransport::LoadSteamConfig(const std::string& dir, const std::string& serial) {
  std::string lower = serial;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return std::tolower(c); });
  std::ifstream f(dir + "/" + lower + "/config.json");
  if (!f) return {};
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

bool UsbTransport::FetchRangeModes(hid_device_* dev, int8_t& gyro, int8_t& accel) {
  // 0x3ce260: feature report 0x01 -> [id][gyroRange][accelRange]
  for (int attempt = 0; attempt < 100; ++attempt) {
    unsigned char buf[65] = {0x01};
    const int n = hid_get_feature_report(dev, buf, sizeof buf);
    if (n >= 3 && buf[0] == 0x01 && buf[1] < 4 && buf[2] < 4) {
      gyro = int8_t(buf[1]); accel = int8_t(buf[2]); return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

std::optional<uint32_t> UsbTransport::ConnectedSerial(hid_device_* receiver) {
  DeviceInfo di;
  if (!QueryDeviceInfo(receiver, &di, 2)) return std::nullopt;
  return di.serial;
}

// #92: whether the device on a receiver is no longer the one it was set up for. A dongle can be
// paired with several devices and serves whichever is on (one at a time), staying plugged in; it was
// identified only when opened, so a 3.0 tracker switched on in place of a 2018 one was taken for it,
// with the 2018's sensor layout. Asked only after a disconnect or a radio gap (Run asks again while
// the receiver doesn't answer). Runs on the probe thread, on a second handle to the receiver: the
// reader thread's handle stays its own. 1: another device, 0: the same one, -1: no answer.
int UsbTransport::OtherDeviceConnected(const std::string& path, const std::string& serial, const std::string& receiver) {
  hid_device* h = nullptr;
  {
    static std::mutex openMutex;  // hid_open_path's error string is process-wide (#49.5)
    std::lock_guard openLock(openMutex);
    h = hid_open_path(path.c_str());
  }
  if (!h) return -1;
  DeviceInfo di;
  const bool ok = QueryDeviceInfo(h, &di, 2) && di.serial != 0;
  hid_close(h);
  if (!ok) return -1;
  char sn[16];
  snprintf(sn, sizeof sn, "LHR-%08X", di.serial);
  if (serial == sn) return 0;
  Log(LogLevel::Info, "%s: receiver %s now has %s; reopening it to set that device up\n", serial.c_str(), receiver.c_str(), sn);
  return 1;
}

void UsbTransport::RunDeviceChecks() {
  std::vector<std::shared_ptr<DeviceCheck>> checks;
  {
    std::lock_guard lock(hapticMutex_);
    checks = deviceChecks_;
  }
  for (auto& c : checks) {
    if (stop_) return;
    const int result = OtherDeviceConnected(c->path, c->serial, c->receiver);
    std::lock_guard lock(hapticMutex_);
    c->result = result;
    std::erase(deviceChecks_, c);
  }
}

bool UsbTransport::QueryDeviceInfo(hid_device_* dev, DeviceInfo* info, int attempts) {
  // 0x3dd8a0: feature 0xff [0x83] then read back a TLV list of 5-byte entries
  // {u8 type; u32 value}; type 2 bit 3 = wireless protocol versioning supported.
  unsigned char req[64] = {0xff, 0x83};
  unsigned char buf[64];
  int n = -1;
  for (int attempt = 0; attempt < attempts; attempt++) {
    if (hid_send_feature_report(dev, req, sizeof req) < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }
    for (int k = 0; k < 10; k++) {
      std::memset(buf, 0, sizeof buf);
      buf[0] = 0xff;
      n = hid_get_feature_report(dev, buf, sizeof buf);
      if (n >= 3 && buf[1] == 0x83 && buf[2] != 0) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (n >= 3 && buf[1] == 0x83 && buf[2] != 0) break;
  }
  if (n < 3 || buf[1] != 0x83) return false;
  const int count = buf[2] / 5;
  for (int i = 0; i < count && 3 + 5 * i + 5 <= n; i++) {
    const unsigned char* e = buf + 3 + 5 * i;
    const uint32_t v = uint32_t(e[1]) | uint32_t(e[2]) << 8 | uint32_t(e[3]) << 16 | uint32_t(e[4]) << 24;
    if (e[0] == 0) info->serial = v;                       // radio-side device serial
    if (e[0] == 2) info->supportsVersioning = (v >> 3) & 1;
  }
  return true;
}

bool UsbTransport::RequestProtocolVersion(hid_device_* dev, int version) {
  // 0x3cc9a0: feature 0xff [0x87, 0x03, 0x00, u16 version]
  unsigned char req[64] = {0xff, 0x87, 0x03, 0x00, uint8_t(version), uint8_t(version >> 8)};
  return hid_send_feature_report(dev, req, sizeof req) >= 0;
}

UsbTransport::OpenResult UsbTransport::OpenEndpoint(const hid_device_info* d, Open* out) {
  // One open at a time: hidapi's hid_open_path resets and sets a process-wide error string,
  // which parallel opens would free twice (#49.5). The config read after it stays parallel.
  static std::mutex openMutex;
  hid_device* h = nullptr;
  {
    std::lock_guard openLock(openMutex);
    h = hid_open_path(d->path);
  }
  if (!h) {
    std::lock_guard lock(pathsMutex_);
    if (failedPaths_.insert(d->path).second) Log(LogLevel::Warn, "cannot open %s\n", d->path);
    return OpenResult::Failed;
  }
  {
    std::lock_guard lock(pathsMutex_);
    failedPaths_.erase(d->path);
  }
  hid_set_nonblocking(h, 1);
  EndpointKind kind{};
  Classify(d->product_id, d->interface_number, kind);
  Open& o = *out;
  o = Open{};
  o.dev = h;
  o.path = d->path;
  o.info.vid = d->vendor_id;
  o.info.pid = d->product_id;
  o.info.iface = uint8_t(d->interface_number);
  o.info.kind = kind;
  o.info.serial = Narrow(d->serial_number);
  const double t0 = HostNow();
  DeviceInfo di;
  // Short query: a receiver without a device keeps answering "nothing yet".
  const bool haveInfo = kind == EndpointKind::Wireless && QueryDeviceInfo(h, &di, 1);
  Log(LogLevel::Debug, "%s pid %04x iface %d: device info %s serial %08X in %.0f ms\n", o.info.serial.c_str(), o.info.pid,
            o.info.iface, haveInfo ? "ok" : "failed", di.serial, (HostNow() - t0) * 1000);
  if (kind == EndpointKind::Wireless && (!haveInfo || di.serial == 0)) {
    // No device on this receiver (yet): reading its range modes and config would
    // only time out (~1.5 s each). The probe thread picks the device up later,
    // like the stock driver's "Connected to receiver".
    o.receiver = o.info.serial;
    SetReceiver(o, "");
    return OpenResult::Idle;
  }
  o.info.ep = nextEp_++;
  if (haveInfo && di.serial != 0) {
    char sn[16];
    snprintf(sn, sizeof sn, "LHR-%08X", di.serial);
    o.receiver = o.info.serial;
    o.info.serial = sn;  // device id = tracked device serial, not the dongle's
    SetReceiver(o, sn);
  }
  CompleteSetup(o, haveInfo && di.supportsVersioning);
  Log(LogLevel::Debug, "%s set up in %.0f ms\n", o.info.serial.c_str(), (HostNow() - t0) * 1000);
  return OpenResult::Ready;
}

void UsbTransport::Enumerate(bool startup) {
  hid_device_info* list = hid_enumerate(kValveVid, 0);
  std::set<std::string> present;
  std::vector<hid_device_info*> fresh;
  for (hid_device_info* d = list; d && !stop_; d = d->next) {
    EndpointKind kind;
    if (!Classify(d->product_id, d->interface_number, kind)) continue;
    present.insert(d->path);
    std::lock_guard lock(pathsMutex_);
    if (!paths_.count(d->path)) fresh.push_back(d);
  }
  // Opened in parallel: each reads its device's config (~0.4 s, wireless ~0.5 s), and a
  // host only accepts devices for a few seconds after Init (Monado: 3 s).
  std::vector<Open> opened(fresh.size());
  std::vector<OpenResult> results(fresh.size(), OpenResult::Failed);
  {
    std::vector<std::thread> workers;
    for (size_t i = 0; i < fresh.size(); ++i)
      workers.emplace_back([&, i] { results[i] = OpenEndpoint(fresh[i], &opened[i]); });
    for (auto& w : workers) w.join();
  }
  // Every opened handle goes to a list Stop() closes, stopping or not (#49.4).
  for (size_t i = 0; i < fresh.size(); ++i) {
    Open& o = opened[i];
    const OpenResult result = results[i];
    if (result == OpenResult::Failed) continue;
    {
      std::lock_guard lock(pathsMutex_);
      paths_.insert(o.path);
    }
    if (!startup) Log(LogLevel::Info, "%s: plugged in (%s)\n", o.info.serial.c_str(), o.path.c_str());
    if (result == OpenResult::Idle) { idle_.push_back(o); continue; }
    if (startup) {
      capture_.WriteEndpoint(o.info);
      decoder_.AddEndpoint(o.info);
      if (o.wantVersion) RequestVersion(o);
      open_.push_back(o);
    } else {
      std::lock_guard lock(pendingMutex_);
      pending_.push_back(o);
      havePending_ = true;
    }
  }
  hid_free_enumeration(list);
  if (startup) return;
  // A receiver unplugged while it had no device: nothing reads it, so notice here.
  for (size_t i = 0; i < idle_.size();) {
    if (present.count(idle_[i].path)) { ++i; continue; }
    Log(LogLevel::Info, "receiver %s unplugged\n", idle_[i].receiver.c_str());
    DropReceiver(idle_[i].receiver);
    hid_close(idle_[i].dev);
    {
      std::lock_guard lock(pathsMutex_);
      paths_.erase(idle_[i].path);
    }
    idle_.erase(idle_.begin() + long(i));
  }
}

void UsbTransport::SetCapture(const std::string& path) {
  std::lock_guard lock(captureMutex_);
  captureRequest_ = path;
  haveCaptureRequest_ = true;
}

bool UsbTransport::Start() {
  if (thread_.joinable() || !open_.empty()) return false;
  decoder_ = ReportDecoder(sink_);  // no stale unwrap/protocol state after Stop/Start
  if (hid_init() != 0) return false;
  if (!opts_.capturePath.empty() && !capture_.Open(opts_.capturePath)) {
    Log(LogLevel::Error, "cannot open capture %s\n", opts_.capturePath.c_str());
    return false;
  }
  nextEp_ = 0;
  stop_ = false;
  Enumerate(true);
  bool unopenable = false;
  {
    std::lock_guard lock(pathsMutex_);
    unopenable = !failedPaths_.empty();
  }
  // Interfaces that exist but can't be opened are held by someone else (SteamVR):
  // fail. With none plugged in at all, run and wait for one (hotplug).
  if (open_.empty() && idle_.empty() && unopenable) return false;
  if (open_.empty() && idle_.empty()) Log(LogLevel::Info, "no Lighthouse receivers yet; waiting for one to be plugged in\n");
  thread_ = std::thread(&UsbTransport::Run, this);
  probe_ = std::thread(&UsbTransport::ProbeMain, this);
  return true;
}

namespace {
// 0x3cd640/0x3e0d90: the sensor type is the low byte of the "hardware id" u32 in
// the firmware version report (feature 0x05; byte 41 with the report id at [0]).
// Receivers relay the request over the radio, so early reads fail; stock retries
// up to 100 times. Valid when the build timestamp at [1..4] and the id are
// non-zero and the type is <= 11. Returns -1 otherwise.
int ReadSensorType(hid_device* dev) {
  for (int attempt = 0; attempt < 100; ++attempt) {
    unsigned char b[65] = {0x05};
    const int n = hid_get_feature_report(dev, b, sizeof b);
    if (n >= 45) {
      uint32_t build = 0, hwid = 0;
      std::memcpy(&build, b + 1, 4);
      std::memcpy(&hwid, b + 41, 4);
      if (!build || !hwid || (hwid & 0xff) > 11) return -1;
      return int(hwid & 0xff);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return -1;
}

// Records the hardware sensor type in the device JSON (TrackedObjectModel reads
// "sensor_type"); captures keep it with the endpoint config.
void AddSensorType(std::string* json, int type) {
  const size_t brace = json->find('{');
  if (brace == std::string::npos || json->find("\"sensor_type\"") != std::string::npos) return;
  json->insert(brace + 1, "\"sensor_type\":" + std::to_string(type) + ",");
}
}  // namespace

void UsbTransport::CompleteSetup(Open& o, bool supportsVersioning) {
  if (o.info.kind == EndpointKind::Buttons) return;  // input only: no sensor type, ranges or config
  if (o.info.kind != EndpointKind::Optical) {
    o.sensorType = ReadSensorType(o.dev);
      Log(LogLevel::Debug, "%s: hardware sensor type %s\n", o.info.serial.c_str(),
              o.sensorType < 0 ? "unavailable (inferring from sync widths)" : std::to_string(o.sensorType).c_str());
  }
  if (o.info.kind != EndpointKind::Optical && !FetchRangeModes(o.dev, o.info.gyroRange, o.info.accelRange))
    Log(LogLevel::Warn, "%s: unable to fetch gyro/accelerometer range modes\n", o.info.serial.c_str());
  // The device's own config, read once per session (as the stock driver: a config
  // rewritten with lighthouse_console takes effect on the next start); SteamVR's cached
  // copy only if the device can't be read. LH_PREFER_STEAM_CONFIG=1: the cache first.
  if (o.info.kind != EndpointKind::Optical) {
    {
      std::lock_guard lock(configMutex_);
      if (const auto it = sessionConfigs_.find(o.info.serial); it != sessionConfigs_.end()) o.info.configJson = it->second;
    }
    static const bool cacheFirst = getenv("LH_PREFER_STEAM_CONFIG") != nullptr;
    if (o.info.configJson.empty() && cacheFirst) o.info.configJson = LoadSteamConfig(opts_.steamLighthouseConfigDir, o.info.serial);
    if (o.info.configJson.empty() && opts_.fetchConfigIfMissing) {
      std::string error;
      const double t0 = HostNow();
      if (ReadDeviceConfig([&](uint8_t* data, size_t size) { return hid_get_feature_report(o.dev, data, size); },
                           &o.info.configJson, &error)) {
        Log(LogLevel::Info, "%s: read device config (%lu bytes) in %.0f ms\n", o.info.serial.c_str(), (unsigned long)o.info.configJson.size(),
                (HostNow() - t0) * 1000);
        std::lock_guard lock(configMutex_);
        sessionConfigs_[o.info.serial] = o.info.configJson;
      } else {
        Log(LogLevel::Warn, "%s: device config unavailable (%s); using SteamVR's cached copy\n", o.info.serial.c_str(), error.c_str());
      }
    }
    if (o.info.configJson.empty()) o.info.configJson = LoadSteamConfig(opts_.steamLighthouseConfigDir, o.info.serial);
  }
  if (o.sensorType >= 0 && !o.info.configJson.empty()) AddSensorType(&o.info.configJson, o.sensorType);
  o.wantVersion = supportsVersioning && (o.info.pid == 0x2101 || o.info.pid == 0x2102);
}

void UsbTransport::RequestVersion(Open& o) {
  wirelessProtocol_ = 1;
  if (!o.versionRequestTime)
    Log(LogLevel::Debug, "%s: protocol versioning supported, requesting %d\n", o.info.serial.c_str(), wirelessProtocol_);
  decoder_.SetVersionNegotiation(o.info.ep);
  decoder_.Wireless(o.info.ep)->SetPendingVersionRequest();
  o.versionRequestTime = HostNow();  // retry even if the first HID write fails
  RequestProtocolVersion(o.dev, wirelessProtocol_);
}

void UsbTransport::SetReceiver(const Open& o, const std::string& connected) {
  std::lock_guard lock(receiversMutex_);
  receivers_[o.receiver] = Receiver{o.receiver, connected, o.info.pid};
}

void UsbTransport::DropReceiver(const std::string& serial) {
  std::lock_guard lock(receiversMutex_);
  receivers_.erase(serial);
}

std::vector<UsbTransport::Receiver> UsbTransport::Receivers() const {
  std::lock_guard lock(receiversMutex_);
  std::vector<Receiver> out;
  for (const auto& [serial, r] : receivers_) out.push_back(r);
  return out;
}

void UsbTransport::OnDeviceGone(const DeviceId& serial) {
  std::lock_guard lock(receiversMutex_);
  for (auto& [id, r] : receivers_) {
    if (r.connected == serial) r.connected.clear();
  }
}

bool UsbTransport::SetPairing(const std::string& receiver, int milliseconds) {
  {
    std::lock_guard lock(receiversMutex_);
    if (!receivers_.count(receiver)) return false;
  }
  std::lock_guard lock(hapticMutex_);
  // A newer command replaces a pending one: only the last on or off counts, and a backlog
  // would reach the receiver late (an "on" after a stop, or an old "off" in a new pairing).
  std::erase_if(pairing_, [&](const PairCommand& c) { return c.receiver == receiver; });
  pairing_.push_back({receiver, std::clamp(milliseconds, 0, 0xffff)});
  return true;
}

void UsbTransport::KeepAlive(Open& o) {
  // A VIVE Tracker 2018 didn't pair with a dongle we kept in pairing mode for over a minute,
  // and did with lighthouse_console, which sends this to a quiet dongle (as stock does).
  const auto report = ReceiverKeepaliveReport();
  const int sent = hid_send_feature_report(o.dev, report.data(), report.size()) == int(report.size()) ? 1 : 0;
  if (sent != o.keepalive)
    Log(LogLevel::Info, "receiver %s: keepalive %s\n", o.receiver.c_str(), sent ? "sent" : "failed");
  o.keepalive = sent;
}

bool UsbTransport::TakePairing(Open& o) {
  if (o.receiver.empty()) return false;
  PairCommand command;
  {
    std::lock_guard lock(hapticMutex_);
    const auto it = std::find_if(pairing_.begin(), pairing_.end(), [&](const PairCommand& c) { return c.receiver == o.receiver; });
    if (it == pairing_.end()) return false;
    command = *it;
    pairing_.erase(it);
  }
  std::array<uint8_t, 64> report{};
  report[0] = 0xff;
  report[1] = 0xad;
  report[2] = 0x03;
  report[3] = command.milliseconds > 0 ? 1 : 0;
  report[4] = uint8_t(command.milliseconds & 0xff);
  report[5] = uint8_t(command.milliseconds >> 8);
  const int n = hid_send_feature_report(o.dev, report.data(), report.size());
  Log(LogLevel::Info, "receiver %s: pairing %s (%d ms): %s\n", o.receiver.c_str(), command.milliseconds ? "on" : "off",
          command.milliseconds, n == int(report.size()) ? "sent" : "failed");
  return n == int(report.size()) && command.milliseconds > 0;
}

std::string UsbTransport::BluetoothReceiver() const {
  std::lock_guard lock(receiversMutex_);
  for (const auto& [serial, r] : receivers_)
    if (IsBluetoothReceiver(r.pid, serial)) return serial;
  return {};
}

bool UsbTransport::HasBluetoothReceiver() const { return !BluetoothReceiver().empty(); }

std::optional<RadioMode> UsbTransport::RunRadioCommand(const std::string& receiver, std::optional<RadioMode> set) {
  auto command = std::make_shared<RadioCommand>();
  command->receiver = receiver;
  command->set = set;
  auto result = command->result.get_future();
  {
    std::lock_guard lock(hapticMutex_);
    radioCommands_.push_back(command);
  }
  if (result.wait_for(std::chrono::seconds(2)) == std::future_status::ready) return result.get();
  // The receiver went away, so nothing will take it.
  std::lock_guard lock(hapticMutex_);
  radioCommands_.erase(std::remove(radioCommands_.begin(), radioCommands_.end(), command), radioCommands_.end());
  return std::nullopt;
}

void UsbTransport::TakeRadioCommands(Open& o) {
  if (o.receiver.empty()) return;
  std::vector<std::shared_ptr<RadioCommand>> commands;
  {
    std::lock_guard lock(hapticMutex_);
    for (auto it = radioCommands_.begin(); it != radioCommands_.end();) {
      if ((*it)->receiver != o.receiver) { ++it; continue; }
      commands.push_back(*it);
      it = radioCommands_.erase(it);
    }
  }
  for (const auto& command : commands) {
    std::optional<RadioMode> answer;
    if (command->set) {
      const auto report = RadioModeReport(*command->set);
      if (hid_send_feature_report(o.dev, report.data(), report.size()) == int(report.size())) {
        answer = command->set;
        o.bluetooth = *command->set == RadioMode::Bluetooth;
      }
      Log(LogLevel::Debug, "receiver %s: radio to %s: %s\n", o.receiver.c_str(),
          *command->set == RadioMode::Bluetooth ? "Bluetooth" : "Valve's", answer ? "sent" : "failed");
    } else {
      const auto query = RadioModeQuery();
      if (hid_send_feature_report(o.dev, query.data(), query.size()) == int(query.size())) {
        std::array<uint8_t, 64> reply{};
        reply[0] = 0xff;
        const int n = hid_get_feature_report(o.dev, reply.data(), reply.size());
        answer = ParseRadioMode(reply.data(), n);
      }
    }
    command->result.set_value(answer);
  }
}

bool UsbTransport::SetBluetoothRadio(bool on) {
  const std::string receiver = BluetoothReceiver();
  if (receiver.empty()) return false;
  if (!on) return RunRadioCommand(receiver, RadioMode::Valve).has_value();
  bool ok = false;
  if (RunRadioCommand(receiver, RadioMode::Bluetooth)) {
    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
      if (attempt) std::this_thread::sleep_for(std::chrono::milliseconds(10));
      ok = RunRadioCommand(receiver, std::nullopt) == RadioMode::Bluetooth;
    }
  }
  if (!ok) {
    Log(LogLevel::Warn, "receiver %s: its radio didn't switch to Bluetooth\n", receiver.c_str());
    RunRadioCommand(receiver, RadioMode::Valve);
  }
  return ok;
}

std::optional<bool> UsbTransport::BluetoothRadioOn() {
  const std::string receiver = BluetoothReceiver();
  if (receiver.empty()) return std::nullopt;
  const auto mode = RunRadioCommand(receiver, std::nullopt);
  if (!mode) return std::nullopt;
  return *mode == RadioMode::Bluetooth;
}

void UsbTransport::ProbeMain() {
  // Someone is waiting for each radio or pairing command, so they don't wait for the slow
  // parts below (a pass takes over a second).
  const auto serve = [this] {
    for (auto& o : idle_) {
      TakeRadioCommands(o);
      TakePairing(o);
    }
    RunDeviceChecks();
  };
  while (!stop_) {
    Enumerate(false);
    for (size_t i = 0; i < idle_.size() && !stop_;) {
      serve();
      Open& o = idle_[i];
      if (o.bluetooth) { ++i; continue; }  // nothing can connect to it for now
      KeepAlive(o);
      DeviceInfo di;
      if (!QueryDeviceInfo(o.dev, &di, 2) || di.serial == 0) { ++i; continue; }
      char sn[16];
      snprintf(sn, sizeof sn, "LHR-%08X", di.serial);
      o.info.serial = sn;
      Log(LogLevel::Info, "%s: connected to receiver %s\n", sn, o.receiver.c_str());
      SetReceiver(o, sn);
      o.info.ep = nextEp_++;
      CompleteSetup(o, di.supportsVersioning);
      {
        std::lock_guard lock(pendingMutex_);
        pending_.push_back(o);
      }
      havePending_ = true;
      idle_.erase(idle_.begin() + long(i));
    }
    for (int k = 0; k < 10 && !stop_; ++k) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      serve();
    }
  }
}

void UsbTransport::Stop() {
  stop_ = true;
  if (probe_.joinable()) probe_.join();
  if (thread_.joinable()) thread_.join();
  for (auto& o : idle_) hid_close(o.dev);
  idle_.clear();
  for (auto& o : pending_) hid_close(o.dev);
  pending_.clear();
  std::set<DeviceId> detached;
  for (auto& o : open_) { hid_close(o.dev); if (!o.disconnected) detached.insert(o.info.serial); }
  open_.clear();
  {
    std::lock_guard lock(pathsMutex_);
    paths_.clear();
    failedPaths_.clear();
  }
  for (const auto& id : detached) sink_->OnDeviceDetached(id);
  capture_.Close();
}

std::vector<EndpointInfo> UsbTransport::Endpoints() const {
  std::vector<EndpointInfo> v;
  for (auto& o : open_) v.push_back(o.info);
  return v;
}

void UsbTransport::Haptic(const DeviceId& serial, double frequency, double amplitude, double duration) {
  std::lock_guard lock(hapticMutex_);
  // Coalesce by device: a newer command replaces a pending one.
  for (auto it = haptics_.begin(); it != haptics_.end();) {
    if (it->serial == serial && it->powerOff) return;
    // Only pending pulses: a queued light-mode command must still go out (#49.2).
    if (it->serial == serial && it->opticalMode < 0) it = haptics_.erase(it); else ++it;
  }
  if (haptics_.size() < 64) haptics_.push_back({serial, HapticReport(frequency, amplitude, duration), HostNow() + 0.1});
}

void UsbTransport::PowerOff(const DeviceId& serial) {
  std::lock_guard lock(hapticMutex_);
  std::erase_if(haptics_, [&](const auto& command) { return command.serial == serial; });
  if (haptics_.size() < 64) haptics_.push_back({serial, PowerOffReport(), HostNow() + 1.0, true});
}

void UsbTransport::SetOpticalSubProtocol(const DeviceId& serial, int sub) {
  std::lock_guard lock(hapticMutex_);
  if (haptics_.size() < 64) haptics_.push_back({serial, OpticalSubProtocolReport(sub), HostNow() + 1.0, false, sub});
}

void UsbTransport::Run() {
  // Mirrors CLighthouseUsbThread::Run: poll each endpoint for up to 10 reports
  // per pass; sleep 1 ms if a pass took < 0.5 ms.
  struct Rep { unsigned char buf[64]; int n; double t; };
  Rep reps[10];
  while (!stop_) {
    const double passStart = HostNow();
    if (havePending_) {
      std::vector<Open> ready;
      {
        std::lock_guard lock(pendingMutex_);
        ready.swap(pending_);
        havePending_ = false;
      }
      for (auto& o : ready) {
        capture_.WriteEndpoint(o.info);
        decoder_.AddEndpoint(o.info);
        if (o.wantVersion) RequestVersion(o);
        open_.push_back(o);
      }
    }
    if (haveCaptureRequest_) {
      std::optional<std::string> path;
      {
        std::lock_guard lock(captureMutex_);
        path.swap(captureRequest_);
        haveCaptureRequest_ = false;
      }
      if (path) {
        capture_.Close();
        if (!path->empty()) {
          if (!capture_.Open(*path)) Log(LogLevel::Error, "cannot open capture %s\n", path->c_str());
          else for (const auto& o : open_) capture_.WriteEndpoint(o.info);
        }
      }
    }
    std::vector<HapticCommand> commands;
    { std::lock_guard lock(hapticMutex_); commands.swap(haptics_); }
    for (const auto& command : commands) {
      DeviceCommandResult result{command.serial, command.powerOff, "not-connected", 0, command.report, command.opticalMode};
      if (passStart > command.deadline) result.status = "expired";
      else for (auto& o : open_) {
        if (o.disconnected || o.info.serial != command.serial) continue;
        if (o.info.kind != EndpointKind::Wireless) {
          if (command.opticalMode < 0 || o.info.kind != EndpointKind::Imu) continue;
          const auto mode = WiredModeReport(command.opticalMode), verbosity = WiredVerbosityReport();
          result.bytes = hid_send_feature_report(o.dev, mode.data(), mode.size());
          if (result.bytes == int(mode.size())) hid_send_feature_report(o.dev, verbosity.data(), verbosity.size());
          result.status = result.bytes == int(mode.size()) ? "written" : "write-failed";
          break;
        }
        result.bytes = hid_send_feature_report(o.dev, command.report.data(), command.report.size());
        result.status = result.bytes == int(command.report.size()) ? "written" : "write-failed";
        break;
      }
      if (opts_.onCommandResult) opts_.onCommandResult(result);
      else if (std::string(result.status) != "written")
        Log(LogLevel::Debug, "%s: %s %s (%d bytes)\n", command.serial.c_str(), command.powerOff ? "power-off" : command.opticalMode >= 0 ? "light mode" : "haptic", result.status, result.bytes);
    }
    for (auto& o : open_) {
      // Pairing a receiver that has a device: the device is replaced, so the receiver
      // is reopened (as one with no device) and the new one picked up when it connects.
      if (!o.disconnected && TakePairing(o)) {
        // Pairing takes the receiver from its device: report it gone now, as the endpoint
        // is closed below in this pass and nothing would say so later (#49.1).
        o.disconnected = true;
        sink_->OnDeviceDetached(o.info.serial);
      }
      if (!o.disconnected) TakeRadioCommands(o);
    }
    for (auto& o : open_) {
      if (o.disconnected) continue;
      // Re-request the wireless protocol if no transition packet arrived (0x3cca50 retry).
      if (o.versionRequestTime != 0) {
        auto* w = decoder_.Wireless(o.info.ep);
        if (w && w->ProtocolVersion() != 2) o.versionRequestTime = 0;
        else if (passStart - o.versionRequestTime > 0.5) RequestVersion(o);
      }
      int count = 0;
      for (; count < 10; count++) {
        reps[count].n = hid_read(o.dev, reps[count].buf, sizeof reps[count].buf);
        reps[count].t = HostNow();
        if (reps[count].n <= 0) {
          if (reps[count].n < 0) o.disconnected = true;
          break;
        }
      }
      for (int i = 0; i < count; i++) {
        const bool last = i == count - 1;
        if (o.info.kind == EndpointKind::Wireless) {
          // #92: after a disconnect or a radio gap, the first data may be another device paired to this
          // receiver. Then it's reopened as one that has no device, as after pairing, and the new device
          // set up when the probe picks it up; its data isn't decoded as the old one's meanwhile.
          const uint8_t id = reps[i].buf[0];
          const bool data = id == 0x23 || id == 0x24;
          // A receiver that doesn't answer is asked again, every kDeviceRecheck for kDeviceCheck: the
          // check without that missed a switch live. The probe thread asks (RunDeviceChecks); this
          // thread only queues the check and reads its answer, so it never waits on the receiver.
          if (id == 0x26 || (data && o.lastData > 0 && reps[i].t - o.lastData > kRadioGap)) {
            o.checkDevice = true;
            o.checkUntil = reps[i].t + kDeviceCheck;
            o.nextCheck = 0;
            // An answer asked for before this may be about the device that was there before.
            if (o.checkPending) {
              o.checkPending = false;
              runningChecks_.erase(o.path);
            }
          }
          if (o.checkPending) {
            std::optional<int> other;
            {
              std::lock_guard lock(hapticMutex_);
              other = runningChecks_[o.path]->result;
            }
            if (other) {
              o.checkPending = false;
              runningChecks_.erase(o.path);
              if (*other >= 0 || reps[i].t > o.checkUntil) o.checkDevice = false;
              else o.nextCheck = reps[i].t + kDeviceRecheck;
              if (*other == 1) {
                o.disconnected = true;
                break;
              }
            }
          }
          if (data && o.checkDevice && !o.checkPending && reps[i].t >= o.nextCheck) {
            auto check = std::make_shared<DeviceCheck>();
            check->path = o.path;
            check->serial = o.info.serial;
            check->receiver = o.receiver;
            runningChecks_[o.path] = check;
            o.checkPending = true;
            std::lock_guard lock(hapticMutex_);
            deviceChecks_.push_back(check);
          }
          if (data) o.lastData = reps[i].t;
        }
        capture_.WriteReport(reps[i].t, o.info.ep, last, reps[i].buf, size_t(reps[i].n));
        decoder_.OnReport(reps[i].t, o.info.ep, last, reps[i].buf, size_t(reps[i].n));
        if (o.info.kind == EndpointKind::Wireless && decoder_.WirelessGeneration(o.info.ep) != o.generation) {
          o.generation = decoder_.WirelessGeneration(o.info.ep);
          Log(LogLevel::Info, "%s: radio epoch changed; resetting protocol and timestamp state\n", o.info.serial.c_str());
          if (o.wantVersion) RequestVersion(o);
        }
      }
      if (o.disconnected) sink_->OnDeviceDetached(o.info.serial);
    }
    // Unplugged: close it and forget its path, so the probe thread reopens it when it
    // comes back (as a new endpoint; the device keeps its id).
    for (auto it = open_.begin(); it != open_.end();) {
      if (!it->disconnected) { ++it; continue; }
      Log(LogLevel::Info, "%s: unplugged (%s)\n", it->info.serial.c_str(), it->path.c_str());
      if (!it->receiver.empty()) DropReceiver(it->receiver);
      hid_close(it->dev);
      decoder_.RemoveEndpoint(it->info.ep);
      {
        std::lock_guard lock(pathsMutex_);
        paths_.erase(it->path);
      }
      it = open_.erase(it);
    }
    if (HostNow() - passStart < 0.0005) platform::SleepFor(std::chrono::milliseconds(1));
  }
}

}  // namespace lh::usb
