#include "telemetry_client.hpp"

#include "common/tracker_names.hpp"

#include <nlohmann/json.hpp>

namespace openhouse::viewer {
namespace {
using json = nlohmann::json;

template <size_t N>
std::array<double, N> Array(const json& j, std::array<double, N> fallback) {
  if (!j.is_array() || j.size() != N) return fallback;
  for (size_t i = 0; i < N; ++i) fallback[i] = j[i].get<double>();
  return fallback;
}
}  // namespace

void TelemetryClient::Disconnect() {
  net::Close(socket_);
  socket_ = net::kInvalid;
  connecting_ = false;
  helloed_ = haveState_ = false;
  buffer_.clear();
  state_ = {};
}

void TelemetryClient::Poll(double now) {
  if (socket_ == net::kInvalid) {
    if (now < nextAttempt_) return;
    nextAttempt_ = now + 1.0;
    socket_ = net::ConnectLoopback(port_);
    if (socket_ == net::kInvalid) return;
    connecting_ = true;
  }
  if (connecting_) {
    const int done = net::ConnectDone(socket_);
    if (done == 0) return;
    if (done < 0) { Disconnect(); return; }
    connecting_ = false;
  }
  char chunk[64 * 1024];
  for (int i = 0; i < 64; ++i) {  // bounded per frame; the rest waits for the next one
    const long n = net::RecvSome(socket_, chunk, sizeof chunk);
    if (n < 0) { Disconnect(); return; }
    if (n == 0) break;
    buffer_.append(chunk, size_t(n));
  }
  size_t start = 0;
  for (size_t nl; (nl = buffer_.find('\n', start)) != std::string::npos; start = nl + 1)
    Handle(buffer_.substr(start, nl - start));
  buffer_.erase(0, start);
  // An unfinished line far longer than any the driver sends: not our driver on the port.
  if (buffer_.size() > (16u << 20)) Disconnect();
}

uint64_t TelemetryClient::SendCommand(const std::string& text) {
  if (!Connected()) return 0;
  json j = json::parse(text, nullptr, false);
  if (!j.is_object()) return 0;
  const uint64_t id = nextCommand_++;
  j["type"] = "command";
  j["id"] = id;
  const std::string line = j.dump() + "\n";
  // Loopback and short: it goes at once (a full send buffer drops it, as a lost reply would).
  if (net::SendSome(socket_, line.data(), line.size()) != long(line.size())) return 0;
  return id;
}

void TelemetryClient::Handle(const std::string& line) {
  const json j = json::parse(line, nullptr, false);
  if (j.is_discarded() || !j.is_object()) return;
  const std::string type = j.value("type", "");
  if (type == "hello") {
    if (!helloed_) ++connectionGeneration_;
    helloed_ = true;
    host_ = j.value("host", "");
    version_ = j.value("version", "");
    pid_ = j.value("pid", 0);
  } else if (type == "log") {
    LogLine l;
    l.time = j.value("t", 0.0);
    lh::ParseLogLevel(j.value("level", "INFO"), &l.level);
    l.serial = j.value("serial", "");
    l.text = j.value("text", "");
    logs_.push_back(std::move(l));
    ++received_;
    while (logs_.size() > kMaxLogs) logs_.pop_front();
  } else if (type == "state") {
    State s;
    s.time = j.value("t", 0.0);
    if (const auto r = j.find("room"); r != j.end() && r->is_object()) {
      s.room.id = r->value("id", "");
      s.room.settled = r->value("settled", false);
      s.room.epoch = r->value("epoch", 0ull);
      s.room.settleSigma = r->value("settleSigma", 0.005);
      s.room.resets = r->value("resets", 0ull);
      s.room.learned = r->value("learned", 0);
      s.room.seen = r->value("seen", 0);
    }
    for (const auto& b : j.value("bases", json::array())) {
      Base x;
      x.serial = b.value("serial", "");
      x.channel = b.value("channel", "");
      x.mode = b.value("mode", 0);
      x.seen = b.value("seen", false);
      x.placed = b.value("placed", false);
      x.origin = b.value("origin", false);
      x.locked = b.value("locked", false);
      x.lh2 = b.value("lh2", false);
      x.channelObservedAt = b.value("channelObservedAt", -1.0);
      x.observations = b.value("observations", 0);
      x.sigma = b.value("sigma", 0.0);
      x.learned = b.value("learned", x.locked || x.sigma <= 0.005);
      x.faults = b.value("faults", 0);
      x.p = Array<3>(b.value("p", json()), x.p);
      x.q = Array<4>(b.value("q", json()), x.q);
      s.bases.push_back(std::move(x));
    }
    for (const auto& d : j.value("devices", json::array())) {
      Device x;
      x.serial = d.value("serial", "");
      x.role = d.value("role", "");
      x.name = lh::telemetry::TrackerDisplayName(j, x.serial);   // trackers from the overlay, any device from its dongle
      x.model = d.value("model", "");
      x.hand = d.value("hand", "");
      x.state = d.value("state", "");
      x.primary = d.value("primary", "");
      x.connected = d.value("connected", false);
      x.enabled = d.value("enabled", true);
      x.charging = d.value("charging", false);
      x.valid = d.value("valid", false);
      if (const auto b = d.find("battery"); b != d.end() && b->is_number()) x.battery = b->get<int>();
      x.p = Array<3>(d.value("p", json()), x.p);
      x.v = Array<3>(d.value("v", json()), x.v);
      x.q = Array<4>(d.value("q", json()), x.q);
      for (const auto& b : d.value("bases", json::array())) x.bases.push_back(b.get<std::string>());
      for (const auto& c : d.value("coverage", json::array()))
        x.coverage.push_back({c.value("base", ""), c.value("frames", 0), c.value("sensors", 0.0)});
      s.devices.push_back(std::move(x));
    }
    for (const auto& r : j.value("receivers", json::array())) {
      Receiver x;
      x.serial = r.value("serial", ""); x.connected = r.value("connected", ""); x.kind = r.value("kind", "watchman");
      x.slot = r.value("slot", -1); x.paired = r.value("paired", true); x.linked = r.value("linked", false);
      x.kBytesPerSec = r.value("kBytesPerSec", 0.0); x.framesPerSec = r.value("framesPerSec", 0.0);
      x.loss = r.value("loss", -1.0); x.drops = r.value("drops", 0.0);
      if (r.contains("rssi") && r["rssi"].is_array() && r["rssi"].size() == 2) {
        x.rssiDongle = r["rssi"][0].get<int>(); x.rssiDevice = r["rssi"][1].get<int>();
      }
      s.receivers.push_back(std::move(x));
    }
    if (const auto st = j.find("settings"); st != j.end() && st->is_object()) s.settingsJson = st->dump();
    state_ = std::move(s);
    haveState_ = true;
  } else if (type == "reply") {
    Reply r;
    r.id = j.value("id", 0ull);
    r.ok = j.value("ok", false);
    r.error = j.value("error", "");
    replies_.push_back(std::move(r));
    if (replies_.size() > 256) replies_.erase(replies_.begin());
  }
}

}  // namespace openhouse::viewer
