// The viewer's connection to a running driver (lh/telemetry.hpp) and the model it
// fills: the latest state and every log line received.
#pragma once
#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "common/net.hpp"
#include "lh/log_level.hpp"

namespace openhouse::viewer {

namespace net = lh::net;

struct LogLine {
  double time = 0;
  lh::LogLevel level = lh::LogLevel::Info;
  std::string serial, text;
};

struct Coverage { std::string base; int frames = 0; double sensors = 0; };

struct Device {
  std::string serial, role, model, hand, state, primary, name;
  bool connected = false, enabled = true, charging = false, valid = false;
  std::optional<int> battery;
  std::array<double, 3> p{}, v{};
  std::array<double, 4> q{1, 0, 0, 0};
  std::vector<std::string> bases;  // in the filter
  std::vector<Coverage> coverage;  // light in the last 0.5 s
};

struct Base {
  std::string serial, channel;
  int mode = 0, observations = 0, faults = 0;
  bool seen = false, placed = false, origin = false, locked = false, learned = false;
  double sigma = 0;
  std::array<double, 3> p{};
  std::array<double, 4> q{1, 0, 0, 0};
};

struct Room { std::string id; bool settled = false; unsigned long long epoch = 0; double settleSigma = 0.005; };

struct State {
  double time = 0;
  Room room;
  std::vector<Device> devices;
  std::vector<Base> bases;
  std::vector<std::pair<std::string, std::string>> receivers;  // serial, connected device
};

class TelemetryClient {
 public:
  explicit TelemetryClient(int port) : port_(port) {}
  ~TelemetryClient() { net::Close(socket_); }

  // Connects (retrying every second), reads what arrived, parses it. Call per frame.
  void Poll(double now);

  bool Connected() const { return socket_ != net::kInvalid && helloed_; }
  int Port() const { return port_; }
  const std::string& Host() const { return host_; }
  const std::string& Version() const { return version_; }  // the driver's; empty before 1.0
  int Pid() const { return pid_; }
  const State& Current() const { return state_; }
  bool HaveState() const { return haveState_; }
  const std::deque<LogLine>& Logs() const { return logs_; }
  void ClearLogs() { logs_.clear(); }
  uint64_t LogsReceived() const { return received_; }

  static constexpr std::size_t kMaxLogs = 50000;

 private:
  void Handle(const std::string& line);
  void Disconnect();

  int port_;
  net::Socket socket_ = net::kInvalid;
  double nextAttempt_ = 0;
  std::string buffer_, host_, version_;
  int pid_ = 0;
  bool helloed_ = false, haveState_ = false;
  State state_;
  std::deque<LogLine> logs_;
  uint64_t received_ = 0;
};

}  // namespace openhouse::viewer

