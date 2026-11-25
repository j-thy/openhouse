// Serves the driver's live state and log to local viewers (lh/telemetry.hpp).
//
// The driver thread hands it snapshots and log lines; a thread of its own accepts
// clients and writes to them, so a slow or stuck viewer never stalls tracking (a
// client more than kMaxQueued behind is dropped).
#pragma once
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "common/net.hpp"
#include "lh/log_level.hpp"
#include "lh/session.hpp"

namespace lh::openvr {

struct RoomStatus {
  bool settled = false;
  unsigned long long epoch = 0;
  double settleSigma = 0;  // m: what a base needs to count as placed
  unsigned long long resets = 0;  // room resets this session
};

class TelemetryServer {
 public:
  TelemetryServer() = default;
  TelemetryServer(const TelemetryServer&) = delete;
  TelemetryServer& operator=(const TelemetryServer&) = delete;
  ~TelemetryServer() { Stop(); }

  // false (and *error) when the port can't be opened; the driver runs on without it.
  bool Start(int port, std::string host, std::string* error = nullptr);
  void Stop();

  void Log(double time, LogLevel level, const std::string& serial, const std::string& text);
  void PublishState(double time, const SessionSnapshot& snapshot, const std::vector<DeviceUpdate>& updates,
                    const std::vector<Session::Receiver>& receivers, const RoomStatus& room);
  // The driver's settings as the overlay shows them: a JSON object, sent in every state.
  void SetSettings(std::string json);

  // Commands from clients (openhouse-overlay): one JSON object per line,
  // {"type":"command","id":N,"cmd":"...", ...}. The driver takes them on its own thread
  // and answers each with Reply(): {"type":"reply","id":N,"ok":B,"error":"..."}.
  struct Command {
    uint64_t client = 0;
    std::string json;
  };
  std::vector<Command> TakeCommands();
  void Reply(uint64_t client, const std::string& json);

  static constexpr std::size_t kBacklog = 2000;            // log lines a new client gets
  static constexpr std::size_t kMaxQueued = 8 << 20;       // bytes

 private:
  struct Client {
    net::Socket socket = net::kInvalid;
    std::string out;
    std::string in;  // a command line still arriving
    uint64_t id = 0;
  };
  static constexpr std::size_t kMaxCommand = 64 << 10;  // bytes: longer lines drop the client
  static constexpr std::size_t kMaxPending = 256;        // commands not taken yet
  void Main();
  void Broadcast(const std::string& line);  // under mutex_

  std::mutex mutex_;
  std::condition_variable wake_;
  std::vector<Client> clients_;
  std::deque<std::string> backlog_;
  std::vector<Command> commands_;
  std::string settings_ = "{}";
  uint64_t nextClient_ = 1;
  net::Socket listener_ = net::kInvalid;
  std::string host_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
};

}  // namespace lh::openvr
