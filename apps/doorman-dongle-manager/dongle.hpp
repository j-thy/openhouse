// doorman-dongle-manager's link to the Doorman Dongles (6767:0727 "Doorman Dongle"): one worker
// thread per dongle polls its state and link statistics and runs the actions the UI queues.
// The UI only reads snapshots (Snapshot()), so it never waits on USB.
//
// The dongle (doorman/dongle-fw/src) answers 64-byte HID OUT reports [cmd][seq][args] with
// IN reports [cmd][seq][status][61 bytes]; the radio stream comes as [0x80/0x81][slot][n][...]
// reports on interface 0 (even slots) and 1 (odd slots). The driver may have the dongle open
// at the same time: HID input reports reach every open handle, and replies are matched by seq.
#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace openhouse::dongle {

constexpr int kSlots = 10;   // the most a dongle has; DongleState::slotCount is what this one has
constexpr int kHops = 16;
constexpr int kHistory = 120;  // samples kept for the graphs (one a second: two minutes)

// What a device on a slot is, as far as anyone knows.
struct Identity {
  std::string serial;  // "LHR-0AC3E7C8"
  std::string model;   // "Knuckles Left", "Vive Tracker", ...
  std::string source;  // "firmware" ('H'), "driver", "config", "remembered"
};

struct SlotState {
  bool used = false, active = false;
  std::string pairId;  // the radio's 8-byte id, hex
  Identity id;
  std::string nickname;  // the user's own name for it (on the dongle, 0x0025+; else settings)
  bool nameRead = false;  // the dongle's name for this slot has been read (firmware with names)
  int rssiDongle = 0, rssiDevice = 0;  // dBm, last
  int rssiMin = 0, rssiMax = 0;        // dBm, over the last poll
  uint32_t msSinceResponse = 0;
  // Rates over the last second (-1 unknown)
  double pollsPerSec = 0, lossPercent = -1, crcPerSec = 0, timeoutsPerSec = 0, dupPerSec = 0;
  double framesPerSec = 0, kBytesPerSec = 0, airKBytesPerSec = 0, dropsPerSec = 0, resendsPerSec = 0;
  double imuHz = 0, lightPerSec = 0, inputPerSec = 0;  // stream contents
  int deviceMissed = 0, deviceCrc = 0;                 // the device's own counters, last poll
  int saturationPercent = -1;                          // answers that left data queued, last 2 s (firmware 0x0028+)
  double deviceDropsPerSec = 0, deviceReplyDropsPerSec = 0;  // dropped in the device's radio (radio fw that reports it)
  uint64_t frames = 0, drops = 0;                      // totals since the dongle started
  std::deque<float> lossHistory, rssiHistory, kbpsHistory;
};

struct DongleState {
  std::string serial, path;
  bool open = false;
  std::string error;
  // CMD_INFO
  int firmware = 0;          // 0x0004
  bool softdeviceVariant = false, watchdog = false;
  int board = 0;  // 0 UF2 (Pro Micro), 1 Holyiot 21017, 2 PCA10059, 3 Kuroshin (Nordic DFU)
  int bank = 0;   // 1 A, 2 B: the bank running (0: older firmware, updates through its bootloader)
  int slotCount = 8;  // slots it has (it says so from firmware 0x002A; older ones have 8)
  bool NordicDfu() const { return board == 1 || board == 2; }
  bool Banked() const { return bank != 0; }   // runs the two-bank layout: updates itself
  uint32_t uptimeMs = 0, resetReason = 0;
  uint32_t crashCount = 0, crashType = 0, crashPc = 0;
  bool linkOn = true;
  bool names = false;    // keeps the devices' names (firmware 0x0025+, cmd 0x34/0x35)
  // pairing
  bool pairing = false;
  int pairTarget = -1;   // slot the pairing goes to (-1: first free)
  double pairUntil = 0;  // host time the pairing window closes (estimate)
  // channels
  uint16_t channelMask = 0;
  std::array<uint8_t, kHops> hopChannel{}, hopLoss{};
  uint8_t currentChannel = 0;
  uint32_t joinsHeard = 0;
  // link timing (cmd 0x2D)
  bool haveTiming = false;
  uint32_t cycleUs = 0, slotUs = 0;
  int answerBytes = 0;         // the answer limit now
  int normalAnswerBytes = 0;   // the limit for this many devices (0x0028+; 0 unknown)
  int saturationPercent = -1;  // worst device (0x0028+)
  bool largeAnswers = false;   // saturated: large answers on (0x0028+)
  // derived
  int joined = 0, paired = 0;
  double kBytesPerSec = 0;
  std::array<SlotState, kSlots> slots;
  // last band scan: -dBm per 1 MHz channel from 2400 + scanFirst (255: none)
  std::vector<uint8_t> scan;
  int scanFirst = 0;
  double scanTime = 0;
};

class Dongle {
 public:
  Dongle(std::string path, std::string serial);
  ~Dongle();
  const std::string& Serial() const { return serial_; }
  const std::string& Path() const { return path_; }
  bool Alive() const { return alive_; }  // false once unplugged (the manager drops it)
  DongleState Snapshot() const;

  // Actions (queued; the result comes back through the callback, on the worker thread).
  using Done = std::function<void(bool ok, const std::string& message)>;
  void Pair(int seconds, int slot, Done done);  // slot -1: the first free one
  void StopPairing(Done done);
  void Forget(int slot, Done done);             // -1: every slot
  void PowerOff(int slot, Done done);           // host command 0x9f "off!" through the radio
  void Identify(int slot, Done done);           // read its config over the radio for its name
  // Read unnamed devices' configs by itself (off while the driver runs: it names them).
  void SetAutoIdentify(bool on) { autoIdentify_ = on; }
  void Scan(Done done);                         // RSSI over 2400..2480 MHz (pauses the link ~0.2 s)
  void Reset(Done done);
  void EnterBootloader(Done done);              // UF2 drive (firmware update)
  // Writes `image` (built for the bank not running) and restarts into it (lh/dongle_bank.hpp).
  // progress gets 0..1 while it writes; done ends it.
  void BankUpdate(std::vector<uint8_t> image, std::function<void(float)> progress, Done done);
  void ClearCrash(Done done);
  void SetNickname(int slot, const std::string& name);
  // Stores the name on the dongle (firmware with names): it goes wherever the dongle goes.
  void SetName(int slot, std::string name, Done done);
  // Names learned elsewhere (the driver's telemetry, the remembered list), by pairing id.
  void SetKnown(const std::string& pairId, const Identity& id);

 private:
  struct Reply { int status = -1; std::array<uint8_t, 61> data{}; };
  void Run();
  bool Open();
  void Close();
  bool Command(uint8_t cmd, const uint8_t* args, size_t n, Reply* reply, int timeoutMs = 300);
  void ReadPending(int timeoutMs);
  void OnReport(const uint8_t* r, int n);
  void OnFrame(int slot, const uint8_t* f, int n);
  void Poll(double now);
  void PollStats(int slot, double now);
  bool FeatureRead(int slot, uint8_t* data, size_t size);
  void Queue(std::function<void()> action);
  void Ident(int slot);

  const std::string path_, serial_;
  static constexpr int kInterfaces = 4;   // interface 0: commands; slot s streams on interface s % 4
  void* dev0_ = nullptr;                   // hid_device*: devs_[0]
  void* devs_[kInterfaces] = {};
  uint8_t seq_ = 0;
  std::thread thread_;
  std::atomic<bool> stop_{false}, alive_{true};

  mutable std::mutex mutex_;  // state_, actions_, known_
  DongleState state_;
  std::deque<std::function<void()>> actions_;
  std::vector<std::pair<std::string, Identity>> known_;

  // worker-thread only
  struct Counters {
    bool have = false;
    double t = 0;
    uint32_t v[15]{};
    uint32_t air = 0, decoded = 0;
    uint16_t devDrops = 0, devReplyDrops = 0;
    uint64_t imu = 0, light = 0, input = 0;
    uint64_t imu0 = 0, light0 = 0, input0 = 0;
  };
  std::array<Counters, kSlots> counters_{};
  std::array<double, kSlots> rssiAvgD_{}, rssiAvgT_{};   // -dBm averages over channels (0: none yet)
  std::array<std::vector<uint8_t>, kSlots> frag_;
  std::array<int, kSlots> packSeq_{};   // packed reports: the next seq expected (-1: wait for an item start)
  std::array<int, kSlots> identTried_{};  // ident attempts made
  std::atomic<bool> autoIdentify_{false};
  std::array<int, kSlots> configTried_{};  // config reads made for a name
  std::array<double, kSlots> configNext_{};
  // a feature read waiting for the device's 'P' answer
  int waitSlot_ = -1;
  std::vector<uint8_t> waitReply_;
  bool waitDone_ = false;
  double nextLive_ = 0, nextSlow_ = 0, nextStats_ = 0;
};

// Every Doorman Dongle plugged in, found again every two seconds.
class DongleManager {
 public:
  DongleManager();
  ~DongleManager();
  void Poll(double now);
  const std::vector<std::shared_ptr<Dongle>>& Dongles() const { return dongles_; }

 private:
  struct Found { std::string path, serial; };
  void Scan();  // background thread: hid_enumerate is slow on Windows (it opens every HID device)

  std::vector<std::shared_ptr<Dongle>> dongles_;
  std::mutex foundMutex_;
  std::vector<Found> found_;
  bool foundFresh_ = false;
  std::atomic<bool> stop_{false};
  std::thread scanner_;
};

double Now();  // seconds, steady clock

}  // namespace openhouse::dongle
