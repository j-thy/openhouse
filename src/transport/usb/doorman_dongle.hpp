// Doorman Dongle (6767:0727, nRF52840): up to 6 trackers over our own radio link
// (doorman: doorman/common/doorman_link.h, ai/doorman/protocol.md).
//
// The dongle streams each device's main-MCU frames (the radio SPI protocol shared by our
// Tracker 1.0 firmware and Valve's main firmwares on our radio). They are the wired reports with
// another first byte, so each is re-labelled and fed to the wired parsers:
//   'p' -> 0x20 IMU slot     'q' -> 0x21 LH1 light records   's' -> 0x28 LH2 stream
//   'Q' -> buttons 0x0100    'n' -> buttons 0x0700 (Index: input + 0xa1 record)
//   'V' -> other buttons-interface reports (0x0400 battery, ...)   'B' -> battery (ours)
//   'P' -> feature report replies (config read etc.; requests go down as 'P' frames)
// Identity: our tracker firmware sends 'H' (serial, IMU ranges, hw id); any other device is
// identified from itself: feature 0x01 (IMU ranges) and the config's device_serial_number.
// Each device becomes virtual wired endpoints (Imu + Optical + Buttons).
//
// Runs on the UsbTransport reader thread (Pump); config reads run on a worker thread, and opening
// the dongle and writing to it on threads of their own.
// Several dongles can run side by side: one DoormanDongle each, keyed by USB serial.
#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "lh/input_sink.hpp"
#include "lh/log_level.hpp"
#include "raw_capture.hpp"

struct hid_device_;
struct hid_device_info;

namespace lh::usb {

class ReportDecoder;

// Tests: DoormanDongle opens the dongle and writes its commands from the calling thread (no opener
// or writer thread), so what was opened and written can be checked right after a reader pass.
extern bool g_doormanInlineIo;

// The dongle's USB data interfaces: interface 0 carries commands and replies, slot s streams on
// interface s % kDoormanInterfaces (dongle-fw usb_descriptors.c).
constexpr int kDoormanInterfaces = 4;
// One Doorman Dongle as hidapi lists it: its interfaces, all with the same USB serial.
struct DoormanDongleFound {
  std::string serial;  // "OHDG-" + USB serial: the dongle's receiver name
  std::array<std::string, kDoormanInterfaces> paths;   // by interface number; paths[0] takes commands
};
// The Doorman Dongles in an enumeration, by serial: a dongle once every interface of its USB
// serial is listed, wherever each is in the list. A serial carried by more than one interface
// of a number (an empty serial counts as one) can't be matched: it is left out, and *shared
// gets it with how many dongles share it.
std::vector<DoormanDongleFound> FindDoormanDongles(const hid_device_info* list,
                                                   std::map<std::string, int>* shared = nullptr);
// hid_enumerate for the dongle's USB id, then FindDoormanDongles.
std::vector<DoormanDongleFound> ScanDoormanDongles(std::map<std::string, int>* shared = nullptr);

// Which dongle carries each device. A device paired to several dongles can show up on two at
// once (it moves between them): the first keeps it, the others wait until it lets go, so a
// device is never set up twice. Reader thread only (and Stop, after the reader has ended).
class DoormanCarriers {
 public:
  struct Carrier { std::string dongle; int slot = -1; };
  // Takes `device` for dongle/slot; if another dongle or slot has it, that one (nothing taken).
  std::optional<Carrier> Take(const DeviceId& device, const std::string& dongle, int slot);
  void Release(const DeviceId& device, const std::string& dongle, int slot);
  // A device plugged in by USB: it's carried by its cable, and a dongle slot it's also linked on lets
  // go of it (both feeding one device doubled its light and mixed two IMU paths). Counted per open
  // interface: a device can be opened more than once.
  void AddWired(const DeviceId& device) { ++wired_[device]; }
  void RemoveWired(const DeviceId& device) {
    const auto it = wired_.find(device);
    if (it != wired_.end() && --it->second <= 0) wired_.erase(it);
  }
  bool Wired(const DeviceId& device) const { return wired_.count(device) != 0; }

 private:
  std::map<DeviceId, Carrier> carriers_;
  std::map<DeviceId, int> wired_;
};

class DoormanDongle {
 public:
  static constexpr int kSlots = 10;   // the most a dongle has (DML_SLOTS); older ones have 8 and never report the rest
  using LogFn = std::function<void(LogLevel, const std::string&)>;
  // serial: "OHDG-..." (DoormanDongleFound::serial). carriers: shared by every dongle of one
  // transport (nullptr: no check).
  DoormanDongle(std::string serial, IInputSink* sink, ReportDecoder* decoder, RawCaptureWriter* capture,
                std::atomic<uint16_t>* nextEp, std::string steamConfigDir, LogFn log, DoormanCarriers* carriers = nullptr);
  ~DoormanDongle();

  // Reader thread. Starts opening the dongle's interfaces on a thread of its own (at most one
  // attempt every 2 s); Pump takes the dongle once they're open. True if open.
  bool Open(const DoormanDongleFound& found, double now);
  bool IsOpen() const { return dev_ != nullptr; }   // reader thread
  void Pump(double now);   // reader thread: read/dispatch; closes the dongle on a read or write error
  void Close();            // detaches every tracker
  std::vector<EndpointInfo> Endpoints() const;
  // A feature report (SET) for a device on this dongle, report id first. [0xff][cmd][len]...
  // reports are sized by their len byte, others by `size`. False: not ours.
  bool Command(const DeviceId& serial, const std::array<uint8_t, 64>& report, const char** status, size_t size = 0);
  // Pairing mode; slot >= 0: into that slot (replacing what's paired there; dongle fw >= 4),
  // -1: the dongle's first free slot. False while the dongle isn't open.
  bool Pair(int milliseconds, int slot = -1);
  // The user's name for a slot's device, kept on the dongle (firmware 0x0025+, cmd 0x34). Any
  // thread; false if the dongle isn't open.
  bool SetName(int slot, const std::string& name);
  const std::string& Serial() const { return serial_; }   // any thread; also while it isn't open
  std::vector<DeviceId> Connected() const;
  // The device was reported gone: its slot shows none until the next stats refresh says otherwise.
  void DeviceGone(const DeviceId& serial);
  // Link stats of each paired slot (dongle command 0x23, polled each second). Any thread.
  struct SlotStat {
    int slot = 0; bool used = false, active = false; DeviceId serial;
    double kBytesPerSec = 0, framesPerSec = 0, lossPercent = -1, dropsPerSec = 0;
    int rssiDongle = 0, rssiDevice = 0, devicesOnDongle = 0;
    std::string name; bool haveName = false;   // the name kept on the dongle (haveName: it keeps names)
  };
  std::vector<SlotStat> SlotStats() const;

 private:
  struct Slot {
    bool used = false, active = false;
    std::array<uint8_t, 8> pairId{}; bool havePairId = false;   // the paired device (0x25): a new one resets the slot
    int identSeq = -1;     // seq of the pending ident request
    double identAskT = 0;
    int identTries = 0;
    bool noIdent = false;  // no 'H' identity: identify from feature 0x01 + config
    bool attached = false;
    // Its device is on another dongle or slot (DoormanCarriers): set up here once that one
    // lets go, with this config.
    bool waiting = false;
    std::string waitConfig;
    DeviceId serial;
    int8_t gyro = -1, accel = -1;
    int sensorType = -1;
    uint16_t epImu = 0, epOpt = 0, epBtn = 0;
    std::array<std::array<uint8_t, 17>, 3> ring{};
    int ringN = 0, ringPos = 0;
    // config read over the radio
    std::thread worker;
    std::atomic<bool> workerDone{false};
    std::string fetchedConfig, fetchedSerial;
    int8_t fetchedGyro = -1, fetchedAccel = -1;
    int fetchedSensorType = -1;
    // link stats: last raw 0x23 counters and the rates derived from them
    bool haveStats = false; double statsT = 0;
    uint32_t polls = 0, respOk = 0, framesUp = 0, bytesUp = 0, drops = 0;
    double kBps = 0, fps = 0, loss = -1, dps = 0; int rssiD = 0, rssiT = 0;
    std::array<uint32_t, 15> raw{};   // the last 0x23 reply as is
    // Silent while connected: data stopped but the slot still answers. Logged (with the radio's own
    // status) and then made to rejoin, so the cause shows in the log.
    double lastDataT = 0, hostBusyT = -1e9; bool streamed = false;
    std::string name; bool haveName = false; int nameSeq = -1;   // its name on the dongle (0x35)
    // IMU sample timestamps out of step (the air codec garbling them): count in a row, and the last.
    uint32_t lastImuTicks = 0; bool haveImuTicks = false; int garbledImu = 0; double garbledRejoinT = -1e9;
    int silentStage = 0; std::array<uint32_t, 15> silentRaw{};
    uint32_t logPolls = 0, logOk = 0, logBytes = 0, logDrops = 0; double logT = 0; bool logHave = false;   // the log line's last counters   // from feature 0x05 (devices without 'H', e.g. stock main firmware)
  };
  struct FeatureWait {
    int slot = -1;
    uint8_t id = 0, tag = 0;
    bool sent = false, done = false;
    std::vector<uint8_t> reply;
  };

  void Drop();
  void Send(uint8_t cmd, const uint8_t* args, size_t n);
  void OnReport(double t, const uint8_t* r, int n, bool last);
  void OnFrame(int slot, double t, const uint8_t* f, int n, bool last);
  void WatchSilent(int slot, double now);
  void OnRadioStatus(int slot, const uint8_t* f, int n);
  void CheckImuStream(int slot, double now, const uint8_t* f);
  void OnIdent(int slot, const uint8_t* f, int n);
  void Attach(int slot, const std::string& config);
  void Detach(int slot, bool handedOver = false);   // handedOver: its USB cable carries it now
  void StartConfigFetch(int slot);
  int FeatureRead(int slot, uint8_t* data, size_t size);  // worker thread
  void Downlink(int slot, const uint8_t* frame, size_t n);
  void RefreshStatsCache();

  const std::string serial_;
  IInputSink* sink_;
  ReportDecoder* decoder_;
  RawCaptureWriter* capture_;
  std::atomic<uint16_t>* nextEp_;
  std::string steamConfigDir_;
  LogFn log_;
  DoormanCarriers* carriers_;

  hid_device_* dev_ = nullptr;
  std::array<hid_device_*, kDoormanInterfaces> devs_{};   // every interface; dev_ is devs_[0]
  // Commands go out on a thread of their own, through a second handle on interface 0: a write waits
  // until the dongle takes it (busy with eight devices, several ms), and written from the reader
  // thread a config read's stream of commands kept it from reading; the dongle's queues filled and
  // dropped the config replies themselves, so the read failed and was retried, over and over.
  hid_device_* writeDev_ = nullptr;
  std::thread writer_;
  std::mutex writeMutex_;
  std::condition_variable writeCv_;
  std::deque<std::array<uint8_t, 65>> writes_;
  bool writerStop_ = false;
  std::atomic<bool> writeFailed_{false};
  void StartWriter();
  void StopWriter();
  // Opening runs on a thread of its own too: hid_open_path takes as long as the system does (half a
  // minute for a dongle that stopped answering), and on the reader thread every device's data waited.
  std::thread opener_;
  std::atomic<bool> openerDone_{false};
  std::array<hid_device_*, kDoormanInterfaces> openedDevs_{};   // the opener's result: every interface or none
  hid_device_* openedWriteDev_ = nullptr;
  bool TakeOpened();   // reader thread: the dongle, once the opener has opened it
  double nextOpen_ = 0, nextLive_ = 0, nextStats_ = 0, nextLinkLog_ = 0;
  int statsSlot_ = 0, statsSeq_ = -1;
  int idsSeq_[2] = {-1, -1};    // the slots (0x25) requests in flight
  double nextIds_ = 0;
  void ResetSlot(int slot);     // its device left or was replaced: identify what comes next afresh
  int liveSeq_[2] = {-1, -1};   // the live (0x29) requests in flight: slots 0-5 and 6-7
  uint8_t seq_ = 0;
  Slot slots_[kSlots];
  std::vector<uint8_t> frag_[kSlots];   // a frame split over several stream reports, per slot
  int packSeq_[kSlots]{};
  int slotCount_ = 8;                   // the slots this dongle has (info byte 60, firmware 0x002A+); 8 until it says                 // packed reports (0x82): the next seq expected, -1 = not in step

  mutable std::mutex mutex_;          // featureWaits_, downlinks_, pairMs_, opened_, slot serials
  bool opened_ = false;
  std::condition_variable cv_;
  std::deque<std::shared_ptr<FeatureWait>> featureWaits_;
  uint8_t nextTag_ = 0;
  std::array<double, kSlots> readsAfter_{};   // Pump thread
  std::deque<std::pair<int, std::vector<uint8_t>>> downlinks_;
  std::optional<int> pairMs_;
  std::vector<std::pair<int, std::string>> nameWrites_;   // SetName, sent by Pump; under mutex_
  double nextNames_ = 0;
  int pairSlot_ = -1;
  std::vector<EndpointInfo> eps_;
  std::vector<SlotStat> statsCache_;   // under mutex_
};

}  // namespace lh::usb
