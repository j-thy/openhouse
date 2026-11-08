#include "room_publisher.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#ifdef __linux__
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#elif defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace lh::openvr {

bool RoomSettle::Close(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double m, double rad) {
  const double d = (a.translation() - b.translation()).norm();
  const double r = Eigen::AngleAxisd(a.rotation().transpose() * b.rotation()).angle();
  return d <= m && r <= rad;
}

bool RoomSettle::Update(const std::vector<Base>& bases, double now) {
  std::map<std::string, Eigen::Isometry3d> poses;
  bool complete = !bases.empty();
  for (const auto& b : bases) {
    if (!b.visible) continue;
    if (!b.Good()) complete = false;
    else poses[b.serial] = b.rawFromBase;
  }
  bool still = complete && poses.size() == window_.size();
  for (const auto& [s, p] : poses) {
    auto w = window_.find(s);
    if (w == window_.end() || !Close(w->second, p, kStill, kStillRad)) still = false;
  }
  if (!still) { window_ = poses; since_ = now; }
  const bool settled = complete && !poses.empty() && now - since_ >= kSettleTime;
  if (settled && !settled_) {
    bool same = !settledPoses_.empty() && settledPoses_.size() == poses.size();
    for (const auto& [s, p] : poses) {
      auto o = settledPoses_.find(s);
      if (o == settledPoses_.end() || !Close(o->second, p, kNewEpoch, kNewEpochRad)) same = false;
    }
    if (!same) { ++epoch_; settledPoses_ = poses; }
  }
  settled_ = settled;
  return settled;
}

void RoomPublisher::Close() {
#ifdef __linux__
  if (shm_) {
    shm_->beat = 0;
    munmap(shm_, sizeof *shm_);
    shm_unlink(path_.c_str());
  }
#elif defined(_WIN32)
  if (shm_) {
    shm_->beat = 0;
    UnmapViewOfFile(shm_);
  }
  if (map_) CloseHandle(static_cast<HANDLE>(map_));
  map_ = nullptr;
#endif
  shm_ = nullptr;
  failed_ = false;
  settle_ = {};
}

bool RoomPublisher::Open() {
#ifdef __linux__
  if (shm_ || failed_) return shm_ != nullptr;
  path_ = OPENHOUSE_ROOM_PATH_PREFIX + std::to_string(unsigned(getuid()));
  const int fd = shm_open(path_.c_str(), O_RDWR | O_CREAT, 0600);
  if (fd < 0) { failed_ = true; return false; }
  void* m = ftruncate(fd, sizeof(openhouse_room)) == 0
                ? mmap(nullptr, sizeof(openhouse_room), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0) : MAP_FAILED;
  close(fd);
  if (m == MAP_FAILED) { failed_ = true; return false; }
  shm_ = static_cast<openhouse_room*>(m);
  std::memset(shm_, 0, sizeof *shm_);
  shm_->magic = OPENHOUSE_ROOM_MAGIC;
  shm_->version = OPENHOUSE_ROOM_VERSION;
  shm_->pid = int32_t(getpid());
  return true;
#elif defined(_WIN32)
  if (shm_ || failed_) return shm_ != nullptr;
  HANDLE h = CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, DWORD(sizeof(openhouse_room)),
                                OPENHOUSE_ROOM_WIN_NAME);
  if (!h) { failed_ = true; return false; }
  void* m = MapViewOfFile(h, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(openhouse_room));
  if (!m) { CloseHandle(h); failed_ = true; return false; }
  map_ = h;
  shm_ = static_cast<openhouse_room*>(m);
  std::memset(shm_, 0, sizeof *shm_);
  shm_->magic = OPENHOUSE_ROOM_MAGIC;
  shm_->version = OPENHOUSE_ROOM_VERSION;
  shm_->pid = int32_t(GetCurrentProcessId());
  return true;
#else
  return false;
#endif
}

void RoomPublisher::Publish(const SessionSnapshot& snapshot, const Eigen::Isometry3d& rawFromWorld, double now,
                            unsigned long long roomResets) {
  if (roomResets != roomResets_) { roomResets_ = roomResets; settle_.Restart(); }
  std::vector<RoomSettle::Base> bases;
  for (const auto& b : snapshot.bases) {
    Eigen::Isometry3d worldFromBase = Eigen::Isometry3d::Identity();
    const auto& o = b.worldFromBase.orientation;
    worldFromBase.linear() = Eigen::Quaterniond(o[0], o[1], o[2], o[3]).normalized().toRotationMatrix();
    worldFromBase.translation() = Eigen::Vector3d(b.worldFromBase.position.data());
    bases.push_back({b.serial, b.seen, b.known, rawFromWorld * worldFromBase,
                     b.sigma, b.locked, b.learned});
  }
  const bool settled = settle_.Update(bases, now);
  settled_ = settled;
  {
    // Logged on a change only: settled or not, and which bases hold it back.
    std::string key = settled ? "settled" : "settling";
    std::string text = std::string("room ") + (settled ? "settled" : "not settled");
    std::string ok, waiting;
    for (const auto& b : bases) {
      if (!b.visible) continue;
      const bool good = b.Good();
      key += "|" + b.serial + (good ? "+" : "-");
      char item[96];
      if (good) std::snprintf(item, sizeof item, " %s (%s%.1f mm)", b.serial.c_str(), b.sigma > RoomSettle::kMaxSigma ? (b.locked ? "locked, " : "well seen, ") : "", b.sigma * 1000);
      else if (!b.placed) std::snprintf(item, sizeof item, " %s (not placed)", b.serial.c_str());
      else std::snprintf(item, sizeof item, " %s (%.1f mm, needs %.0f)", b.serial.c_str(), b.sigma * 1000, RoomSettle::kMaxSigma * 1000);
      (good ? ok : waiting) += item;
    }
    if (key != lastLogKey_) {
      lastLogKey_ = key;
      text += ": placed" + (ok.empty() ? std::string(" none") : ok);
      if (!waiting.empty()) text += "; waiting for" + waiting;
      char e[48];
      std::snprintf(e, sizeof e, " (epoch %llu)", static_cast<unsigned long long>(settle_.epoch()));
      pendingLog_ = text + e;
    }
  }
#if defined(__linux__) || defined(_WIN32)
  if (!Open()) return;
  auto* seq = reinterpret_cast<volatile uint32_t*>(&shm_->seq);
  __atomic_store_n(&shm_->seq, *seq + 1, __ATOMIC_RELEASE);
  __atomic_thread_fence(__ATOMIC_SEQ_CST);
#ifdef _WIN32
  shm_->beat = double(GetTickCount64()) / 1000.0;
#else
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  shm_->beat = double(ts.tv_sec) + 1e-9 * double(ts.tv_nsec);
#endif
  shm_->epoch = settle_.epoch();
  shm_->settled = settled;
  std::snprintf(shm_->room, sizeof shm_->room, "%s", snapshot.roomId.c_str());
  int n = 0;
  for (size_t i = 0; i < bases.size() && n < OPENHOUSE_ROOM_MAX_BASES; ++i) {
    const auto& b = bases[i];
    auto& out = shm_->bases[n++];
    std::memset(&out, 0, sizeof out);
    std::snprintf(out.serial, sizeof out.serial, "%s", b.serial.c_str());
    out.visible = b.visible;
    out.placed = b.placed;
    out.origin = snapshot.bases[i].origin;
    const Eigen::Quaterniond q(b.rawFromBase.rotation());
    const Eigen::Vector3d t = b.rawFromBase.translation();
    for (int k = 0; k < 3; ++k) out.p[k] = t[k];
    out.q[0] = q.w(); out.q[1] = q.x(); out.q[2] = q.y(); out.q[3] = q.z();
    out.sigma = b.sigma;
  }
  shm_->count = n;
  __atomic_thread_fence(__ATOMIC_SEQ_CST);
  __atomic_store_n(&shm_->seq, *seq + 1, __ATOMIC_RELEASE);
#else
  (void)settled;
#endif
}

}  // namespace lh::openvr
