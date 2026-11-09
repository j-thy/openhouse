// Core EKF of vortex::CLighthouseDirect: process noise, covariance prediction,
// measurement update and correction rejection. Higher-level logic (IMU/optical
// update orchestration, bootstrap, base slots) lives in lighthouse_direct.*.
//
// Semantics established against the original with tests/oracle/explore_predict.cpp
// and verified by tests/oracle/test_direct_filter.cpp:
//   predict:  xLin = f(x); F = ∂f/∂x;  Ppred = F_n P_n F_nᵀ + Q_n   (x, P unchanged)
//   update:   H = ∂h/∂x(xLin); S = H Ppred Hᵀ + R; K = Ppred Hᵀ S⁻¹;
//             dx = K (z − h(xLin)); reject(dx)?; x = xLin + dx; normalize q;
//             P_n = (I − K H) Ppred (I − K H)ᵀ + K R Kᵀ
// where _n is the active top-left n×n block (n = active state dimension).
#pragma once

#include <ceres/jet.h>

#include <Eigen/Dense>
#include <functional>
#include <vector>

#include "residuals.hpp"

namespace lh::direct {

using Vec = Eigen::VectorXd;
using Mat = Eigen::MatrixXd;
using Jet = ceres::Jet<double, kStateSize>;

// A residual block: m residuals as a function of the full state (templated twice).
struct Residual {
  int m = 1;
  std::function<void(const double*, double*)> eval;
  std::function<void(const Jet*, Jet*)> evalJet;
};

// Why a frame's light was refused. Diagnostics only: the EKF maths, the limits and
// their constants are unchanged. Before this a rejected frame left no trace beyond
// rejectCount, so a log could say "this base's light keeps being rejected" but never
// which limit refused it - which is the difference between a bad calibration, a
// reflection, and a room that disagrees with the device.
enum RejectReason {
  kRejectNone = 0,
  kRejectFewRows,       // fewer than 3 optical sensor rows in the frame
  kRejectNaN,           // non-finite correction
  kRejectImuRot,        // IMU rotation correction over its limit
  kRejectGravMag,       // gravity magnitude correction over its limit
  kRejectGravTilt,      // gravity tilt correction over its limit
  kRejectOmega,         // angular-rate correction over its limit
  kRejectAltBaseTrans,  // an alternate base's position correction over its limit
  kRejectAltBaseRot,    // an alternate base's rotation correction over its limit
};

inline const char* RejectReasonName(int reason) {
  switch (reason) {
    case kRejectFewRows: return "too few sensor rows";
    case kRejectNaN: return "non-finite correction";
    case kRejectImuRot: return "IMU rotation over limit";
    case kRejectGravMag: return "gravity magnitude over limit";
    case kRejectGravTilt: return "gravity tilt over limit";
    case kRejectOmega: return "angular rate over limit";
    case kRejectAltBaseTrans: return "alternate base position over limit";
    case kRejectAltBaseRot: return "alternate base rotation over limit";
    default: return "unknown";
  }
}

class DirectFilter {
 public:
  DirectFilter();

  // State and covariance.
  double x[kStateSize] = {};
  double xLin[kStateSize] = {};
  Mat P;            // 49×49 (+0x5d8)
  Mat Ppred;        // n×n   (+0x950)
  Mat Q;            // 49×49 (+0x968)
  int n = 31;       // active dimension (+0x5d0)
  uint64_t updateCount = 0;  // +0x7a0
  // Not stock (0 = off): optical rows whose innovation is beyond this many sigmas of
  // their predicted spread are left out of the update (reflections, one bad sensor),
  // unless more than half of a sweep's rows are: then the prediction is what's off.
  double gateSigma = 0;
  uint64_t gatedRows = 0;
  int lastOutliers = 0;          // last Update: optical rows beyond gateSigma, before gating
  double lastWorstSigma = 0;     // last Update: largest |innovation| / predicted sigma
  // Whole frames rejected as throwing the device far (see Update), and the current run.
  static constexpr double kFrameRejectMove = 0.10;  // m
  static constexpr int kMaxFrameRejectRun = 6;
  uint64_t rejectedFrames = 0;
  bool frameGateOff = false;  // set per update by the caller (see LighthouseDirect)
  int frameRejectRun = 0;
  double omegaCorrectionLimit2 = 1e300;  // +0x408 (|dω|² limit)
  bool altBaseActive[3] = {false, false, false};  // base slots 1..3 in use (+0x1d0..)
  // Not stock: slots whose base is being relearned (knocked): a correction moving it
  // beyond the per-update limit is scaled down to it instead of rejected.
  bool altBaseRelearn[3] = {false, false, false};
  // Not stock: during violent motion (saturated or near-limit IMU samples) the slow
  // states are held (see Update); the light then corrects the device's motion only.
  bool holdSlowStates = false;
  bool imuUpdate = false;  // the update in progress is an IMU sample's (LighthouseDirect::ImuUpdate)
  uint32_t flags = 0;        // +0xf8
  int rejectCount = 0;       // +0xfc
  // Diagnostics: which limit refused the last frame, and how many sensor rows it had.
  int lastRejectReason = kRejectNone;
  int lastRejectRows = 0;

  void DefaultState();                        // 0x3511a0 (state only)
  void InitialCovariance();                   // 0x350280 (diagonal only)
  void ComputeProcessNoise(double dt);         // 0x355990
  void Predict(double dtPos, double dtRot);    // 0x366610 (uses current Q)
  // 0x36cae0. Returns false if rejected (flags |= 0x200).
  bool Update(const std::vector<Residual>& res, const Mat& R, const Vec& z, bool gate = false);
  bool RejectCorrection(const Vec& dx);  // 0x34f570 (records lastRejectReason)
  // Runtime safety check for reusing IMU calibration after a tracking reset.
  // Does not change the verified EKF update/rejection maths.
  bool CalibrationHealthy() const;
};

}  // namespace lh::direct
