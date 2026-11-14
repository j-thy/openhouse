#include <string>
#include "direct_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdio>
#include <cstdlib>
#include <iterator>

namespace lh::direct {
namespace {

// Process noise constants (function-local statics in the original).
constexpr double kJerkDensity = 0.25;                       // 0x1a9bc8
constexpr double kOmegaDensity = 1.0966227112321507;        // 0x1a99d8
constexpr double kGravTiltNoise = 8.461594994075239e-08;    // 0x1a9910
constexpr double kGyroBiasNoise = 3.3846379976300955e-09;   // 0x1a9808
constexpr double kAccelBiasXYNoise = 6.678498904340278e-09; // 0x1a9c28
constexpr double kAccelBiasZNoise = 2.6713995617361114e-08; // 0x1a9690
constexpr double kAccelScaleNoise = 1.1111111111111111e-11; // 0x1a9ab8
constexpr double kDiagEpsilon = 1e-11;                      // 0x1a9630

// Correction limits (0x34f570)
constexpr double kMaxImuRot = 0.17453292519943295;          // 10°
constexpr double kMaxImuRotConverged = 0.03490658503988659; // 2°
constexpr double kMaxGravMag = 0.4903325;                   // 0.05 g
constexpr double kMaxGravMagConverged = 0.0980665;          // 0.01 g
constexpr double kMaxGravTilt = 0.06981317007977318;        // 4°
constexpr double kMaxAltBaseTrans = 0.1;
constexpr double kMaxAltBaseRot = 0.017453292519943295;     // 1°

}  // namespace

DirectFilter::DirectFilter() : P(Mat::Zero(kStateSize, kStateSize)), Q(Mat::Zero(kStateSize, kStateSize)) {}

bool DirectFilter::CalibrationHealthy() const {
  for (int i = 18; i < 31; ++i) if (!std::isfinite(x[i])) return false;
  if (std::abs(x[18] - 9.80665) > gravityTolerance ||
      x[28] * x[28] + x[29] * x[29] + x[30] * x[30] > 0.06853891945200943) return false;
  for (int i = 25; i < 28; ++i) if (x[i] <= 0) return false;
  if (P.rows() < 31 || P.cols() < 31 || !P.block(18, 18, 13, 13).allFinite()) return false;
  for (int i = 18; i < 31; ++i) if (P(i, i) < 0) return false;
  return true;
}

void DirectFilter::DefaultState() {
  std::fill(std::begin(x), std::end(x), 0.0);
  x[2] = -2;
  x[10] = -0.7071067811865476;
  x[11] = 0.7071067811865476;
  x[18] = 9.80665;
  x[25] = x[26] = x[27] = 1;
}

void DirectFilter::InitialCovariance() {
  // Like the original, does not clear existing off-diagonal entries.
  P.diagonal().setConstant(1e-6);
  P.diagonal().segment<3>(0).setConstant(2.5e-5);
  P.diagonal().segment<3>(3).setConstant(0.04000000000000001);
  P.diagonal().segment<3>(6).setConstant(1);
  P.diagonal().segment<4>(9).setConstant(0.00047596471841673214);
  P.diagonal().segment<3>(13).setConstant(1.713472986300236);
  P(16, 16) = 1.0966227112321507;
  P(17, 17) = 0.15421256876702122;
}

void DirectFilter::ComputeProcessNoise(double dt) {
  Q.diagonal().setZero();
  // pos/vel/acc: q·G·Gᵀ per axis with G = [dt²/2, dt, 1] (0x354560)
  const double q = kJerkDensity;
  const double dt2 = dt * dt;
  const double a = dt2 * dt2 * 0.25 * q;   // pos-pos
  const double b = dt * dt2 * 0.5 * q;     // pos-vel
  const double c = dt2 * q * 0.5;          // pos-acc
  const double d = dt2 * q;                // vel-vel
  const double e = dt * q;                 // vel-acc
  for (int i = 0; i < 3; i++) {
    const int p = kPos + i, v = kVel + i, ac = kAcc + i;
    for (int r : {p, v, ac})
      for (int k = 0; k < 9; k++) Q(r, k) = 0;
    Q(p, p) = a; Q(p, v) = b; Q(p, ac) = c;
    Q(v, p) = b; Q(v, v) = d; Q(v, ac) = e;
    Q(ac, p) = c; Q(ac, v) = e; Q(ac, ac) = q;
  }
  // quaternion / angular velocity block (0x3556d0)
  {
    const double s = kOmegaDensity * dt;
    const double w = x[9], qx = x[10], qy = x[11], qz = x[12];
    const double aq = dt * 0.08333333333333333 * dt * s;  // s·dt²/12
    const double bq = dt * s * 0.25;
    double Xi[4][3] = {{-qx, -qy, -qz}, {w, -qz, qy}, {qz, w, -qx}, {-qy, qx, w}};
    double QQ[4][4];
    QQ[0][0] = (qz * qz + qy * qy + qx * qx) * aq;
    QQ[0][1] = w * -qx * aq; QQ[0][2] = -qy * w * aq; QQ[0][3] = -(w * qz * aq);
    QQ[1][1] = (qy * qy + (w * w + qz * qz)) * aq;
    QQ[1][2] = qy * -qx * aq; QQ[1][3] = qz * -qx * aq;
    QQ[2][2] = ((w * w + qz * qz) + qx * qx) * aq;
    QQ[2][3] = -qy * qz * aq;
    QQ[3][3] = aq * (w * w + qy * qy + qx * qx);
    for (int r = 0; r < 4; r++)
      for (int k = r; k < 4; k++) Q(kQuat + r, kQuat + k) = Q(kQuat + k, kQuat + r) = QQ[r][k];
    for (int r = 0; r < 4; r++)
      for (int k = 0; k < 3; k++) Q(kQuat + r, kOmega + k) = Q(kOmega + k, kQuat + r) = Xi[r][k] * bq;
    for (int r = 0; r < 3; r++)
      for (int k = 0; k < 3; k++) Q(kOmega + r, kOmega + k) = r == k ? s : 0.0;
  }
  Q(16, 16) = Q(17, 17) = kGravTiltNoise * dt;
  for (int i = 19; i < 22; i++) Q(i, i) = kGyroBiasNoise * dt;
  Q(22, 22) = Q(23, 23) = kAccelBiasXYNoise * dt;
  Q(24, 24) = kAccelBiasZNoise * dt;
  for (int i = 25; i < 28; i++) Q(i, i) = dt * kAccelScaleNoise;
  for (int i = 0; i < kStateSize; i++) Q(i, i) += kDiagEpsilon;
}

void DirectFilter::Predict(double dtPos, double dtRot) {
  Jet xj[kStateSize], r[kStateSize];
  for (int i = 0; i < kStateSize; i++) xj[i] = Jet(x[i], i);
  StateTimestep(xj, dtPos, dtRot, r);
  for (int i = 0; i < kStateSize; i++) xLin[i] = r[i].a;
  // Ppred = F P F^T + Q. F is the identity except in the rows of the states that move over a
  // step (position, velocity, orientation: 10 of 31-49, with ~37 entries off the identity), so
  // only those rows of F P and those columns of (F P) F^T are summed; the others are copies.
  // The same products as the dense form up to summation order; the dense pair of 49x49 GEMMs
  // per IMU sample was most of the driver's CPU time (#39, #50).
  struct Row { int i; int count; int j[kStateSize]; double v[kStateSize]; };
  Row rows[kStateSize];
  int moving = 0;
  for (int i = 0; i < n; i++) {
    Row& row = rows[moving];
    row.i = i;
    row.count = 0;
    bool identity = true;
    for (int j = 0; j < n; j++) {
      const double f = r[i].v[j];
      if (f != (i == j ? 1.0 : 0.0)) identity = false;
      if (f != 0.0) { row.j[row.count] = j; row.v[row.count] = f; ++row.count; }
    }
    if (!identity) ++moving;
  }
  const auto Pn = P.topLeftCorner(n, n);
  Mat A = Pn;  // F P
  for (int k = 0; k < moving; k++) {
    const Row& row = rows[k];
    A.row(row.i).setZero();
    for (int t = 0; t < row.count; t++) A.row(row.i).noalias() += row.v[t] * Pn.row(row.j[t]);
  }
  Ppred = A;  // (F P) F^T
  for (int k = 0; k < moving; k++) {
    const Row& row = rows[k];
    Ppred.col(row.i).setZero();
    for (int t = 0; t < row.count; t++) Ppred.col(row.i).noalias() += row.v[t] * A.col(row.j[t]);
  }
  Ppred += Q.topLeftCorner(n, n);
}

bool DirectFilter::RejectCorrection(const Vec& dx) {
  // Same limits, same order of test: only the reason is recorded (see RejectReason).
  const auto refuse = [this](int reason) {
    lastRejectReason = reason;
    lastRejectSlot = -1;
    lastRejectOver = lastRejectLimit = 0;
    return true;
  };
  // A scalar limit (IMU rotation, gravity, angular rate): record the value and the limit
  // it was compared against, not just which one it was. The alternate-base limits below
  // already did this; without it for these, a log cannot tell a correction that is 2.1
  // deg over a 2 deg converged limit (a threshold question) from a 30 deg one (a
  // calibration question), which is exactly the ambiguity the diagnostic exists to end.
  const auto refuseValue = [this](int reason, double over, double limit) {
    lastRejectReason = reason;
    lastRejectSlot = -1;
    lastRejectOver = over;
    lastRejectLimit = limit;
    return true;
  };
  // An alternate base's limit: record which base slot tripped it and by how much.
  const auto refuseAlt = [this](int reason, int slot, double over, double limit) {
    lastRejectReason = reason;
    lastRejectSlot = slot;
    lastRejectOver = over;
    lastRejectLimit = limit;
    return true;
  };
  for (int i = 0; i < dx.size(); i++)
    if (std::isnan(dx[i] - dx[i])) return refuse(kRejectNaN);
  auto d = [&](int i) { return i < dx.size() ? dx[i] : 0.0; };
  const double rot = std::sqrt(d(30) * d(30) + d(28) * d(28) + d(29) * d(29));
  if (kMaxImuRot < rot) return refuseValue(kRejectImuRot, rot, kMaxImuRot);
  bool skipTilt = false;
  if (updateCount < 0x105) {
    const double g = std::fabs(d(18));
    if (kMaxGravMag < g) return refuseValue(kRejectGravMag, g, kMaxGravMag);
    if (updateCount < 0xc9) skipTilt = true;
  } else {
    if (kMaxImuRotConverged < rot) return refuseValue(kRejectImuRot, rot, kMaxImuRotConverged);
    const double g = std::fabs(d(18));
    if (kMaxGravMag < g) return refuseValue(kRejectGravMag, g, kMaxGravMag);
    if (kMaxGravMagConverged < g) return refuseValue(kRejectGravMag, g, kMaxGravMagConverged);
  }
  const double tilt = std::sqrt(d(16) * d(16) + d(17) * d(17));
  if (!skipTilt && kMaxGravTilt < tilt) return refuseValue(kRejectGravTilt, tilt, kMaxGravTilt);
  const double w2 = d(13) * d(13) + d(14) * d(14) + d(15) * d(15);
  if (omegaCorrectionLimit2 <= w2) return refuseValue(kRejectOmega, std::sqrt(w2), std::sqrt(omegaCorrectionLimit2));
  for (int s = 0; s < 3; s++) {
    if (!altBaseActive[s]) continue;
    const int k = kAltBaseIndex[s];
    const double trans = std::sqrt(d(k + 2) * d(k + 2) + d(k) * d(k) + d(k + 1) * d(k + 1));
    if (kMaxAltBaseTrans < trans) return refuseAlt(kRejectAltBaseTrans, s, trans, kMaxAltBaseTrans);
    const double rotAlt = std::sqrt(d(k + 5) * d(k + 5) + d(k + 3) * d(k + 3) + d(k + 4) * d(k + 4));
    if (kMaxAltBaseRot < rotAlt) return refuseAlt(kRejectAltBaseRot, s, rotAlt, kMaxAltBaseRot);
  }
  // A frame that passes must leave no diagnostic behind: a stale slot or limit from the
  // previous refusal would be read as belonging to this frame.
  lastRejectReason = kRejectNone;
  lastRejectSlot = -1;
  lastRejectOver = lastRejectLimit = 0;
  return false;
}

bool DirectFilter::Update(const std::vector<Residual>& res, const Mat& R, const Vec& z, bool gate) {
  int m = 0;
  for (auto& r : res) m += r.m;
  Mat H(m, n);
  Vec h(m);
  Jet xj[kStateSize];
  for (int i = 0; i < kStateSize; i++) xj[i] = Jet(xLin[i], i);
  int row = 0;
  std::vector<Jet> out;
  for (auto& r : res) {
    out.resize(size_t(r.m));
    r.evalJet(xj, out.data());
    for (int k = 0; k < r.m; k++, row++) {
      h[row] = out[size_t(k)].a;
      for (int j = 0; j < n; j++) H(row, j) = out[size_t(k)].v[j];
    }
  }
  Mat PHt = Ppred * H.transpose();
  Mat S = H * PHt + R;
  Vec y = z - h;
  Mat Rk = R;
  lastOutliers = 0; lastWorstSigma = 0;
  const int opticalRows = m - (res.empty() ? 0 : res.back().m);
  for (int i = 0; i < opticalRows; ++i) {
    const double sigma = std::abs(y[i]) / std::sqrt(S(i, i));
    lastWorstSigma = std::max(lastWorstSigma, sigma);
    lastOutliers += gateSigma > 0 && sigma > gateSigma;
  }
  if (gate && gateSigma > 0 && m >= 4) {
    // Rows of the last residual block are not optical (normalizers): never gated.
    std::vector<int> keep;
    int outliers = 0;
    const int optical = m - (res.empty() ? 0 : res.back().m);
    for (int i = 0; i < m; ++i) {
      const bool out = i < optical && y[i] * y[i] > gateSigma * gateSigma * S(i, i);
      outliers += out;
      if (!out) keep.push_back(i);
    }
    if (outliers > 0 && 2 * outliers <= optical && optical - outliers >= 3) {
      const int k = int(keep.size());
      Mat H2(k, n), R2(k, k);
      Vec y2(k);
      for (int a = 0; a < k; ++a) {
        H2.row(a) = H.row(keep[size_t(a)]);
        y2[a] = y[keep[size_t(a)]];
        for (int b = 0; b < k; ++b) R2(a, b) = R(keep[size_t(a)], keep[size_t(b)]);
      }
      H = std::move(H2); y = std::move(y2); Rk = std::move(R2);
      PHt = Ppred * H.transpose();
      S = H * PHt + Rk;
      gatedRows += uint64_t(outliers);
    }
  }
  const Mat Sinv = S.ldlt().solve(Mat::Identity(S.rows(), S.cols()));
  if (holdSlowStates && !imuUpdate) {
    // Held, the other bases' per-update limits (altTrans, altRot) no longer catch a
    // wrong frame: its whole correction would land in the device pose (one reflected
    // sensor 0.24 rad off moved it 43 cm, 1 m within 0.3 s). Frames of fewer than three
    // sensor rows are rejected. (A chi-square gate doesn't tell them apart: the
    // predicted spread is far too small in violent motion, median 71 per row.)
    if (opticalRows < 3) {
      flags |= 0x200;
      rejectCount++;
      lastRejectReason = kRejectFewRows;
      lastRejectRows = opticalRows;
      return false;
    }
  }
  Mat K = PHt * Sinv;
  // Not stock (Schmidt "consider" states): during violent motion the slow states (base
  // tilt and gravity, IMU calibration, the other bases' poses) are held: their gain
  // rows are zeroed, so the update corrects the device's motion only and the Joseph
  // form below keeps their uncertainty and correlations consistent.
  if (holdSlowStates) {
    // (testswing, held: none / calibration / + other bases / + tilt: tracking 87 / 95
    // / 92 / 98 %, the light used 42 / 52 / 90 / 84 %, jumps > 5 cm 142 / 58 / 67 / 3.)
    K.middleRows(16, std::min(n, 49) - 16).setZero();
  } else if (holdImuCalibration && n > 30) {
    K.middleRows(18, 13).setZero();
  }
  Vec dx = K * y;
  // Not stock: a relearning (knocked) base whose light asks for more than the
  // per-update limit moves only itself, by the largest step the limit allows, and
  // keeps its uncertainty: it walks to its new place without dragging the device
  // (a knocked base 20 cm away was otherwise rejected on every frame, for good:
  // ms4-01; scaling the whole correction instead pulled the device off the others).
  bool walked = false;
  for (int s = 0; s < 3 && !walked; s++) {
    if (!altBaseActive[s] || !altBaseRelearn[s]) continue;
    const int k = kAltBaseIndex[s];
    if (k + 6 > dx.size()) continue;
    const double trans = dx.segment<3>(k).norm(), rot = dx.segment<3>(k + 3).norm();
    if (trans <= kMaxAltBaseTrans && rot <= kMaxAltBaseRot) continue;
    const double scale = std::min({1.0, 0.9 * kMaxAltBaseTrans / std::max(trans, 1e-12), 0.9 * kMaxAltBaseRot / std::max(rot, 1e-12)});
    Vec only = Vec::Zero(dx.size());
    only.segment<6>(k) = scale * dx.segment<6>(k);
    dx = only;
    walked = true;
  }
  if (RejectCorrection(dx)) {
    flags |= 0x200;
    rejectCount++;
    lastRejectRows = opticalRows;
    return false;
  }
  // Not stock: one frame that disagrees with the prediction as a whole and would throw the
  // device far. The row gate stands aside when most rows are out, so such a frame was
  // applied in full: in Beat Saber play (light from a base whose pose or slot doesn't
  // fit, mostly single-axis, mid-swing) one moved the pose 40-90 cm and the filter followed
  // the wrong track out of the room. No single frame, a few ms after the last, moves a
  // device 10 cm. Gating on sigma instead (> 100) also dropped good light from still
  // devices, whose predicted spread is tiny (vrchat-01: coasting 4.9 -> 10 %).
  // No cap on how many in a row: if it is the filter that's wrong, DeviceTracker's refused-
  // light reset reacquires after 0.5 s of light that fits nothing. (A cap of 6 frames was
  // written here but never acted: the IMU samples between frames, which share this Update,
  // reset its count. Making it act, either applying the 7th frame or reacquiring at once,
  // was worse in the Beat Saber stand-ins - a 52 cm jump in one replay; fastmotion4/5 one
  // more fly-away and 1 % less tracking - so the behaviour live-tested here is kept, #47.1.)
  // Nor while the filter is still settling (fewer than 200 updates since a bootstrap or
  // reacquire): its large corrections then are the right ones (all 322 frames this gate
  // took from vrchat-01's 100-200 s were there). It acts in violent motion too, where
  // nothing found (sigma, base slot) told bad light from big real corrections: there it
  // costs the extreme testswing capture (valid 94 -> 84 %), but without it a mid-game
  // fly-away in Beat Saber came back. Chosen for Beat Saber; live-tested there
  // 2026-10-22 (no fly-aways).
  if (gate && !frameGateOff && gateSigma > 0 && updateCount >= 200 && opticalRows > 0 &&
      2 * lastOutliers > opticalRows &&
      dx.size() >= 3 && dx.head<3>().norm() > kFrameRejectMove) {
    ++rejectedFrames;
    return false;
  }
  for (int i = 0; i < kStateSize; i++) x[i] = xLin[i] + (i < n ? dx[i] : 0.0);
  const double inv = 1.0 / std::sqrt(x[12] * x[12] + x[9] * x[9] + x[10] * x[10] + x[11] * x[11]);
  for (int i = 9; i < 13; i++) x[i] *= inv;
  if (walked) { P.topLeftCorner(n, n) = Ppred; return true; }
  // The dense Joseph form stays: expanding or regrouping it (n^2 m instead of n^3, another 25 %
  // of replay CPU) cost testswing ~1.4 % tracking in fresh rooms, losing its numerical care.
  const Mat IKH = Mat::Identity(n, n) - K * H;
  P.topLeftCorner(n, n) = IKH * Ppred * IKH.transpose() + K * Rk * K.transpose();
  return true;
}

}  // namespace lh::direct
