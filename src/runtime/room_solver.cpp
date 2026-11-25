#include "room_solver.hpp"

#include <ceres/ceres.h>
#include <ceres/product_manifold.h>
#include <ceres/rotation.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <set>

#include "tracking/lighthouse_model.hpp"
#include "tracking/pose_solver.hpp"

namespace lh::runtime {
namespace {

std::vector<OpticalSample> Raw(const std::vector<KeySample>& in) {
  std::vector<OpticalSample> out;
  out.reserve(in.size());
  for (const auto& k : in) out.push_back(k.sample);
  return out;
}

constexpr double kSigma = 1e-4;        // rad, typical LH1 sample noise at room distances
constexpr double kHuber = 3.0;         // in sigmas
constexpr int kCalPerAxis = model::kCalPerAxis;

struct Block {
  double v[7] = {1, 0, 0, 0, 0, 0, 0};  // q (w x y z), t (x y z): one block per pose
  double* q = v;
  double* t = v + 4;
  Block() = default;
  Block(const Block& o) { std::copy(o.v, o.v + 7, v); }
  Block& operator=(const Block& o) { std::copy(o.v, o.v + 7, v); return *this; }
  void Set(const Eigen::Isometry3d& pose) {
    const Eigen::Quaterniond r(pose.rotation());
    q[0] = r.w(); q[1] = r.x(); q[2] = r.y(); q[3] = r.z();
    t[0] = pose.translation().x(); t[1] = pose.translation().y(); t[2] = pose.translation().z();
  }
  Eigen::Isometry3d Get() const {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized().toRotationMatrix();
    pose.translation() = Eigen::Vector3d(t[0], t[1], t[2]);
    return pose;
  }
};

// Tangent-space covariance (6x6: rotation, translation) of each free base after a solve.
// Every residual ties one keyframe to one base, so the keyframes are eliminated block by
// block (Schur complement, as DENSE_SCHUR does) and only the small base system is
// inverted. ceres::Covariance factors the whole problem instead: with SuiteSparse that
// is far more work than needed, and without it (Windows) Eigen's SparseQR took ~90% of
// the session's CPU and hundreds of MB.
bool BaseCovariances(ceres::Problem& problem, const std::vector<double*>& keyframes,
                     const std::vector<double*>& bases, std::vector<Eigen::Matrix<double, 6, 6>>* out) {
  if (bases.empty()) return false;
  ceres::Problem::EvaluateOptions eo;
  eo.parameter_blocks = keyframes;  // order: keyframes, then bases; others held constant
  eo.parameter_blocks.insert(eo.parameter_blocks.end(), bases.begin(), bases.end());
  eo.apply_loss_function = true;
  ceres::CRSMatrix j;
  if (!problem.Evaluate(eo, nullptr, nullptr, nullptr, &j)) return false;
  const int kfCols = 6 * int(keyframes.size()), nb = 6 * int(bases.size());
  if (j.num_cols != kfCols + nb) return false;
  std::vector<Eigen::Matrix<double, 6, 6>> hkk(keyframes.size(), Eigen::Matrix<double, 6, 6>::Zero());
  std::vector<Eigen::MatrixXd> hkb(keyframes.size(), Eigen::MatrixXd::Zero(6, nb));
  Eigen::MatrixXd hbb = Eigen::MatrixXd::Zero(nb, nb);
  for (int r = 0; r < j.num_rows; ++r) {
    for (int a = j.rows[r]; a < j.rows[r + 1]; ++a) {
      const int ca = j.cols[a];
      for (int b = j.rows[r]; b < j.rows[r + 1]; ++b) {
        const int cb = j.cols[b];
        const double v = j.values[a] * j.values[b];
        if (ca < kfCols && cb < kfCols) {
          if (ca / 6 == cb / 6) hkk[ca / 6](ca % 6, cb % 6) += v;  // one keyframe per residual
        } else if (ca < kfCols) {
          hkb[ca / 6](ca % 6, cb - kfCols) += v;
        } else if (cb >= kfCols) {
          hbb(ca - kfCols, cb - kfCols) += v;
        }
      }
    }
  }
  Eigen::MatrixXd s = hbb;
  for (size_t k = 0; k < keyframes.size(); ++k) {
    const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(hkk[k]);
    if (ldlt.info() != Eigen::Success || !(ldlt.vectorD().array() > 0).all()) return false;
    s.noalias() -= hkb[k].transpose() * ldlt.solve(hkb[k]);
  }
  const Eigen::LDLT<Eigen::MatrixXd> ldlt(s);
  if (ldlt.info() != Eigen::Success || !(ldlt.vectorD().array() > 0).all()) return false;
  const Eigen::MatrixXd cov = ldlt.solve(Eigen::MatrixXd::Identity(nb, nb));
  out->resize(bases.size());
  for (size_t i = 0; i < bases.size(); ++i) (*out)[i] = cov.block<6, 6>(6 * int(i), 6 * int(i));
  return true;
}

// One LH1 sample. Device pose at the sample time = keyframe pose advanced by the
// filter's velocity and angular rate (both tiny over one LH1 cycle).
struct AngleResidual {
  Eigen::Vector3d sensor;        // device (IMU) frame, already rotated by the angular-rate delta
  Eigen::Vector3d offset;        // world-frame displacement over dt (velocity * dt)
  int axis;
  const double* cal;             // the base's 14 values (both axes)
  double angle;
  int model = 1;                 // base measurement model

  template <typename T>
  bool operator()(const T* base, const T* device, T* residual) const {
    const T* bq = base;
    const T* bt = base + 4;
    const T* dq = device;
    const T* dt = device + 4;
    const T s[3] = {T(sensor.x()), T(sensor.y()), T(sensor.z())};
    T w[3];
    ceres::QuaternionRotatePoint(dq, s, w);
    for (int i = 0; i < 3; ++i) w[i] += dt[i] + T(offset[i]) - bt[i];
    const T inv[4] = {bq[0], -bq[1], -bq[2], -bq[3]};
    T b[3];
    ceres::QuaternionRotatePoint(inv, w, b);
    residual[0] = (model::BaseAngle(model, axis, b[0], b[1], b[2], cal) - T(angle)) / T(kSigma);
    return true;
  }
};

// AngleResidual with the rigid transform differentiated by hand: only the sweep model is
// auto-differentiated, against the 3 coordinates of the sensor in the base frame (Jet<3>
// instead of Jet<14> through both rotations). Same Jacobian, several times cheaper.
class AngleCost : public ceres::SizedCostFunction<1, 7, 7> {
 public:
  explicit AngleCost(const AngleResidual& r) : r_(r) {}

  bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
    const double* bq = parameters[0];
    const double* bt = parameters[0] + 4;
    const double* dq = parameters[1];
    const double* dt = parameters[1] + 4;
    const Eigen::Quaterniond qb = Eigen::Quaterniond(bq[0], bq[1], bq[2], bq[3]).normalized();
    const Eigen::Quaterniond qd = Eigen::Quaterniond(dq[0], dq[1], dq[2], dq[3]).normalized();
    const Eigen::Matrix3d rb = qb.toRotationMatrix(), rd = qd.toRotationMatrix();
    const Eigen::Vector3d rotated = rd * r_.sensor;
    const Eigen::Vector3d w = rotated + Eigen::Vector3d(dt[0], dt[1], dt[2]) + r_.offset - Eigen::Vector3d(bt[0], bt[1], bt[2]);
    const Eigen::Vector3d b = rb.transpose() * w;
    using J3 = ceres::Jet<double, 3>;
    const J3 angle = model::BaseAngle(r_.model, r_.axis, J3(b.x(), 0), J3(b.y(), 1), J3(b.z(), 2), r_.cal);
    if (!std::isfinite(angle.a)) {  // LH2: outside the plane model's domain; no information
      residuals[0] = 0;
      for (int k = 0; k < 2; ++k)
        if (jacobians && jacobians[k]) std::fill(jacobians[k], jacobians[k] + 7, 0.0);
      return true;
    }
    residuals[0] = (angle.a - r_.angle) / kSigma;
    if (!jacobians) return true;
    const Eigen::RowVector3d dAngle = angle.v.transpose() / kSigma;  // d residual / d b
    if (jacobians[0]) {  // base: b = Rb^T (w), via conj(qb) rotating w; and -Rb^T for its translation
      const Eigen::Matrix<double, 3, 4> dq4 =
          RotateJacobian(Eigen::Quaterniond(qb.w(), -qb.x(), -qb.y(), -qb.z()), w) *
          Eigen::Vector4d(1, -1, -1, -1).asDiagonal();
      Eigen::Map<Eigen::Matrix<double, 1, 7>> j(jacobians[0]);
      j.head<4>() = dAngle * dq4;
      j.tail<3>() = -dAngle * rb.transpose();
    }
    if (jacobians[1]) {  // device: w = Rd s + t
      Eigen::Map<Eigen::Matrix<double, 1, 7>> j(jacobians[1]);
      const Eigen::RowVector3d dW = dAngle * rb.transpose();
      j.head<4>() = dW * RotateJacobian(qd, r_.sensor);
      j.tail<3>() = dW;
    }
    return true;
  }

 private:
  // d(R(q) p) / d(w, x, y, z) for a unit q, from R(q) p = p + 2w (v x p) + 2 v x (v x p).
  // Only the tangent directions (orthogonal to q) reach the solver through the quaternion
  // manifold, so the normalization's radial term doesn't matter.
  static Eigen::Matrix<double, 3, 4> RotateJacobian(const Eigen::Quaterniond& q, const Eigen::Vector3d& p) {
    const Eigen::Vector3d v = q.vec();
    Eigen::Matrix3d px;
    px << 0, -p.z(), p.y(), p.z(), 0, -p.x(), -p.y(), p.x(), 0;
    Eigen::Matrix<double, 3, 4> j;
    j.col(0) = 2 * v.cross(p);
    j.rightCols<3>() = -2 * q.w() * px + 2 * (v.dot(p) * Eigen::Matrix3d::Identity() + v * p.transpose() -
                                              2 * p * v.transpose());
    return j;
  }
  AngleResidual r_;
};

BaseConfig ConfigFrom(uint32_t serial, const std::array<double, 14>& cal, int baseModel = 1) {
  BaseConfig c;
  c.model = baseModel;
  c.valid = true;
  c.serial = serial;
  c.cal.assign(cal.begin(), cal.end());
  return c;
}

Eigen::Isometry3d SolverPose(const solver::Pose& p) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = Eigen::Quaterniond(p.q[0], p.q[1], p.q[2], p.q[3]).normalized().toRotationMatrix();
  t.translation() = Eigen::Vector3d(p.p[0], p.p[1], p.p[2]);
  return t;
}

// Robust average of pose candidates: largest 10 cm / 3 deg consensus set, then mean.
std::optional<Eigen::Isometry3d> Consensus(const std::vector<Eigen::Isometry3d>& c) {
  if (c.empty()) return std::nullopt;
  size_t best = 0, bestCount = 0;
  auto close = [](const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
    return (a.translation() - b.translation()).norm() < 0.10 &&
           Eigen::AngleAxisd(a.rotation().transpose() * b.rotation()).angle() < 0.0524;
  };
  for (size_t i = 0; i < c.size(); ++i) {
    size_t n = 0;
    for (const auto& o : c) n += close(c[i], o);
    if (n > bestCount) { bestCount = n; best = i; }
  }
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  Eigen::Vector4d q = Eigen::Vector4d::Zero();
  const Eigen::Quaterniond ref(c[best].rotation());
  for (const auto& o : c) {
    if (!close(c[best], o)) continue;
    t += o.translation();
    Eigen::Quaterniond r(o.rotation());
    if (r.dot(ref) < 0) r.coeffs() *= -1;
    q += r.coeffs();
  }
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = t / double(bestCount);
  out.linear() = Eigen::Quaterniond(q.normalized()).toRotationMatrix();
  return out;
}


}  // namespace

RoomSolveOutput SolveRoom(const RoomSolveInput& in) {
  const auto start = std::chrono::steady_clock::now();
  RoomSolveOutput out;
  if (!in.origin || !in.known.count(in.origin)) {
    out.summary = "no origin";
    return out;
  }
  // ---- initial device poses ------------------------------------------------------
  struct KF {
    const Keyframe* k;
    bool fresh = false;
    const TrackedObjectModel* model;
    Block pose;
    Eigen::Matrix3d worldFromPrimaryR = Eigen::Matrix3d::Identity();
    bool ok = false;
  };
  std::vector<KF> kfs;
  const size_t firstFresh = in.fresh < 0 ? 0 : in.keyframes.size() - std::min(in.keyframes.size(), size_t(in.fresh));
  for (size_t index = 0; index < in.keyframes.size(); ++index) {
    const auto& kp = in.keyframes[index];
    const auto model = in.models.find(kp->device);
    if (model == in.models.end()) continue;
    KF kf{kp.get(), index >= firstFresh, model->second.get(), {}, {}, false};
    const auto primary = in.known.find(kp->primary);
    if (primary != in.known.end()) {
      kf.pose.Set(primary->second * kp->primaryFromDevice);
      kf.worldFromPrimaryR = primary->second.rotation();
      kf.ok = true;
    } else {
      for (const auto& [base, samples] : kp->samples) {
        const auto known = in.known.find(base);
        const auto cal = in.calibration.find(base);
        if (known == in.known.end() || cal == in.calibration.end()) continue;
        const auto bm = in.baseModels.find(base);
        const auto r = solver::Solve(*kf.model, ConfigFrom(base, cal->second, bm == in.baseModels.end() ? 1 : bm->second),
                                     Raw(samples));
        if (r.status == 2 || !std::isfinite(r.cost) || r.cost > 1e-6) continue;
        kf.pose.Set(known->second * SolverPose(r.pose));
        kf.worldFromPrimaryR = kf.pose.Get().rotation() * kp->primaryFromDevice.rotation().transpose();
        kf.ok = true;
        break;
      }
    }
    if (kf.ok) kfs.push_back(kf);
  }
  // ---- initial base poses ----------------------------------------------------------
  std::map<uint32_t, Block> bases;
  std::map<uint32_t, int> seen;
  for (const auto& kf : kfs)
    for (const auto& [base, s] : kf.k->samples)
      if (in.calibration.count(base)) ++seen[base];
  for (const auto& [base, count] : seen) {
    const auto known = in.known.find(base);
    if (known != in.known.end()) {
      bases[base].Set(known->second);
      continue;
    }
    std::vector<Eigen::Isometry3d> candidates;
    for (const auto& kf : kfs) {
      const auto s = kf.k->samples.find(base);
      if (s == kf.k->samples.end()) continue;
      const auto bm = in.baseModels.find(base);
      const auto r = solver::Solve(*kf.model, ConfigFrom(base, in.calibration.at(base), bm == in.baseModels.end() ? 1 : bm->second),
                                   Raw(s->second));
      // The bootstrap solver often stops at its iteration limit with an excellent fit.
      if (r.status == 2 || !std::isfinite(r.cost) || r.cost > 1e-6) continue;
      candidates.push_back(kf.pose.Get() * SolverPose(r.pose).inverse());
      if (candidates.size() >= 40) break;
    }
    if (auto pose = Consensus(candidates)) bases[base].Set(*pose);
  }
  if (bases.size() < 2) {
    out.summary = "need two bases";
    return out;
  }

  // ---- bundle adjustment ------------------------------------------------------------------
  auto solve = [&](const std::vector<char>& use, std::vector<double>* kfRms) {
    ceres::Problem::Options po;
    po.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem(po);
    ceres::ProductManifold<ceres::QuaternionManifold, ceres::EuclideanManifold<3>> poseManifold;
    auto* ordering = new ceres::ParameterBlockOrdering;
    std::set<uint32_t> used;
    int observations = 0;
    std::vector<std::vector<ceres::ResidualBlockId>> ids(kfs.size());
    for (size_t i = 0; i < kfs.size(); ++i) {
      if (!use[i]) continue;
      auto& kf = kfs[i];
      int linked = 0;
      for (const auto& [base, s] : kf.k->samples) linked += bases.count(base) != 0;
      if (linked < 2) continue;
      for (const auto& [base, samples] : kf.k->samples) {
        auto b = bases.find(base);
        if (b == bases.end()) continue;
        const auto& cal = in.calibration.at(base);
        const auto bm = in.baseModels.find(base);
        const int baseModel = bm == in.baseModels.end() ? 1 : bm->second;
        const Eigen::Isometry3d worldFromBase = b->second.Get(), worldFromDevice = kf.pose.Get();
        for (const auto& ks : samples) {
          const auto& s = ks.sample;
          if (s.axis < 0 || s.axis > 1) continue;
          // Same facing gate as the filter (0x36f740, cos >= 0.3): back-facing hits
          // are reflections.
          const Eigen::Vector3d point = worldFromDevice * ks.point;
          const Eigen::Vector3d normal = worldFromDevice.rotation() * ks.normal;
          const Eigen::Vector3d toSensor = (point - worldFromBase.translation()).normalized();
          if (-normal.dot(toSensor) < 0.3) continue;
          const AngleResidual residual{ks.point, Eigen::Vector3d::Zero(), s.axis, cal.data(), s.angle, baseModel};
          auto* cost = new AngleCost(residual);
          ids[i].push_back(problem.AddResidualBlock(cost, new ceres::HuberLoss(kHuber), b->second.v, kf.pose.v));
          ++observations;
          // Only a base with a residual is in the problem: one whose samples all failed the
          // facing gate isn't, and SetManifold on it was a fatal Ceres error (abort).
          used.insert(base);
        }
      }
      if (!ids[i].empty()) {
        problem.SetManifold(kf.pose.v, &poseManifold);
        ordering->AddElementToGroup(kf.pose.v, 0);
      }
    }
    if (observations == 0) { delete ordering; return std::make_pair(false, 0); }
    for (uint32_t base : used) {
      auto& b = bases.at(base);
      problem.SetManifold(b.v, &poseManifold);
      ordering->AddElementToGroup(b.v, 1);
      if (base == in.origin || (!in.refineBases && in.known.count(base))) problem.SetParameterBlockConstant(b.v);
    }
    if (!used.count(in.origin)) {
      // Gauge: keep every already-known base fixed when the origin is not observed.
      bool fixed = false;
      for (uint32_t base : used)
        if (in.known.count(base)) {
          problem.SetParameterBlockConstant(bases.at(base).v);
          fixed = true;
        }
      if (!fixed) { delete ordering; return std::make_pair(false, 0); }
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.linear_solver_ordering.reset(ordering);
    options.max_num_iterations = 40;
    options.num_threads = 4;
    options.function_tolerance = 1e-9;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (kfRms) {
      kfRms->assign(kfs.size(), -1);
      for (size_t i = 0; i < kfs.size(); ++i) {
        if (ids[i].empty()) continue;
        double sum = 0;
        for (auto id : ids[i]) {
          double cost = 0;
          problem.EvaluateResidualBlock(id, false, &cost, nullptr, nullptr);
          sum += 2 * cost;  // cost = r²/2 (before loss)
        }
        (*kfRms)[i] = std::sqrt(sum / double(ids[i].size())) * kSigma;
      }
    }
    // Covariance of the solved bases (tangent space), when computable.
    if (summary.IsSolutionUsable()) {
      std::vector<double*> kfBlocks, freeBases;
      for (size_t i = 0; i < kfs.size(); ++i)
        if (!ids[i].empty()) kfBlocks.push_back(kfs[i].pose.v);
      for (uint32_t base : used)
        if (base != in.origin && !problem.IsParameterBlockConstant(bases.at(base).v))
          freeBases.push_back(bases.at(base).v);
      std::vector<Eigen::Matrix<double, 6, 6>> covs;
      const bool haveCov = BaseCovariances(problem, kfBlocks, freeBases, &covs);
      for (uint32_t base : used) {
        auto& sb = out.bases[base];
        sb.worldFromBase = bases.at(base).Get();
        if (base == in.origin || problem.IsParameterBlockConstant(bases.at(base).v)) continue;
        const auto at = std::find(freeBases.begin(), freeBases.end(), bases.at(base).v) - freeBases.begin();
        if (haveCov) {
          const auto& c = covs[size_t(at)];  // tangent: [rotation(3), translation(3)]
          sb.rotationVariance = 4 * c.diagonal().head<3>().sum();  // tangent δ is a half angle
          sb.positionVariance = c.diagonal().tail<3>().sum();
        } else {
          sb.positionVariance = 1e-4;
          sb.rotationVariance = 1e-4;
        }
      }
    }
    out.observations = observations;
    return std::make_pair(summary.IsSolutionUsable(), int(summary.iterations.size()));
  };

  std::vector<char> use(kfs.size(), 1), excluded(kfs.size(), 0);
  std::vector<double> rms;
  auto [ok, iterations] = solve(use, &rms);
  if (!ok) { out.summary = "solve failed"; return out; }
  // A device whose keyframes typically fit several times worse than every other device's
  // is out of this solve: its own model or calibration is off, and its many keyframes,
  // each under the per-keyframe limit below, dragged the room around (live: a headset
  // 0.3-0.7 m / 5-14 deg off the room's base pairs moved locked bases 5-6 cm per solve).
  {
    std::map<std::string, std::vector<double>> perDevice;
    for (size_t i = 0; i < kfs.size(); ++i)
      if (rms[i] >= 0) perDevice[kfs[i].k->device].push_back(rms[i]);
    std::map<std::string, double> medians;
    for (auto& [device, v] : perDevice) {
      if (v.size() < 10) continue;
      std::sort(v.begin(), v.end());
      medians[device] = v[v.size() / 2];
    }
    // Only a clearly broken device, one at a time, and never most of the window:
    // controllers fit ~3x worse than trackers by nature (fast, grazing), and leaving them
    // out at 3x cut a live solve from 395 keyframes to 87 and moved the room 35-41 mm.
    if (medians.size() >= 3) {
      std::string worst;
      double worstRatio = 0;
      for (const auto& [device, m] : medians) {
        std::vector<double> others;
        for (const auto& [d, o] : medians) if (d != device) others.push_back(o);
        std::sort(others.begin(), others.end());
        const double ratio = m / std::max(others[others.size() / 2], 1e-9);
        if (m > 3e-4 && ratio > worstRatio) { worstRatio = ratio; worst = device; }
      }
      if (worstRatio > 8 && perDevice[worst].size() * 3 <= kfs.size()) {
        out.excludedDevices.emplace_back(worst, worstRatio);
        for (size_t i = 0; i < kfs.size(); ++i)
          if (kfs[i].k->device == worst) { use[i] = 0; excluded[i] = 1; }
      }
      if (!out.excludedDevices.empty()) {
        out.bases.clear();
        std::tie(ok, iterations) = solve(use, &rms);
        if (!ok) { out.summary = "solve failed after leaving a device out"; return out; }
      }
    }
  }
  // Reject keyframes that do not fit (wrong pose, reflections), then solve again.
  std::vector<double> sorted;
  for (double r : rms) if (r >= 0) sorted.push_back(r);
  std::sort(sorted.begin(), sorted.end());
  const double median = sorted.empty() ? 0 : sorted[sorted.size() / 2];
  const double limit = std::max(4 * median, 5e-4);
  int rejected = 0;
  for (size_t i = 0; i < kfs.size(); ++i)
    if (!excluded[i] && rms[i] > limit) { use[i] = 0; ++rejected; }
  if (rejected) {
    out.bases.clear();
    std::tie(ok, iterations) = solve(use, &rms);
    if (!ok) { out.summary = "solve failed after rejection"; return out; }
  }
  std::map<uint32_t, std::pair<int, int>> seenRejected;
  std::map<uint32_t, std::map<std::string, int>> rejectedBy;  // base -> device -> rejected keyframes
  std::map<uint32_t, std::set<std::string>> seenBy;           // base -> devices with a keyframe of it
  for (size_t i = 0; i < kfs.size(); ++i) {
    if (excluded[i]) continue;
    for (const auto& [base, s] : kfs[i].k->samples) {
      seenBy[base].insert(kfs[i].k->device);
      ++seenRejected[base].first;
      if (!use[i]) { ++seenRejected[base].second; ++rejectedBy[base][kfs[i].k->device]; }
    }
  }
  // Per device, its samples of a base at the solved pose against the room's pose: the
  // devices that fit the solved one clearly better are the ones whose light says it moved.
  auto deviceSupport = [&](uint32_t base, const Eigen::Isometry3d& known, int* checked, int* supporting) {
    std::map<std::string, std::array<double, 3>> acc;  // device -> (sum at solved, sum at known, keyframes)
    const auto& cal = in.calibration.at(base);
    const auto bm = in.baseModels.find(base);
    const int baseModel = bm == in.baseModels.end() ? 1 : bm->second;
    Block knownBlock;
    knownBlock.Set(known);
    const double* solvedV = bases.at(base).v;
    for (size_t i = 0; i < kfs.size(); ++i) {
      if (!use[i] || !kfs[i].k->settled) continue;
      const auto s = kfs[i].k->samples.find(base);
      if (s == kfs[i].k->samples.end()) continue;
      auto& a = acc[kfs[i].k->device];
      a[2] += 1;
      for (const auto& ks : s->second) {
        if (ks.sample.axis < 0 || ks.sample.axis > 1) continue;
        const AngleCost cost(AngleResidual{ks.point, Eigen::Vector3d::Zero(), ks.sample.axis, cal.data(), ks.sample.angle, baseModel});
        double r = 0;
        const double* p1[2] = {solvedV, kfs[i].pose.v};
        cost.Evaluate(p1, &r, nullptr);
        a[0] += std::min(r * r, kHuber * kHuber * 4);
        const double* p2[2] = {knownBlock.v, kfs[i].pose.v};
        cost.Evaluate(p2, &r, nullptr);
        a[1] += std::min(r * r, kHuber * kHuber * 4);
      }
    }
    for (const auto& [device, a] : acc) {
      if (a[2] < 5) continue;
      ++*checked;
      if (a[1] > 1.5 * a[0]) ++*supporting;
    }
  };
  // A base whose every keyframe was rejected is in no solve: reported at its room pose
  // with no keyframes, so the caller still sees its rejections.
  for (const auto& [base, sr] : seenRejected) {
    const auto known = in.known.find(base);
    if (sr.second > 0 && !out.bases.count(base) && known != in.known.end()) out.bases[base].worldFromBase = known->second;
  }
  // Per-base fit statistics.
  std::map<uint32_t, std::pair<double, int>> fit;
  std::map<uint32_t, int> samples, fresh;
  std::map<uint32_t, std::map<std::string, std::pair<double, double>>> when;  // base -> device -> time range
  for (size_t i = 0; i < kfs.size(); ++i) {
    if (!use[i] || rms[i] < 0) continue;
    for (const auto& [base, s] : kfs[i].k->samples) {
      if (!bases.count(base)) continue;
      auto [range, added] = when[base].try_emplace(kfs[i].k->device, kfs[i].k->time, kfs[i].k->time);
      range->second.first = std::min(range->second.first, kfs[i].k->time);
      range->second.second = std::max(range->second.second, kfs[i].k->time);
      fit[base].first += rms[i] * rms[i];
      ++fit[base].second;
      samples[base] += int(s.size());
      fresh[base] += kfs[i].fresh;
    }
  }
  double total = 0;
  int count = 0;
  for (auto& [base, sb] : out.bases) {
    sb.keyframes = fit[base].second;
    sb.seen = seenRejected[base].first;
    sb.rejected = seenRejected[base].second;
    for (const auto& [device, n] : rejectedBy[base]) sb.rejectingDevices += n >= 5;
    sb.seeingDevices = int(seenBy[base].size());
    if (const auto known = in.known.find(base); known != in.known.end() && base != in.origin && in.calibration.count(base) && bases.count(base))
      deviceSupport(base, known->second, &sb.checkedDevices, &sb.supportingDevices);
    sb.fresh = fresh[base];
    sb.devices = int(when[base].size());
    for (const auto& [device, range] : when[base]) sb.span = std::max(sb.span, range.second - range.first);
    sb.rms = sb.keyframes ? std::sqrt(fit[base].first / sb.keyframes) : 0;
    // The covariance takes every sample as independent at kSigma. They are not: the
    // samples of a keyframe share its device pose errors (the filter's motion over the
    // window, the device's model), and solves from each device alone disagree by
    // 1-4 cm where it claims 0.5 mm. Count a keyframe as one measurement, at the
    // fit's own noise.
    if (sb.keyframes && samples[base] > sb.keyframes) {
      const double noise = std::max(1.0, sb.rms / kSigma);
      const double inflate = noise * noise * double(samples[base]) / double(sb.keyframes);
      sb.positionVariance *= inflate;
      sb.rotationVariance *= inflate;
    }
    total += fit[base].first;
    count += fit[base].second;
  }
  out.rms = count ? std::sqrt(total / count) : 0;
  out.keyframesUsed = int(std::count(use.begin(), use.end(), 1));
  out.keyframesRejected = rejected;
  out.ok = true;
  out.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  char text[160];
  std::snprintf(text, sizeof text, "%d keyframes (%d rejected), %d samples, rms %.0f urad, %d iterations, %.0f ms",
                out.keyframesUsed, rejected, out.observations, out.rms * 1e6, iterations, out.seconds * 1000);
  out.summary = text;
  return out;
}

std::optional<Eigen::Isometry3d> SolveFromBase(const Keyframe& keyframe, uint32_t base, const TrackedObjectModel& model,
                                               const std::array<double, 14>& calibration, int baseModel) {
  const auto s = keyframe.samples.find(base);
  if (s == keyframe.samples.end()) return std::nullopt;
  const auto r = solver::Solve(model, ConfigFrom(base, calibration, baseModel), Raw(s->second));
  if (r.status == 2 || !std::isfinite(r.cost) || r.cost > 1e-6) return std::nullopt;
  return SolverPose(r.pose);
}

bool KeyframeStore::Add(std::shared_ptr<const Keyframe> keyframe, const std::optional<Eigen::Isometry3d>& pose) {
  if (pose) {
    for (const auto& e : entries_) {
      if (e.keyframe->device != keyframe->device || !e.pose) continue;
      if ((e.pose->translation() - pose->translation()).norm() < 0.03 &&
          Eigen::AngleAxisd(e.pose->rotation().transpose() * pose->rotation()).angle() < 0.05 &&
          std::fabs(e.keyframe->time - keyframe->time) < 30.0)
        return false;  // nearly the same view; keep the store diverse
    }
  }
  if (entries_.size() >= capacity_) entries_.erase(entries_.begin());
  entries_.push_back({std::move(keyframe), pose});
  return true;
}

void KeyframeStore::DropBase(uint32_t base) {
  entries_.erase(std::remove_if(entries_.begin(), entries_.end(),
                                [&](const Entry& e) { return e.keyframe->primary == base || e.keyframe->samples.contains(base); }),
                 entries_.end());
}

std::vector<std::shared_ptr<const Keyframe>> KeyframeStore::All() const {
  std::vector<std::shared_ptr<const Keyframe>> out;
  out.reserve(entries_.size());
  for (const auto& e : entries_) out.push_back(e.keyframe);
  return out;
}

}  // namespace lh::runtime
