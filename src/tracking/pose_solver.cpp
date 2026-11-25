#include "pose_solver.hpp"

#include <ceres/jet.h>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "lighthouse_model.hpp"

namespace lh::solver {
namespace {

constexpr double kGolden1 = 0.3819660112501051;  // 0x1a9a98
constexpr double kGolden2 = 0.6180339887498949;  // 0x1a9b18
constexpr double kMaxRange = 20.0;               // 0x1a9700
constexpr double kBehindPenalty = 1e50;          // 0x1a9a08
constexpr double kLineSearchTol = 0.01;          // 0x1a9738
constexpr double kStopTol = 1e-8;                // 0x1a99c0
constexpr double kSpread = 0.0085;               // 0x1a9748
constexpr double kPi = 3.141592653589793;
constexpr double kMultiStartNoise = 2.5 * 0.0001884955592153876;
constexpr double kTwoPi = 6.283185307179586;

using Jet7 = ceres::Jet<double, 7>;

double Wrap(double r) {
  // 0x3a2220
  if (kPi <= r || r < -kPi) {
    if (kPi < r) r -= kTwoPi;
    if (r < -kPi) r += kTwoPi;
  }
  return r;
}

struct Term {
  int sensorIndex = 0;
  int axes = 0;  // bit0: axis 0 residual, bit1: axis 1 residual; 0: quaternion normalizer
};

template <typename T>
void EvalTerm(const Term& t, const TrackedObjectModel& obj, const double* cal14, int baseModel, const T* x, T* out, int* n) {
  if (t.axes == 0) {
    out[(*n)++] = 0.25 * (1.0 - (x[3] * x[3] + x[4] * x[4] + x[5] * x[5] + x[6] * x[6]));
    return;
  }
  // Sensor point in base frame, operation order of 0x3abdf0.
  const auto& s = obj.points[size_t(t.sensorIndex)];
  const T w = x[3], qx = x[4], qy = x[5], qz = x[6];
  const T nx2 = -(qx * qx);
  T d0 = ((qx * qy - w * qz) * s[1] - (qy * qy + qz * qz) * s[0]) + (w * qy + qx * qz) * s[2];
  T d1 = (w * qz + qx * qy) * s[0] + (nx2 - qz * qz) * s[1] + (qy * qz - qx * w) * s[2];
  T d2 = s[2] * (nx2 - qy * qy) + s[1] * (qy * qz + qx * w) + s[0] * (qx * qz - w * qy);
  const T px = d0 + d0 + x[0] + s[0];
  const T py = x[1] + s[1] + d1 + d1;
  const T pz = d2 + d2 + x[2] + s[2];
  if (t.axes & 1) out[(*n)++] = model::BaseAngle(baseModel, 0, px, py, pz, cal14);
  if (t.axes & 2) out[(*n)++] = model::BaseAngle(baseModel, 1, px, py, pz, cal14);
}

struct Problem {
  const TrackedObjectModel* obj;
  const double* cal;
  int model = 1;  // base measurement model (BaseConfig::model)
  std::vector<Term> terms;
  std::vector<double> z;
  int N = 0;

  template <typename T>
  void Eval(const T* x, std::vector<T>* h) const {
    h->resize(size_t(N));
    int n = 0;
    for (const auto& t : terms) EvalTerm(t, *obj, cal, model, x, h->data(), &n);
  }
  // Line-search objective (0x3a26c0): cost at x − α·step with q renormalized.
  double Cost(const double* x, const Eigen::Matrix<double, 7, 1>& step, double alpha) const {
    double y[7];
    for (int i = 0; i < 7; i++) y[i] = x[i] - step[i] * alpha;
    const double qn2 = y[4] * y[4] + y[6] * y[6] + y[5] * y[5] + y[3] * y[3];
    if (0.0 < qn2) {
      const double qn = std::sqrt(qn2);
      for (int i = 3; i < 7; i++) y[i] /= qn;
    }
    if (0.0 <= y[2]) return (alpha + 1.0) * kBehindPenalty;
    std::vector<double> h;
    Eval(y, &h);
    double c = 0;
    for (int i = 0; i < N; i++) {
      const double r = Wrap(h[size_t(i)] - z[size_t(i)]);
      c = r * r + c;
    }
    return detail::LineSearchCostOrPenalty(c, alpha);
  }
};

bool NonCoplanar(const TrackedObjectModel& obj, const std::vector<int>& channels) {
  // 0x3a2e80: some triangle with height ≥ 8.5 mm and a fourth sensor off its plane.
  std::vector<const std::array<double, 3>*> p;
  for (int c : channels) {
    const int idx = obj.IndexOf(c);
    if (idx < 0 || size_t(idx) >= obj.points.size()) return false;
    p.push_back(&obj.points[size_t(idx)]);
  }
  const size_t n = p.size();
  if (n < 3) return false;
  for (size_t i = 0; i < n; i++)
    for (size_t j = i + 1; j < n; j++)
      for (size_t k = j + 1; k < n; k++) {
        const auto &a = *p[i], &b = *p[j], &c = *p[k];
        const double ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        const double ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        const double bc[3] = {c[0] - b[0], c[1] - b[1], c[2] - b[2]};
        double nx = ab[1] * ac[2] - ab[2] * ac[1];
        double ny = ab[2] * ac[0] - ab[0] * ac[2];
        double nz = ab[0] * ac[1] - ab[1] * ac[0];
        double e = std::max(ac[0] * ac[0] + ac[1] * ac[1] + ac[2] * ac[2], ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]);
        e = std::max(e, bc[0] * bc[0] + bc[1] * bc[1] + bc[2] * bc[2]);
        const double n2 = nx * nx + ny * ny + nz * nz;
        if (n2 != 0.0) {
          const double inv = 1.0 / std::sqrt(n2);
          nx *= inv; ny *= inv; nz *= inv;
        }
        if (!(kSpread <= std::sqrt(n2 / e))) continue;
        for (size_t m = 0; m < n; m++) {
          if (m == i || m == j || m == k) continue;
          const auto& d = *p[m];
          const double dist = (d[0] - a[0]) * nx + (d[1] - a[1]) * ny + (d[2] - a[2]) * nz;
          if (kSpread < std::fabs(dist)) return true;
        }
      }
  return false;
}

}  // namespace

namespace detail {
double LineSearchCostOrPenalty(double cost, double alpha) {
  // A non-finite LH2 residual is outside the model domain, not an inexpensive
  // trial. Give it the same ordered penalty as a pose behind the base. Otherwise
  // fa <= fb and fa < fb are both false for NaN, and the line search walks toward
  // the invalid side until the whole pose becomes NaN.
  return std::isfinite(cost) ? cost : (alpha + 1.0) * kBehindPenalty;
}
}  // namespace detail

bool SamplesSufficient(const TrackedObjectModel& obj, const std::vector<OpticalSample>& s) {
  // 0x3a2e30 / 0x3a3370
  int axisHits[2] = {0, 0}, both = 0;
  uint8_t mask[32] = {};
  for (const auto& x : s) {
    if (x.axis < 0 || x.axis > 1 || x.sensor < 0 || x.sensor >= 32) continue;
    axisHits[x.axis]++;
    mask[x.sensor] |= uint8_t(1 << x.axis);
  }
  for (int i = 0; i < 32; i++) both += mask[i] == 3;
  if (axisHits[0] < 4 || axisHits[1] < 4) return false;
  if (3 < both) {
    std::vector<int> ch;
    for (int i = 0; i < 32; i++)
      if (mask[i] == 3) ch.push_back(i);
    if (NonCoplanar(obj, ch)) return true;
    if (both == axisHits[0]) return false;
  }
  if (both == axisHits[1]) return false;
  for (int axis = 0; axis < 2; axis++) {
    std::vector<int> ch;
    for (int i = 0; i < 32; i++)
      if (mask[i] & (1 << axis)) ch.push_back(i);
    if (!NonCoplanar(obj, ch)) return false;
  }
  return true;
}

Result Solve(const TrackedObjectModel& obj, const BaseConfig& base, std::vector<OpticalSample> samples,
             const Pose* initial, double noise) {
  Result res;
  std::sort(samples.begin(), samples.end(), [](const OpticalSample& a, const OpticalSample& b) {
    return a.sensor < b.sensor || (a.sensor == b.sensor && a.axis < b.axis);
  });
  if (!SamplesSufficient(obj, samples) || base.cal.size() < 14) {
    res.status = 2;
    return res;
  }
  Problem pr;
  pr.obj = &obj;
  pr.cal = base.cal.data();
  pr.model = base.model;
  for (size_t i = 0; i < samples.size();) {
    const auto& s = samples[i];
    const int idx = obj.IndexOf(s.sensor);
    if (idx < 0) {  // map::at throws in the original
      res.status = 2;
      return res;
    }
    if (s.axis != 0) {
      pr.terms.push_back({idx, 2});
      pr.z.push_back(s.angle);
      i += 1;
    } else if (i + 1 < samples.size() && samples[i + 1].sensor == s.sensor && samples[i + 1].axis == 1) {
      pr.terms.push_back({idx, 3});
      pr.z.push_back(s.angle);
      pr.z.push_back(samples[i + 1].angle);
      i += 2;
    } else {
      pr.terms.push_back({idx, 1});
      pr.z.push_back(s.angle);
      i += 1;
    }
  }
  const double sampleCount = double(pr.z.size());
  pr.terms.push_back({0, 0});
  pr.z.push_back(0.0);
  pr.N = int(pr.z.size());
  const double convThreshold = noise * noise * sampleCount;

  double x[7] = {0, 0, -2.0, 1, 0, 0, 0};
  if (initial) {
    x[0] = initial->p[0]; x[1] = initial->p[1]; x[2] = initial->p[2];
    x[3] = initial->q[0]; x[4] = initial->q[1]; x[5] = initial->q[2]; x[6] = initial->q[3];
  }
  bool converged = false;
  int itersLeft = 0x28;
  double lastCost = 0;
  auto finish = [&]() {
    const double dof = pr.N == 0 ? -1.0 : double(pr.N - 1);
    res.cost = lastCost / dof;
    res.error = std::sqrt(std::sqrt(x[2] * x[2] + x[0] * x[0] + x[1] * x[1]) * res.cost);
    res.pose.p = {x[0], x[1], x[2]};
    const double k = 1.0 / std::sqrt(x[3] * x[3] + x[6] * x[6] + x[5] * x[5] + x[4] * x[4]);
    res.pose.q = {x[3] * k, x[4] * k, x[5] * k, x[6] * k};
    res.status = converged ? 0 : 1;
    return res;
  };
  for (;;) {
    res.iterations++;
    const double pn2 = x[2] * x[2] + x[0] * x[0] + x[1] * x[1];
    const double pn = std::sqrt(pn2);
    if (kMaxRange < pn) {
      if (0.0 < pn2) {
        x[2] /= pn; x[0] /= pn; x[1] /= pn;
      }
      converged = false;
    }
    // Residuals and Jacobian.
    Jet7 xj[7];
    for (int i = 0; i < 7; i++) xj[i] = Jet7(x[i], i);
    std::vector<Jet7> h;
    pr.Eval(xj, &h);
    Eigen::MatrixXd J(pr.N, 7);
    Eigen::VectorXd r(pr.N);
    double cost0 = 0;
    for (int i = 0; i < pr.N; i++) {
      r[i] = Wrap(h[size_t(i)].a - pr.z[size_t(i)]);
      cost0 += r[i] * r[i];
      for (int k = 0; k < 7; k++) J(i, k) = h[size_t(i)].v[k];
    }
    const Eigen::Matrix<double, 7, 1> step = (J.transpose() * J).ldlt().solve(J.transpose() * r);
    // Golden-section line search on α ∈ [0, 1].
    double lo = 0, hi = 1.0, a = kGolden1, b = kGolden2;
    double fa = pr.Cost(x, step, a), fb = pr.Cost(x, step, b);
    int it = 2;
    for (;;) {
      const double m = fa <= fb ? fa : fb;
      if (cost0 <= m || fa < fb) {
        const double na = (lo - b) * kGolden2 + b;
        const double nf = pr.Cost(x, step, na);
        hi = b;
        fb = fa;
        b = a;
        fa = nf;
        a = na;
      } else {
        const double nb = (hi - a) * kGolden2 + a;
        const double nf = pr.Cost(x, step, nb);
        lo = a;
        fa = fb;
        a = b;
        fb = nf;
        b = nb;
      }
      if (it == 0x1f) break;
      it++;
      if (!((std::fabs(a) + std::fabs(b)) * kLineSearchTol < std::fabs(hi - lo))) {
        it--;
        break;
      }
    }
    res.lineSearchEvals += it + 1;
    const double fbest = fa <= fb ? fa : fb;
    const double alpha = fa < fb ? a : b;
    if (cost0 < fbest && converged) return finish();
    lastCost = fbest;
    for (int i = 0; i < 7; i++) x[i] = x[i] - step[i] * alpha;
    const double qk = 1.0 / std::sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5] + x[6] * x[6]);
    for (int i = 3; i < 7; i++) x[i] *= qk;
    if (fbest < convThreshold && fbest < convThreshold / std::sqrt(x[2] * x[2] + x[0] * x[0] + x[1] * x[1]))
      converged = true;
    const double predicted = (J * step).squaredNorm();
    if (predicted < (fbest + 1.0) * kStopTol || --itersLeft == 0) return finish();
  }
}

namespace {
// cos of the angle between each sensor's normal and its ray to the base, at a pose.
double Facing(const TrackedObjectModel& obj, const Pose& pose, int sensor) {
  const int idx = obj.IndexOf(sensor);
  if (idx < 0) return -1;
  const Eigen::Quaterniond q(pose.q[0], pose.q[1], pose.q[2], pose.q[3]);
  const Eigen::Vector3d pt = q * Eigen::Vector3d(obj.points[size_t(idx)].data()) + Eigen::Vector3d(pose.p.data());
  const Eigen::Vector3d n = q * Eigen::Vector3d(obj.normals[size_t(idx)].data());
  return n.dot(-pt.normalized());
}
bool FacesAll(const TrackedObjectModel& obj, const Pose& pose, const std::vector<OpticalSample>& samples, double minimum) {
  for (const auto& s : samples) if (Facing(obj, pose, s.sensor) < minimum) return false;
  return true;
}
// The sensor whose samples fit `pose` worst (largest |residual|).
int WorstSensor(const TrackedObjectModel& obj, const BaseConfig& base, const Pose& pose, const std::vector<OpticalSample>& samples) {
  const Eigen::Quaterniond q(pose.q[0], pose.q[1], pose.q[2], pose.q[3]);
  const Eigen::Vector3d t(pose.p.data());
  int worst = -1;
  double largest = -1;
  for (const auto& s : samples) {
    const int idx = obj.IndexOf(s.sensor);
    if (idx < 0 || s.axis < 0 || s.axis > 1) continue;
    const Eigen::Vector3d pt = q * Eigen::Vector3d(obj.points[size_t(idx)].data()) + t;
    const double r = std::abs(Wrap(model::BaseAngle(base.model, s.axis, pt.x(), pt.y(), pt.z(), base.cal.data()) - s.angle));
    if (r > largest) { largest = r; worst = s.sensor; }
  }
  return worst;
}

// Drop what faces away and solve again, until nothing does.
Result WithoutReflections(const TrackedObjectModel& obj, const BaseConfig& base, std::vector<OpticalSample> samples,
                          Result r, std::vector<int>* dropped, double noise) {
  constexpr double kFacing = 0.2;
  for (int round = 0; round < 3 && std::isfinite(r.cost); ++round) {
    std::vector<int> away;
    for (const auto& s : samples)
      if (Facing(obj, r.pose, s.sensor) < kFacing && std::find(away.begin(), away.end(), s.sensor) == away.end()) away.push_back(s.sensor);
    if (away.empty()) return r;
    samples.erase(std::remove_if(samples.begin(), samples.end(), [&](const OpticalSample& s) {
      return std::find(away.begin(), away.end(), s.sensor) != away.end(); }), samples.end());
    if (dropped) dropped->insert(dropped->end(), away.begin(), away.end());
    const Result again = Solve(obj, base, samples, &r.pose, noise);
    if (again.status == 2) { r.status = 2; return r; }
    r = again;
  }
  if (!FacesAll(obj, r.pose, samples, kFacing)) r.status = 1;
  return r;
}
}  // namespace

Result SolveMultiStartImpl(const TrackedObjectModel& obj, const BaseConfig& base, const std::vector<OpticalSample>& samples,
                           bool* ambiguous, std::vector<int>* dropped);

Result SolveMultiStart(const TrackedObjectModel& obj, const BaseConfig& base, const std::vector<OpticalSample>& samples,
                       bool* ambiguous, std::vector<int>* dropped) {
  Result r = SolveMultiStartImpl(obj, base, samples, ambiguous, dropped);
  return r;
}

Result SolveMultiStartImpl(const TrackedObjectModel& obj, const BaseConfig& base, const std::vector<OpticalSample>& samples,
                           bool* ambiguous, std::vector<int>* dropped) {
  if (ambiguous) *ambiguous = false;
  if (dropped) dropped->clear();
  std::vector<int> bestDropped;
  Result best;
  best.status = 2;
  if (base.cal.size() < 14 || samples.empty()) return best;
  // The ray: the direction whose predicted sweep angles match the mean measured ones.
  double mean[2] = {0, 0};
  int count[2] = {0, 0};
  for (const auto& s : samples)
    if (s.axis == 0 || s.axis == 1) { mean[s.axis] += s.angle; ++count[s.axis]; }
  if (!count[0] || !count[1]) return best;
  mean[0] /= count[0]; mean[1] /= count[1];
  Eigen::Vector3d ray(0, 0, -1);
  double rayError = 1e300;
  for (int i = -30; i <= 30; ++i)
    for (int k = -30; k <= 30; ++k) {
      const double u = i * 0.06, v = k * 0.06;
      const double a0 = model::BaseAngle(base.model, 0, u, v, -1.0, base.cal.data());
      const double a1 = model::BaseAngle(base.model, 1, u, v, -1.0, base.cal.data());
      const double e = Wrap(a0 - mean[0]) * Wrap(a0 - mean[0]) + Wrap(a1 - mean[1]) * Wrap(a1 - mean[1]);
      if (e < rayError) { rayError = e; ray = Eigen::Vector3d(u, v, -1).normalized(); }
    }
  // 24 orientations: the rotation group of the cube.
  std::vector<Eigen::Quaterniond> turns;
  for (int face = 0; face < 6; ++face) {
    static const Eigen::Vector3d axes[6] = {Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
                                            -Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ()};
    const Eigen::Quaterniond toFace = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), axes[face]);
    for (int roll = 0; roll < 4; ++roll) turns.push_back(toFace * Eigen::Quaterniond(Eigen::AngleAxisd(roll * kPi / 2, Eigen::Vector3d::UnitZ())));
  }
  std::vector<Result> found;
  // The best few distinct near misses, each with the reflections left out of it: one
  // strong outlier can pull even a start at the true pose well away.
  struct Miss { Result r; std::vector<int> away; };
  std::vector<Miss> misses;
  auto keepMiss = [&](const Result& r, const std::vector<int>& away) {
    for (auto& m : misses)
      if ((Eigen::Vector3d(m.r.pose.p.data()) - Eigen::Vector3d(r.pose.p.data())).norm() < 0.05) {
        if (r.cost < m.r.cost) m = {r, away};
        return;
      }
    misses.push_back({r, away});
    std::sort(misses.begin(), misses.end(), [](const Miss& a, const Miss& b) { return a.r.cost < b.r.cost; });
    if (misses.size() > 8) misses.pop_back();
  };
  for (double distance : {1.0, 2.0, 3.5}) {
    for (const auto& q : turns) {
      Pose start;
      start.p = {ray.x() * distance, ray.y() * distance, ray.z() * distance};
      start.q = {q.w(), q.x(), q.y(), q.z()};
      // A fit within 2.5x the stock noise: a still device can sit on a few sensors
      // whose stock-noise fit never quite converges (live: 330 urad rms for minutes).
      const Result r = Solve(obj, base, samples, &start, kMultiStartNoise);
      if (r.status == 2) return r;  // insufficient samples: no start helps
      if (!std::isfinite(r.cost) || r.pose.p[2] > -0.1) continue;
      // A fit that needs a sensor facing away isn't the pose; neither is a miss
      // caused by one. Either way: without them.
      if (r.status == 1) keepMiss(r, {});  // trimming may find the outlier the facing test can't
      std::vector<int> away;
      const Result clean = FacesAll(obj, r.pose, samples, 0.2) ? r
                                                              : WithoutReflections(obj, base, samples, r, &away, kMultiStartNoise);
      if (!std::isfinite(clean.cost) || clean.pose.p[2] > -0.1) continue;
      if (clean.status != 0) {
        if (clean.status == 1 && !away.empty()) keepMiss(clean, away);
        continue;
      }
      found.push_back(clean);
      if (best.status != 0 || clean.cost < best.cost) { best = clean; bestDropped = away; }
    }
  }
  // Trim the worst-fitting sensor from each near miss, up to three times.
  for (const auto& miss : misses) {
    if (best.status == 0) break;
    std::vector<int> out = miss.away;
    std::vector<OpticalSample> subset;
    for (const auto& s : samples) if (std::find(out.begin(), out.end(), s.sensor) == out.end()) subset.push_back(s);
    Result r = miss.r;
    for (int round = 0; round < 3; ++round) {
      const int worst = WorstSensor(obj, base, r.pose, subset);
      if (worst < 0) break;
      subset.erase(std::remove_if(subset.begin(), subset.end(), [worst](const OpticalSample& s) { return s.sensor == worst; }), subset.end());
      out.push_back(worst);
      r = Solve(obj, base, subset, &r.pose, kMultiStartNoise);
      if (r.status == 2 || !std::isfinite(r.cost)) break;
      if (r.status == 0 && r.pose.p[2] <= -0.1 && FacesAll(obj, r.pose, subset, 0.2)) {
        found.push_back(r);
        if (best.status != 0 || r.cost < best.cost) { best = r; bestDropped = out; }
        break;
      }
    }
  }
  if (best.status != 0) return best;
  if (dropped) *dropped = bestDropped;
  // Another converged pose that differs (> 5 cm or > 15 deg) and fits within 2x.
  const Eigen::Vector3d bp(best.pose.p.data());
  const Eigen::Quaterniond bq(best.pose.q[0], best.pose.q[1], best.pose.q[2], best.pose.q[3]);
  for (const auto& r : found) {
    const Eigen::Quaterniond q(r.pose.q[0], r.pose.q[1], r.pose.q[2], r.pose.q[3]);
    const bool different = (Eigen::Vector3d(r.pose.p.data()) - bp).norm() > 0.05 || bq.angularDistance(q) > 0.26;
    if (different && r.cost < 2 * best.cost && ambiguous) *ambiguous = true;
  }
  return best;
}

}  // namespace lh::solver
