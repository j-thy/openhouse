#include "hand_skeleton.hpp"

#include "platform/platform.hpp"
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <vector>

// Port of the stock Index hand (driver_lighthouse CHandPoseLogic_IndexController,
// vtable 0x470988): setup 0x381470, per-frame state 0x27ff20, thumb state choice
// 0x27fd10, the WithController graph 0x280c70 (fingers 0x2808d0, A-button override
// 0x280bc0) and the WithoutController graph 0x280e80 (thumb 0x280cd0). Its building
// blocks: poses from a library frame (0x287020) or a clip at normalized time
// (0x286f50), masked blends (0x286bc0), a 3x3 bilinear pose grid (0x27c280), a
// critically damped spring (0x27cc40) and a Schlick bias curve (0x27ce90).
namespace lh::openvr {
namespace {
using json = nlohmann::json;
using Bones = HandSkeleton::Bones;

uint32_t Word(const std::vector<uint8_t>& b, size_t at) {
  if (at + 4 > b.size()) throw std::runtime_error("truncated GLB");
  uint32_t v; std::memcpy(&v, b.data() + at, 4); return v;
}
Eigen::Quaternionf Q(const vr::VRBoneTransform_t& b) {
  return {b.orientation.w, b.orientation.x, b.orientation.y, b.orientation.z};
}
void SetQ(vr::VRBoneTransform_t& b, Eigen::Quaternionf q) {
  q.normalize(); b.orientation = {q.w(), q.x(), q.y(), q.z()};
}
// Children of the wrist: thumb 2, index 6, middle 11, ring 16, pinky 21.
bool IsMetacarpal(int bone) { return bone == 2 || (bone >= 6 && bone < 26 && (bone - 6) % 5 == 0); }

// Valve's bone masks (resources/anims/bonemask_*.txt): one finger's chain.
enum Mask { kThumb, kIndex, kMiddle, kRing, kPinky };
int MaskBegin(Mask m) { return m == kThumb ? 2 : 6 + 5 * (int(m) - 1); }
int MaskEnd(Mask m) { return m == kThumb ? 6 : MaskBegin(m) + 5; }

// a -> b by w on one finger (0x286bc0); w outside [0, 1] is clamped as stock does.
void Blend(Bones& a, const Bones& b, float w, Mask m) {
  w = std::clamp(std::isfinite(w) ? w : 0.f, 0.f, 1.f);
  for (int i = MaskBegin(m); i < MaskEnd(m); ++i) {
    SetQ(a[i], Q(a[i]).slerp(w, Q(b[i])));
    for (int k = 0; k < 3; ++k) a[i].position.v[k] += w * (b[i].position.v[k] - a[i].position.v[k]);
  }
}
Bones Lerp(const Bones& a, const Bones& b, float w) {
  w = std::clamp(std::isfinite(w) ? w : 0.f, 0.f, 1.f);
  Bones out = a;
  for (int i = 0; i < 31; ++i) {
    SetQ(out[i], Q(a[i]).slerp(w, Q(b[i])));
    for (int k = 0; k < 3; ++k) out[i].position.v[k] += w * (b[i].position.v[k] - a[i].position.v[k]);
  }
  return out;
}

// 0x27ce90 mode 0: Schlick's bias.
float Bias(float x, float b) { return x / ((1.f / b - 2.f) * (1.f - x) + 1.f); }
// 0x27d080: v from [lo, hi] onto [out0, out1], clamped.
float Remap(float v, float lo, float hi, float out0, float out1) {
  const float t = hi == lo ? 0.f : std::clamp((v - lo) / (hi - lo), 0.f, 1.f);
  return out0 + (out1 - out0) * t;
}
// 0x27cc40: critically damped spring toward target at rate omega (1/s).
void Spring(float target, float dt, float omega, float* value, float* velocity) {
  if (!(dt > 0) || omega < 1e-4f) return;
  const float e = std::exp(-dt * omega);
  const float x = *value - target;
  const float v = omega * x + *velocity;
  const float nx = (dt * v + x) * e;
  *velocity = e * v - omega * nx;
  *value = nx + target;
}

// Recomputes the aux (fingertip summary) bones from the finger chains.
void FixAux(Bones& out) {
  static constexpr int parent[26] = {-1,0,1,2,3,4,1,6,7,8,9,1,11,12,13,14,1,16,17,18,19,1,21,22,23,24};
  std::array<Eigen::Isometry3f, 26> global;
  for (int i = 0; i < 26; ++i) {
    Eigen::Isometry3f local = Eigen::Isometry3f::Identity();
    local.linear() = Q(out[i]).toRotationMatrix();
    local.translation() = Eigen::Vector3f(out[i].position.v);
    global[i] = parent[i] < 0 ? local : global[parent[i]] * local;
  }
  for (int f = 0; f < 5; ++f) {
    const auto tip = global[f == 0 ? 5 : 10 + (f - 1) * 5];
    SetQ(out[26 + f], Eigen::Quaternionf(tip.rotation()));
    for (int k = 0; k < 3; ++k) out[26 + f].position.v[k] = tip.translation()[k];
  }
}

// Every frame of one of Valve's 31-bone hand GLBs, in OpenVR's convention.
std::vector<Bones> LoadFrames(const std::string& path, bool left) {
  std::ifstream f(lh::platform::FsPath(path), std::ios::binary);
  if (!f) throw std::runtime_error("cannot open " + path);
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)), {});
  if (Word(bytes, 0) != 0x46546c67 || Word(bytes, 4) != 2 || Word(bytes, 8) != bytes.size() ||
      Word(bytes, 16) != 0x4e4f534a) throw std::runtime_error("invalid GLB header");
  const size_t n = Word(bytes, 12), bin = 28 + n;
  if (bin > bytes.size() || Word(bytes, 24 + n) != 0x004e4942) throw std::runtime_error("missing GLB BIN");
  const json j = json::parse(bytes.begin() + 20, bytes.begin() + 20 + n);
  if (j.at("nodes").size() != 32 || j.at("nodes")[1].at("name") != "REF:Root")
    throw std::runtime_error("unsupported Valve hand hierarchy");
  auto values = [&](int index, size_t frame, int count) {
    const auto& a = j.at("accessors").at(index);
    const auto& v = j.at("bufferViews").at(a.at("bufferView").get<int>());
    if (a.at("componentType") != 5126 || a.contains("sparse") ||
        a.at("type") != (count == 3 ? "VEC3" : "VEC4")) throw std::runtime_error("unsupported accessor");
    frame = std::min(frame, a.at("count").get<size_t>() - 1);
    const size_t at = bin + v.value("byteOffset", size_t(0)) + a.value("byteOffset", size_t(0)) +
                      frame * v.value("byteStride", size_t(count * 4));
    if (at + count * 4 > bytes.size()) throw std::runtime_error("bad accessor bounds");
    std::vector<float> out(count);
    std::memcpy(out.data(), bytes.data() + at, count * 4);
    for (float x : out) if (!std::isfinite(x)) throw std::runtime_error("nonfinite bone");
    return out;
  };
  const auto& anim = j.at("animations").at(0);
  const size_t frames = j.at("accessors").at(anim.at("samplers").at(0).at("input").get<int>()).at("count").get<size_t>();
  if (frames == 0) throw std::runtime_error("empty animation in " + path);
  std::vector<Bones> out(frames);
  for (size_t frame = 0; frame < frames; ++frame) {
    Bones& bones = out[frame];
    for (int i = 0; i < 31; ++i) {
      auto& b = bones[size_t(i)]; const auto& node = j.at("nodes")[size_t(i) + 1];
      auto t = node.value("translation", std::vector<float>{0, 0, 0});
      auto q = node.value("rotation", std::vector<float>{0, 0, 0, 1});
      for (const auto& ch : anim.at("channels")) {
        if (ch.at("target").at("node") != i + 1) continue;
        const auto& s = anim.at("samplers").at(ch.at("sampler").get<int>());
        const auto kind = ch.at("target").at("path").get<std::string>();
        if (kind == "translation") t = values(s.at("output"), frame, 3);
        if (kind == "rotation") q = values(s.at("output"), frame, 4);
      }
      b.position = {{t.at(0), t.at(1), t.at(2), 1}};
      SetQ(b, Eigen::Quaternionf(q.at(3), q.at(0), q.at(1), q.at(2)));
      // Valve GLB has a glTF root frame. OpenVR root uses a half turn about Y.
      // Only wrist and auxiliary bones are directly parented to root.
      if (i == 1 || i >= 26) {
        b.position.v[0] *= -1; b.position.v[2] *= -1;
        SetQ(b, Eigen::Quaternionf(0, 0, 1, 0) * Q(b));
      }
      // Left hand: OpenVR's left skeleton is the right one reflected across x, but
      // every finger bone frame is also turned half a turn about its own x axis
      // (Valve's reference poses: a left proximal has the right one's rotation and
      // the negated offset). So: the wrist is a plain reflection; bones whose parent
      // is the (reflected) wrist or root take the reflection and then that turn; bones
      // inside a finger keep their rotation, offset negated.
      if (left && i != 0) {
        auto r = Q(b);
        const Eigen::Quaternionf mirrored(r.w(), r.x(), -r.y(), -r.z());
        if (i == 1) {
          b.position.v[0] *= -1;
          SetQ(b, mirrored);
        } else if (IsMetacarpal(i) || i >= 26) {
          b.position.v[0] *= -1;
          SetQ(b, mirrored * Eigen::Quaternionf(0, 1, 0, 0));
        } else {
          for (int k = 0; k < 3; ++k) b.position.v[k] *= -1;
        }
      }
    }
  }
  return out;
}

// A clip at normalized time t in [0, 1] (0x286f50 with its "normalized" flag).
Bones Sample(const std::vector<Bones>& clip, float t) {
  const float at = std::clamp(std::isfinite(t) ? t : 0.f, 0.f, 1.f) * float(clip.size() - 1);
  const size_t i = std::min(size_t(at), clip.size() - 1);
  if (i + 1 >= clip.size()) return clip.back();
  return Lerp(clip[i], clip[i + 1], at - float(i));
}

// 0x27c280: a 3x3 pose grid blended bilinearly at (x, y) in [-1, 1]^2; frames are
// row-major, row from y, column from x.
Bones Grid(const std::vector<Bones>& lib, const std::array<int, 9>& frames, float x, float y) {
  x = std::clamp(std::isfinite(x) ? x : 0.f, -1.f, 1.f);
  y = std::clamp(std::isfinite(y) ? y : 0.f, -1.f, 1.f);
  const int row = std::clamp(int(std::floor(y + 1)), 0, 1), col = std::clamp(int(std::floor(x + 1)), 0, 1);
  auto at = [&](int r, int c) -> const Bones& { return lib[size_t(frames[size_t(r * 3 + c)])]; };
  const Bones low = Lerp(at(row, col), at(row, col + 1), x + 1 - float(col));
  const Bones high = Lerp(at(row + 1, col), at(row + 1, col + 1), x + 1 - float(col));
  return Lerp(low, high, y + 1 - float(row));
}

// vr_glove_right_poses.glb frames used by the stock thumb states (0x381470).
constexpr std::array<int, 9> kPadGrid = {17, 16, 18, 14, 13, 15, 11, 10, 12};
constexpr std::array<int, 9> kStickGrid = {24, 23, 25, 21, 19, 22, 27, 26, 28};
constexpr int kStateFrame[9] = {-1, -1, 32, 35, 37, 33, 5, 4, 1};
constexpr int kOpen = 0, kTriggerCurl = 30;
// hand_right_poses.glb (WithoutController thumb, 0x280cd0)
constexpr int kThumbRest = 13, kThumbOut = 10, kThumbBack = 14;
}  // namespace

bool HandSkeleton::Load(const std::string& path, bool left, std::string* error) {
  try {
    glove_ = LoadFrames(path, left);
    if (glove_.size() < 38) throw std::runtime_error("unexpected Valve hand pose library " + path);
    grip_ = glove_[1];
    FixAux(grip_);
    // The stock WithoutController hand: SteamVR's shared hand animations, next to
    // the Index driver's squeeze clip. Without them, the glove library's grip pose
    // stands in for the fist.
    namespace fs = std::filesystem;
    const fs::path anims = lh::platform::FsPath(path).parent_path();
    fs::path shared = anims / ".." / ".." / ".." / ".." / "resources" / "anims";
    if (const std::string dir = lh::platform::Env("LH_HAND_ANIMS"); !dir.empty()) shared = lh::platform::FsPath(dir);
    hand_.clear(); close_.clear(); squeeze_.clear();
    try {
      hand_ = LoadFrames(lh::platform::PathUtf8(shared / "hand_right_poses.glb"), left);
      close_ = LoadFrames(lh::platform::PathUtf8(shared / "hand_right_closeanim.glb"), left);
      if (hand_.size() < 15 || close_.size() < 2) throw std::runtime_error("unexpected hand animations");
      try { squeeze_ = LoadFrames(lh::platform::PathUtf8(anims / "indexcontroller_anim_closesqueeze.glb"), left); }
      catch (const std::exception&) { squeeze_.clear(); }
    } catch (const std::exception& e) {
      hand_.clear(); close_.clear(); squeeze_.clear();
      if (error) *error = std::string("WithoutController fist unavailable (") + e.what() + "); using the grip pose";
    }
    left_ = left;
    state_ = {};
    state_.thumbFrom = glove_[size_t(kStateFrame[8])];
    if (error && hand_.size()) error->clear();
    return true;
  } catch (const std::exception& e) {
    glove_.clear();
    if (error) *error = e.what();
    return false;
  }
}

// 0x27fd10: which thumb pose, from the buttons the thumb is on. OpenVR button ids
// as the stock logic tests them: B = ApplicationMenu (1), A = Grip (2), System (0),
// trackpad = Axis0 (32), thumbstick = Axis3 (35). 0 none, 1 touched, 2 pressed.
int HandSkeleton::ThumbState(const ControllerInput& in) {
  auto level = [](bool touch, bool press) { return press ? 2 : touch ? 1 : 0; };
  const int b = level(in.bTouch, in.bClick), a = level(in.aTouch, in.aClick);
  const int system = level(in.systemTouch, in.systemClick);
  if (a == 2 && b == 2) return 5;
  if (b == 2) return 4;
  if (a == 2) return 3;
  if (in.trackpadTouch) return 0;
  if (!in.thumbstickTouch && !in.thumbstickClick && in.thumbstickX == 0 && in.thumbstickY == 0) {
    if (a == 1 && b == 1) return 2;
    if (system == 2) return 6;
    return system == 1 ? 7 : 8;
  }
  return 1;
}

void HandSkeleton::Update(const ControllerInput& in, double seconds) {
  if (glove_.empty()) return;
  const float dt = std::isfinite(seconds) ? float(std::clamp(seconds, 0.0, 0.25)) : 0.f;
  State& s = state_;
  auto finite = [](double v) { return std::isfinite(v) ? std::clamp(float(v), 0.f, 1.f) : 0.f; };
  const float fingers[4] = {finite(in.fingerIndex), finite(in.fingerMiddle), finite(in.fingerRing), finite(in.fingerPinky)};
  static constexpr float kBias[4] = {0.5f, 0.25f, 0.3f, 0.3f};
  for (int f = 0; f < 4; ++f) {
    Spring(fingers[f], dt, 250, &s.curl[f], &s.curlV[f]);
    Spring(Bias(fingers[f], kBias[f]), dt, 250, &s.fist[f], &s.fistV[f]);
  }
  // The open hand at rest, by how the controller is held (0x27ff20's end, from the pose 0x27fcb0 keeps):
  // the hand frame is the controller's turned 44 deg about -x (cos/sin 22 deg: 0.927, 0.375). Its +z
  // pointing up blends frame 0 toward frame 8, its -x pointing up adds frame 9, each through a spring.
  if (haveOrientation_) {
    const Eigen::Quaterniond q(orientation_[0], orientation_[1], orientation_[2], orientation_[3]);
    const Eigen::Quaterniond hand = q.normalized() * Eigen::Quaterniond(0.9270064234733582, -0.3750454783439636, 0, 0);
    const double up = (hand * Eigen::Vector3d(0, 0, 1)).y(), side = (hand * Eigen::Vector3d(-1, 0, 0)).y();
    const float restUp = up < -1 ? 0.f : up > 1 ? 1.f : float((up + 1) * 0.5);
    const float restSide = side < 0 ? 0.f : float(std::min(side, 1.0));
    Spring(restUp, dt, 40, &s.restUp, &s.restUpV);
    Spring(restSide, dt, 40, &s.restSide, &s.restSideV);
  }
  s.trigger = finite(in.trigger);
  // Pulling the trigger closes the index all the way without the controller; on a wand,
  // which has no finger sensors, as far as it's pulled.
  Spring(indexFromTrigger_ ? s.trigger : s.trigger > 0 ? 1.f : s.fist[0], dt, 250, &s.indexClose, &s.indexCloseV);
  const float force = finite(in.gripForce);
  s.squeeze = force > 0 ? Bias(std::min(1.f, force), 0.8f) : 0.f;
  // The thumb's close-clip time follows index and middle (0x2803ed..0x2804b7): it
  // opens at once, and closes with a slower spring.
  const float n = float(std::max<size_t>(close_.size(), 2) - 1);
  const float closeTarget = std::min(Remap(s.indexClose, 0.4f, 0.92f, 21.f / n, 1.f), Remap(s.fist[1], 0.4f, 0.92f, 18.f / n, 1.f));
  if (closeTarget < s.thumbClose) { s.thumbClose = closeTarget; s.thumbCloseV = 0; }
  else Spring(closeTarget, dt, 25, &s.thumbClose, &s.thumbCloseV);
  // WithoutController thumb position: on the stick (outward), on the pad, curled
  // onto A/B, gliding between them (no snap: going from one button to another passes
  // moments with nothing touched). Lifted off everything, it rises at once to the
  // open hand's thumb from where it was, and comes down again on the next touch. The
  // left hand mirrors x.
  // Lifting off and settling back both take ~0.2 s (a critically damped spring at 30/s).
  constexpr float kThumbMove = 60, kThumbRise = 30, kThumbLower = 30;
  const float side = left_ ? -1.f : 1.f;
  bool target = true;
  float tx = 0, ty = 0;
  // A deflected stick has the thumb on it too, as in ThumbState: a stick whose touch sensor
  // fails still moves the thumb.
  if (in.thumbstickTouch || in.thumbstickClick || in.thumbstickX != 0 || in.thumbstickY != 0) {
    tx = Remap(-side * float(in.thumbstickX), -1, 1, 0.2f, 1.f); ty = float(in.thumbstickY);
  } else if (in.trackpadTouch) {
    tx = Remap(-side * float(in.trackpadX), -1, 1, -0.2f, 0.2f); ty = float(in.trackpadY);
  } else if (in.aTouch || in.bTouch || in.aClick || in.bClick) {
    tx = -1; ty = 0;
  } else {
    target = false;
  }
  if (!std::isfinite(tx) || !std::isfinite(ty)) tx = ty = 0;
  s.thumbLiftedFor = target ? 0.f : s.thumbLiftedFor + dt;
  if (target) {
    Spring(tx, dt, kThumbMove, &s.thumbX, &s.thumbXV);
    Spring(ty, dt, kThumbMove, &s.thumbY, &s.thumbYV);
  }
  const bool rise = !target;
  Spring(rise ? 1.f : 0.f, dt, rise ? kThumbRise : kThumbLower, &s.thumbRaise, &s.thumbRaiseV);
  s.thumbRaise = std::clamp(s.thumbRaise, 0.f, 1.f);
  // WithController thumb state machine, cross-faded in 0.2 s. Stock takes 0.9 s back to rest
  // (nothing touched): the thumb looked slow to lift off the buttons.
  const int state = ThumbState(in);
  s.padX = -side * float(std::isfinite(in.trackpadX) ? in.trackpadX : 0);
  s.padY = float(std::isfinite(in.trackpadY) ? in.trackpadY : 0);
  s.stickX = -side * float(std::isfinite(in.thumbstickX) ? in.thumbstickX : 0);
  s.stickY = float(std::isfinite(in.thumbstickY) ? in.thumbstickY : 0);
  if (state != s.thumbState) {
    s.thumbFrom = ThumbPose(s, true);  // where the thumb is now, mid-fade or not
    s.thumbState = state;
    s.fade = 0;
  }
  s.fade = std::min(1.f, s.fade + dt / 0.2f);
}

Bones HandSkeleton::ThumbPose(const State& s, bool blended) const {
  Bones target;
  if (s.thumbState == 0) target = Grid(glove_, kPadGrid, s.padX, s.padY);
  else if (s.thumbState == 1) target = Grid(glove_, kStickGrid, s.stickX, s.stickY);
  else target = glove_[size_t(kStateFrame[s.thumbState])];
  return blended && s.fade < 1 ? Lerp(s.thumbFrom, target, s.fade) : target;
}

// The open hand without the controller (0x280e80's first blends): frame 0 toward frame 8, then 9.
Bones HandSkeleton::Rest(const State& s) const {
  return Lerp(Lerp(hand_[0], hand_[8], s.restUp), hand_[9], s.restSide);
}

Bones HandSkeleton::Current(bool withController) const {
  if (glove_.empty()) return {};
  const State& s = state_;
  Bones out;
  if (withController) {
    // 0x2808d0: the thumb state's pose holds the controller; each finger opens from
    // it by its sensor, the index curls further onto the trigger.
    out = ThumbPose(s, true);
    const Bones& open = glove_[kOpen];
    if (s.trigger > 0) Blend(out, Lerp(out, glove_[kTriggerCurl], s.trigger), 1, kIndex);
    else Blend(out, open, 1 - s.curl[0], kIndex);
    Blend(out, open, 1 - s.curl[1], kMiddle);
    Blend(out, open, 1 - s.curl[2], kRing);
    Blend(out, open, 1 - s.curl[3], kPinky);
  } else if (!hand_.empty()) {
    // 0x280e80: each finger closes along the fist clip (squeezing, the squeeze clip)
    const Bones rest = Rest(s);
    out = rest;
    // The index goes back to frame 0 as the other three close (their mean closure).
    Blend(out, hand_[0], std::clamp((s.fist[1] + s.fist[2] + s.fist[3]) / 3, 0.f, 1.f), kIndex);
    auto fist = [&](float t) { return squeeze_.empty() ? Sample(close_, t) : Lerp(Sample(close_, t), Sample(squeeze_, t), s.squeeze); };
    Blend(out, fist(s.indexClose), s.indexClose, kIndex);
    Blend(out, fist(s.fist[1]), s.fist[1], kMiddle);
    Blend(out, fist(s.fist[2]), s.fist[2], kRing);
    Blend(out, fist(s.fist[3]), s.fist[3], kPinky);
    // 0x280cd0: the thumb between rest, out over the stick, back on the pad and
    // curled onto the face buttons.
    Bones thumb = s.thumbX < 0 ? Lerp(hand_[kThumbRest], fist(s.thumbClose), -s.thumbX)
                               : Lerp(hand_[kThumbRest], hand_[kThumbOut], s.thumbX);
    thumb = s.thumbY < 0 ? Lerp(thumb, hand_[kThumbBack], -s.thumbY) : Lerp(thumb, hand_[kThumbRest], s.thumbY);
    Blend(out, Lerp(thumb, rest, s.thumbRaise), 1, kThumb);
  } else {
    // No shared animations installed: close onto the glove library's grip.
    out = glove_[kOpen];
    for (int f = 0; f < 4; ++f) Blend(out, glove_[1], f == 0 ? s.indexClose : s.fist[f], Mask(kIndex + f));
    Blend(out, ThumbPose(s, true), 1, kThumb);
  }
  FixAux(out);
  return out;
}

Bones HandSkeleton::Evaluate(const ControllerInput& input, bool withController) const {
  // Settled: every spring and fade at its target.
  HandSkeleton settled = *this;
  settled.Update(input, 0);
  settled.state_.fade = 1;
  for (int k = 0; k < 40; ++k) settled.Update(input, 0.25);
  return settled.Current(withController);
}
}
