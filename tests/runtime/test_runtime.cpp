#include "common/test_paths.hpp"
#include <set>
// Tests for the lh runtime: room map, Index input, device roles, and an end-to-end
// session replay of the recorded motion capture.
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include "platform/platform.hpp"
#include <random>
#include <tuple>
#include <sstream>
#include <thread>

#include <nlohmann/json.hpp>
#include <optional>

#include "lh/session.hpp"
#include "runtime/device_role.hpp"
#include "runtime/device_tracker.hpp"
#include "runtime/index_input.hpp"
#include "runtime/mapped_input.hpp"
#include "runtime/model_scale.hpp"
#include "runtime/room_map.hpp"
#include "runtime/room_solver.hpp"
#include "tracking/lighthouse_model.hpp"
#ifdef LH_TEST_USB
#include "transport/usb/raw_capture.hpp"
#endif

namespace lh::runtime {
struct DeviceTrackerTestAccess {
  static LighthouseDirect& Core(DeviceTracker& d) { return *d.filter_; }
  static direct::DirectFilter& Filter(DeviceTracker& d) { return d.filter_->FilterForTest(); }
  static bool HasCarry(const DeviceTracker& d) { return d.carry_.has_value(); }
  static void SetHeldPose(DeviceTracker& d, const Eigen::Isometry3d& pose) { d.heldPose_ = pose; }
  static bool HasHeldPose(const DeviceTracker& d) { return d.heldPose_.has_value(); }
  static void UpdateOutput(DeviceTracker& d) { d.UpdateOutput(); }
  static void Check(DeviceTracker& d, double now) { d.latestImu_ = now; d.CheckHealth(); }
  static void Acquired(DeviceTracker& d) { d.OnAcquired(); }
  static void Frame(DeviceTracker& d, const OpticalFrame& f) { d.HandleFrame(f); }
};
}

namespace {
using namespace lh;
using namespace lh::runtime;

const std::string kCapture = TestCapture("session-20261016-142931Z-147083");

std::string ReadFile(const std::string& path) {
  std::ifstream in(path);
  std::stringstream s;
  s << in.rdbuf();
  return s.str();
}

Eigen::Isometry3d Pose(double x, double y, double z, double yawDeg) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = Eigen::AngleAxisd(yawDeg * M_PI / 180, Eigen::Vector3d::UnitY()).toRotationMatrix();
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

BaseConfig Config(uint32_t serial) {
  BaseConfig c;
  c.valid = true;
  c.serial = serial;
  c.cal.assign(14, 0.0);
  return c;
}

TEST(DeviceTrackerOutput, InvalidTrackingClearsAHeldPose) {
  RoomMap room;
  TrackedObjectModel model;
  DeviceTracker tracker("LHR-HELD", model, &room);
  DeviceTrackerTestAccess::SetHeldPose(tracker, Pose(1.0, 2.0, 3.0, 0.0));
  ASSERT_TRUE(DeviceTrackerTestAccess::HasHeldPose(tracker));

  // No bootstrap and no located primary base make RawOutput invalid. The old
  // loss path returned without clearing heldPose_, allowing a later valid output
  // to reuse this stale location after the device had been moved while lost.
  DeviceTrackerTestAccess::UpdateOutput(tracker);
  EXPECT_FALSE(tracker.Output().valid);
  EXPECT_FALSE(DeviceTrackerTestAccess::HasHeldPose(tracker));
}

TEST(DeviceRole, ComesFromDeviceClassOnly) {
  EXPECT_EQ(RoleFromConfig(R"({"device_class":"hmd","model_number":"Vive. MV"})"), DeviceRole::Hmd);
  EXPECT_EQ(RoleFromConfig(R"({"device_class":"controller","model_number":"Knuckles Right"})"), DeviceRole::Controller);
  EXPECT_EQ(RoleFromConfig(R"({"device_class":"controller","render_model":"{knuckles}valve_controller_knu_1_0_left"})"),
            DeviceRole::Controller);
  // An Index controller the user mapped as a tracker stays a tracker.
  EXPECT_EQ(RoleFromConfig(R"({"device_class":"generic_tracker","model_number":"Knuckles Left"})"), DeviceRole::Tracker);
  // Physically a Vive wand, configured as a tracker.
  EXPECT_EQ(RoleFromConfig(R"({"device_class":"generic_tracker","render_model":"vr_controller_vive_1_5"})"),
            DeviceRole::Tracker);
  // Vive wands (1.0 and the 2.0 Pro) are controllers; other controllers stay trackers.
  EXPECT_EQ(RoleFromConfig(R"({"device_class":"controller","model_number":"Vive. Controller MV"})"), DeviceRole::Controller);
  EXPECT_EQ(RoleFromConfig(R"({"device_class":"controller","model_number":"VIVE Controller Pro MV"})"),
            DeviceRole::Controller);
  EXPECT_EQ(RoleFromConfig(R"({"device_class":"controller","model_number":"Something Else"})"), DeviceRole::Tracker);
  EXPECT_EQ(ControllerTypeFromConfig(R"({"device_class":"controller","render_model":"vr_controller_vive_1_5"})"),
            ControllerType::ViveWand);
  EXPECT_EQ(ControllerTypeFromConfig(R"({"device_class":"controller","model_number":"Knuckles Right"})"),
            ControllerType::Index);
}

TEST(ViveWandInput, DecodesButtonsTriggerAndTrackpad) {
  ControllerInputDecoder d(ControllerType::ViveWand);
  InputReport r;
  r.hasButtons = true; r.buttons = 0x01 | 0x02 | 0x04 | 0x10 | 0x20;  // trigger, pad touch+click, grip, menu
  r.hasTrigger = true; r.trigger = 255;
  r.hasAxes = true; r.axisX = 16384; r.axisY = -16384;
  EXPECT_TRUE(d.Apply(r, 1.0));
  const auto& s = d.State();
  EXPECT_TRUE(s.triggerClick && s.trackpadTouch && s.trackpadClick && s.gripClick && s.menuClick);
  EXPECT_FALSE(s.systemClick || s.aClick || s.thumbstickClick);
  EXPECT_DOUBLE_EQ(s.trackpadX, 0.5);
  EXPECT_DOUBLE_EQ(s.trackpadY, -0.5);
  EXPECT_DOUBLE_EQ(s.thumbstickX, 0.0);  // a wand has no thumbstick
  InputReport release;
  release.hasButtons = true; release.buttons = 0x08;  // system only, finger off the pad
  d.Apply(release, 2.0);
  EXPECT_TRUE(d.State().systemClick);
  EXPECT_FALSE(d.State().gripClick);
  EXPECT_DOUBLE_EQ(d.State().trackpadX, 0.0);
}

// Shiftall FlipVR: device_class controller, mapped-input firmware, its own profile. The
// stock driver makes it a controller; it was a tracker here before.
TEST(MappedInput, FlipVrIsAMappedController) {
  const std::string flip = R"({"device_class":"controller","manufacturer":"Shiftall Inc.",
    "model_number":"FlipVR Controller VC1B Left","render_model":"{shiftall}flipvr_controller_vc1b_left_2_0",
    "input_profile":"{shiftall}/input/flipvr_controller_vc1b_profile.json","resource_root":"shiftall",
    "tracked_controller_role":"left_hand","firmware_config":{"mapped_input":true}})";
  EXPECT_EQ(RoleFromConfig(flip), DeviceRole::Controller);
  EXPECT_EQ(ControllerTypeFromConfig(flip), ControllerType::Mapped);
  // Without mapped input it stays what it was (a tracker), and wands/Index are unchanged.
  EXPECT_EQ(RoleFromConfig(R"({"device_class":"controller","model_number":"Something Else",
    "input_profile":"{x}/input/p.json","firmware_config":{"mapped_input":false}})"), DeviceRole::Tracker);
  EXPECT_EQ(ControllerTypeFromConfig(R"({"device_class":"controller","model_number":"Knuckles Right",
    "firmware_config":{"mapped_input":true},"input_profile":"{indexcontroller}/input/index_controller_profile.json"})"),
            ControllerType::Index);
  // A tracker role still wins.
  EXPECT_EQ(RoleFromConfig(R"({"device_class":"controller","tracked_controller_role":"generic_tracker",
    "input_profile":"{x}/input/p.json","firmware_config":{"mapped_input":true}})"), DeviceRole::Tracker);
}

TEST(MappedInput, DecoderKeepsTheRawRecord) {
  ControllerInputDecoder d(ControllerType::Mapped);
  InputReport legacy;
  legacy.hasButtons = true; legacy.buttons = 0xff;
  EXPECT_FALSE(d.Apply(legacy, 1.0));  // not this device's format
  InputReport r;
  r.format = InputFormat::Raw;
  r.raw = {1, 2, 3, 4, 5, 6};
  EXPECT_TRUE(d.Apply(r, 2.0));
  EXPECT_EQ(d.State().mapped, r.raw);
  EXPECT_FALSE(d.Apply(r, 3.0));  // same record
  EXPECT_DOUBLE_EQ(d.State().hostTime, 3.0);
}

// The components of {shiftall}/input/flipvr_controller_vc1b_profile.json (abridged).
const char* kFlipProfile = R"({
  "controller_type": "flipvr_controller_vc1b",
  "input_source": {
    "/input/joystick": {"type": "joystick"}, "/input/trigger": {"type": "trigger"},
    "/input/grip": {"type": "trigger"},
    "/input/a": {"type": "button", "side": "right"}, "/input/b": {"type": "button", "side": "right"},
    "/input/x": {"type": "button", "side": "left"}, "/input/y": {"type": "button", "side": "left"},
    "/input/system": {"type": "button"}
  },
  "input_components": {
    "/input/joystick/x": {"bit_offset": 0, "bit_width": 8, "min": 0, "max": 255, "type": "scalar", "unit": "twos_complement"},
    "/input/joystick/y": {"bit_offset": 8, "bit_width": 8, "min": 0, "max": 255, "type": "scalar", "unit": "twos_complement"},
    "/input/trigger/value": {"bit_offset": 16, "bit_width": 8, "min": 0, "max": 255, "type": "scalar", "unit": "onesided"},
    "/input/grip/value": {"bit_offset": 24, "bit_width": 8, "min": 0, "max": 255, "type": "scalar", "unit": "onesided"},
    "/input/a/touch": {"bit_offset": 32, "type": "bool"}, "/input/x/touch": {"bit_offset": 32, "type": "bool"},
    "/input/b/touch": {"bit_offset": 33, "type": "bool"}, "/input/y/touch": {"bit_offset": 33, "type": "bool"},
    "/input/trigger/touch": {"bit_offset": 34, "type": "bool"},
    "/input/joystick/touch": {"bit_offset": 35, "type": "bool"},
    "/input/a/click": {"bit_offset": 36, "type": "bool"}, "/input/x/click": {"bit_offset": 36, "type": "bool"},
    "/input/b/click": {"bit_offset": 37, "type": "bool"}, "/input/y/click": {"bit_offset": 37, "type": "bool"},
    "/input/system/click": {"bit_offset": 38, "type": "bool"},
    "/input/trigger/click": {"bit_offset": 39, "type": "bool"},
    "/input/grip/click": {"bit_offset": 40, "type": "bool"},
    "/input/joystick/click": {"bit_offset": 41, "type": "bool"},
    "/output/haptic": {"type": "vibration"}
  }
})";

TEST(MappedInput, ParsesTheProfileForOneHand) {
  using lh::runtime::MappedComponent;
  lh::runtime::MappedProfile left, right, both;
  std::string error;
  ASSERT_TRUE(lh::runtime::ParseMappedProfile(kFlipProfile, "left_hand", &left, &error)) << error;
  ASSERT_TRUE(lh::runtime::ParseMappedProfile(kFlipProfile, "right_hand", &right, &error)) << error;
  ASSERT_TRUE(lh::runtime::ParseMappedProfile(kFlipProfile, "", &both, &error)) << error;
  EXPECT_EQ(left.controllerType, "flipvr_controller_vc1b");
  auto has = [](const lh::runtime::MappedProfile& p, const std::string& path) {
    return std::any_of(p.components.begin(), p.components.end(), [&](const MappedComponent& c) { return c.path == path; });
  };
  EXPECT_TRUE(has(left, "/input/x/click") && has(left, "/input/y/touch"));
  EXPECT_FALSE(has(left, "/input/a/click") || has(left, "/input/b/touch"));
  EXPECT_TRUE(has(right, "/input/a/click") && !has(right, "/input/x/click"));
  EXPECT_TRUE(has(left, "/output/haptic") && has(left, "/input/system/click") && has(left, "/input/joystick/x"));
  EXPECT_EQ(both.components.size(), 19u);
  EXPECT_EQ(left.components.size(), 15u);
  std::string bad;
  lh::runtime::MappedProfile none;
  EXPECT_FALSE(lh::runtime::ParseMappedProfile(R"({"controller_type":"x"})", "", &none, &bad));
  EXPECT_FALSE(lh::runtime::ParseMappedProfile("not json", "", &none, &bad));
}

TEST(MappedInput, ReadsTheBitsOfARecord) {
  lh::runtime::MappedProfile p;
  std::string error;
  ASSERT_TRUE(lh::runtime::ParseMappedProfile(kFlipProfile, "right_hand", &p, &error)) << error;
  auto value = [&](const std::vector<uint8_t>& record, const std::string& path) {
    for (const auto& c : p.components)
      if (c.path == path) return lh::runtime::MappedValue(record, c);
    return std::optional<double>();
  };
  // At rest (recorded from a FlipVR): stick 0x7f 0x7f is centered, not full scale.
  const std::vector<uint8_t> rest = {0x7f, 0x7f, 0x2c, 0x00, 0x00, 0x00};
  EXPECT_NEAR(*value(rest, "/input/joystick/x"), 0.0, 0.01);
  EXPECT_NEAR(*value(rest, "/input/joystick/y"), 0.0, 0.01);
  EXPECT_DOUBLE_EQ(*value(rest, "/input/a/click"), 0.0);
  // Stick full left and up, trigger half, grip full; A touched, A and grip clicked.
  const std::vector<uint8_t> record = {0x00, 0xff, 0x80, 0xff, 0x01 | 0x10, 0x01};
  EXPECT_DOUBLE_EQ(*value(record, "/input/joystick/x"), -1.0);
  EXPECT_DOUBLE_EQ(*value(record, "/input/joystick/y"), 1.0);
  EXPECT_NEAR(*value(record, "/input/trigger/value"), 128.0 / 255.0, 1e-12);
  EXPECT_DOUBLE_EQ(*value(record, "/input/grip/value"), 1.0);
  EXPECT_DOUBLE_EQ(*value(record, "/input/a/touch"), 1.0);
  EXPECT_DOUBLE_EQ(*value(record, "/input/b/touch"), 0.0);
  EXPECT_DOUBLE_EQ(*value(record, "/input/a/click"), 1.0);
  EXPECT_DOUBLE_EQ(*value(record, "/input/grip/click"), 1.0);
  EXPECT_DOUBLE_EQ(*value(record, "/input/joystick/click"), 0.0);
  EXPECT_FALSE(value({0x00, 0x00}, "/input/grip/click").has_value());  // too short for its bit
  EXPECT_FALSE(value(record, "/output/haptic").has_value());

  ControllerInput named;
  lh::runtime::MappedToControllerInput(p, record, &named);
  EXPECT_DOUBLE_EQ(named.thumbstickX, -1.0);
  EXPECT_DOUBLE_EQ(named.thumbstickY, 1.0);
  EXPECT_NEAR(named.trigger, 128.0 / 255.0, 1e-12);
  EXPECT_TRUE(named.aTouch && named.aClick && named.gripClick);
  EXPECT_FALSE(named.bClick || named.systemClick || named.triggerClick);
  EXPECT_DOUBLE_EQ(named.gripForce, 1.0);
  EXPECT_DOUBLE_EQ(named.fingerMiddle, 1.0);
}

// lighthouse_scale.json, as the stock driver applies it: a FlipVR right controller's own
// points are Shiftall's reference to 0.4 mm, so its geometry takes the reference's scale.
TEST(ModelScale, AppliesTheMatchingReference) {
  using lh::runtime::ScaleReference;
  std::vector<ScaleReference> refs;
  std::string error;
  ASSERT_TRUE(lh::runtime::ParseScaleReferences(R"({
    "Other": {"scale": 0.9, "modelPoints": [[0.1, 0, 0], [0, 0.1, 0]]},
    "FlipVR Controller VC1B Right": {"scale": 1.04819540747,
      "modelPoints": [[0.0900820717, 0.03899635, 0.0344087742], [0.0833031312, 0.044179827, 0.0245249346],
                      [0.0770357102, 0.0486424901, 0.0294898432]]},
    "Broken": {"modelPoints": [[0, 0, 0]]}})", &refs, &error)) << error;
  ASSERT_EQ(refs.size(), 2u);  // "Broken" has no scale
  // The device's own points, a fraction of a millimetre off the reference.
  std::vector<std::array<double, 3>> points = {
      {0.0903927162, 0.0390972607, 0.0344037004}, {0.0833, 0.0442, 0.0245}, {0.0770, 0.0486, 0.0295}};
  const auto before = points;
  auto match = lh::runtime::ApplyScaleReference(refs, &points);
  ASSERT_NE(match.reference, nullptr);
  EXPECT_EQ(match.reference->name, "FlipVR Controller VC1B Right");
  for (size_t i = 0; i < points.size(); ++i)
    for (int k = 0; k < 3; ++k) EXPECT_NEAR(points[i][k], before[i][k] * 1.04819540747, 1e-12);

  // Another point count (the FlipVR left: 26 own points, 25 in the reference): unchanged.
  std::vector<std::array<double, 3>> two = {{0.0903927162, 0.0390972607, 0.0344037004}, {0.0833, 0.0442, 0.0245}};
  EXPECT_EQ(lh::runtime::ApplyScaleReference(refs, &two).reference, nullptr);
  EXPECT_DOUBLE_EQ(two[0][0], 0.0903927162);

  // The same CAD measured at another size: logged as a mismatch, not scaled.
  std::vector<std::array<double, 3>> big = {{0.1004, 0, 0}, {0, 0.1004, 0}};
  match = lh::runtime::ApplyScaleReference(refs, &big);
  EXPECT_EQ(match.reference, nullptr);
  ASSERT_EQ(match.mismatched.size(), 1u);
  EXPECT_EQ(match.mismatched[0], "Other");
  EXPECT_DOUBLE_EQ(big[0][0], 0.1004);

  std::vector<ScaleReference> none;
  EXPECT_FALSE(lh::runtime::ParseScaleReferences("[1, 2]", &none, &error));
}

TEST(IndexInput, DecodesButtonsAxesAndExtendedRecord) {
  ControllerInputDecoder d;
  InputReport r;
  r.hasButtons = true; r.buttons = 0x01 | 0x10 | 0x08;  // trigger click, A, system
  r.hasTrigger = true; r.trigger = 255;
  r.hasAxes = true; r.axisX = 16384; r.axisY = -32768;
  EXPECT_TRUE(d.Apply(r, 1.0));
  EXPECT_TRUE(d.State().triggerClick);
  EXPECT_TRUE(d.State().aClick);
  EXPECT_TRUE(d.State().systemClick);
  EXPECT_FALSE(d.State().bClick);
  EXPECT_DOUBLE_EQ(d.State().trigger, 1.0);
  EXPECT_DOUBLE_EQ(d.State().thumbstickX, 0.5);   // trackpad not touched -> thumbstick
  EXPECT_DOUBLE_EQ(d.State().thumbstickY, -1.0);
  // Trackpad touch routes the axes to the trackpad.
  InputReport pad;
  pad.hasButtons = true; pad.buttons = 0x02;
  pad.hasAxes = true; pad.axisX = -16384; pad.axisY = 8192;
  d.Apply(pad, 2.0);
  EXPECT_TRUE(d.State().trackpadTouch);
  EXPECT_DOUBLE_EQ(d.State().trackpadX, -0.5);
  EXPECT_DOUBLE_EQ(d.State().thumbstickX, 0.5);  // unchanged
  InputReport ext;
  ext.hasExtended = true;
  ext.extended = {0xa1, 0x01 | 0x20 | 0x40, 51, 102, 153, 204, 255, 0};
  d.Apply(ext, 3.0);
  EXPECT_TRUE(d.State().triggerTouch);
  EXPECT_TRUE(d.State().bTouch);
  EXPECT_TRUE(d.State().thumbstickTouch);
  EXPECT_FALSE(d.State().aTouch);
  EXPECT_DOUBLE_EQ(d.State().fingerMiddle, 0.2);
  EXPECT_DOUBLE_EQ(d.State().fingerRing, 0.4);
  EXPECT_DOUBLE_EQ(d.State().fingerPinky, 0.6);
  EXPECT_DOUBLE_EQ(d.State().fingerIndex, 0.8);
  EXPECT_DOUBLE_EQ(d.State().gripForce, 1.0);
  // Same state again: no change reported.
  EXPECT_FALSE(d.Apply(ext, 4.0));
  // Unknown raw format is ignored.
  InputReport raw;
  raw.format = InputFormat::Raw;
  raw.raw = {1, 2, 3};
  EXPECT_FALSE(d.Apply(raw, 5.0));
}

TEST(RoomMap, TiltRoundTripAndLeveling) {
  for (double pitch : {0.0, 0.3, 2.4}) {
    for (double roll : {-0.05, 0.0, 0.7}) {
      const auto t = TiltFromUp(UpFromTilt(pitch, roll));
      EXPECT_NEAR(UpFromTilt(t[0], t[1]).dot(UpFromTilt(pitch, roll)), 1.0, 1e-12);
      for (int method : {0, 1}) {
        // Levelling maps the base "up" direction onto world +Y.
        const Eigen::Vector3d up = Leveling(pitch, roll, method) * UpFromTilt(pitch, roll);
        EXPECT_NEAR(up.y(), 1.0, 1e-9) << pitch << " " << roll << " " << method;
      }
    }
  }
}

TEST(RoomMap, ImportsSteamUniversesAndSelectsByVisibleBase) {
  const auto db = ReadFile(kCapture + "/lighthousedb.json");
  if (db.empty()) GTEST_SKIP() << "no lighthousedb snapshot";
  RoomMap room;
  ASSERT_TRUE(room.ImportSteamDatabase(db));
  EXPECT_EQ(room.KnownConfigs().count(0xA101B835u), 1u);
  EXPECT_EQ(room.Active(), nullptr);
  auto [config, dynamic] = room.KnownConfigs().at(0xE236DC40u);
  room.OnBaseSeen(config, dynamic);
  ASSERT_NE(room.Active(), nullptr);
  EXPECT_EQ(room.Active()->id, "steamvr-1770781428");
  const RoomBase* origin = room.Base(0xA101B835u);
  const RoomBase* other = room.Base(0xE236DC40u);
  ASSERT_TRUE(origin && other);
  EXPECT_TRUE(origin->origin);
  EXPECT_LT(origin->worldFromBase.translation().norm(), 1e-9);
  // SteamVR logged "RELATIONSHIP bases A101B835 <-> e236dc40 distance 4.92m".
  EXPECT_NEAR((other->worldFromBase.translation() - origin->worldFromBase.translation()).norm(), 4.92, 0.05);
}

// The whole room never moves as a step under a tracked device: a better tilt turns it
// at 2 deg/s the first time and 0.05 deg/s after that (live: 2.3 deg at once moved
// every device 17 cm), and a room-solver refinement glides at 2 cm/s.
TEST(RoomMap, LevelAndSolverCorrectionsAreNeverSteps) {
  RoomMap room;
  room.OnBaseSeen(Config(1), {});
  ASSERT_TRUE(room.EstablishOrigin(1, {0.0, 0.0}, 1e-3));
  room.Observe("dev", 1, 2, Pose(3, 0.5, -2, 40).inverse() * room.Base(1)->worldFromBase, 1e-5, 1e-6, 0.0);
  ASSERT_NE(room.Base(2), nullptr);
  auto angle = [&] { return Eigen::AngleAxisd(room.Base(1)->worldFromBase.rotation()).angle(); };
  const double tilt = 2.0 * M_PI / 180;
  double t = 0, largest = 0, last = angle();
  for (; t < 3; t += 0.01) {
    room.RefineOriginTilt(1, {tilt, 0}, 1e-5, t);
    largest = std::max(largest, std::abs(angle() - last));
    last = angle();
  }
  EXPECT_NEAR(angle(), tilt, 1e-6) << "the first level arrives within ~1 s";
  EXPECT_LT(largest, 2.01 * M_PI / 180 * 0.01 + 1e-9) << "at most 2 deg/s";
  // A later, better estimate 1 deg away: 0.05 deg/s.
  const double before = angle();
  for (int i = 0; i < 100; ++i, t += 0.01) room.RefineOriginTilt(1, {tilt + M_PI / 180, 0}, 1e-6, t);
  EXPECT_NEAR(angle() - before, 0.05 * M_PI / 180, 1e-4);
  // A solver refinement 3 cm off: glides, not jumps.
  const Eigen::Isometry3d at = room.Base(2)->worldFromBase;
  room.ApplySolved(2, Eigen::Translation3d(0.03, 0, 0) * at, 1e-6, 1e-7, 10, 1e-4);  // first solve: placed at once
  const Eigen::Isometry3d placed = room.Base(2)->worldFromBase;
  room.ApplySolved(2, Eigen::Translation3d(0.03, 0, 0) * placed, 1e-6, 1e-7, 11, 1e-4);
  EXPECT_TRUE(room.Base(2)->worldFromBase.isApprox(placed, 1e-12)) << "not applied as a step";
  room.Advance(t);
  for (int i = 0; i < 100; ++i) { t += 0.01; room.Advance(t); }
  EXPECT_NEAR((room.Base(2)->worldFromBase.translation() - placed.translation()).norm(), 0.02, 1e-6);  // 2 cm/s for 1 s
  for (int i = 0; i < 100; ++i) { t += 0.01; room.Advance(t); }
  EXPECT_NEAR((room.Base(2)->worldFromBase.translation() - placed.translation()).norm(), 0.03, 1e-3);  // the solve, weighed against what the room knew
}

TEST(RoomMap, LearnsRefinesAndRelearnsBases) {
  RoomMap room;
  room.OnBaseSeen(Config(1), {});
  ASSERT_NE(room.Active(), nullptr);
  EXPECT_TRUE(room.Empty());
  ASSERT_TRUE(room.EstablishOrigin(1, {0.0, 0.0}, 1e-6));
  EXPECT_FALSE(room.EstablishOrigin(2, {0.0, 0.0}, 1e-6));  // only for an empty room
  const Eigen::Isometry3d worldFrom2 = Pose(3, 0.5, -2, 40);
  const Eigen::Isometry3d altFromPrimary = worldFrom2.inverse() * room.Base(1)->worldFromBase;
  room.Observe("dev", 1, 2, altFromPrimary, 1e-5, 1e-6, 0.0);
  ASSERT_NE(room.Base(2), nullptr);
  EXPECT_TRUE(room.Base(2)->worldFromBase.isApprox(worldFrom2, 1e-9));
  // Observations closer than 0.5 s from the same device are rate limited.
  const uint64_t revision = room.Revision();
  room.Observe("dev", 1, 2, altFromPrimary, 1e-5, 1e-6, 0.1);
  EXPECT_EQ(room.Revision(), revision);
  // Small disagreements are blended in, the origin never moves.
  const Eigen::Isometry3d shifted = Pose(3.02, 0.5, -2, 40);
  for (int i = 0; i < 50; ++i) room.Observe("dev", 1, 2, shifted.inverse(), 1e-5, 1e-6, 1.0 + i);
  EXPECT_NEAR(room.Base(2)->worldFromBase.translation().x(), 3.02, 0.005);
  EXPECT_TRUE(room.Base(1)->worldFromBase.isApprox(Eigen::Isometry3d::Identity()));
  // A base that keeps disagreeing by > 10 cm, seen by one device only, is not
  // overwritten (that device may be the one that is wrong): it is marked uncertain and
  // left to the room solver.
  const Eigen::Isometry3d moved = Pose(4, 0.5, -2, 40);
  for (int i = 0; i < 10; ++i) room.Observe("dev", 1, 2, moved.inverse(), 1e-5, 1e-6, 100.0 + i);
  EXPECT_NEAR(room.Base(2)->worldFromBase.translation().x(), 3.02, 0.01);
  EXPECT_GE(room.Base(2)->positionVariance, 3 * 0.01 * 0.01);
  // A primary that is not in the map is located from a known alternate base.
  room.Observe("dev2", 3, 2, Pose(0, 0, 1, 0), 1e-5, 1e-6, 300.0);
  ASSERT_NE(room.Base(3), nullptr);
  EXPECT_TRUE(room.Base(3)->worldFromBase.isApprox(room.Base(2)->worldFromBase * Pose(0, 0, 1, 0), 1e-9));
}

// A room saved while its origin stood upright (A101B835 on 2026-10-18: lighthousedb.json
// gravity [0.024, -1.0, -0.008]), the base remounted at its usual tilt ([0.128, -0.738,
// 0.663]) before the next session: that session re-levels the room from the base's
// accelerometer, keeping the bases' layout, and relearns the others when its reset
// count says it was power cycled.
// By default a turned origin makes the stored room stale: a new room is learned (the
// headset otherwise fought the old layout: teststill, 2026-10-23).
// A well-placed base is locked: solves far off don't move it; only its accelerometer
// turning (a bump) lets the solver re-place it.
TEST(RoomMap, LockedBaseMovesOnlyWhenBumped) {
  RoomMap room;
  BaseDynamicState level;
  level.accelDir = {0.f, 1.f, 0.f};
  room.OnBaseSeen(Config(1), level);
  ASSERT_TRUE(room.EstablishOrigin(1, {0.0, 0.0}, 1e-6));
  room.OnBaseSeen(Config(2), level);
  const Eigen::Isometry3d at = Pose(3, 0.5, -2, 40);
  room.ApplySolved(2, at, 1e-6, 1e-7, 150, 1e-4);  // placed from 150 keyframes
  ASSERT_TRUE(RoomMap::Locked(*room.Base(2)));
  const Eigen::Isometry3d away = Eigen::Translation3d(0.3, 0, 0) * at;
  for (int i = 0; i < 5; ++i) room.ApplySolved(2, away, 1e-6, 1e-7, 150, 1e-4);
  EXPECT_TRUE(room.Base(2)->worldFromBase.isApprox(at, 1e-9));
  BaseDynamicState turned;
  turned.accelDir = {0.f, 0.94f, 0.34f};  // 20 deg
  room.OnBaseSeen(Config(2), turned);
  room.OnBaseSeen(Config(2), turned);
  EXPECT_FALSE(RoomMap::Locked(*room.Base(2)));
  room.ApplySolved(2, away, 1e-6, 1e-7, 80, 1e-4);  // re-placed from enough keyframes (>= 40, 80 from one device)
  EXPECT_NEAR(room.Base(2)->worldFromBase.translation().x(), away.translation().x(), 1e-6);
}

TEST(RoomMap, TurnedOriginStartsANewRoom) {
  const auto path = (std::filesystem::temp_directory_path() / ("lh-test-stale-" + std::to_string(lh::platform::ProcessId()) + ".json")).string();
  const Eigen::Vector3d upright(0.024, -1.0, -0.008), mounted(0.128, -0.738, 0.663);
  BaseDynamicState saved, live;
  saved.accelDir = {float(upright.x()), float(upright.y()), float(upright.z())};
  live.accelDir = {float(mounted.x()), float(mounted.y()), float(mounted.z())};
  saved.sysUnlockCount = live.sysUnlockCount = 13;
  {
    RoomMap room;
    room.OnBaseSeen(Config(1), saved);
    ASSERT_TRUE(room.EstablishOrigin(1, TiltFromUp(upright), 1e-6));
    room.Observe("dev", 1, 2, Pose(3, 0.5, -2, 40).inverse() * room.Base(1)->worldFromBase, 1e-5, 1e-6, 0.0);
    ASSERT_TRUE(room.Save(path));
  }
  RoomMap room;
  ASSERT_TRUE(room.Load(path));
  std::filesystem::remove(path);
  room.OnBaseSeen(Config(1), live);
  ASSERT_NE(room.Active(), nullptr);
  EXPECT_TRUE(room.Empty());
  EXPECT_EQ(room.Base(2), nullptr);
  bool said = false;
  for (const auto& line : room.TakeLogs()) said |= line.find("is stale") != std::string::npos;
  EXPECT_TRUE(said);
  // An origin that kept its tilt keeps its room.
  RoomMap same;
  {
    RoomMap r;
    r.OnBaseSeen(Config(1), saved);
    ASSERT_TRUE(r.EstablishOrigin(1, TiltFromUp(upright), 1e-6));
    ASSERT_TRUE(r.Save(path));
  }
  ASSERT_TRUE(same.Load(path));
  std::filesystem::remove(path);
  same.OnBaseSeen(Config(1), saved);
  EXPECT_FALSE(same.Empty());
}

TEST(RoomMap, PowerCycledOriginKeepsTheOthersLoosely) {
  const auto path = (std::filesystem::temp_directory_path() / ("lh-test-relevel-" + std::to_string(lh::platform::ProcessId()) + ".json")).string();
  const Eigen::Vector3d upright(0.024, -1.0, -0.008), mounted(0.128, -0.738, 0.663);
  BaseDynamicState saved, live;
  saved.accelDir = {float(upright.x()), float(upright.y()), float(upright.z())};
  saved.sysUnlockCount = 13;
  live.accelDir = {float(mounted.x()), float(mounted.y()), float(mounted.z())};
  live.sysUnlockCount = 13;
  // A power-cycled origin (new reset count: rotor locks since power-up, not a move): the
  // others' places are kept with little weight, not thrown away; a level that agrees
  // with the accelerometer isn't touched.
  Eigen::Isometry3d level, kept;
  {
    RoomMap r;
    r.OnBaseSeen(Config(1), live);
    ASSERT_TRUE(r.EstablishOrigin(1, TiltFromUp(mounted), 1.5e-4));
    r.Observe("dev", 1, 2, Pose(3, 0.5, -2, 40).inverse() * r.Base(1)->worldFromBase, 1e-5, 1e-6, 0.0);
    level = r.Base(1)->worldFromBase;
    ASSERT_NE(r.Base(2), nullptr);
    kept = r.Base(2)->worldFromBase;
    ASSERT_TRUE(r.Save(path));
  }
  RoomMap again;
  ASSERT_TRUE(again.Load(path));
  std::filesystem::remove(path);
  BaseDynamicState reset = live;
  reset.sysUnlockCount = 14;
  again.OnBaseSeen(Config(1), reset);
  EXPECT_TRUE(again.Base(2)->worldFromBase.isApprox(kept, 1e-6));
  EXPECT_GE(again.Base(2)->positionVariance, 3e-4);
  EXPECT_LE(again.Base(2)->observations, 30);
  EXPECT_TRUE(again.TakeRelearned().empty());
  ASSERT_NE(again.Base(1), nullptr);
  EXPECT_TRUE(again.Base(1)->worldFromBase.isApprox(level, 1e-9));
}

// A hand behind the body: no light at all for 1.5 s. The device coasts on its IMU and
// keeps its track (the stock 2 s optical timeout), then resets keeping its calibration.
// It used to be reset after 0.5 s with the calibration discarded: the inertial advance
// moves the filter past the newest pulse, which read as a receiver's new epoch.
TEST(DeviceRecovery, OcclusionCoastsForTheStockTimeoutAndKeepsCalibration) {
  TrackedObjectModel model;
  model.points = {{{-0.08, -0.07, -0.02}}, {{0.08, -0.07, 0.02}}, {{-0.08, 0.07, 0.03}},
                  {{0.08, 0.07, -0.03}}, {{0, 0, 0.08}}, {{0.04, 0.01, -0.06}}};
  model.normals.assign(6, {0, 0, 1});
  for (int i = 0; i < 6; ++i) model.channelToIndex[i] = i;
  RoomMap room;
  DeviceTracker tracker("LHR-HIDDEN", model, &room);
  BaseConfig base;
  base.valid = true; base.serial = 1; base.cal.assign(14, 0.0);
  tracker.OnBaseConfig(base, {});
  LighthouseDirect::Seed seed;
  seed.base = 1; seed.calibrated = true; seed.time = 10.0;
  seed.baseFromImu.translation() = Eigen::Vector3d(0, 0, -2);
  ASSERT_TRUE(DeviceTrackerTestAccess::Core(tracker).SeedPose(seed));
  for (double t = 10.05; t < 11.9; t += 0.05) {
    DeviceTrackerTestAccess::Check(tracker, t);
    ASSERT_TRUE(DeviceTrackerTestAccess::Core(tracker).Bootstrapped()) << "reset " << t - 10 << " s into the occlusion";
  }
  DeviceTrackerTestAccess::Check(tracker, 12.1);
  EXPECT_FALSE(DeviceTrackerTestAccess::Core(tracker).Bootstrapped());
  EXPECT_TRUE(DeviceTrackerTestAccess::HasCarry(tracker));
  bool timeout = false;
  for (const auto& line : tracker.TakeLogs()) {
    timeout |= line.find("no optical samples") != std::string::npos;
    EXPECT_EQ(line.find("no forward light"), std::string::npos) << line;
  }
  EXPECT_TRUE(timeout);

  // A new epoch (power cycle): light keeps arriving timestamped behind the filter.
  ASSERT_TRUE(DeviceTrackerTestAccess::Core(tracker).SeedPose(seed));
  for (int i = 0; i < 1000; ++i) tracker.OnPulse({0.001 * i, 1e-5, i % 6});
  DeviceTrackerTestAccess::Check(tracker, 10.2);
  EXPECT_FALSE(DeviceTrackerTestAccess::Core(tracker).Bootstrapped());
  bool epoch = false;
  for (const auto& line : tracker.TakeLogs()) epoch |= line.find("no forward light") != std::string::npos;
  EXPECT_TRUE(epoch);
}

// Switching LH1 -> LH2 mode, the last sorted frames reached the filter 0.17-0.31 s late, behind the
// time the IMU had carried it to, and the LH2 light only came 0.5-1.5 s after the switch. Light the
// filter skips for being late isn't light that doesn't fit: it used to restart tracking 0.45 s in
// (live: 12 of 45 such switches). Light that comes in time and is refused still does.
TEST(DeviceRecovery, LateLightIsNotRefusedLight) {
  TrackedObjectModel model;
  model.points = {{{-0.08, -0.07, -0.02}}, {{0.08, -0.07, 0.02}}, {{-0.08, 0.07, 0.03}},
                  {{0.08, 0.07, -0.03}}, {{0, 0, 0.08}}, {{0.04, 0.01, -0.06}}};
  model.normals.assign(6, {0, 0, 1});
  for (int i = 0; i < 6; ++i) model.channelToIndex[i] = i;
  RoomMap room;
  DeviceTracker tracker("LHR-SWITCHING", model, &room);
  BaseConfig base;
  base.valid = true; base.serial = 1; base.cal.assign(14, 0.0);
  tracker.OnBaseConfig(base, {});
  LighthouseDirect::Seed seed;
  seed.base = 1; seed.calibrated = true; seed.time = 10.0;
  seed.baseFromImu.translation() = Eigen::Vector3d(0, 0, -2);
  ASSERT_TRUE(DeviceTrackerTestAccess::Core(tracker).SeedPose(seed));
  auto frame = [](double t, int axis) {
    OpticalFrame f;
    f.baseSerial = 1; f.axis = axis; f.syncTime = t; f.leftSyncIsReal = f.rightSyncIsReal = true;
    for (int i = 0; i < 6; ++i) f.samples.push_back({i, axis, t + 0.004, 0.5, 1e-5});
    return f;
  };
  auto restarted = [&] {
    bool r = false;
    for (const auto& line : tracker.TakeLogs()) r |= line.find("none fits the pose") != std::string::npos;
    return r;
  };
  for (int i = 0; i < 12; ++i) DeviceTrackerTestAccess::Frame(tracker, frame(9.7 + 0.008 * i, i & 1));  // late
  DeviceTrackerTestAccess::Check(tracker, 10.6);
  EXPECT_TRUE(DeviceTrackerTestAccess::Core(tracker).Bootstrapped());
  EXPECT_FALSE(restarted());
  for (int i = 0; i < 12; ++i) DeviceTrackerTestAccess::Frame(tracker, frame(10.62 + 0.008 * i, i & 1));  // in time
  DeviceTrackerTestAccess::Check(tracker, 10.8);
  EXPECT_FALSE(DeviceTrackerTestAccess::Core(tracker).Bootstrapped());
  EXPECT_TRUE(restarted());
}

TEST(DeviceRecovery, InvalidImuCalibrationIsNeverCarriedOver) {
  TrackedObjectModel model;
  model.points = {{{-0.08, -0.07, -0.02}}, {{0.08, -0.07, 0.02}}, {{-0.08, 0.07, 0.03}},
                  {{0.08, 0.07, -0.03}}, {{0, 0, 0.08}}, {{0.04, 0.01, -0.06}}};
  model.normals.assign(6, {0, 0, 1});
  for (int i = 0; i < 6; ++i) model.channelToIndex[i] = i;
  RoomMap room;
  DeviceTracker tracker("LHR-INVALID", model, &room);
  BaseConfig base;
  base.valid = true; base.serial = 1; base.cal.assign(14, 0.0);
  tracker.OnBaseConfig(base, {});
  {
    // ResetFilter() replaces the filter object, so never hold this reference across it.
    auto& core = DeviceTrackerTestAccess::Core(tracker);
    LighthouseDirect::Seed seed;
    seed.base = 1; seed.calibrated = true;
    ASSERT_TRUE(core.SeedPose(seed));
    ASSERT_TRUE(core.Bootstrapped());
    DeviceTrackerTestAccess::Filter(tracker).x[28] = 0.5;  // fails the stock 0.262 rad check
    ASSERT_FALSE(core.AdvanceInertial(2.0));  // nothing new to integrate in this fixture
    EXPECT_FALSE(core.Filter().CalibrationHealthy());
  }
  DeviceTrackerTestAccess::Check(tracker, 10.0);
  EXPECT_FALSE(DeviceTrackerTestAccess::Core(tracker).Bootstrapped());
  EXPECT_FALSE(DeviceTrackerTestAccess::HasCarry(tracker));
  bool health = false, discarded = false;
  for (const auto& line : tracker.TakeLogs()) {
    health |= line.find("IMU misalignment") != std::string::npos;
    discarded |= line.find("discarding rejected IMU calibration") != std::string::npos;
  }
  EXPECT_TRUE(health);
  EXPECT_TRUE(discarded);
  EXPECT_NE(DeviceTrackerTestAccess::Core(tracker).Filter().x[28], 0.5);  // reset, not rejected state
  DeviceTrackerTestAccess::Acquired(tracker);
}

TEST(RoomMap, PersistsRoomsConfigsAndOotxFrames) {
  const auto path = (std::filesystem::temp_directory_path() / ("lh-test-roommap-" + std::to_string(lh::platform::ProcessId()) + ".json")).string();
  std::filesystem::remove(path);
  {
    RoomMap room;
    BaseDynamicState dynamic;
    dynamic.mode = 2;
    dynamic.sysUnlockCount = 7;
    room.OnBaseSeen(Config(0x11), dynamic);
    room.EstablishOrigin(0x11, {0.1, -0.2}, 1e-6);
    room.SetOotxFrame(0x11, {0, 1, 1, 0, 0xff, 1});
    ASSERT_TRUE(room.Save(path));
  }
  RoomMap loaded;
  ASSERT_TRUE(loaded.Load(path));
  ASSERT_EQ(loaded.KnownConfigs().count(0x11u), 1u);
  EXPECT_EQ(loaded.KnownConfigs().at(0x11u).second.mode, 2);
  EXPECT_EQ(loaded.OotxFrames().at(0x11u), (std::vector<uint8_t>{0, 1, 1, 0, 0xff, 1}));
  loaded.OnBaseSeen(Config(0x11), {});
  ASSERT_NE(loaded.Base(0x11), nullptr);
  EXPECT_TRUE(loaded.Base(0x11)->origin);
  // A different reset count marks the stored pose as uncertain (base may have moved).
  RoomMap second;
  ASSERT_TRUE(second.Load(path));
  BaseDynamicState moved;
  moved.sysUnlockCount = 8;
  second.OnBaseSeen(Config(0x11), moved);
  EXPECT_TRUE(second.Base(0x11)->origin);  // the origin itself is never relearned
  std::filesystem::remove(path);
}

struct ReplayResult {
  std::vector<SessionEvent> events;
  std::map<std::string, int> valid, total;
  SessionSnapshot last;
};

ReplayResult Replay(const std::string& roomPath, const std::string& capture = kCapture) {
  SessionOptions options;
  options.replayPath = capture + "/input.lhraw";
  options.replayRealtime = false;
  options.roomMapPath = roomPath;
  auto session = Session::Create(options);
  ReplayResult result;
  std::string error;
  const bool started = session->Start(&error);
  EXPECT_TRUE(started) << error;
  if (!started) return result;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
  while (!session->Finished()) {
    if (std::chrono::steady_clock::now() > deadline) { ADD_FAILURE() << "replay timed out"; break; }
    for (auto& e : session->PollEvents()) result.events.push_back(e);
    const auto snap = session->Snapshot();
    for (const auto& d : snap.devices) {
      ++result.total[d.info.serial];
      result.valid[d.info.serial] += d.pose.valid;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  for (auto& e : session->PollEvents()) result.events.push_back(e);
  result.last = session->Snapshot();
  session->Stop();
  return result;
}

// What a host is handed while a controller is waved around and hidden: no frame-to-
// frame jump (stock: 19.7 cm when light came back) and no inertial fly-away while it
// is hidden (stock: 23.4 cm). captures/controller-moving-hidden, every pose at 100 Hz.
TEST(OutputStage, NoJumpsAndNoFlyAwayWhenHidden) {
  const std::string capture = TestCapture("controller-moving-hidden/input.lhraw");
  if (!std::filesystem::exists(capture)) GTEST_SKIP();
  const auto trace = (std::filesystem::temp_directory_path() / ("lh-test-trace-" + std::to_string(lh::platform::ProcessId()) + ".jsonl")).string();
  SessionOptions options;
  options.replayPath = capture;
  options.replayRealtime = false;
  options.roomMapPath = "";
  options.replayTracePath = trace;
  auto session = Session::Create(options);
  std::string error;
  ASSERT_TRUE(session->Start(&error)) << error;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
  while (!session->Finished() && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(5));
  session->Stop();
  session.reset();  // closes the trace (Windows can't remove an open file)
  struct P { double t; bool valid; std::string state; Eigen::Vector3d p; };
  std::map<std::string, std::vector<P>> per;
  std::ifstream in(trace);
  for (std::string line; std::getline(in, line);) {
    const auto j = nlohmann::json::parse(line);
    const auto& x = j["position"];
    per[j["serial"].get<std::string>()].push_back({j["t"].get<double>(), j["valid"].get<int>() != 0, j["state"].get<std::string>(),
                                                   Eigen::Vector3d(x[0].get<double>(), x[1].get<double>(), x[2].get<double>())});
  }
  in.close();
  std::filesystem::remove(trace);
  ASSERT_FALSE(per.empty());
  int coasting = 0, valid = 0;
  for (const auto& [serial, poses] : per) {
    double step = 0, drift = 0;
    std::optional<Eigen::Vector3d> anchor;
    for (size_t i = 1; i < poses.size(); ++i) {
      const auto &a = poses[i - 1], &b = poses[i];
      valid += b.valid;
      if (a.valid && b.valid && b.t - a.t < 0.02) step = std::max(step, (b.p - a.p).norm());
      if (b.valid && b.state == "coasting") {
        ++coasting;
        if (!anchor) anchor = a.p;
        drift = std::max(drift, (b.p - *anchor).norm());
      } else {
        anchor.reset();
      }
    }
    EXPECT_LT(step, 0.10) << serial << ": a host frame jumped";
    EXPECT_LT(drift, 0.05) << serial << ": flew away while hidden";
  }
  EXPECT_GT(valid, 1000);
  EXPECT_GT(coasting, 20) << "the capture should hide the controller";
}

bool HasEvent(const ReplayResult& r, const std::string& text) {
  for (const auto& e : r.events)
    if (e.text.find(text) != std::string::npos) return true;
  return false;
}

// Recorded with the tracker and controller lying on the floor: one sensor on each
// reads ~2.3 mrad off (reflection), so the full-set bootstrap never converged
// and neither device tracked. Leaving the worst sensor out lets both bootstrap.
TEST(SessionReplay, BootstrapsDespiteOneBadSensor) {
#ifndef LH_TEST_USB
  GTEST_SKIP() << "replay built without USB/capture layer";
#else
  const std::string capture = TestCapture("floor-reflection");
  if (!std::filesystem::exists(capture + "/input.lhraw")) GTEST_SKIP() << "no floor capture";
  const auto room = (std::filesystem::temp_directory_path() / ("lh-test-floor-room-" + std::to_string(lh::platform::ProcessId()) + ".json")).string();
  std::filesystem::remove(room);
  const auto r = Replay(room, capture);
  std::filesystem::remove(room);
  for (const char* serial : {"LHR-0AC3E7C8", "LHR-13DA81D1"}) {
    EXPECT_TRUE(HasEvent(r, std::string(serial) + ": ----- BOOTSTRAPPED")) << serial;
    EXPECT_GT(r.valid.count(serial) ? r.valid.at(serial) : 0, 0) << serial;
  }
#endif
}

TEST(SessionReplay, ControllerPowerCycleStartsFreshTrackingEpoch) {
#ifndef LH_TEST_USB
  GTEST_SKIP() << "replay built without USB/capture layer";
#else
  if (!std::filesystem::exists(kCapture + "/input.lhraw")) GTEST_SKIP();
  const auto file = std::filesystem::temp_directory_path() / ("lh-reconnect-" + std::to_string(lh::platform::ProcessId()) + ".lhraw");
  usb::RawCaptureReader reader;
  ASSERT_TRUE(reader.Open(kCapture + "/input.lhraw"));
  std::vector<usb::EndpointInfo> endpoints;
  std::vector<usb::RawReport> reports;
  double start = 0;
  reader.ReadAll([&](const auto& ep) { if (ep.serial == "LHR-13DA81D1") endpoints.push_back(ep); },
                 [&](const auto& r) {
    bool wanted = false;
    for (const auto& ep : endpoints) wanted |= ep.ep == r.ep;
    if (!wanted) return;
    if (!start) start = r.hostTime;
    if (r.hostTime - start < 8.0) reports.push_back(r);
  });
  ASSERT_FALSE(reports.empty());
  {
    usb::RawCaptureWriter writer;
    ASSERT_TRUE(writer.Open(file.string()));
    for (const auto& ep : endpoints) writer.WriteEndpoint(ep);
    for (const auto& r : reports) writer.WriteReport(r.hostTime, r.ep, r.lastInBatch, r.data.data(), r.data.size());
    const uint8_t off = 0x26;
    writer.WriteReport(reports.back().hostTime + 0.1, reports.back().ep, true, &off, 1);
    // Same controller boots again with its old device timestamps. Only host time advances.
    for (const auto& r : reports) writer.WriteReport(r.hostTime + 12, r.ep, r.lastInBatch, r.data.data(), r.data.size());
    writer.Close();
  }
  SessionOptions options;
  options.replayPath = file.string(); options.replayRealtime = false;
  options.steamDatabasePath = kCapture + "/lighthousedb.json"; options.saveRoomMap = false;
  auto session = Session::Create(options);
  std::string error;
  ASSERT_TRUE(session->Start(&error)) << error;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
  while (!session->Finished() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  EXPECT_TRUE(session->Finished());
  int activations = 0, bootstraps = 0;
  for (const auto& e : session->PollEvents()) {
    if (e.serial != "LHR-13DA81D1") continue;
    activations += e.type == SessionEventType::DeviceActivated;
    bootstraps += e.text.find("BOOTSTRAPPED base") != std::string::npos;
  }
  const auto snapshot = session->Snapshot();
  ASSERT_EQ(snapshot.devices.size(), 1u);
  EXPECT_TRUE(snapshot.devices.front().pose.valid);
  EXPECT_EQ(activations, 2);
  EXPECT_EQ(bootstraps, 2);
  session->Stop();
  std::filesystem::remove(file);
#endif
}

TEST(SessionReplay, TracksBothDevicesInTheSharedRoom) {
#ifndef LH_TEST_USB
  GTEST_SKIP() << "replay built without USB/capture layer";
#endif
  if (!std::filesystem::exists(kCapture + "/input.lhraw")) GTEST_SKIP() << "no motion capture";
  const auto room = (std::filesystem::temp_directory_path() / ("lh-test-session-room-" + std::to_string(lh::platform::ProcessId()) + ".json")).string();
  std::filesystem::remove(room);
  const auto first = Replay(room);
  // Roles come from the device JSON; receivers without tracking configs are hidden.
  ASSERT_EQ(first.last.devices.size(), 2u);
  std::map<std::string, const DeviceState*> devices;
  for (const auto& d : first.last.devices) devices[d.info.serial] = &d;
  ASSERT_TRUE(devices.count("LHR-0704E07A") && devices.count("LHR-13DA81D1"));
  EXPECT_EQ(devices["LHR-0704E07A"]->info.role, DeviceRole::Hmd);
  EXPECT_EQ(devices["LHR-13DA81D1"]->info.role, DeviceRole::Controller);
  EXPECT_TRUE(devices["LHR-13DA81D1"]->input.has_value());
  EXPECT_FALSE(devices["LHR-0704E07A"]->input.has_value());
  EXPECT_TRUE(devices["LHR-13DA81D1"]->info.batteryPercent.has_value());
  EXPECT_EQ(first.last.roomId, "steamvr-1770781428");
  // Both devices track in world space for nearly the whole recording and end tracking
  // with both bases in their filter.
  for (const auto& [serial, d] : devices) {
    EXPECT_EQ(d->pose.state, TrackingState::Tracking) << serial;
    EXPECT_TRUE(d->pose.valid) << serial;
    EXPECT_EQ(d->pose.bases.size(), 2u) << serial;
    EXPECT_GT(double(first.valid.at(serial)) / first.total.at(serial), 0.9) << serial;
  }
  // The second device identifies bases from the first device's OOTX frames; devices
  // that lose tracking re-acquire with their calibration carried over.
  EXPECT_TRUE(HasEvent(first, "identified base"));
  EXPECT_TRUE(HasEvent(first, "REACQUIRED base"));
  EXPECT_TRUE(HasEvent(first, "added base E236DC40 from the room map"));
  // Second run with the saved room map: the first device identifies its bases from
  // the persisted OOTX frames instead of waiting for a full decode.
  const auto second = Replay(room);
  bool identifiedBeforeAnyDecode = false;
  for (const auto& e : second.events) {
    if (e.text.find("identified base") != std::string::npos) { identifiedBeforeAnyDecode = true; break; }
    if (e.type == SessionEventType::BaseStationSeen) break;
  }
  EXPECT_TRUE(identifiedBeforeAnyDecode || HasEvent(second, "LHR-0704E07A: identified base"));
  std::filesystem::remove(room);
}

}  // namespace

namespace {
using namespace lh;
using namespace lh::runtime;

// Simulated room: exact LH1 angles for a real Knuckles model at known poses.
TEST(RoomSolver, RecoversPerturbedBaseFromSimulatedSweeps) {
  const auto config = ReadFile(kCapture + "/device-configs/device-1.json");
  if (config.empty()) GTEST_SKIP() << "no device config";
  TrackedObjectModel raw;
  ASSERT_TRUE(TrackedObjectModel::FromJson(config, &raw));
  auto model = std::make_shared<const TrackedObjectModel>(raw.InImuFrame());
  std::array<double, 14> calA{}, calB{};
  for (int i = 0; i < 14; ++i) { calA[size_t(i)] = 0.001 * (i % 5); calB[size_t(i)] = -0.0007 * (i % 3); }
  // Base A at the origin looking down -Z; base B 4 m away, facing back toward A's area.
  const Eigen::Isometry3d worldFromA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d worldFromB = Eigen::Isometry3d::Identity();
  worldFromB.linear() = Eigen::AngleAxisd(M_PI * 0.9, Eigen::Vector3d::UnitY()).toRotationMatrix();
  worldFromB.translation() = Eigen::Vector3d(0.8, 0.2, -4.0);
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0, 5e-5);
  std::uniform_real_distribution<double> u(-1, 1);
  RoomSolveInput in;
  in.origin = 1;
  in.calibration[1] = calA;
  in.calibration[2] = calB;
  in.models["dev"] = model;
  in.known[1] = worldFromA;
  for (int k = 0; k < 40; ++k) {
    Eigen::Isometry3d worldFromDevice = Eigen::Isometry3d::Identity();
    worldFromDevice.translation() = Eigen::Vector3d(0.6 * u(rng), 0.4 * u(rng), -2.0 + 0.5 * u(rng));
    // Mostly turned so sensors face both bases (they sit on opposite sides).
    worldFromDevice.linear() = (Eigen::AngleAxisd(M_PI / 2 + 0.6 * u(rng), Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(0.4 * u(rng), Eigen::Vector3d::UnitX())).toRotationMatrix();
    auto kf = std::make_shared<Keyframe>();
    kf->device = "dev";
    kf->time = k;
    kf->primary = 1;
    kf->primaryFromDevice = worldFromA.inverse() * worldFromDevice;
    for (auto [serial, worldFromBase, cal] : {std::tuple{1u, worldFromA, calA}, std::tuple{2u, worldFromB, calB}}) {
      for (size_t i = 0; i < model->points.size(); ++i) {
        const Eigen::Vector3d p = worldFromDevice * Eigen::Vector3d(model->points[i].data());
        const Eigen::Vector3d n = worldFromDevice.rotation() * Eigen::Vector3d(model->normals[i].data());
        const Eigen::Vector3d b = worldFromBase.inverse() * p;
        if (b.z() > -0.2 || -n.dot((p - worldFromBase.translation()).normalized()) < 0.5) continue;
        for (int axis = 0; axis < 2; ++axis) {
          KeySample s;
          s.sample.axis = axis;
          s.sample.time = k;
          s.sample.sensor = 0;
          for (const auto& [channel, index] : model->channelToIndex) if (size_t(index) == i) s.sample.sensor = channel;
          s.sample.angle = model::Lh1Angle(axis, b.x(), b.y(), b.z(), cal.data() + axis * model::kCalPerAxis) + noise(rng);
          s.point = Eigen::Vector3d(model->points[i].data());
          s.normal = Eigen::Vector3d(model->normals[i].data());
          kf->samples[serial].push_back(s);
        }
      }
    }
    if (kf->samples.size() == 2 && kf->samples[1].size() >= 8 && kf->samples[2].size() >= 8) in.keyframes.push_back(kf);
  }
  ASSERT_GE(in.keyframes.size(), 10u);
  // Start B 10 cm / 2 deg off: the solver must pull it back.
  Eigen::Isometry3d guess = worldFromB;
  guess.translation() += Eigen::Vector3d(0.06, -0.05, 0.06);
  guess.linear() = guess.linear() * Eigen::AngleAxisd(0.035, Eigen::Vector3d::UnitX()).toRotationMatrix();
  in.known[2] = guess;
  const auto out = SolveRoom(in);
  ASSERT_TRUE(out.ok) << out.summary;
  ASSERT_TRUE(out.bases.count(2));
  const auto& solved = out.bases.at(2).worldFromBase;
  EXPECT_LT((solved.translation() - worldFromB.translation()).norm(), 0.002) << out.summary;
  EXPECT_LT(Eigen::AngleAxisd(solved.rotation().transpose() * worldFromB.rotation()).angle(), 0.0005);
  EXPECT_LT(out.rms, 1e-4);
  EXPECT_GT(out.bases.at(2).positionVariance, 0);
  // The origin never moves.
  EXPECT_TRUE(out.bases.at(1).worldFromBase.isApprox(worldFromA));
}

}  // namespace

namespace {
struct MsReplay {
  std::vector<SessionEvent> events;
  SessionSnapshot snapshot;
  std::vector<std::pair<double, bool>> valid;  // (t, valid) of one device, 100 Hz
};
MsReplay MsRun(const std::string& capture, const std::string& only, double to, bool mixedLight = false) {
  const auto trace = (std::filesystem::temp_directory_path() / ("lh-test-ms-" + std::to_string(lh::platform::ProcessId()) + ".jsonl")).string();
  SessionOptions options;
  options.replayPath = capture;
  options.replayRealtime = false;
  options.replayTo = to;
  options.mixedLight.enabled = mixedLight;
  if (!only.empty()) options.replayOnly = {only};
  options.saveRoomMap = false;
  options.roomMapPath = (std::filesystem::temp_directory_path() / ("lh-test-ms-room-" + std::to_string(lh::platform::ProcessId()) + ".json")).string();
  options.replayTracePath = trace;
  auto session = Session::Create(options);
  MsReplay r;
  std::string error;
  EXPECT_TRUE(session->Start(&error)) << error;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
  while (!session->Finished() && std::chrono::steady_clock::now() < deadline) {
    for (auto& e : session->PollEvents()) r.events.push_back(std::move(e));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  for (auto& e : session->PollEvents()) r.events.push_back(std::move(e));
  r.snapshot = session->Snapshot();
  session->Stop();
  session.reset();  // closes the trace (Windows can't remove an open file)
  {
    std::ifstream in(trace);
    for (std::string line; std::getline(in, line);) {
      const auto j = nlohmann::json::parse(line);
      r.valid.push_back({j["t"].get<double>(), j["valid"].get<int>() != 0});
    }
  }
  std::filesystem::remove(trace);
  return r;
}
bool MsHasEvent(const MsReplay& r, const std::string& text) {
  for (const auto& e : r.events) if (e.text.find(text) != std::string::npos) return true;
  return false;
}
}  // namespace

// lhbs multi-station mode, 2 stations at slots 0 and 1 of N = 4 (captures/ms-a): the
// layout is detected, both stations' OOTX decode (the slave's only on its own sweeping
// flashes) and the controller tracks from the start.
TEST(MultiStation, DetectsDecodesAndTracks) {
  const std::string capture = TestCapture("ms-a/input.lhraw");
  if (!std::filesystem::exists(capture)) GTEST_SKIP();
  const auto r = MsRun(capture, "LHR-13DA81D1", 40);
  EXPECT_TRUE(MsHasEvent(r, "multi-station layout detected"));
  std::set<std::string> seen;
  for (const auto& b : r.snapshot.bases) if (b.seen) seen.insert(b.serial);
  EXPECT_TRUE(seen.count("LHB-E236DC40"));
  EXPECT_TRUE(seen.count("LHB-A101B835"));
  ASSERT_FALSE(r.valid.empty());
  const double start = r.valid.front().first;
  int n = 0, ok = 0;
  for (const auto& [t, v] : r.valid) if (t - start > 5) { ++n; ok += v; }
  EXPECT_GT(ok, 0.95 * n) << ok << "/" << n;
}

// The 1.0 + 2.0 switch on in a room with no 2.0 station (lhbs multi-station, whose SteamVR
// database has a 2.0 station of another room): never in use, not even before the room is known,
// so multi-station detection and tracking are as with it off (DetectsDecodesAndTracks).
TEST(MixedLight, NeverInUseInARoomWithout2Point0Stations) {
  const std::string capture = TestCapture("ms-a/input.lhraw");
  if (!std::filesystem::exists(capture)) GTEST_SKIP();
  const auto r = MsRun(capture, "LHR-13DA81D1", 40, true);
  EXPECT_FALSE(MsHasEvent(r, "1.0 + 2.0 base stations together: in use"));
  EXPECT_TRUE(MsHasEvent(r, "multi-station layout detected"));
  ASSERT_FALSE(r.valid.empty());
  const double start = r.valid.front().first;
  int n = 0, ok = 0;
  for (const auto& [t, v] : r.valid) if (t - start > 5) { ++n; ok += v; }
  EXPECT_GT(ok, 0.95 * n) << ok << "/" << n;
}

// In a 2.0 room it's in use, and LH2 light alone (the recording's mode) tracks as before.
TEST(MixedLight, InUseWith2Point0StationsAndLh2LightStillTracks) {
  const std::string capture = TestCapture("lh2-one-controller/input.lhraw");
  if (!std::filesystem::exists(capture)) GTEST_SKIP();
  const auto r = MsRun(capture, "", 0, true);
  EXPECT_TRUE(MsHasEvent(r, "1.0 + 2.0 base stations together: in use"));
  ASSERT_FALSE(r.valid.empty());
  const double start = r.valid.front().first;
  int n = 0, ok = 0;
  for (const auto& [t, v] : r.valid) if (t - start > 10) { ++n; ok += v; }
  EXPECT_GT(ok, 0.97 * n) << ok << "/" << n;
}

// A stock 2-station room never switches to the multi-station layout.
TEST(MultiStation, StockRoomStaysStock) {
  const std::string capture = TestCapture("controller-moving-hidden/input.lhraw");
  if (!std::filesystem::exists(capture)) GTEST_SKIP();
  const auto r = MsRun(capture, "", 0);
  EXPECT_FALSE(MsHasEvent(r, "multi-station"));
  EXPECT_FALSE(r.valid.empty());
}

// N = 2, policy ALL (captures/ms-n2-all): the slave's OOTX arrives at the stock rate,
// and with either station blocked (60-75 s, 75-90 s) the device stays tracked: the
// primary switches to the visible base instead of drifting off with it (it ran off at
// 1.4 m/s before).
TEST(MultiStation, PolicyAllAndBlockedStations) {
  const std::string capture = TestCapture("ms-n2-all/input.lhraw");
  if (!std::filesystem::exists(capture)) GTEST_SKIP();
  const auto r = MsRun(capture, "LHR-13DA81D1", 0);
  EXPECT_TRUE(MsHasEvent(r, "policy ALL"));
  EXPECT_TRUE(MsHasEvent(r, "switching to base A101B835"));
  EXPECT_FALSE(MsHasEvent(r, "position out of bounds"));
  ASSERT_FALSE(r.valid.empty());
  const double start = r.valid.front().first;
  int n = 0, ok = 0;
  for (const auto& [t, v] : r.valid) if (t - start > 5) { ++n; ok += v; }
  EXPECT_GT(ok, 0.995 * n) << ok << "/" << n;
}

// Stations woken from standby (captures/ms-n2-startup): light starts ~49 s in, the
// master alone first (tracked by the stock path), the slave ~5 s later. Rotors spinning
// up must not wedge the multi timing reference, and the handover to the multi path must
// not lose the device.
TEST(MultiStation, WakeFromStandbyHandover) {
  const std::string capture = TestCapture("ms-n2-startup/input.lhraw");
  if (!std::filesystem::exists(capture)) GTEST_SKIP();
  const auto r = MsRun(capture, "LHR-13DA81D1", 0);
  // (Short stock IMU-alignment resets also happen in stock rooms; they recover in
  // < 0.1 s. What must not happen is an outage at the handover.)
  double detected = -1;
  for (const auto& e : r.events)
    if (detected < 0 && e.text.find("multi-station layout detected") != std::string::npos) detected = e.hostTime;
  ASSERT_GT(detected, 0);
  double gapStart = -1, worst = 0;
  for (const auto& [t, v] : r.valid) {
    if (t < detected) continue;
    if (!v && gapStart < 0) gapStart = t;
    if (v && gapStart >= 0) { worst = std::max(worst, t - gapStart); gapStart = -1; }
  }
  EXPECT_LT(worst, 0.3);
  std::set<std::string> seen;
  for (const auto& b : r.snapshot.bases) if (b.seen) seen.insert(b.serial);
  EXPECT_TRUE(seen.count("LHB-A101B835"));
}

// Regression on the bundled 2.0 recordings (tests/data/captures): every device tracks
// (>= 97 % valid after a 10 s warm-up) and never jumps 20 cm between host frames.
// lh2-one-controller: one Index controller, 31 s. lh2-five-devices: two controllers and
// three trackers in a four-base 2.0 room, the first 30 s.
class Lh2Regression : public ::testing::TestWithParam<const char*> {};
TEST_P(Lh2Regression, TracksEveryDeviceWithoutJumps) {
  const std::string capture = TestCapture(std::string(GetParam()) + "/input.lhraw");
  if (!std::filesystem::exists(capture)) GTEST_SKIP() << capture;
  const auto trace = (std::filesystem::temp_directory_path() / ("lh-test-lh2-" + std::to_string(lh::platform::ProcessId()) + ".jsonl")).string();
  SessionOptions options;
  options.replayPath = capture;
  options.replayRealtime = false;
  options.roomMapPath = "";
  options.replayTracePath = trace;
  auto session = Session::Create(options);
  std::string error;
  ASSERT_TRUE(session->Start(&error)) << error;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(180);
  while (!session->Finished() && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(5));
  session->Stop();
  session.reset();
  struct P { double t; bool valid; Eigen::Vector3d p; };
  std::map<std::string, std::vector<P>> per;
  double t0 = -1;
  std::ifstream in(trace);
  for (std::string line; std::getline(in, line);) {
    const auto j = nlohmann::json::parse(line);
    const auto& x = j["position"];
    const double t = j["t"].get<double>();
    if (t0 < 0) t0 = t;
    if (t - t0 < 10) continue;
    per[j["serial"].get<std::string>()].push_back({t, j["valid"].get<int>() != 0,
                                                   Eigen::Vector3d(x[0].get<double>(), x[1].get<double>(), x[2].get<double>())});
  }
  in.close();
  std::filesystem::remove(trace);
  ASSERT_FALSE(per.empty());
  for (const auto& [serial, poses] : per) {
    int valid = 0;
    double step = 0;
    for (size_t i = 0; i < poses.size(); ++i) {
      valid += poses[i].valid;
      if (i && poses[i - 1].valid && poses[i].valid && poses[i].t - poses[i - 1].t < 0.05)
        step = std::max(step, (poses[i].p - poses[i - 1].p).norm());
    }
    EXPECT_GE(double(valid) / poses.size(), 0.97) << serial;
    EXPECT_LT(step, 0.20) << serial << ": a host frame jumped";
  }
}
INSTANTIATE_TEST_SUITE_P(Bundled, Lh2Regression, ::testing::Values("lh2-one-controller", "lh2-five-devices"));

namespace {
lh::MixedLightOptions MixedFromSettings(const std::string& text, bool cliOn = false) {
  const auto path = (std::filesystem::temp_directory_path() /
                     ("lh-test-mixed-settings-" + std::to_string(lh::platform::ProcessId()) + ".json")).string();
  { std::ofstream(path) << text; }
  SessionOptions options;
  options.replayPath = "no-such-capture.lhraw";  // Start reads the settings, then fails to open it
  options.settingsPath = path;
  options.mixedLight.enabled = cliOn;
  auto session = Session::Create(options);
  std::string error;
  session->Start(&error);
  const auto mixed = session->MixedLight();
  session->Stop();
  session.reset();
  std::filesystem::remove(path);
  return mixed;
}
}  // namespace

TEST(MixedLight, SettingsFileTurnsItOnWithItsOptions) {
  const auto m = MixedFromSettings(R"({"mixedLight": {"enabled": true, "trackers": false, "scout": false}})");
  EXPECT_TRUE(m.enabled);
  EXPECT_TRUE(m.controllers);
  EXPECT_FALSE(m.trackers);
  EXPECT_FALSE(m.scout);
  EXPECT_FALSE(m.preferLh2) << "LH1 mode once the rotors are known, unless asked";
  EXPECT_TRUE(MixedFromSettings(R"({"mixedLight": {"enabled": true, "preferLh2": true}})").preferLh2);
}

TEST(MixedLight, SettingsFileAcceptsABool) {
  EXPECT_TRUE(MixedFromSettings(R"({"mixedLight": true})").enabled);
  EXPECT_FALSE(MixedFromSettings(R"({"mixedLight": false})").enabled);
  EXPECT_FALSE(MixedFromSettings(R"({})").enabled);
}

TEST(MixedLight, ABadValueLeavesItOff) {
  EXPECT_FALSE(MixedFromSettings(R"({"mixedLight": {"enabled": "yes"}})").enabled);
  EXPECT_FALSE(MixedFromSettings(R"({"mixedLight": 1})").enabled);
}

TEST(MixedLight, TheCommandLineWins) {
  EXPECT_TRUE(MixedFromSettings(R"({"mixedLight": false})", true).enabled);
}

// #86: a room learned before device model scales were applied (no "modelScale") keeps its
// bases, unlocked and less certain, once: the solver refines them in the same room.
TEST(RoomMap, RoomLearnedWithoutModelScalesIsRefinedOnce) {
  const auto path = (std::filesystem::temp_directory_path() / ("lh-test-scale-" + std::to_string(lh::platform::ProcessId()) + ".json")).string();
  {
    std::ofstream f(path);
    f << R"({"rooms": [{"id": "room-1", "source": "lh", "updated": 1, "bases": [
          {"serial": 1, "origin": true, "observations": 0, "positionVariance": 0, "rotationVariance": 0,
           "pose": {"position": [0, 0, 0], "orientation": [1, 0, 0, 0]}},
          {"serial": 2, "observations": 500, "positionVariance": 1e-6, "rotationVariance": 1e-7,
           "pose": {"position": [3, 0.5, -2], "orientation": [1, 0, 0, 0]}}]}]})";
  }
  RoomMap room;
  ASSERT_TRUE(room.Load(path));
  room.OnBaseSeen(Config(1), {});
  room.OnBaseSeen(Config(2), {});
  ASSERT_NE(room.Base(2), nullptr);
  EXPECT_FALSE(RoomMap::Locked(*room.Base(2)));
  EXPECT_GE(room.Base(2)->positionVariance, 1e-4);
  EXPECT_LE(room.Base(2)->positionVariance, 0.02 * 0.02);  // still learned (40 keyframes, under 20 mm)
  EXPECT_GE(room.Base(2)->observations, lh::BaseStationState::kLearnedObservations);
  EXPECT_NEAR(room.Base(2)->worldFromBase.translation().x(), 3, 1e-9);  // kept, not reset
  EXPECT_TRUE(room.Base(1)->origin);
  bool said = false;
  for (const auto& line : room.TakeLogs()) said |= line.find("model scales") != std::string::npos;
  EXPECT_TRUE(said);
  ASSERT_TRUE(room.Save(path));
  std::ifstream in(path);
  EXPECT_EQ(nlohmann::json::parse(in).at("rooms")[0].at("modelScale"), lh::runtime::Room::kModelScale);
  in.close();  // Windows can't remove an open file
  RoomMap again;
  ASSERT_TRUE(again.Load(path));
  std::filesystem::remove(path);
  for (const auto& line : again.TakeLogs()) EXPECT_EQ(line.find("model scales"), std::string::npos);
}
