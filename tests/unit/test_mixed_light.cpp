#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include "runtime/mixed_light.hpp"

using lh::runtime::ChooseMixedLight;
using lh::runtime::Lh2Light;
using lh::runtime::Lh2LightEnough;
using lh::runtime::MixedLightStep;
using lh::runtime::MixedLightView;

namespace {

constexpr double kVisits = 45;

// A device in LH2 mode, tracking, knowing its rotors.
MixedLightView InLh2(double inMode) {
  MixedLightView v;
  v.mode = 1;
  v.inMode = inMode;
  v.sinceSent = inMode;
  v.readyRotors = 3;
  v.tracking = true;
  v.calm = true;
  return v;
}

// A device in LH1 mode, tracking, with light.
MixedLightView InLh1(double inMode) {
  MixedLightView v = InLh2(inMode);
  v.mode = 0;
  return v;
}

int Mode(const MixedLightView& v) { return ChooseMixedLight(v, kVisits).mode; }

}  // namespace

TEST(MixedLight, StartsInLh2) {
  MixedLightView v;
  EXPECT_EQ(Mode(v), 1);
  v.stayLh2 = true;
  EXPECT_EQ(Mode(v), 1);
}

TEST(MixedLight, LeavesLh2OnceItKnowsItsRotors) {
  EXPECT_EQ(Mode(InLh2(1.0)), -1);  // however ready, a visit lasts long enough to hear a new rotor
  EXPECT_EQ(Mode(InLh2(2.5)), 0);
}

TEST(MixedLight, WaitsForARotorStillBeingLearned) {
  MixedLightView v = InLh2(5.0);
  v.learningRotors = 1;
  EXPECT_EQ(Mode(v), -1);
  v.inMode = 8.0;
  EXPECT_EQ(Mode(v), 0);  // but not for ever
}

TEST(MixedLight, GivesAStationTimeToBeIdentified) {
  MixedLightView v = InLh2(20.0);
  v.unnamedRotors = 1;
  EXPECT_EQ(Mode(v), -1);
  v.inMode = 25.0;
  EXPECT_EQ(Mode(v), 0);
}

TEST(MixedLight, WithNoPoseStaysLongerToIdentifyAStation) {
  MixedLightView v = InLh2(59.0);
  v.unnamedRotors = 1;
  v.tracking = false;
  EXPECT_EQ(Mode(v), -1);
  v.inMode = 60.0;
  EXPECT_EQ(Mode(v), 0);
}

TEST(MixedLight, WaitsForTrackingToComeBack) {
  MixedLightView v = InLh2(5.0);
  v.tracking = false;
  EXPECT_EQ(Mode(v), -1);
  v.inMode = 8.0;
  EXPECT_EQ(Mode(v), 0);
}

TEST(MixedLight, StationsWakingAreWaitedFor) {
  MixedLightView v = InLh2(29.0);
  v.readyRotors = 0;
  v.tracking = false;
  v.dark = 29.0;
  EXPECT_NE(Mode(v), 0);  // LH1 mode now would see the 1.0 station first
  v.dark = v.inMode = 30.0;
  EXPECT_EQ(Mode(v), 0);
}

TEST(MixedLight, AVisitInDarknessEndsEarly) {
  MixedLightView v = InLh2(4.0);
  v.tracking = false;
  v.dark = 4.0;
  EXPECT_EQ(Mode(v), 0);
  v.inMode = 2.0;  // the switch itself takes half a second
  v.dark = 2.0;
  EXPECT_EQ(Mode(v), -1);
}

// MixedLightOptions::preferLh2: LH2 mode while the 2.0 stations light the device well.
TEST(MixedLight, The2Point0StationsLightADeviceTogether) {
  EXPECT_TRUE(Lh2LightEnough({{20, 4.0}}));               // one station, a pose by itself
  EXPECT_FALSE(Lh2LightEnough({{19, 4.0}}));
  EXPECT_TRUE(Lh2LightEnough({{40, 3.6}, {38, 3.7}}));    // a Knuckles lit by two, neither enough alone
  EXPECT_TRUE(Lh2LightEnough({{25, 2.8}, {26, 3.1}}));    // the one lighting too few sensors left out
  EXPECT_FALSE(Lh2LightEnough({{25, 2.8}, {25, 3.1}}));
  EXPECT_FALSE(Lh2LightEnough({{40, 2.9}}));              // too few sensors a frame, however often
  EXPECT_FALSE(Lh2LightEnough({{8, 9.0}, {20, 3.5}}));    // a station seen in a few frames isn't counted
  EXPECT_FALSE(Lh2LightEnough({}));
}

TEST(MixedLight, PreferringLh2StaysThereWhileThe2Point0LightIsEnough) {
  MixedLightView v = InLh2(2.5);
  v.preferLh2 = true;
  v.lh2Covered = 2.5;
  EXPECT_EQ(Mode(v), -1);  // where it would have left with its rotors ready
  v.inMode = v.lh2Covered = 600.0;
  EXPECT_EQ(Mode(v), -1);
  v.preferLh2 = false;
  EXPECT_EQ(Mode(v), 0);
}

TEST(MixedLight, PreferringLh2LeavesWhenThe2Point0LightRunsShort) {
  MixedLightView v = InLh2(600.0);
  v.preferLh2 = true;
  v.lh2Short = 0.1;
  EXPECT_EQ(Mode(v), -1);  // a sweep or two missed
  v.lh2Short = 0.25;
  const MixedLightStep step = ChooseMixedLight(v, kVisits);
  EXPECT_EQ(step.mode, 0);
  EXPECT_EQ(std::string(step.why), "too little 2.0 light");
  v.inMode = 1.0;
  EXPECT_EQ(Mode(v), -1);  // but a visit still lasts long enough to hear a new rotor
  v.inMode = 5.0;
  v.learningRotors = 1;
  EXPECT_EQ(Mode(v), -1);  // and to learn one
}

TEST(MixedLight, PreferringLh2StaysWhileARotorRelearns) {
  MixedLightView v = InLh2(87.0);
  v.preferLh2 = true;
  v.lh2Covered = 87.0;
  v.learningRotors = 1;
  EXPECT_EQ(Mode(v), -1);  // the light is enough: it's learned here
  v.learningRotors = 0;
  v.unnamedRotors = 1;
  EXPECT_EQ(Mode(v), -1);  // and a station named
  v.lh2Covered = 0;
  v.lh2Short = 0.25;
  const MixedLightStep step = ChooseMixedLight(v, kVisits);
  EXPECT_EQ(step.mode, 0);  // short of light: a visit's limits, as without preferLh2
  EXPECT_EQ(std::string(step.why), "visit over");
  v.unnamedRotors = 0;
  v.learningRotors = 1;
  v.lh2Covered = 87.0;
  v.lh2Short = 0;
  v.preferLh2 = false;
  EXPECT_EQ(std::string(ChooseMixedLight(v, kVisits).why), "visit over");
}

TEST(MixedLight, PreferringLh2LeavesWhenLostOnThe2Point0LightAlone) {
  MixedLightView v = InLh2(600.0);
  v.preferLh2 = true;
  v.tracking = false;
  v.lh2Short = 0.6;
  v.lost = 0.5;
  EXPECT_EQ(Mode(v), -1);  // long enough to recover where it is
  v.lost = 1.0;
  const MixedLightStep step = ChooseMixedLight(v, kVisits);
  EXPECT_EQ(step.mode, 0);
  EXPECT_EQ(std::string(step.why), "lost on 2.0 light");
  v.dark = 5.0;
  v.lost = 0.5;
  EXPECT_EQ(std::string(ChooseMixedLight(v, kVisits).why), "no 2.0 light");
}

TEST(MixedLight, PreferringLh2GoesBackOnceThe2Point0LightHasHeld) {
  MixedLightView v = InLh1(30.0);
  v.preferLh2 = true;
  v.lh2Covered = 9.0;
  EXPECT_EQ(Mode(v), -1);  // not held long enough
  v.lh2Covered = 10.0;
  const MixedLightStep step = ChooseMixedLight(v, kVisits);
  EXPECT_EQ(step.mode, 1);
  EXPECT_EQ(std::string(step.why), "2.0 light is enough");
  v.calm = false;
  EXPECT_EQ(Mode(v), -1);  // half a second without light: not while moving
  v.calm = true;
  v.inMode = 19.0;
  EXPECT_EQ(Mode(v), -1);  // nor straight back: at most one such gap in 20 s
  v.inMode = 30.0;
  v.tracking = false;
  EXPECT_EQ(Mode(v), -1);  // lost, the usual rule decides
  v.tracking = true;
  v.preferLh2 = false;
  EXPECT_EQ(Mode(v), -1);  // off: only the visits
}

TEST(MixedLight, PreferringLh2WaitsForA2Point0StationToBePlaced) {
  // LH2 mode gives no pose yet: staying there would leave the device blind.
  MixedLightView v = InLh2(2.5);
  v.preferLh2 = true;
  v.lh2Covered = 2.5;
  v.lh2Unplaced = true;
  EXPECT_EQ(Mode(v), 0);
  v = InLh1(30.0);
  v.preferLh2 = true;
  v.lh2Covered = 30.0;
  v.lh2Unplaced = true;
  EXPECT_EQ(Mode(v), -1);
}

TEST(MixedLight, VisitsWhenCalm) {
  EXPECT_EQ(Mode(InLh1(44.0)), -1);
  EXPECT_EQ(Mode(InLh1(45.0)), 1);
}

TEST(MixedLight, AVisitWaitsOutMovementButNotForEver) {
  MixedLightView v = InLh1(60.0);
  v.calm = false;
  EXPECT_EQ(Mode(v), -1);
  v.inMode = 90.0;
  EXPECT_EQ(Mode(v), 1);
}

TEST(MixedLight, VisitsCanBeTurnedOff) {
  EXPECT_EQ(ChooseMixedLight(InLh1(1000.0), 0).mode, -1);
  MixedLightView lost = InLh1(1000.0);
  lost.lost = 2.0;
  EXPECT_EQ(ChooseMixedLight(lost, 0).mode, 1);  // going for help still happens
}

TEST(MixedLight, LostInLh1GoesToLh2) {
  MixedLightView v = InLh1(10.0);
  v.tracking = false;
  v.lost = 0.5;
  EXPECT_EQ(Mode(v), -1);  // long enough to recover where it is
  v.lost = 1.0;
  EXPECT_EQ(Mode(v), 1);
  v.inMode = 2.0;
  EXPECT_EQ(Mode(v), -1);  // not straight back after arriving lost
}

TEST(MixedLight, NoLightInLh1GoesToLh2) {
  MixedLightView v = InLh1(10.0);
  v.dark = 5.0;
  EXPECT_EQ(Mode(v), 1);
}

TEST(MixedLight, LooksForRotorsAgainWhenItKnowsNone) {
  MixedLightView v = InLh1(14.0);
  v.readyRotors = 0;
  EXPECT_EQ(Mode(v), -1);
  v.inMode = 15.0;
  EXPECT_EQ(Mode(v), 1);
}

TEST(MixedLight, ResendsItsModeWhileDark) {
  MixedLightView v = InLh2(10.0);
  v.readyRotors = 0;
  v.tracking = false;
  v.dark = 10.0;
  v.sinceSent = 4.0;
  EXPECT_EQ(Mode(v), -1);
  v.sinceSent = 5.0;
  const MixedLightStep step = ChooseMixedLight(v, kVisits);
  EXPECT_EQ(step.mode, 1);
  EXPECT_EQ(std::string(step.why), "re-send");
}

TEST(MixedLight, AHeadsetStaysInLh2) {
  MixedLightView v = InLh2(1000.0);
  v.stayLh2 = true;
  EXPECT_EQ(Mode(v), -1);
  v.dark = 5.0;
  v.sinceSent = 5.0;
  EXPECT_EQ(Mode(v), 1);  // re-sent, the same mode
  v.mode = 0;
  v.dark = 0;
  EXPECT_EQ(Mode(v), 1);
  v.mode = 1;
  v.tracking = false;
  v.lost = 100;
  EXPECT_EQ(Mode(v), -1);
}

TEST(MixedLight, EachKindOfDeviceHasItsOwnSwitch) {
  using lh::DeviceRole;
  using lh::runtime::MixesLight;
  lh::MixedLightOptions options;  // the defaults: controllers and trackers, never the headset
  EXPECT_FALSE(options.enabled);
  EXPECT_TRUE(MixesLight(DeviceRole::Controller, options));
  EXPECT_FALSE(MixesLight(DeviceRole::Hmd, options));
  EXPECT_TRUE(MixesLight(DeviceRole::Tracker, options));
  EXPECT_FALSE(MixesLight(DeviceRole::BaseStation, options));
  options.controllers = false;
  EXPECT_FALSE(MixesLight(DeviceRole::Controller, options));
  EXPECT_TRUE(MixesLight(DeviceRole::Tracker, options));
  options.trackers = false;
  EXPECT_FALSE(MixesLight(DeviceRole::Tracker, options));
}

namespace {
// A rotor slowing steadily, as one warming up: 0.005 ppm more each second.
constexpr double kPeriod = 0.02, kRamp = 0.005e-6;
double Warming(double t) { return kPeriod * (1 + kRamp * t); }
}  // namespace

TEST(RotorScout, FollowsTheRotorsSpeed) {
  auto s = lh::runtime::StartRotorScout(0, kPeriod);
  double drift = 0;
  for (double t = 0.25; t <= 60 + 1e-9; t += 0.25) {
    const double p = Warming(t);
    drift = lh::runtime::StepRotorScout(s, t, &p);
  }
  EXPECT_NEAR(drift, kRamp * 60 * 60 / 2, 0.05e-6);  // 9 us: the integral of the change
  EXPECT_FALSE(s.lost);
}

TEST(RotorScout, KeepsTheRateThroughAHoleThenLetsGo) {
  auto s = lh::runtime::StartRotorScout(0, kPeriod);
  double drift = 0;
  for (double t = 0.25; t <= 40 + 1e-9; t += 0.25) {
    const double p = Warming(t);
    drift = lh::runtime::StepRotorScout(s, t, &p);
  }
  const double at40 = drift, rate = s.rate;  // 0.2 ppm
  for (double t = 40.25; t <= 60 + 1e-9; t += 0.25) drift = lh::runtime::StepRotorScout(s, t, nullptr);
  EXPECT_FALSE(s.lost);
  EXPECT_NEAR(drift, at40 + rate * 20, 0.01e-6);  // the last rate kept through 20 s without the scout
  for (double t = 60.25; t <= 65 + 1e-9; t += 0.25) drift = lh::runtime::StepRotorScout(s, t, nullptr);
  EXPECT_TRUE(s.lost);
  const double frozen = drift;
  for (double t = 65.25; t <= 90 + 1e-9; t += 0.25) {
    const double p = Warming(t);
    drift = lh::runtime::StepRotorScout(s, t, &p);
  }
  EXPECT_TRUE(s.lost);  // and then the line is on its own, even when the scout comes back
  EXPECT_DOUBLE_EQ(drift, frozen);
}

TEST(RotorScout, ABadPeriodCountsAsNone) {
  auto s = lh::runtime::StartRotorScout(0, kPeriod);
  double drift = 0;
  for (double t = 0.25; t <= 40 + 1e-9; t += 0.25) {
    const double p = Warming(t);
    drift = lh::runtime::StepRotorScout(s, t, &p);
  }
  const double at40 = drift, rate = s.rate;
  const double wild = kPeriod * (1 + 5e-6);
  for (double t = 40.25; t <= 45 + 1e-9; t += 0.25) drift = lh::runtime::StepRotorScout(s, t, &wild);
  EXPECT_NEAR(drift, at40 + rate * 5, 0.01e-6);  // the rate kept, not 5 ppm
}

TEST(RotorScout, AJumpInTheScoutsPeriodIsFollowedOnlySlowly) {
  auto s = lh::runtime::StartRotorScout(0, kPeriod);
  const double jumped = kPeriod * (1 + 2e-6);  // within the 3 ppm limit, but no rotor changes so fast
  double drift = 0;
  for (double t = 0.25; t <= 5 + 1e-9; t += 0.25) drift = lh::runtime::StepRotorScout(s, t, &jumped);
  EXPECT_NEAR(s.rate, 0.25e-6, 0.01e-6);  // 0.05 ppm a second
  EXPECT_LT(drift, 1e-6);  // against 10 us if taken at once
}

TEST(MixedLight, SwitchedOffInLh1GoesToLh2AtOnce) {
  MixedLightView v = InLh1(10.0);
  v.calm = false;  // even mid-swing: it was asked for
  v.stayLh2 = true;
  const MixedLightStep step = ChooseMixedLight(v, kVisits);
  EXPECT_EQ(step.mode, 1);
  EXPECT_EQ(std::string(step.why), "stays in LH2");
}

// The original Vive headset, Vive wands, the 2017 Vive Tracker: no LH2 mode, so told to go there
// they keep sending LH1 pulses, and never a beam word.
TEST(MixedLight, TellsA1Point0OnlyDeviceApart) {
  using lh::runtime::LooksLh1Only;
  MixedLightView v;
  v.mode = 1;
  v.inMode = 8.0;
  v.lh1Pulses = true;
  EXPECT_TRUE(LooksLh1Only(v));
  v.inMode = 7.9;
  EXPECT_FALSE(LooksLh1Only(v)) << "not before the command was sent again";
  v.inMode = 30.0;
  v.everLh2 = true;
  EXPECT_FALSE(LooksLh1Only(v)) << "it has sent 2.0 light: the command was lost, the device can";
  v.everLh2 = false;
  v.lh1Pulses = false;
  EXPECT_FALSE(LooksLh1Only(v)) << "dark in LH2 mode: no 2.0 station in view, or one waking";
  v.lh1Pulses = true;
  v.mode = 0;
  EXPECT_FALSE(LooksLh1Only(v)) << "in LH1 mode, LH1 pulses are what every device sends";
  v.mode = 1;
  v.stayLh2 = true;  // a headset
  EXPECT_TRUE(LooksLh1Only(v));
}

// In use for the room's 2.0 stations, none of which has been seen: they may be off, so every device, the
// headset too, takes a short look in LH2 mode and goes back to the 1.0 light, rather than waiting blind.
TEST(MixedLight, OnlyLooksForUnseen2Point0Stations) {
  MixedLightView v = InLh2(0);
  v.readyRotors = 0;
  v.tracking = false;
  v.lh2Unseen = true;
  v.inMode = v.dark = v.sinceSent = 2.9;
  EXPECT_EQ(Mode(v), -1);
  v.inMode = v.dark = v.sinceSent = 3.0;
  EXPECT_EQ(Mode(v), 0) << "nothing seen in 3 s: back to the 1.0 light";
  v.stayLh2 = true;  // a headset
  EXPECT_EQ(Mode(v), 0);
  v.lh2Unseen = false;
  EXPECT_EQ(Mode(v), -1) << "once one is seen, the headset stays in LH2 mode";
  v.stayLh2 = false;
  EXPECT_EQ(Mode(v), -1) << "and the others wait for stations still waking";

  MixedLightView lh1 = InLh1(14.9);
  lh1.readyRotors = 0;
  lh1.lh2Unseen = true;
  lh1.stayLh2 = true;
  EXPECT_EQ(Mode(lh1), -1) << "a headset on the 1.0 light while none is seen";
  lh1.inMode = 15.0;
  EXPECT_EQ(Mode(lh1), 1) << "until the next look";
}

// A 2.0 station seen in a room learned from 1.0 stations alone, where none is placed yet: LH2 mode gives no
// pose, so a device only learns the rotors there and stays on the 1.0 light, where it can place them.
// Live, without this every device cycled 8 s blind in LH2 mode and 3 s in LH1 mode (too short to
// pick up the 1.0 light), the headset stayed in LH2 mode, and tracking never came back.
TEST(MixedLight, With2Point0StationsNotInTheRoomStaysOnThe1Point0Light) {
  MixedLightView v = InLh2(2.5);
  v.tracking = false;
  v.lost = 999;
  EXPECT_EQ(Mode(v), -1) << "placed, it would wait for a pose";
  v.lh2Unplaced = true;
  EXPECT_EQ(Mode(v), 0) << "rotors learned: back to the 1.0 light";
  v.stayLh2 = true;  // a headset
  EXPECT_EQ(Mode(v), 0);
  v.inMode = v.sinceSent = 1.0;
  EXPECT_EQ(Mode(v), -1) << "a visit still lasts long enough to learn the rotors";

  MixedLightView lh1 = InLh1(10.0);
  lh1.tracking = false;
  lh1.lost = 10.0;
  EXPECT_EQ(Mode(lh1), 1) << "placed, LH2 mode helps a lost device";
  lh1.lh2Unplaced = true;
  EXPECT_EQ(Mode(lh1), -1) << "unplaced it can't: stay and pick up the 1.0 light";
  lh1.stayLh2 = true;
  EXPECT_EQ(Mode(lh1), -1) << "the headset too";
  lh1.inMode = kVisits;
  EXPECT_EQ(Mode(lh1), 1) << "visits go on, to keep the rotors";
}

// In use only where there's a 2.0 station to sort, and not for a room whose 2.0 stations stay dark
// (switched off, or not woken): there devices would go on looking for their light.
TEST(MixedLight, InUseOnlyWithA2Point0StationToSort) {
  using lh::runtime::ChooseMixedRoom;
  using lh::runtime::MixedRoomStep;
  using lh::runtime::MixedRoomView;
  MixedRoomView v;
  v.enabled = true;
  EXPECT_EQ(ChooseMixedRoom(v), MixedRoomStep::NotInUse) << "no 2.0 station known: a 1.0-only room";
  v.lh2InRoom = true;
  EXPECT_EQ(ChooseMixedRoom(v), MixedRoomStep::InUse);
  v.enabled = false;
  EXPECT_EQ(ChooseMixedRoom(v), MixedRoomStep::NotInUse) << "switched off";
  v.enabled = true;

  v.inUse = 89;
  EXPECT_EQ(ChooseMixedRoom(v), MixedRoomStep::InUse) << "the stations may still be waking";
  v.inUse = 90;
  EXPECT_EQ(ChooseMixedRoom(v), MixedRoomStep::RoomDark);
  v.sinceLh2Light = 10;
  EXPECT_EQ(ChooseMixedRoom(v), MixedRoomStep::InUse) << "a device had 2.0 light lately";
  v.inUse = 30;
  v.sinceLh2Light = 1e300;
  EXPECT_EQ(ChooseMixedRoom(v), MixedRoomStep::InUse) << "counted from when it came into use";

  v.inUse = 600;
  v.lh2Seen = true;
  EXPECT_EQ(ChooseMixedRoom(v), MixedRoomStep::InUse) << "a 2.0 station seen this session: it's lit";
  v.lh2InRoom = false;
  EXPECT_EQ(ChooseMixedRoom(v), MixedRoomStep::InUse);
}

namespace {

using lh::runtime::ChooseLightMode;
using lh::runtime::LightModeView;

// A device in a room with both kinds of station, both seen, 1.0 + 2.0 not in use.
LightModeView InMixedRoom(int mode, double inMode, double dark) {
  LightModeView v;
  v.mode = mode;
  v.inMode = inMode;
  v.dark = dark;
  v.knownLh1 = v.knownLh2 = v.seenLh1 = v.seenLh2 = true;
  return v;
}

}  // namespace

TEST(LightMode, StartsAsStock) {
  LightModeView v;
  v.knownLh1 = v.knownLh2 = true;
  EXPECT_EQ(ChooseLightMode(v).mode, 0) << "a room with both kinds: LH1, the default";
  v.knownLh1 = false;
  EXPECT_EQ(ChooseLightMode(v).mode, 1) << "only 2.0 stations";
}

TEST(LightMode, TriesTheOtherModeWhenDarkWithout2Point0Light) {
  LightModeView v = InMixedRoom(1, 3.7, 2.5);
  EXPECT_EQ(ChooseLightMode(v).mode, 0) << "never sent 2.0 light: stock's choice";
  v.everLh2 = true;
  v.sinceLh2Light = 12;
  EXPECT_EQ(ChooseLightMode(v).mode, 0) << "no 2.0 light from anyone for 12 s: they're off";
}

TEST(LightMode, StaysInLh2While2Point0StationsAreLit) {
  // Live, 2026-11-11: dark 2.5 s just after the switch went off, while the tracker beside it had 2.0 light.
  LightModeView v = InMixedRoom(1, 3.7, 2.5);
  v.everLh2 = true;
  v.sinceLh2Light = 0.5;
  EXPECT_EQ(ChooseLightMode(v).mode, -1);
  v.inMode = v.dark = 5.0;
  const auto step = ChooseLightMode(v);
  EXPECT_EQ(step.mode, 1);
  EXPECT_EQ(std::string(step.why), "re-send");
}

TEST(LightMode, LeavesLh1While2Point0StationsAreLit) {
  LightModeView v = InMixedRoom(0, 3.0, 0.0);  // tracking on the 1.0 light, which holds it there under stock
  v.everLh2 = true;
  v.sinceLh2Light = 0.1;
  const auto step = ChooseLightMode(v);
  EXPECT_EQ(step.mode, 1);
  EXPECT_EQ(std::string(step.why), "2.0 stations lit");
  v.inMode = 1.0;
  EXPECT_EQ(ChooseLightMode(v).mode, -1) << "not straight after a change";
  v.inMode = 3.0;
  v.everLh2 = false;
  EXPECT_EQ(ChooseLightMode(v).mode, -1) << "a device that never sent 2.0 light (a 1.0-only one) stays";
}

TEST(LightMode, OneKindSeenKeepsThatMode) {
  LightModeView v = InMixedRoom(1, 3.0, 3.0);
  v.seenLh2 = false;
  EXPECT_EQ(ChooseLightMode(v).mode, 0) << "only 1.0 stations seen";
  v.mode = 0;
  EXPECT_EQ(ChooseLightMode(v).mode, -1);
  v.inMode = 5.0;
  EXPECT_EQ(ChooseLightMode(v).mode, 0) << "re-sent now and then";
}

TEST(LightMode, KnownRoomIsOnlyAStartingGuess) {
  // SteamVR's database lists 2.0 bases only (an earlier setup); the room really has 1.0 bases.
  LightModeView v;
  v.knownLh2 = true;
  EXPECT_EQ(ChooseLightMode(v).mode, 1) << "starts in LH2";
  v.mode = 1; v.dark = v.inMode = 5;
  EXPECT_NE(ChooseLightMode(v).mode, 0) << "dark a few seconds: still the known room's mode (or it re-sent)";
  v.dark = v.inMode = 15;
  EXPECT_EQ(ChooseLightMode(v).mode, 0) << "dark in LH2 that long: try LH1";
  v.mode = 0; v.dark = v.inMode = 5;
  EXPECT_NE(ChooseLightMode(v).mode, 1) << "LH1 gets its time to find a station";
  v.dark = v.inMode = 10;
  EXPECT_EQ(ChooseLightMode(v).mode, 1) << "nothing in LH1 either: back to the known mode";
  v.dark = 0; v.inMode = 30;
  EXPECT_NE(ChooseLightMode(v).mode, 1) << "light in LH1: it stays";
}

TEST(Lh1Keyframes, AlwaysWithout2Point0Light) {
  lh::runtime::Lh1KeyframeView v;  // a 1.0-only room: no beam word from anyone, ever
  EXPECT_TRUE(lh::runtime::Lh1KeyframesUsable(v));
  v.sinceLh2Light = 30;
  v.lh2Stations = 3;
  EXPECT_TRUE(lh::runtime::Lh1KeyframesUsable(v)) << "2.0 stations seen, but dark for 30 s: turned off";
}

TEST(Lh1Keyframes, NotFromUnsortedLightWhile2Point0IsLit) {
  lh::runtime::Lh1KeyframeView v;
  v.sinceLh2Light = 0.2;
  v.lh2Stations = 3;
  EXPECT_FALSE(lh::runtime::Lh1KeyframesUsable(v)) << "switched off, or a 1.0-only device: not sorting";
  v.sorting = true;
  EXPECT_FALSE(lh::runtime::Lh1KeyframesUsable(v)) << "sorting, no rotor learned yet (a replay starting mid-session)";
  v.knownRotors = 2;
  EXPECT_FALSE(lh::runtime::Lh1KeyframesUsable(v)) << "a station seen whose rotor this device hasn't learned";
  v.knownRotors = 3;
  EXPECT_TRUE(lh::runtime::Lh1KeyframesUsable(v));
}

TEST(Lh1Keyframes, SortingNeedsARotorBeforeAnyStationIsSeen) {
  lh::runtime::Lh1KeyframeView v;
  v.sinceLh2Light = 0.2;  // 2.0 light, its station not identified yet
  v.sorting = true;
  EXPECT_FALSE(lh::runtime::Lh1KeyframesUsable(v));
  v.knownRotors = 1;
  EXPECT_TRUE(lh::runtime::Lh1KeyframesUsable(v));
}

namespace {

using lh::runtime::ForeignSweeps;
using lh::runtime::SweepSample;

// n samples 20 us apart from `from`, off the pose by `offset` plus `step` per sample.
void Sweep(std::vector<SweepSample>* v, double from, int n, double offset, double step) {
  for (int i = 0; i < n; ++i) v->push_back({from + i * 20e-6, offset + i * step, true});
}

int Dropped(const std::vector<bool>& d) { return int(std::count(d.begin(), d.end(), true)); }

}  // namespace

TEST(ForeignSweeps, ASweepOffAlikeIsTheStationsOwn) {
  std::vector<SweepSample> v;
  Sweep(&v, 1.0, 6, 0.05, 0.0002);  // 50 mrad off, all alike: the pose's error or the timing's
  EXPECT_EQ(Dropped(ForeignSweeps(v)), 0);
}

TEST(ForeignSweeps, ASweepAtAnotherTiltIsDropped) {
  std::vector<SweepSample> v;
  Sweep(&v, 1.0, 6, 0.0, 0.02);  // each sensor 20 mrad further off than the last
  EXPECT_EQ(Dropped(ForeignSweeps(v)), 6);
}

TEST(ForeignSweeps, TooFewToJudgeAreKept) {
  std::vector<SweepSample> v;
  Sweep(&v, 1.0, 2, 0.0, 0.05);
  EXPECT_EQ(Dropped(ForeignSweeps(v)), 0);
  v.push_back({1.0 + 40e-6, 0.3, false});  // a third, but the pose has nothing for it
  EXPECT_EQ(Dropped(ForeignSweeps(v)), 0);
}

TEST(ForeignSweeps, TwoSweepsInOneFrameAreJudgedApart) {
  std::vector<SweepSample> v;
  Sweep(&v, 1.0, 5, 0.001, 0.0001);   // the station's own
  Sweep(&v, 1.003, 5, 0.1, 0.03);     // 3 ms on: a 2.0 sweep
  v.push_back({1.003 + 50e-6, 0.0, false});  // in that sweep, unjudged: it goes with it
  std::sort(v.begin(), v.end(), [](const SweepSample& a, const SweepSample& b) { return a.time < b.time; });
  const auto d = ForeignSweeps(v);
  EXPECT_EQ(Dropped(d), 6);
  for (size_t i = 0; i < v.size(); ++i) EXPECT_EQ(d[i], v[i].time > 1.002) << i;
}
