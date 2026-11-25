// Not stock: which light mode a device should be in, in a room with both kinds of base station
// (ai/driver/mixed_stations.md).
//
// In LH1 mode a device reports hits from both kinds, and PulseSorter tells them apart, but only
// for 2.0 rotors it has heard real beam words from. Those come only in LH2 mode, where a 1.0
// station gives nothing. So a device starts in LH2 mode, moves to LH1 mode once it knows the
// rotors it can see, and comes back now and then. Going to LH2 mode costs about half a second
// without light, so routine visits wait for a calm moment, and the same half second is no loss
// to a device that has lost tracking, which LH2 mode handles better on hard swings.
//
// A device can stay in LH2 mode throughout, by kind (MixedLightOptions, from the SteamVR
// settings page). The headset always does.
//
// A first-generation device (the original Vive headset, Vive wands, the 2017 Vive Tracker) has no
// LH2 mode and sees only 1.0 stations, so none of this is for it. It doesn't say so, but it gives
// itself away: told to go to LH2 mode it keeps sending LH1 pulses and never a beam word, where a
// device that can stops within about 30 ms. Then it's left out (LooksLh1Only).
//
// Live only: a replay's modes are in its recording.
#pragma once

#include <vector>

#include "lh/session.hpp"

namespace lh::runtime {

// s in LH1 mode between visits to LH2 mode. A rotor's line, learned on a visit, runs off by about
// 2.5 us (typical) to 9 us (worst tenth) over 45-60 s, against windows of 30 us and up. The second
// while the scout (below) covers every rotor the device is using.
constexpr double kVisitEvery = 45, kScoutedVisitEvery = 90;

// Light in LH1 mode, weighted by how much it scatters (LighthouseDirect::SetMixedLightScatter). From the
// frames the filter took in one session, per hit: LH2 words 0.06-0.07 mrad off the pose typically,
// sorted 2.0 hits 0.11-0.14, the 1.0 station's 0.16-0.21. Weighted as LH2 words, that was 2-2.5 times
// LH2 mode's jitter; these brought it near LH2 mode's, with no lag.
constexpr double kSortedHitScatter = 1e-6;    // s, of a sorted 2.0 hit's timing
constexpr double kLh1HitScatter = 0.5e-3;     // rad, of a 1.0 station's angle
// A 2.0 rotor's sorted hits are off all alike, wandering over a few tenths of a second
// (tracking/rotor_wobble.hpp): the rotor's real beam words sit 0.25-0.3 us rms (0.09 mrad) off its line,
// and on a tracker lying still a frame's hits were off together by 0.09-0.17 mrad. Followed per rotor and
// taken out, as far as the pose is known better: this far it may be off, and this long it lasts.
constexpr double kRotorWobble = 0.3e-3;       // rad
constexpr double kRotorWobbleMemory = 0.2;    // s

// MixedLightOptions::preferLh2 (a test): LH2 mode while the 2.0 light is enough, LH1 mode only while it
// isn't. In LH1 mode a sorted hit's angle hangs on its rotor keeping to a line, which it doesn't
// (tracking/rotor_wobble.hpp), and trackers lying still swayed 0.22-0.41 mm rms against 0.07-0.12 mm in LH2
// mode, 0.12-0.36 mm with the wobble followed (live 2026-11-11: 0.17 and 0.08 mm in LH2 mode against 0.50
// and 0.12 mm in LH1 mode). Enough (Lh2LightEnough): kPreferHits sensor hits in the last 0.5 s from the 2.0
// stations together, counting each seen in kPreferFrames frames (of about 50) with kPreferSensors sensors a
// frame. It was one station lighting 4 sensors a frame in 20 frames, a pose by itself; but LH2 mode tracks
// on the stations together, and Knuckles lit 3-4 sensors a frame by each of two fell short of that for up to
// 17 % of their time there: live they left after 2.6-11 s while tracking 93-97 % of it, and waited 20 s to
// come back. Going to LH1 mode costs 10-20 ms, so it's left as soon as the light has been short for
// kPreferShort; coming back costs 0.5-1.5 s without new light (live), so that waits for kPreferCovered of
// enough light without a break, a calm moment, and kPreferMinLh1 in LH1 mode: at worst one such gap every
// 20 s, where the light comes and goes.
constexpr int kPreferFrames = 10;
constexpr double kPreferSensors = 3, kPreferHits = 80;
constexpr double kPreferShort = 0.25;    // s
constexpr double kPreferCovered = 10.0;  // s
constexpr double kPreferMinLh1 = 20.0;   // s

// Whether a device of this kind reads both kinds of base station.
bool MixesLight(DeviceRole role, const MixedLightOptions& options);

// One 2.0 station's light on a device in the last 0.5 s: frames, and the mean sensors lit per frame.
struct Lh2Light {
  int frames = 0;
  double sensors = 0;
};
// MixedLightOptions::preferLh2: whether the 2.0 stations light the device well enough to stay in LH2 mode.
bool Lh2LightEnough(const std::vector<Lh2Light>& stations);

struct MixedLightView {
  int mode = -1;           // 0 = LH1, 1 = LH2, -1 = not sent yet
  double inMode = 0;       // s since this mode was chosen
  double dark = 0;         // s without this mode's kind of light
  double sinceSent = 0;    // s since the mode was last sent
  bool stayLh2 = false;    // never leave LH2 mode
  int readyRotors = 0;     // rotors the sorter can predict
  int learningRotors = 0;  // heard right now, short of words
  int unnamedRotors = 0;   // ready and heard right now, with no station identified for them
  bool tracking = false;   // has a pose from light right now
  double lost = 0;         // s since it last did
  bool calm = false;       // slow enough to go half a second without light
  bool lh1Pulses = false;  // LH1 pulses within the last half second
  bool everLh2 = false;    // has sent a beam word this session
  // In use for the room's 2.0 stations, none seen yet this session: they may be off, so every device
  // (the headset too) only looks for their light now and then, tracking on the 1.0 light in between.
  bool lh2Unseen = false;
  // A 2.0 station seen, but none in the room in use yet (a room learned from 1.0 stations alone): LH2 mode
  // gives no pose until a device on the 1.0 light places one from its sorted hits. So every device, the
  // headset too, visits LH2 mode only to learn the rotors, and stays on the 1.0 light even when lost there.
  bool lh2Unplaced = false;
  // MixedLightOptions::preferLh2. lh2Covered: s a 2.0 station has lit the device well without a break (0:
  // none does now); lh2Short: s since one last did (0: one does now).
  bool preferLh2 = false;
  double lh2Covered = 0, lh2Short = 0;
};

struct MixedLightStep {
  int mode = -1;  // the mode to send; -1: send nothing
  const char* why = nullptr;
};

// visitEvery: s in LH1 mode between visits to LH2 mode; 0: only when lost or dark.
MixedLightStep ChooseMixedLight(const MixedLightView& view, double visitEvery);

// A device that can't do LH2 mode: still sending LH1 pulses 8 s after being told to go to LH2 mode,
// past a re-send of the command (5 s), and never a beam word.
bool LooksLh1Only(const MixedLightView& view);

// Whether 1.0 + 2.0 is in use (Session::UpdateMixedRoom): only where there's a 2.0 station to sort.
struct MixedRoomView {
  bool enabled = false;          // the switch
  bool lh2Seen = false;          // a 2.0 station seen this session
  bool lh2InRoom = false;        // the room in use has one, and it wasn't found dark
  double inUse = -1;             // s in use so far; -1: not in use
  double sinceLh2Light = 1e300;  // s since any device last had 2.0 light
};
enum class MixedRoomStep { NotInUse, InUse, RoomDark };
// s in use for the room's 2.0 stations alone with no 2.0 light from any device: they're off or asleep, and
// devices stop looking for their light (MixedLightView::lh2Unseen). Stations SteamVR woke showed their first
// light 35-65 s after it started.
constexpr double kLh2RoomDark = 90;
// RoomDark: not in use for that room from now on (until the switch is turned on again).
MixedRoomStep ChooseMixedRoom(const MixedRoomView& view);

// The light mode while 1.0 + 2.0 isn't in use (switched off, or a room without a 2.0 station). As stock:
// from the room's stations, and in a room with both kinds, a device without usable light (a 1.0 sync flash
// or a beam word) for 2.5 s tries the other mode. Not stock: a device that has sent 2.0 light stays in LH2
// mode while any device sees 2.0 light. In LH1 mode the 2.0 stations' sweeps reach it unsorted beside the
// 1.0 stations' light and keep breaking its pose, and the 1.0 light then holds it there. In a recording, a
// tracker dark for 2.5 s just after the switch went off had a good pose for 38 % of the next two minutes in
// LH1 mode, against 95 % for the one beside it, still in LH2 mode. A device out of the 2.0 stations' view
// waits in the dark instead.
struct LightModeView {
  int mode = -1;                 // 0 = LH1, 1 = LH2, -1 = not sent yet
  double inMode = 0;             // s since this mode was chosen
  double dark = 0;               // s without usable light
  bool knownLh1 = false;         // the room has a 1.0 station (SteamVR's database)
  bool knownLh2 = false;         // ... a 2.0 station
  bool seenLh1 = false;          // a 1.0 station seen this session
  bool seenLh2 = false;          // ... a 2.0 station
  bool everLh2 = false;          // this device has sent a beam word this session
  double sinceLh2Light = 1e300;  // s since any device last had 2.0 light
};
// s since any device's last beam word that still counts as 2.0 stations lit. Past it (they've been turned
// off), stock's choice applies again.
constexpr double kLh2Lit = 10;
// s dark in the mode SteamVR's database suggests before the other is tried, and how long that one is tried
constexpr double kKnownProbe = 15;
constexpr double kKnownOther = 10;
MixedLightStep ChooseLightMode(const LightModeView& view);

// Whether a device's LH1 frames may go into keyframes for the room map (DeviceTracker::SetLh1Keyframes). An
// LH1 frame takes every pulse in its station's sweep window, so a 2.0 station's sweep that the device doesn't
// sort lands in it as wrong angles. The filter checks each sample against its pose and drops those; the room
// solver takes keyframes as they are. In a recording, unsorted stretches gave 40-55 % of the trackers' 1.0
// samples 10 mrad to 0.8 rad off, and the solver, fitting them, loosened and relearned the 1.0 stations.
// So while 2.0 light is about, LH1 frames go into keyframes only from a device that sorts, knowing every 2.0
// station seen this session. Off: switched off, a 1.0-only device, or rotors not learned yet (a replay
// starting mid-session, a station not yet visited). With no 2.0 light (a 1.0-only room), always on.
struct Lh1KeyframeView {
  double sinceLh2Light = 1e300;  // s since any device last had 2.0 light
  int lh2Stations = 0;           // 2.0 stations seen this session
  bool sorting = false;          // the device sorts its LH1 pulses
  int knownRotors = 0;           // rotors it can predict, with a real beam word within kRotorFresh
};
// s since a rotor's last real beam word that it still counts as known: two visits' worth with the scout
// (kScoutedVisitEvery). Its line is learned once and kept, but runs off while no words come.
constexpr double kRotorFresh = 2 * kScoutedVisitEvery;
bool Lh1KeyframesUsable(const Lh1KeyframeView& view);

// A 1.0 frame takes every pulse in its sweep window, so a 2.0 sweep that wasn't sorted out lands in it as
// 1.0 samples (a rotor not learned yet, a 1.0-only device, the switch off). Against where the pose puts the
// frame's own station's sweep, one sweep's samples (hits less than kSweepGap apart) are off alike: by the
// pose's error, or the timing's. A 2.0 sweep crosses the device at another tilt and speed, so its sensors
// are off by different amounts. So a sweep of at least kSweepMinSamples samples spreading more than
// kSweepSpread about their median is dropped, before the filter or the room map sees it. In a recording
// (2026-11-11 13:02, two 1.0 and three 2.0 stations): of the samples over 20 mrad off, 70 % of a 3.0
// tracker's and 51 % of a 2018 tracker's went, of those within 5 mrad 0.7 % and 0.3 %. No timing needed,
// only a pose, and one that holds: turning fast, its error spreads a sweep's own samples too, and checked
// then a struggling tracker lost more light and tracking (3 more losses in that recording); above
// kSweepMaxTurn the check waits.
constexpr double kSweepGap = 150e-6, kSweepSpread = 4e-3;  // s, rad
constexpr int kSweepMinSamples = 3;
constexpr double kSweepMaxTurn = 4.0;  // rad/s
struct SweepSample {
  double time = 0;      // the pulse's start
  double residual = 0;  // rad, its angle less the pose's for the frame's station and axis
  bool judged = false;  // the pose has one
};
// In time order; true for each sample to drop.
std::vector<bool> ForeignSweeps(const std::vector<SweepSample>& samples);

// The scout. Between a device's visits to LH2 mode its line for a 2.0 rotor runs off, about half of it
// because the rotor's speed changes (stations warm for minutes after waking: 1-2.5 ppm over 10-15 min,
// seen by every device alike). A device in LH2 mode all the while, the headset, measures that speed, so
// the line can follow it: the time it has run off is the integral of the rotor's relative period change
// since the line was learned. Only the change is passed on, so the two devices' clocks needn't line up
// closely. In replays, 45-60 s after a visit this took a line from 2.5 to 1.5 us off typically and 6.9
// to 4.1 us for the worst tenth, so a device whose every rotor in use has a scout visits half as often.
struct RotorScout {
  double reference = 0;  // the scout's period for the rotor when the line was learned
  double drift = 0;      // s the line has run off since
  double rate = 0;       // relative period change, as last heard
  double lastHeard = 0;  // when the scout last had the period
  double lastStep = 0;
  bool lost = false;     // the scout was quiet too long: the line is on its own until the next visit
};
RotorScout StartRotorScout(double now, double period);
// One step at `now`, with the scout's period if it has one now. Returns the drift for the line.
double StepRotorScout(RotorScout& scout, double now, const double* period);

}  // namespace lh::runtime
