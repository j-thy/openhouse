// The driver's live telemetry: what openhouse-overlay shows and the commands it sends.
//
// The driver listens on 127.0.0.1 (never other hosts) and streams newline-delimited
// JSON to every client. On connect a client gets {"type":"hello"} and the recent log
// backlog; then {"type":"state"} about 20 times a second and {"type":"log"} for every
// message, at every level (the client decides what to show). The protocol only adds
// fields; clients ignore what they don't know.
//
//   hello  {"type":"hello","protocol":1,"name":"openhouse","host":"...","pid":N}
//   log    {"type":"log","t":S,"level":"DEBUG|INFO|WARN|ERR","serial":"...","text":"..."}
//   state  {"type":"state","t":S,
//           "room":{"id":"...","settled":B,"epoch":N,"settleSigma":M,"resets":N,
//                   "learned":N,"seen":N},
//           "bases":[{"serial","channel","mode","lh2","seen","placed","origin","locked",
//                     "observations","sigma","learned","status","p":[x,y,z],"q":[w,x,y,z],"faults"}],
//           "devices":[{"serial","role","model","connected","enabled","battery","charging",
//                       "state","result","valid","p","q","v","primary","bases":[...],
//                       "coverage":[{"base","frames","sensors"}],"basesSeen":N}],
//           "receivers":[{"serial","connected"}],
//           "settings":{"positionSmoothingMs","rotationSmoothingMs","predictWithAcceleration",
//                       "hideLostDevices","mixedLight","mixedLight...","host","steamvr",
//                       "recording","recordingPath","reportBusy","report":{"count","path","error"},"steamRooms",
//                       "trackerNames":{"LHR-...":"..."}}}
//
// Clients may send commands, one JSON object per line (openhouse-overlay):
//   {"type":"command","id":N,"cmd":"set","key":"<SteamVR settings key>","value":V}
//   {"type":"command","id":N,"cmd":"record","on":B}       Record Session (off saves the report)
//   {"type":"command","id":N,"cmd":"resetRoom"}           {"cmd":"saveReport"}
//   {"type":"command","id":N,"cmd":"identify"|"powerOff","serial":"LHR-..."}
//   {"type":"command","id":N,"cmd":"enable","serial":"LHR-...","enabled":B}
//   {"type":"command","id":N,"cmd":"renameTracker","serial":"LHR-...","name":"..."}
//       Names are trimmed of spaces and limited to 63 UTF-8 bytes without ASCII control characters.
//       An empty name clears the alias for a known tracker.
//   {"type":"command","id":N,"cmd":"pair","receiver":"<dongle serial>","seconds":S}
// and get {"type":"reply","id":N,"ok":B,"error":"..."} back. Under SteamVR "set" writes the
// settings page's value (kept); elsewhere it lasts for the session.
//
// Positions are metres in the driver's room frame; sigma metres (1-sigma, total). A base's
// "status" is "not seen", "not placed", "learning" or "learned" ("learned": good enough
// to track and align against; the room settles once every seen base is). "resets" bumps
// when the room is reset (SteamVR settings, or its file deleted while running).
#pragma once

#include <cstddef>

namespace lh::telemetry {

constexpr int kProtocolVersion = 1;
constexpr int kDefaultPort = 36840;
constexpr std::size_t kTrackerNameMaxBytes = 63;
// LH_TELEMETRY_PORT, else kDefaultPort; 0 turns the server off.
int PortFromEnvironment();

}  // namespace lh::telemetry
