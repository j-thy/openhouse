// The status panels: room, devices, base stations and a top-down map.
#pragma once
#include "telemetry_client.hpp"

namespace openhouse::viewer {

enum class Detail { Basic, Advanced };

void DrawConnection(const TelemetryClient& client);
void DrawRoom(const State& state);
void DrawDevices(const State& state, Detail detail);
void DrawBases(const State& state, Detail detail);
void DrawMap(const State& state);
void DrawReceivers(const State& state);

}  // namespace openhouse::viewer
