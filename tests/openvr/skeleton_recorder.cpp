// Records what an app sees of the right hand from a running vrserver (stock driver or ours), for
// comparing the two: the controller's pose, SteamVR's skeletal summary (finger curl and splay, what
// VRChat's finger tracking reads) and all 31 bones in both motion ranges, as JSON lines.
// Usage: lh_openvr_skeleton_recorder ABSOLUTE_ACTION_MANIFEST OUT.jsonl [SECONDS]
// (the manifest is tests/openvr/probe_actions.json, next to its bindings file)
#include <openvr.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace {
void Bones(FILE* f, const char* name, const vr::VRBoneTransform_t* b) {
  std::fprintf(f, ",\"%s\":[", name);
  for (int i = 0; i < 31; ++i)
    std::fprintf(f, "%s[%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f]", i ? "," : "", b[i].position.v[0], b[i].position.v[1],
                 b[i].position.v[2], b[i].orientation.w, b[i].orientation.x, b[i].orientation.y, b[i].orientation.z);
  std::fprintf(f, "]");
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) { std::fprintf(stderr, "usage: %s ABSOLUTE_ACTION_MANIFEST OUT.jsonl [SECONDS]\n", argv[0]); return 2; }
  vr::EVRInitError init;
  auto* system = vr::VR_Init(&init, vr::VRApplication_Background);
  if (!system) { std::fprintf(stderr, "VR_Init: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(init)); return 1; }
  auto* input = vr::VRInput();
  vr::VRActionHandle_t hand = 0;
  vr::VRActionSetHandle_t set = 0;
  if (input->SetActionManifestPath(argv[1]) != vr::VRInputError_None) { std::fprintf(stderr, "bad manifest\n"); return 1; }
  input->GetActionHandle("/actions/probe/in/hand", &hand);
  input->GetActionSetHandle("/actions/probe", &set);
  FILE* out = std::fopen(argv[2], "w");
  if (!out) { std::fprintf(stderr, "cannot write %s\n", argv[2]); return 1; }
  const double seconds = argc > 3 ? std::atof(argv[3]) : 40;
  const auto start = std::chrono::steady_clock::now();
  int rows = 0;
  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
    const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    vr::VRActiveActionSet_t active{};
    active.ulActionSet = set;
    input->UpdateActionState(&active, sizeof(active), 1);
    vr::InputSkeletalActionData_t data{};
    if (input->GetSkeletalActionData(hand, &data, sizeof(data)) != vr::VRInputError_None || !data.bActive) continue;
    vr::VRBoneTransform_t with[31], without[31];
    if (input->GetSkeletalBoneData(hand, vr::VRSkeletalTransformSpace_Parent, vr::VRSkeletalMotionRange_WithController, with, 31) !=
            vr::VRInputError_None ||
        input->GetSkeletalBoneData(hand, vr::VRSkeletalTransformSpace_Parent, vr::VRSkeletalMotionRange_WithoutController, without,
                                   31) != vr::VRInputError_None)
      continue;
    vr::VRSkeletalSummaryData_t summary{};
    input->GetSkeletalSummaryData(hand, vr::VRSummaryType_FromDevice, &summary);
    vr::EVRSkeletalTrackingLevel level = vr::VRSkeletalTracking_Estimated;
    input->GetSkeletalTrackingLevel(hand, &level);
    const auto right = system->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_RightHand);
    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
    system->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, poses, vr::k_unMaxTrackedDeviceCount);
    const auto& m = right < vr::k_unMaxTrackedDeviceCount ? poses[right].mDeviceToAbsoluteTracking : vr::HmdMatrix34_t{};
    std::fprintf(out, "{\"t\":%.4f,\"level\":%d,\"pose\":[", t, int(level));
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 4; ++c) std::fprintf(out, "%s%.5f", r || c ? "," : "", m.m[r][c]);
    std::fprintf(out, "],\"curl\":[%.4f,%.4f,%.4f,%.4f,%.4f],\"splay\":[%.4f,%.4f,%.4f,%.4f]", summary.flFingerCurl[0],
                 summary.flFingerCurl[1], summary.flFingerCurl[2], summary.flFingerCurl[3], summary.flFingerCurl[4],
                 summary.flFingerSplay[0], summary.flFingerSplay[1], summary.flFingerSplay[2], summary.flFingerSplay[3]);
    Bones(out, "with", with);
    Bones(out, "without", without);
    std::fprintf(out, "}\n");
    if (++rows % 120 == 0) { std::printf("%d rows\n", rows); std::fflush(stdout); }
  }
  std::fclose(out);
  std::printf("%d rows written to %s\n", rows, argv[2]);
  vr::VR_Shutdown();
  return rows ? 0 : 1;
}
