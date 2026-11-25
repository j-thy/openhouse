#pragma once
#include <openvr_driver.h>
#include <array>
#include <string>
#include <vector>
#include "lh/session.hpp"

namespace lh::openvr {
// The stock Index hand: thumb poses chosen by what the thumb touches, fingers from
// the capacitive sensors, a real fist without the controller. Estimated, not optical
// joint tracking. Poses are loaded from the user's installed Valve assets; no
// proprietary assets are redistributed.
class HandSkeleton {
 public:
  using Bones = std::array<vr::VRBoneTransform_t, 31>;
  // vr_glove_right_poses.glb (required). SteamVR's resources/anims hand_right_*.glb
  // are found from it (or LH_HAND_ANIMS) for the WithoutController fist.
  bool Load(const std::string& glovePoses, bool left, std::string* error);
  // Advances the smoothing and thumb fades by `seconds` with the latest input.
  void Update(const ControllerInput& input, double seconds);
  // A Vive wand: no finger sensors, so the index bends as far as the trigger is pulled
  // (on an Index any pull closes it all the way: the finger is already round the trigger).
  void SetIndexFromTrigger(bool on) { indexFromTrigger_ = on; }
  // The controller's orientation as apps see it (w, x, y, z; y up), for the open hand's rest pose:
  // stock lets the open hand settle by how the controller is held. Kept until the next valid one.
  void SetOrientation(const std::array<double, 4>& wxyz) { orientation_ = wxyz; haveOrientation_ = true; }
  // The current hand, after Update.
  Bones Current(bool withController) const;
  // Stateless: the hand once settled on this input.
  Bones Evaluate(const ControllerInput& input, bool withController) const;
  const Bones& GripLimit() const { return grip_; }

 private:
  struct State {
    float curl[4] = {}, curlV[4] = {};  // index..pinky, WithController
    float fist[4] = {}, fistV[4] = {};  // biased, WithoutController
    float indexClose = 0, indexCloseV = 0;
    float trigger = 0, squeeze = 0;
    float thumbClose = 0, thumbCloseV = 0;
    float thumbX = 0, thumbY = 0, thumbXV = 0, thumbYV = 0;
    float thumbLiftedFor = 0;  // s with the thumb on nothing
    float thumbRaise = 0, thumbRaiseV = 0;  // 0 on the controller .. 1 the open hand's thumb
    float restUp = 0, restUpV = 0, restSide = 0, restSideV = 0;  // the open hand's rest pose (Update)
    int thumbState = 8;
    float fade = 1;
    float padX = 0, padY = 0, stickX = 0, stickY = 0;
    Bones thumbFrom{};
  };
  static int ThumbState(const ControllerInput& input);
  Bones ThumbPose(const State& s, bool blended) const;

  std::vector<Bones> glove_, hand_, close_, squeeze_;
  Bones grip_{};
  bool left_ = false;
  State state_;
  bool indexFromTrigger_ = false;
  std::array<double, 4> orientation_{1, 0, 0, 0};
  bool haveOrientation_ = false;
  Bones Rest(const State& s) const;
};
}
