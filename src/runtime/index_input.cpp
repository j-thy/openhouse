#include "index_input.hpp"

#include <tuple>

namespace lh::runtime {
namespace {
bool SameInput(const ControllerInput& a, const ControllerInput& b) {
  auto key = [](const ControllerInput& c) {
    return std::tie(c.systemClick, c.aClick, c.bClick, c.triggerClick, c.thumbstickClick, c.trackpadTouch,
                    c.trackpadClick, c.gripClick, c.menuClick,
                    c.systemTouch, c.aTouch, c.bTouch, c.triggerTouch, c.thumbstickTouch, c.trigger, c.thumbstickX,
                    c.thumbstickY, c.trackpadX, c.trackpadY, c.trackpadForce, c.gripForce, c.fingerIndex,
                    c.fingerMiddle, c.fingerRing, c.fingerPinky, c.mapped);
  };
  return key(a) == key(b);  // field-wise: the struct has padding bytes
}
}  // namespace

bool ControllerInputDecoder::Apply(const InputReport& r, double hostTime) {
  if (type_ == ControllerType::Mapped) {
    // The whole record, every report: its bits are named by the device's input profile
    // (the driver reads it). Anything else such a device sends is not ours to read.
    if (r.format != InputFormat::Raw || r.raw.empty()) return false;
    const bool changed = state_.mapped != r.raw;
    state_.mapped = r.raw;
    state_.hostTime = hostTime;
    return changed;
  }
  if (r.format == InputFormat::Raw) return false;
  const ControllerInput before = state_;
  auto& s = state_;
  if (type_ == ControllerType::ViveWand) {
    if (r.hasButtons) {
      s.triggerClick = r.buttons & 0x01;
      s.trackpadTouch = r.buttons & 0x02;
      s.trackpadClick = r.buttons & 0x04;
      s.systemClick = r.buttons & 0x08;
      s.gripClick = r.buttons & 0x10;
      s.menuClick = r.buttons & 0x20;
    }
    if (r.hasTrigger) s.trigger = r.trigger / 255.0;
    // A click is a touch too. Letting go, the touch bit can drop a report or two before
    // the click's: zeroed there, the pad would read as a click at its very centre, which
    // a binding split into regions takes as another button. The last position is kept
    // while it's pressed.
    const bool touched = s.trackpadTouch;
    if (s.trackpadClick) s.trackpadTouch = true;
    if (r.hasAxes && touched) { s.trackpadX = r.axisX / 32768.0; s.trackpadY = r.axisY / 32768.0; }
    if (!s.trackpadTouch) s.trackpadX = s.trackpadY = 0;
    s.hostTime = hostTime;
    return !SameInput(before, s);
  }
  if (r.hasButtons) {
    s.triggerClick = r.buttons & 0x01;
    s.trackpadTouch = r.buttons & 0x02;
    s.thumbstickClick = r.buttons & 0x04;
    s.systemClick = r.buttons & 0x08;
    s.aClick = r.buttons & 0x10;
    s.bClick = r.buttons & 0x20;
  }
  if (r.hasTrigger) s.trigger = r.trigger / 255.0;
  if (r.hasAxes) {
    const double x = r.axisX / 32768.0, y = r.axisY / 32768.0;
    if (s.trackpadTouch) {
      s.trackpadX = x; s.trackpadY = y;
    } else {
      s.thumbstickX = x; s.thumbstickY = y;
    }
  }
  if (!s.trackpadTouch) s.trackpadX = s.trackpadY = 0;
  if (r.hasExtended && r.extended[0] == 0xa1) {
    const uint8_t touch = r.extended[1];
    s.triggerTouch = touch & 0x01;
    s.systemTouch = touch & 0x08;
    s.aTouch = touch & 0x10;
    s.bTouch = touch & 0x20;
    s.thumbstickTouch = touch & 0x40;
    s.fingerMiddle = r.extended[2] / 255.0;
    s.fingerRing = r.extended[3] / 255.0;
    s.fingerPinky = r.extended[4] / 255.0;
    s.fingerIndex = r.extended[5] / 255.0;
    s.gripForce = r.extended[6] / 255.0;
    s.trackpadForce = r.extended[7] / 255.0;
  }
  s.hostTime = hostTime;
  return !SameInput(before, s);
}

}  // namespace lh::runtime
