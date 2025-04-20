#include "select_resolution.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mlx90640 {

void NumberPresenceDetectorHysteresis::control(float value) {
  this->publish_state(value);
  this->parent_->set_presence_detector_hysteresis(value);
}

}  // namespace ld2420
}  // namespace esphome
