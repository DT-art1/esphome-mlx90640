#include "select_refresh_rate.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mlx90640 {

void SelectRefreshRate::control(const std::string &value) {
  this->publish_state(value);
  this->parent_->set_refresh_rate(value);
}

}  // namespace ld2420
}  // namespace esphome
