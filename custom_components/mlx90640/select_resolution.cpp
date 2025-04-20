#include "select_resolution.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mlx90640 {

void SelectResolution::control(const std::string &value) {
  this->publish_state(value);
      ESP_LOGD("SelectResolution","New refresh rate %s -> %p", value.c_str() , this->parent_);

  this->parent_->set_resolution(value);
}

}  // namespace ld2420
}  // namespace esphome
