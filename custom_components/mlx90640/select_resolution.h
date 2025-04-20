#pragma once

#include "mlx90640.h"
#include "esphome/components/select/select.h"

namespace esphome {
namespace mlx90640 {

class MLX90640;

class SelectResolution : public select::Select, public Parented<mlx90640::MLX90640> {
 public:
  SelectResolution() = default;

 protected:
  void control(const std::string &value) override;
};

}  // namespace ld2420
}  // namespace esphome
