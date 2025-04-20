#pragma once

#include "mlx90640.h"
#include "esphome/components/number/number.h"

namespace esphome {
namespace mlx90640 {

class MLX90640;

class NumberPresenceDetectorThreshold : public number::Number, public Parented<mlx90640::MLX90640> {
 public:
  NumberPresenceDetectorThreshold() = default;

 protected:
  void control(float value) override;
};

}  // namespace ld2420
}  // namespace esphome
