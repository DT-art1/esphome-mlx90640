# MLX90640 for ESPHome

Custom ESPHome component for the MLX90640 thermal camera.
This implementation depends on esphome/esphome#7639

## Installation

In your ESPHome `yaml`:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/DT-art1/esphome-mlx90640
      ref: main
    components: [mlx90640]
