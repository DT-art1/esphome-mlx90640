#pragma once

#include "esphome.h"

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/select/select.h"
#include "esphome/components/number/number.h"
#include "esphome/core/component.h"

#include "mlx90640_i2c_driver.h"
#include "mlx90640_API.h"

namespace esphome {
namespace mlx90640 {

class MLX90640CameraImage : public camera::CameraImage {
 public:
  MLX90640CameraImage(uint8_t *data, size_t length, uint8_t requester);
  uint8_t *get_data_buffer() override;
  size_t get_data_length() override;
  bool was_requested_by(camera::CameraRequester requester) const override;
  virtual ~MLX90640CameraImage();
 protected:
  uint8_t *data_;
  size_t length_;
  uint8_t requesters_;
};

/* ---------------- CameraImageReader class ---------------- */
class MLX90640CameraImageReader : public camera::CameraImageReader {
 public:
  void set_image(std::shared_ptr<camera::CameraImage> image) override;
  size_t available() const override;
  uint8_t *peek_data_buffer() override;
  void consume_data(size_t consumed) override;
  void return_image() override;

 protected:
  std::shared_ptr<MLX90640CameraImage> image_;
  size_t offset_{0};
};

enum RefreshRate {
  IR_REFRESH_RATE_0_DOT_5_HZ = 0,
  IR_REFRESH_RATE_1_HZ = 1,
  IR_REFRESH_RATE_2_HZ = 2,
  IR_REFRESH_RATE_4_HZ = 3,
  IR_REFRESH_RATE_8_HZ = 4,
  IR_REFRESH_RATE_16_HZ = 5,
  IR_REFRESH_RATE_32_HZ = 6,
  IR_REFRESH_RATE_64_HZ = 7,
};

enum Resolution {
  RESOLUTION_16_BIT = 0,
  RESOLUTION_17_BIT = 1,
  RESOLUTION_18_BIT = 2,
  RESOLUTION_19_BIT = 3,
};

static const std::map<std::string, uint8_t> Refresh_mappings {
  {"IR_REFRESH_RATE_0_DOT_5_HZ", IR_REFRESH_RATE_0_DOT_5_HZ},
  {"IR_REFRESH_RATE_1_HZ", IR_REFRESH_RATE_1_HZ},
  {"IR_REFRESH_RATE_2_HZ", IR_REFRESH_RATE_2_HZ},
  {"IR_REFRESH_RATE_4_HZ", IR_REFRESH_RATE_4_HZ},
  {"IR_REFRESH_RATE_8_HZ", IR_REFRESH_RATE_8_HZ},
  {"IR_REFRESH_RATE_16_HZ", IR_REFRESH_RATE_16_HZ},
  {"IR_REFRESH_RATE_32_HZ", IR_REFRESH_RATE_32_HZ},
  {"IR_REFRESH_RATE_64_HZ", IR_REFRESH_RATE_64_HZ},
};
static const std::map<uint8_t, std::string> Refresh_mappings_to_string {
  {IR_REFRESH_RATE_0_DOT_5_HZ, "IR_REFRESH_RATE_0_DOT_5_HZ"},
  {IR_REFRESH_RATE_1_HZ, "IR_REFRESH_RATE_1_HZ"},
  {IR_REFRESH_RATE_2_HZ, "IR_REFRESH_RATE_2_HZ"},
  {IR_REFRESH_RATE_4_HZ, "IR_REFRESH_RATE_4_HZ"},
  {IR_REFRESH_RATE_8_HZ, "IR_REFRESH_RATE_8_HZ"},
  {IR_REFRESH_RATE_16_HZ, "IR_REFRESH_RATE_16_HZ"},
  {IR_REFRESH_RATE_32_HZ, "IR_REFRESH_RATE_32_HZ"},
  {IR_REFRESH_RATE_64_HZ, "IR_REFRESH_RATE_64_HZ"},
};

static const std::map<std::string, uint8_t> Resolution_mappings {
  {"RESOLUTION_16_BIT", RESOLUTION_16_BIT},
  {"RESOLUTION_17_BIT", RESOLUTION_17_BIT},
  {"RESOLUTION_18_BIT", RESOLUTION_18_BIT},
  {"RESOLUTION_19_BIT", RESOLUTION_19_BIT},
};

static const std::map<uint8_t, std::string> Resolution_mappings_to_string {
  {RESOLUTION_16_BIT, "RESOLUTION_16_BIT"},
  {RESOLUTION_17_BIT, "RESOLUTION_17_BIT"},
  {RESOLUTION_18_BIT, "RESOLUTION_18_BIT"},
  {RESOLUTION_19_BIT, "RESOLUTION_19_BIT"},
};

class MLX90640 : public camera::Camera, public i2c::I2CDevice {
  public:

    enum Mode {
      INTERLEAVED = 0,
      CHESS_PATTERN = 1,
    };

    enum State {
      STATE_CLEAR_NEW_DATA_READY_BIT = 0,
      STATE_WAIT_FOR_SUB_PAGE_0,
      STATE_CALCULATE_SUB_PAGE_0,
      STATE_WAIT_FOR_SUB_PAGE_1,
      STATE_CALCULATE_SUB_PAGE_1,
      STATE_PUBLISH_IMAGE,
      STATE_PUBLISH_SENSORS,
    };

/* ---------------- Camera implementation class ---------------- */
    void request_image(camera::CameraRequester requester) override;
    void start_stream(camera::CameraRequester requester) override {}
    void stop_stream(camera::CameraRequester requester) override {}
    void add_image_callback(std::function<void(std::shared_ptr<camera::CameraImage>)> &&callback) override;
    camera::CameraImageReader* create_image_reader() override;

    void print_memory_8_bytes(uint8_t* address);
    void print_memory_16_bytes(uint8_t* address);
    void print_memory_32_bytes(uint8_t* address);
    void print_ee();
    void print_device_id();

    uint16_t getStatusRegister();

    void setup() override;
    void dump_config() override;
    void loop() override;
    float get_setup_priority() const override;

    void set_configuration();
    void calculate_temperatur_();
    const char* i2c_to_string(esphome::i2c::ErrorCode error);
    const char* api_to_string(int error);
    i2c::ErrorCode i2c_read(uint16_t startAddress, uint16_t size, uint16_t *data, bool stop = true);
    i2c::ErrorCode i2c_read_max_message_size(uint16_t startAddress, uint16_t size, uint16_t *data, bool stop = true);
    i2c::ErrorCode i2c_write(uint16_t writeAddress, uint16_t data);
    int get_frame();
 protected:
    State state = STATE_CLEAR_NEW_DATA_READY_BIT;

  public:
    static MLX90640* device;
    uint16_t ee_data[832] = {0};
    uint16_t frame[834] = {0};
    float framebuf[768] = {0};
    paramsMLX90640 parameters;
//    uint16_t i2c_max_message_size {256};
    uint16_t i2c_max_message_size {16};
    int message = 0;
    std::shared_ptr<MLX90640CameraImage> current_image_;
    CallbackManager<void(std::shared_ptr<camera::CameraImage>)> new_image_callback_{};
    uint32_t max_update_interval_{1000};
    uint32_t idle_update_interval_{5000};
    uint32_t last_idle_request_{0};
    uint32_t last_image_update_{0};

    uint32_t publish_sensors_timeout_{10000};
    uint32_t last_sensors_update_{0};

    uint8_t single_requesters_{0};
    uint8_t stream_requesters_{0};
    bool has_requested_image_() const { return this->single_requesters_ || this->stream_requesters_; }

    void set_default_resolution(Resolution resolution) { resolution_ = resolution; }
    void set_default_refresh_rate(RefreshRate refresh_rate) { refresh_rate_ = refresh_rate; }
    void set_default_presence_detector_threshold(float presence_detector_threshold) {presence_detector_threshold_ = presence_detector_threshold; }
    void set_default_presence_detector_hysteresis(float presence_detector_hysteresis) {presence_detector_hysteresis_ = presence_detector_hysteresis; }

    void set_ambient_sensor(sensor::Sensor *sensor) { ambient_sensor_ = sensor; }
    void set_minimum_temperature_sensor(sensor::Sensor *sensor) { minimum_temperature_sensor_ = sensor; }
    void set_maximum_temperature_sensor(sensor::Sensor *sensor) { maximum_temperature_sensor_ = sensor; }

    void set_presence_detector_sensor(binary_sensor::BinarySensor *sensor) { presence_detector_sensor_ = sensor; }
    void set_presence_detector_threshold_sensor(number::Number *sensor) { presence_detector_threshold_sensor_ = sensor; }
    void set_presence_detector_hysteresis_sensor(number::Number *sensor) { presence_detector_hysteresis_sensor_ = sensor; }
    void set_presence_detector_threshold(float value) { presence_detector_threshold_ = value; }
    void set_presence_detector_hysteresis(float value) { presence_detector_hysteresis_ = value; }

    void set_refresh_rate_select(select::Select *select) { refresh_rate_select_ = select; }
    void set_refresh_rate(const std::string &refresh_rate);
    void set_resolution_select(select::Select *select) { resolution_select_ = select; }
    void set_resolution(const std::string &refresh_rate);

    Resolution resolution_{RESOLUTION_16_BIT};
    RefreshRate refresh_rate_{IR_REFRESH_RATE_2_HZ};

    sensor::Sensor *ambient_sensor_{nullptr};
    sensor::Sensor *minimum_temperature_sensor_{nullptr};
    sensor::Sensor *maximum_temperature_sensor_{nullptr};

    binary_sensor::BinarySensor *presence_detector_sensor_{nullptr};

    number::Number *presence_detector_threshold_sensor_{nullptr};
    number::Number *presence_detector_hysteresis_sensor_{nullptr};

    select::Select *refresh_rate_select_{nullptr};
    select::Select *resolution_select_{nullptr};

    float ambient_temperature_{0};
    float minimum_temperature_{0};
    float maximum_temperature_{0};

    float presence_detector_threshold_{3.5f};
    float presence_detector_hysteresis_{0.2f};
};

}  // namespace mlx90614
}  // namespace esphome
