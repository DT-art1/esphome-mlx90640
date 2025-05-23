#include "mlx90640.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

esphome::mlx90640::MLX90640* mlx_device = nullptr;

inline uint16_t i2ctohs(uint16_t i2cshort) { return esphome::convert_big_endian(i2cshort); }
inline uint16_t htoi2cs(uint16_t hostshort) { return esphome::convert_big_endian(hostshort); }

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data) {
  return mlx_device->i2c_read(startAddress, nMemAddressRead, data);
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
  return mlx_device->i2c_write(writeAddress, data);
}

int MLX90640_I2CGeneralReset() {
//is sending 0x06 to address 0x00
//  return mlx_device->i2c_write(0x00, 0x06);
  return 0;
}

namespace esphome {
namespace mlx90640 {

static const char *const TAG = "mlx90640";
/* ---------------- CameraImage class ---------------- */
MLX90640CameraImage::MLX90640CameraImage(uint8_t *data, size_t length, uint8_t requesters) : data_(data), length_(length), requesters_(requesters) {}

uint8_t *MLX90640CameraImage::get_data_buffer() {
  return data_;
}

size_t MLX90640CameraImage::get_data_length() {
  return length_;
}

bool MLX90640CameraImage::was_requested_by(camera::CameraRequester requester) const {
  return (this->requesters_ & (1 << requester)) != 0;
}

MLX90640CameraImage::~MLX90640CameraImage() {
  free(data_);
}

/* ---------------- CameraImageReader class ---------------- */
void MLX90640CameraImageReader::set_image(std::shared_ptr<camera::CameraImage> image) {
//  ESP_LOGD(TAG,"MLX90640CameraImageReader::set_image()");
  this->image_ = std::static_pointer_cast<MLX90640CameraImage>(image);
  //this->image_ = std::move(image);
  this->offset_ = 0;
}

size_t MLX90640CameraImageReader::available() const {
  if (!this->image_)
    return 0;

  return this->image_->get_data_length() - this->offset_;
}

void MLX90640CameraImageReader::return_image() {
//  ESP_LOGD(TAG,"MLX90640CameraImageReader::return_image()");
  this->image_.reset();
}

void MLX90640CameraImageReader::consume_data(size_t consumed) { this->offset_ += consumed; }

uint8_t *MLX90640CameraImageReader::peek_data_buffer() { return this->image_->get_data_buffer() + this->offset_; }

void MLX90640::request_image(camera::CameraRequester requester) {
  single_requesters_ |= (1U << requester);
}

void MLX90640::add_image_callback(std::function<void(std::shared_ptr<camera::CameraImage>)> &&callback) {
    ESP_LOGD(TAG,"MLX90640CameraImageReader::add_image_callback()");
    this->new_image_callback_.add(std::move(callback));
}

camera::CameraImageReader* MLX90640::create_image_reader() {
  return new MLX90640CameraImageReader;
}

void MLX90640::print_memory_8_bytes(uint8_t* address) {
  ESP_LOGD(TAG, "%p: %02X%02X %02X%02X %02X%02X %02X%02X", address,
           address[0], address[1], address[2], address[3], address[4], address[5], address[6], address[7]);
}

void MLX90640::print_memory_16_bytes(uint8_t* address) {
  ESP_LOGD(TAG, "%p: %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X", address,
           address[0], address[1], address[2], address[3], address[4], address[5], address[6], address[7],
           address[8], address[9], address[10], address[11], address[12], address[13], address[14], address[15]);
}

void MLX90640::print_memory_32_bytes(uint8_t* address) {
  ESP_LOGD(TAG, "%p: %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X", address,
           address[0], address[1], address[2], address[3], address[4], address[5], address[6], address[7],
           address[8], address[9], address[10], address[11], address[12], address[13], address[14], address[15],
           address[16], address[17], address[18], address[19], address[20], address[21], address[22], address[23],
           address[24], address[25], address[26], address[27], address[28], address[29], address[30], address[31]);
}

void MLX90640::print_ee() {
    for (int i = 0; i < 52; ++i) {
      print_memory_32_bytes(reinterpret_cast<uint8_t*>(ee_data + 16*i));
    }
}

void MLX90640::print_device_id() {
  uint8_t* address = reinterpret_cast<uint8_t*>(ee_data + 7);
  ESP_LOGD(TAG, "DEVICE ID: %02X%02X %02X%02X %02X%02X",
           address[0], address[1], address[2], address[3], address[4], address[5]);
}

void MLX90640::setup() {
  state = STATE_CLEAR_NEW_DATA_READY_BIT;
  ::mlx_device = this;

  ESP_LOGCONFIG(TAG, "Setting up MLX90640...");
  esphome::i2c::ErrorCode error = (esphome::i2c::ErrorCode)MLX90640_DumpEE(0x33, ee_data);
  print_ee();
  print_device_id();
  if (error) {
    ESP_LOGE(TAG,"MLX90640_DumpEE failed. Error: %s", i2c_to_string(error));
    mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "Loaded EE content.");

  int status = MLX90640_ExtractParameters(ee_data, &parameters);
  if (status == MLX90640_EEPROM_DATA_ERROR) {
    ESP_LOGE(TAG,"MLX90640_ExtractParameters failed. Error: %s", api_to_string(error));
    mark_failed();
    return;
  } else {
    ESP_LOGW(TAG,"Parameter extraction: %s", api_to_string(status));
  }

  ESP_LOGCONFIG(TAG, "Extracted Parameters from EE.");

  set_configuration();
}

float MLX90640::get_setup_priority() const {
  return setup_priority::LATE;
}

void MLX90640::set_configuration() {
  MLX90640_SetResolution(0x33, resolution_);
  MLX90640_SetRefreshRate(0x33, refresh_rate_);

//  MLX90640_SetInterleavedMode(0x33);
  MLX90640_SetChessMode(0x33);

  ESP_LOGCONFIG(TAG, "Resolution: %i Refreshrate: %i Mode: %i",
    MLX90640_GetCurResolution(0x33),
    MLX90640_GetRefreshRate(0x33),
    MLX90640_GetCurMode(0x33));

  if (refresh_rate_select_ != nullptr)
    refresh_rate_select_->publish_state(Refresh_mappings_to_string.at(refresh_rate_));

  if (resolution_select_ != nullptr)
    resolution_select_->publish_state(Resolution_mappings_to_string.at(resolution_));

  if (presence_detector_threshold_sensor_ != nullptr)
    presence_detector_threshold_sensor_->publish_state(presence_detector_threshold_);

  if (presence_detector_hysteresis_sensor_ != nullptr)
    presence_detector_hysteresis_sensor_->publish_state(presence_detector_hysteresis_);
}

void MLX90640::dump_config() {
  ESP_LOGCONFIG(TAG, "MLX90640:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with MLX90640 failed!");
  }
}

const char* MLX90640::i2c_to_string(esphome::i2c::ErrorCode error) {
  switch (error) {
    case esphome::i2c::NO_ERROR: return "NO_ERROR";
    case esphome::i2c::ERROR_INVALID_ARGUMENT: return "ERROR_INVALID_ARGUMENT";
    case esphome::i2c::ERROR_NOT_ACKNOWLEDGED: return "ERROR_NOT_ACKNOWLEDGED";
    case esphome::i2c::ERROR_TIMEOUT: return "ERROR_TIMEOUT";
    case esphome::i2c::ERROR_NOT_INITIALIZED: return "ERROR_NOT_INITIALIZED";
    case esphome::i2c::ERROR_TOO_LARGE: return "ERROR_TOO_LARGE";
    case esphome::i2c::ERROR_UNKNOWN: return "ERROR_UNKNOWN";
    case esphome::i2c::ERROR_CRC: return "ERROR_CRC";
    default: return "ERROR_UNHANDLED_TYPE";
  }
}

const char* MLX90640::api_to_string(int error) {
  switch (-error) {
    case MLX90640_NO_ERROR: return "MLX90640_NO_ERROR";
    case MLX90640_I2C_NACK_ERROR: return "MLX90640_I2C_NACK_ERROR";
    case MLX90640_I2C_WRITE_ERROR: return "MLX90640_I2C_WRITE_ERROR";
    case MLX90640_BROKEN_PIXELS_NUM_ERROR: return "MLX90640_BROKEN_PIXELS_NUM_ERROR";
    case MLX90640_OUTLIER_PIXELS_NUM_ERROR: return "MLX90640_OUTLIER_PIXELS_NUM_ERROR";
    case MLX90640_BAD_PIXELS_NUM_ERROR: return "MLX90640_BAD_PIXELS_NUM_ERROR";
    case MLX90640_ADJACENT_BAD_PIXELS_ERROR: return "MLX90640_ADJACENT_BAD_PIXELS_ERROR";
    case MLX90640_EEPROM_DATA_ERROR: return "MLX90640_EEPROM_DATA_ERROR";
    case MLX90640_FRAME_DATA_ERROR: return "MLX90640_FRAME_DATA_ERROR";
    case MLX90640_MEAS_TRIGGER_ERROR: return "MLX90640_MEAS_TRIGGER_ERROR";
    default: return "MLX90640_UNHANDLED_TYPE";
  }
}

i2c::ErrorCode MLX90640::i2c_read(uint16_t startAddress, uint16_t size, uint16_t *data, bool stop) {
  while (size > i2c_max_message_size) {
    esphome::i2c::ErrorCode error = i2c_read_max_message_size(startAddress, i2c_max_message_size, data, false);
    if (error != esphome::i2c::NO_ERROR)
      return error;

    size -= i2c_max_message_size;
    startAddress += i2c_max_message_size;
    data += i2c_max_message_size;
  }

  esphome::i2c::ErrorCode error = i2c_read_max_message_size(startAddress, size, data, false);

  // if (size == 1)
  //   ESP_LOGD(TAG, "MLX90640::i2c_read() error: %i size: %i  %04X@%#04X", error, size, *data, startAddress);
  // else
  //   ESP_LOGD(TAG, "MLX90640::i2c_read() error: %i size: %i  @%#04X", error, size, startAddress);

  return error;
}

i2c::ErrorCode MLX90640::i2c_read_max_message_size(uint16_t startAddress, uint16_t size, uint16_t *data, bool stop) {
  esphome::i2c::ErrorCode error = read_register16(startAddress, reinterpret_cast<uint8_t *>(data), size * 2, stop);

//  ESP_LOGD(TAG, "MLX90640::i2c_read_max_message_size() error: %i size: %i data:%p @%#04X", error, size, data, startAddress);

  if (error != esphome::i2c::ErrorCode::ERROR_OK)
    return error;

  for (size_t i = 0; i < size; i++)
    data[i] = i2ctohs(data[i]);

  return esphome::i2c::ErrorCode::ERROR_OK;
}

i2c::ErrorCode MLX90640::i2c_write(uint16_t writeAddress, uint16_t data) {
  data = htoi2cs(data);

//  ESP_LOGD(TAG, "MLX90640::i2c_write() %04X@%#04X", data, writeAddress);

  return write_register16(writeAddress, reinterpret_cast<const uint8_t *>(&data), 2);
}

uint16_t MLX90640::getStatusRegister() {
    uint16_t statusRegister = 0;
    int status = MLX90640_I2CRead(0x33, MLX90640_STATUS_REG, 1, &statusRegister);
    if (status) {
      ESP_LOGD(TAG,"getStatusRegister failed. Status: %i. Error: %s", status, api_to_string(status));
    }

    return statusRegister;
}

void MLX90640::calculate_temperatur_() {
  float emissivity = 0.95;
  ambient_temperature_ = MLX90640_GetTa(frame, &parameters) - 8.0f;
  MLX90640_CalculateTo(frame, &parameters, emissivity, ambient_temperature_, framebuf);
  MLX90640_BadPixelsCorrection((&parameters)->brokenPixels, framebuf, 1/*1 = CHESS, 0 = INTERLEAVED*/, &parameters); 
  MLX90640_BadPixelsCorrection((&parameters)->outlierPixels, framebuf, 1, &parameters); 
}

void MLX90640::loop() {
//  ESP_LOGD(TAG, "MLX90640::loop() state: %i", this->state);

  switch (state) {

    case STATE_CLEAR_NEW_DATA_READY_BIT: {
      int status = MLX90640_I2CWrite(0x33, MLX90640_STATUS_REG, MLX90640_INIT_STATUS_VALUE);
      if (status) {
        ESP_LOGD(TAG,"STATE_CLEAR_NEW_DATA_READY_BIT failed. Status: %i. Error: %s", status, api_to_string(status));
        return;
      }
      state = STATE_WAIT_FOR_SUB_PAGE_0;
      return;
    }
    break;

    case STATE_WAIT_FOR_SUB_PAGE_0: {
      uint16_t statusRegister = getStatusRegister();
      if (!MLX90640_GET_DATA_READY(statusRegister))
        return;

      // Wrong subpage.
      if (MLX90640_GET_FRAME(statusRegister) == 1) {
        state = STATE_CLEAR_NEW_DATA_READY_BIT;
        return;
      }

      // uint16_t statusRegister = getStatusRegister();
      // if (MLX90640_GET_FRAME(statusRegister) == 0) {
        int status = MLX90640_GetFrameData(0x33, frame);
        if (status < 0) 
          ESP_LOGD(TAG,"STATE_WAIT_FOR_SUB_PAGE_0 failed. Status: %i. Error: %s", status, api_to_string(status));

        state = STATE_CALCULATE_SUB_PAGE_0;
//        state = STATE_PUBLISH_IMAGE;
//state = STATE_EXTRACT_TEMPERATURE;
      // }
    }
    break;
    case STATE_CALCULATE_SUB_PAGE_0: {
        calculate_temperatur_(); 
        state = STATE_WAIT_FOR_SUB_PAGE_1;
    }
    break;
    case STATE_WAIT_FOR_SUB_PAGE_1: {
      uint16_t statusRegister = getStatusRegister();
      if (!MLX90640_GET_DATA_READY(statusRegister))
        return;

      // Wrong subpage.
      if (MLX90640_GET_FRAME(statusRegister) == 0) {
        state = STATE_CLEAR_NEW_DATA_READY_BIT;
        return;
      }

      int status = MLX90640_GetFrameData(0x33, frame);
      if (status < 0) 
        ESP_LOGD(TAG,"STATE_WAIT_FOR_SUB_PAGE_1 failed. Status: %i. Error: %s", status, api_to_string(status));

      state = STATE_CALCULATE_SUB_PAGE_1;
      return;
    }
    break;
    case STATE_CALCULATE_SUB_PAGE_1: {
        calculate_temperatur_(); 
        state = STATE_PUBLISH_IMAGE;
    }
    break;
    case STATE_PUBLISH_IMAGE: {
        state = STATE_PUBLISH_SENSORS;
        // request idle image every idle_update_interval
        const uint32_t now = millis();
        // if (this->idle_update_interval_ != 0 && (now - this->last_idle_request_) > this->idle_update_interval_) {
        //   this->last_idle_request_ = now;
        //   this->request_image(camera::IDLE);
        //   ESP_LOGD(TAG,"MLX90640::loop() request_image.");
        // } else {
        //   return;
        // }

        // // Check if we should fetch a new image
        // if (!this->has_requested_image_())
        //   return;
        this->request_image(camera::IDLE);

        if (this->current_image_.use_count() > 1) {
          ESP_LOGD(TAG,"MLX90640::loop() image is still in use.");
          return;
        }
        // if (now - this->last_update_ <= this->max_update_interval_)
        //   return;
        JPEGENCODE jpe;
        JPEGENC jpg;
        uint8_t ucMCU[64]; // fake image data 8x8

        int width = 32 * 8;// 32 * 8 -> 256;
        int height = 24 * 8;// 24 * 8 -> 196;

        int size = 8192; // test with an output buffer
        uint8_t* buffer = (uint8_t *)malloc(size);
        int rc = jpg.open(buffer, size);
        if (rc != JPEGE_SUCCESS) {
          ESP_LOGD(TAG,"MLX90640::loop() jpeg open failed. Error: %i", rc);
          free(buffer);
          return;
        }
        rc = jpg.encodeBegin(&jpe, width, height, JPEGE_PIXEL_GRAYSCALE, JPEGE_SUBSAMPLE_444, JPEGE_Q_BEST);
        if (rc != JPEGE_SUCCESS) {
          ESP_LOGD(TAG,"MLX90640::loop() jpeg encode failed. Error: %i", rc);
          free(buffer);
          return;
        }
        memset(ucMCU, 0, sizeof(ucMCU));

      float minT = 1000.0f;
      float maxT = -1000.0f;

      for (int i = 0 ; i < 32*24 ; i++) {
        float t = framebuf[i];
        if (t == 0.0f)
          continue;
        minT = minT < t ? minT : t;
        maxT = maxT > t ? maxT : t;
      }

      minimum_temperature_ = minT;
      maximum_temperature_ = maxT;

      //ESP_LOGD(TAG, "Temperature Range %.2f - %.2f C", minT, maxT);

        int iMCUCount = ((width + jpe.cx-1)/ jpe.cx) * ((height + jpe.cy-1) / jpe.cy);
        for (uint16_t i=0; i<iMCUCount && rc == JPEGE_SUCCESS; i++) {
          float t = framebuf[i];
          float rangeT = maxT - minT;
          if (rangeT > 0.1f) {
            t = (t - minT) / rangeT;
            if (t < 0)
              t = 0.0f;
            if (t > 1.0f)
              t = 1.0f;
          }
          uint8_t color = (t * 255.0f);
      for (int i=0; i<64; i++)
        ucMCU[i] = color;

//          ESP_LOGD(TAG,"MLX90640::loop() add MCU(%i) at position y(%i) x(%i) cy(%i) cx(%i)",i, jpe.y, jpe.x, jpe.cy, jpe.cx);

        rc = jpg.addMCU(&jpe, ucMCU, 8);
      }
      int datasize = jpg.close();

      //ESP_LOGD(TAG,"MLX90640::loop() jpeg encoded. Size: %i", datasize);
      current_image_ = std::make_shared<MLX90640CameraImage>(buffer, datasize, this->single_requesters_ | this->stream_requesters_);
      new_image_callback_.call(this->current_image_);
      last_image_update_ = now;
      single_requesters_ = 0;
    }
    break;
    case STATE_PUBLISH_SENSORS: {
      state = STATE_CLEAR_NEW_DATA_READY_BIT;

      const uint32_t now = millis();

      if (now - last_sensors_update_ > publish_sensors_timeout_) {
        last_sensors_update_ = now;
        if (this->ambient_sensor_ != nullptr)
          this->ambient_sensor_->publish_state(ambient_temperature_);

        if (this->minimum_temperature_sensor_ != nullptr)
          this->minimum_temperature_sensor_->publish_state(minimum_temperature_);

        if (this->maximum_temperature_sensor_ != nullptr)
          this->maximum_temperature_sensor_->publish_state(maximum_temperature_);
      }

      if (presence_detector_sensor_ != nullptr) {
        if (minimum_temperature_ + presence_detector_threshold_ < maximum_temperature_) {
          presence_detector_sensor_->publish_state(true);
        }

        if (minimum_temperature_ + presence_detector_threshold_ - presence_detector_hysteresis_ > maximum_temperature_) {
          presence_detector_sensor_->publish_state(false);
        }
      }
    }
    break;

    default: {
    }
    break;
  }
}

void MLX90640::set_refresh_rate(const std::string &refresh_rate) {
  MLX90640_SetRefreshRate(0x33, Refresh_mappings.at(refresh_rate));
}

void MLX90640::set_resolution(const std::string &resolution) {
  ESP_LOGD(TAG,"MLX90640::set_resolution %s", resolution.c_str());
  MLX90640_SetResolution(0x33, Resolution_mappings.at(resolution));
}

}  // namespace mlx90640
}  // namespace esphome
