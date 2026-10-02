#include "rcxazair.h"
#include "esphome/core/log.h"

namespace esphome {
namespace rcxazair {

static const char *const TAG = "rcxazair";

static const espbt::ESPBTUUID CONTROL_SERVICE_UUID =
    espbt::ESPBTUUID::from_raw("00010203-0405-0607-0809-0a0b0c0d1910");
static const espbt::ESPBTUUID CONTROL_CHARACTERISTIC_UUID =
    espbt::ESPBTUUID::from_raw("00010203-0405-0607-0809-0a0b0c0d2b10");

void Rcxazair::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                   esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      auto *chr = this->parent_->get_characteristic(CONTROL_SERVICE_UUID, CONTROL_CHARACTERISTIC_UUID);
      if (chr == nullptr) {
        ESP_LOGE(TAG, "[%s] No control service found at device, not an rcxazair?",
                 this->parent_->address_str());
        break;
      }
      this->char_handle_ = chr->handle;
      ESP_LOGI(TAG, "[%s] got characteristic!",
               this->parent_->address_str());

      auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(), chr->handle);
      if (status) {
        ESP_LOGE(TAG, "[%s] esp_ble_gattc_register_for_notify failed, status=%d",
                 this->parent_->address_str(), status);
      }
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.handle != this->char_handle_)
        break;
      this->handle_message(param->notify.value, param->notify.value_len);
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      this->char_handle_ = 0;
      break;
    }

    default:
      break;
  }
}

static uint16_t parse_be16(uint8_t *payload) {
  return (static_cast<uint16_t>(payload[0]) << 8) | payload[1];
}

void Rcxazair::handle_message(uint8_t *payload, uint16_t length) {
  if (length < 4) {
    ESP_LOGW(TAG, "Malformed message: too short");
    return;
  }

  // Header verification: 0x23 0x01
  if (payload[0] != 0x23 || payload[1] != 0x01) {
    ESP_LOGV(TAG, "Unhandled header: %02x %02x", payload[0], payload[1]);
    return;
  }

  uint8_t message_type = payload[3];

  switch (message_type) {
    case 0x02: {
      // Message 0x02: Formaldehyde (CH2O), TVOC, CO2
      if (length < 11) {
        ESP_LOGW(TAG, "Malformed 0x02 message: too short");
        return;
      }
      if (this->ch2o_sensor_ != nullptr) {
        float ch2o = parse_be16(payload + 4) / 100.0f;
        this->ch2o_sensor_->publish_state(ch2o);
      }
      if (this->tvoc_sensor_ != nullptr) {
        float tvoc = parse_be16(payload + 6) / 100.0f;
        this->tvoc_sensor_->publish_state(tvoc);
      }
      if (this->co2_sensor_ != nullptr) {
        uint16_t co2 = parse_be16(payload + 8);
        this->co2_sensor_->publish_state(co2);
      }
      break;
    }

    case 0x03: {
      // Message 0x03: Temperature, Humidity
      if (length < 9) {
        ESP_LOGW(TAG, "Malformed 0x03 message: too short");
        return;
      }
      if (this->temp_sensor_ != nullptr) {
        float temp = parse_be16(payload + 4) / 10.0f;
        this->temp_sensor_->publish_state(temp);
      }
      if (this->hum_sensor_ != nullptr) {
        float humidity = parse_be16(payload + 6) / 10.0f;
        this->hum_sensor_->publish_state(humidity);
      }
      break;
    }

    default:
      ESP_LOGI(TAG, "[%s] Got unknown message type %x",
               this->parent_->address_str(), message_type);
      break;
  }
}

}  // namespace rcxazair
}  // namespace esphome
