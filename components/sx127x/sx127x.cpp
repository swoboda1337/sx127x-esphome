#include "sx127x.h"

namespace esphome {
namespace sx127x {

static const char *const TAG = "sx127x";

static const uint8_t REG_OP_MODE         = 0x01;
static const uint8_t REG_FRF_MSB         = 0x06;
static const uint8_t REG_FRF_MID         = 0x07;
static const uint8_t REG_FRF_LSB         = 0x08;
static const uint8_t REG_RX_BW           = 0x12;
static const uint8_t REG_OOK_PEAK        = 0x14;
static const uint8_t REG_OOK_FIX         = 0x15;
static const uint8_t REG_SYNC_CONFIG     = 0x27;
static const uint8_t REG_PACKET_CONFIG_1 = 0x30;
static const uint8_t REG_PACKET_CONFIG_2 = 0x31;
static const uint8_t REG_IRQ_FLAGS_1     = 0x3e;
static const uint8_t REG_IRQ_FLAGS_2     = 0x3f;
static const uint8_t REG_VERSION         = 0x42;

static const uint8_t MODE_LF_ON = 0x08;
static const uint8_t MODE_SLEEP = 0x00;
static const uint8_t MODE_STDBY = 0x01;
static const uint8_t MODE_RX_FS = 0x04;
static const uint8_t MODE_RX    = 0x05;

static const uint8_t OOK_TRESHOLD_PEAK = 0x08;

uint8_t SX127x::read_register_(uint8_t reg) {
  return this->single_transfer_((uint8_t)reg & 0x7f, 0x00);
}

void SX127x::write_register_(uint8_t reg, uint8_t value) {
  this->single_transfer_((uint8_t)reg | 0x80, value);
}

uint8_t SX127x::single_transfer_(uint8_t address, uint8_t value) {
  uint8_t response;
  this->delegate_->begin_transaction();
  this->nss_pin_->digital_write(false);
  this->delegate_->transfer(address);
  response = this->delegate_->transfer(value);
  this->nss_pin_->digital_write(true);
  this->delegate_->end_transaction();
  return response;
}

void SX127x::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SX127x...");

  // setup nss and set high
  this->nss_pin_->setup();
  this->nss_pin_->digital_write(true);

  // setup reset and toggle to reset chip
  this->rst_pin_->setup();
  this->rst_pin_->digital_write(false);
  delay(1);
  this->rst_pin_->digital_write(true);
  delay(10);

  // start spi
  this->spi_setup();

  // check silicon version to make sure hw is ok
  if (this->read_register_(REG_VERSION) != 0x12) {
    mark_failed();
    return;
  }

  // set modulation and make sure transceiver is in sleep mode
  this->write_register_(REG_OP_MODE, this->modulation_ | MODE_LF_ON | MODE_SLEEP);
  delay(1);

  // set freq
  uint64_t frf = ((uint64_t)this->frequency_ << 19) / 32000000;
  this->write_register_(REG_FRF_MSB, (uint8_t)((frf >> 16) & 0xFF));
  this->write_register_(REG_FRF_MID, (uint8_t)((frf >> 8) & 0xFF));
  this->write_register_(REG_FRF_LSB, (uint8_t)((frf >> 0) & 0xFF));
  
  // set the channel bw
  this->write_register_(REG_RX_BW, this->bandwidth_);

  // disable packet mode
  this->write_register_(REG_PACKET_CONFIG_1, 0x00);
  this->write_register_(REG_PACKET_CONFIG_2, 0x00);

  // disable bit synchronizer and sync generation
  this->write_register_(REG_SYNC_CONFIG, 0x00);
  this->write_register_(REG_OOK_PEAK, OOK_TRESHOLD_PEAK);

  // set ook floor
  this->write_register_(REG_OOK_FIX, int((128 + this->ook_floor_) * 2 + 0.5));

  // enable rx mode  
  this->write_register_(REG_OP_MODE, this->modulation_ | MODE_LF_ON | MODE_STDBY);
  delay(1);
  this->write_register_(REG_OP_MODE, this->modulation_ | MODE_LF_ON | MODE_RX_FS);
  delay(1);
  this->write_register_(REG_OP_MODE, this->modulation_ | MODE_LF_ON | MODE_RX);
}

void SX127x::dump_config() {
  uint32_t rx_bw_mant = 16 + (this->bandwidth_ >> 3) * 4;
  uint32_t rx_bw_exp = this->bandwidth_ & 0x7;
  float rx_bw = (float)32000000 / (rx_bw_mant * (2 << (rx_bw_exp + 2))) * 2;
  ESP_LOGCONFIG(TAG, "SX127x:");
  ESP_LOGCONFIG(TAG, "  Frequency: %f MHz", (float)this->frequency_ / 1000000);
  ESP_LOGCONFIG(TAG, "  Bandwidth: %.1f kHz", (float)rx_bw / 1000);
  ESP_LOGCONFIG(TAG, "  Modulation: %s", this->modulation_ == MODULATION_FSK ? "FSK" : "OOK");
  if (this->modulation_ == MODULATION_OOK) {
    ESP_LOGCONFIG(TAG, "  Floor: %.1f dBm", this->ook_floor_);
  }
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Configuring SX127x failed");
  }
}

}  // namespace sx127x
}  // namespace esphome
