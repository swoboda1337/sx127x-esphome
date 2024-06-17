#pragma once

#include "esphome/components/spi/spi.h"

namespace esphome {
namespace sx127x {

enum SX127xRxBw : uint8_t {
  RX_BANDWIDTH_2_6 = 0x17,
  RX_BANDWIDTH_3_1 = 0x0F,
  RX_BANDWIDTH_3_9 = 0x07,
  RX_BANDWIDTH_5_2 = 0x16,
  RX_BANDWIDTH_6_3 = 0x0E,
  RX_BANDWIDTH_7_8 = 0x06,
  RX_BANDWIDTH_10_4 = 0x15,
  RX_BANDWIDTH_12_5 = 0x0D,
  RX_BANDWIDTH_15_6 = 0x05,
  RX_BANDWIDTH_20_8 = 0x14,
  RX_BANDWIDTH_25_0 = 0x0C,
  RX_BANDWIDTH_31_3 = 0x04,
  RX_BANDWIDTH_41_7 = 0x13,
  RX_BANDWIDTH_50_0 = 0x0B,
  RX_BANDWIDTH_62_5 = 0x03,
  RX_BANDWIDTH_83_3 = 0x12,
  RX_BANDWIDTH_100_0 = 0x0A,
  RX_BANDWIDTH_125_0 = 0x02,
  RX_BANDWIDTH_166_7 = 0x11,
  RX_BANDWIDTH_200_0 = 0x09,
  RX_BANDWIDTH_250_0 = 0x01
};

enum SX127xMod : uint8_t {
  MODULATION_FSK = 0x00,
  MODULATION_OOK = 0x20
};

enum SX127xPaPin : uint8_t {
  RFO = 0x00,
  PA_BOOST = 0x80
};

class SX127x : public Component,
               public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, 
                                     spi::CLOCK_POLARITY_LOW, 
                                     spi::CLOCK_PHASE_LEADING,
                                     spi::DATA_RATE_8MHZ> { 
 public:
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void setup() override;
  void dump_config() override;
  void set_rst_pin(InternalGPIOPin *rst_pin) { this->rst_pin_ = rst_pin; }
  void set_nss_pin(InternalGPIOPin *nss_pin) { this->nss_pin_ = nss_pin; }
  void set_frequency(uint32_t frequency) { this->frequency_ = frequency; }
  void set_modulation(SX127xMod modulation) { this->modulation_ = modulation; }
  void set_rx_start(bool start) { this->rx_start_ = start; }
  void set_rx_floor(float floor) { this->rx_floor_ = floor; }
  void set_rx_bandwidth(SX127xRxBw bandwidth) { this->rx_bandwidth_ = bandwidth; }
  void set_pa_pin(SX127xPaPin pin) { this->pa_pin_ = pin; }
  void set_pa_power(uint32_t power) { this->pa_power_ = power; }
  void set_mode_standby();
  void set_mode_tx();
  void set_mode_rx();

 protected:
  void write_register_(uint8_t address, uint8_t value);
  uint8_t single_transfer_(uint8_t address, uint8_t value);
  uint8_t read_register_(uint8_t address);
  InternalGPIOPin *rst_pin_{nullptr};
  InternalGPIOPin *nss_pin_{nullptr};
  SX127xPaPin pa_pin_;
  SX127xRxBw rx_bandwidth_; 
  SX127xMod modulation_;
  uint32_t frequency_;
  uint32_t pa_power_;
  float rx_floor_;
  bool rx_start_;
};

}  // namespace sx127x
}  // namespace esphome