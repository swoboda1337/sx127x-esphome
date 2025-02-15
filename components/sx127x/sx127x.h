#pragma once

#include "sx127x_reg.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include <vector>

namespace esphome {
namespace sx127x {

class SX127x : public Component,
               public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING,
                                     spi::DATA_RATE_8MHZ> {
 public:
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  void set_bitrate(uint32_t bitrate) { this->bitrate_ = bitrate; }
  void set_bitsync(bool bitsync) { this->bitsync_ = bitsync; }
  void set_crc_enable(bool crc_enable) { this->crc_enable_ = crc_enable; }
  void set_dio0_pin(InternalGPIOPin *dio0_pin) { this->dio0_pin_ = dio0_pin; }
  void set_frequency(uint32_t frequency) { this->frequency_ = frequency; }
  void set_fsk_fdev(uint32_t fdev) { this->fsk_fdev_ = fdev; }
  void set_fsk_ramp(SX127xPaRamp ramp) { this->fsk_ramp_ = ramp; }
  void set_mode_rx();
  void set_mode_standby();
  void set_mode_tx();
  void set_modulation(SX127xOpMode modulation) { this->modulation_ = modulation; }
  void set_pa_pin(SX127xPaConfig pin) { this->pa_pin_ = pin; }
  void set_pa_power(uint32_t power) { this->pa_power_ = power; }
  void set_payload_length(uint8_t payload_length) { this->payload_length_ = payload_length; }
  void set_preamble_errors(uint8_t preamble_errors) { this->preamble_errors_ = preamble_errors; }
  void set_preamble_polarity(uint8_t preamble_polarity) { this->preamble_polarity_ = preamble_polarity; }
  void set_preamble_size(uint8_t preamble_size) { this->preamble_size_ = preamble_size; }
  void set_rst_pin(InternalGPIOPin *rst_pin) { this->rst_pin_ = rst_pin; }
  void set_rx_bandwidth(SX127xRxBw bandwidth) { this->rx_bandwidth_ = bandwidth; }
  void set_rx_floor(float floor) { this->rx_floor_ = floor; }
  void set_rx_start(bool start) { this->rx_start_ = start; }
  void set_shaping(SX127xPaRamp shaping) { this->shaping_ = shaping; }
  void set_sync_value(const std::vector<uint8_t> &sync_value) { this->sync_value_ = sync_value; }
  void run_image_cal();
  void configure();
  void transmit_packet(const std::vector<uint8_t> &packet);
  Trigger<std::vector<uint8_t>> *get_packet_trigger() const { return this->packet_trigger_; };

 protected:
  void configure_fsk_ook_();
  void configure_lora_();
  void set_mode_(SX127xOpMode mode);
  void write_fifo_(const std::vector<uint8_t> &packet);
  void read_fifo_(std::vector<uint8_t> &packet);
  void write_register_(uint8_t reg, uint8_t value);
  uint8_t single_transfer_(uint8_t reg, uint8_t value);
  uint8_t read_register_(uint8_t reg);
  Trigger<std::vector<uint8_t>> *packet_trigger_{new Trigger<std::vector<uint8_t>>()};
  std::vector<uint8_t> sync_value_;
  InternalGPIOPin *dio0_pin_{nullptr};
  InternalGPIOPin *rst_pin_{nullptr};
  SX127xPaConfig pa_pin_;
  SX127xRxBw rx_bandwidth_;
  SX127xOpMode modulation_;
  SX127xPaRamp shaping_;
  SX127xPaRamp fsk_ramp_;
  uint32_t fsk_fdev_;
  uint32_t frequency_;
  uint32_t bitrate_;
  uint32_t pa_power_;
  uint32_t payload_length_;
  uint8_t preamble_polarity_;
  uint8_t preamble_size_;
  uint8_t preamble_errors_;
  float rx_floor_;
  bool rx_start_;
  bool bitsync_;
  bool crc_enable_;
};

}  // namespace sx127x
}  // namespace esphome
