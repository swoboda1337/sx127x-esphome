#include "sx127x.h"

#include <cinttypes>

namespace esphome {
namespace sx127x {

static const char *const TAG = "sx127x";
static const uint32_t FXOSC = 32000000u;

void IRAM_ATTR HOT SX127xStore::gpio_intr(SX127xStore *arg) { arg->dio0_irq = true; }

uint8_t SX127x::read_register_(uint8_t reg) {
  this->enable();
  this->write_byte(reg & 0x7F);
  reg = this->read_byte();
  this->disable();
  return reg;
}

void SX127x::write_register_(uint8_t reg, uint8_t value) {
  this->enable();
  this->write_byte(reg | 0x80);
  this->write_byte(value);
  this->disable();
}

void SX127x::read_fifo_(std::vector<uint8_t> &packet) {
  this->enable();
  this->write_byte(REG_FIFO & 0x7F);
  this->read_array(packet.data(), packet.size());
  this->disable();
}

void SX127x::write_fifo_(const std::vector<uint8_t> &packet) {
  this->enable();
  this->write_byte(REG_FIFO | 0x80);
  this->write_array(packet.data(), packet.size());
  this->disable();
}

void SX127x::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SX127x...");

  // setup reset
  this->rst_pin_->setup();

  // setup dio0
  if (this->dio0_pin_) {
    this->dio0_pin_->setup();
    this->dio0_pin_->attach_interrupt(SX127xStore::gpio_intr, &this->store_, gpio::INTERRUPT_RISING_EDGE);
  }

  // start spi
  this->spi_setup();

  // configure rf
  this->configure();
}

void SX127x::configure() {
  // toggle chip reset
  this->rst_pin_->digital_write(false);
  delay(1);
  this->rst_pin_->digital_write(true);
  delay(10);

  // check silicon version to make sure hw is ok
  if (this->read_register_(REG_VERSION) != 0x12) {
    this->mark_failed();
    return;
  }

  // set modulation and make sure transceiver is in sleep mode
  this->write_register_(REG_OP_MODE, this->modulation_ | MODE_SLEEP);
  delay(1);

  // set freq
  uint64_t frf = ((uint64_t) this->frequency_ << 19) / FXOSC;
  this->write_register_(REG_FRF_MSB, (uint8_t) ((frf >> 16) & 0xFF));
  this->write_register_(REG_FRF_MID, (uint8_t) ((frf >> 8) & 0xFF));
  this->write_register_(REG_FRF_LSB, (uint8_t) ((frf >> 0) & 0xFF));

  // set fdev
  uint32_t fdev = std::min(this->fsk_fdev_ / 61, (uint32_t) 0x3FFF);
  this->write_register_(REG_FDEV_MSB, (uint8_t) ((fdev >> 8) & 0xFF));
  this->write_register_(REG_FDEV_LSB, (uint8_t) ((fdev >> 0) & 0xFF));

  // set the channel bw
  this->write_register_(REG_RX_BW, this->rx_bandwidth_);

  // set bitrate
  uint64_t bitrate = (FXOSC + this->bitrate_ / 2) / this->bitrate_;  // round up
  this->write_register_(REG_BITRATE_MSB, (uint8_t) ((bitrate >> 8) & 0xFF));
  this->write_register_(REG_BITRATE_LSB, (uint8_t) ((bitrate >> 0) & 0xFF));

  // configure dio mapping
  if (this->payload_length_ > 0) {
    this->write_register_(REG_DIO_MAPPING1, DIO0_MAPPING_00);
  } else {
    this->write_register_(REG_DIO_MAPPING1, DIO0_MAPPING_11);
  }

  // configure rx and afc
  uint8_t trigger = (this->preamble_size_ > 0) ? TRIGGER_PREAMBLE : TRIGGER_RSSI;
  this->write_register_(REG_AFC_FEI, AFC_AUTO_CLEAR_ON);
  if (this->modulation_ == MOD_FSK) {
    this->rx_config_ = AFC_AUTO_ON | AGC_AUTO_ON | trigger;
  } else {
    this->rx_config_ = AGC_AUTO_ON | trigger;
  }
  this->write_register_(REG_RX_CONFIG, this->rx_config_);

  // configure packet mode
  this->write_register_(REG_PACKET_CONFIG_1, 0x00);
  if (this->payload_length_ > 0) {
    this->write_register_(REG_PACKET_CONFIG_2, PACKET_MODE);
  } else {
    this->write_register_(REG_PACKET_CONFIG_2, CONTINUOUS_MODE);
  }
  this->write_register_(REG_PAYLOAD_LENGTH, this->payload_length_);

  // config pa
  if (this->pa_pin_ == PA_PIN_BOOST) {
    this->pa_power_ = std::max(this->pa_power_, (uint32_t) 2);
    this->pa_power_ = std::min(this->pa_power_, (uint32_t) 17);
    this->write_register_(REG_PA_CONFIG, (this->pa_power_ - 2) | this->pa_pin_ | PA_MAX_POWER);
  } else {
    this->pa_power_ = std::min(this->pa_power_, (uint32_t) 14);
    this->write_register_(REG_PA_CONFIG, (this->pa_power_ - 0) | this->pa_pin_ | PA_MAX_POWER);
  }
  this->write_register_(REG_PA_RAMP, this->shaping_ | this->fsk_ramp_);
  this->write_register_(REG_FIFO_THRESH, TX_START_FIFO_EMPTY);

  // config bit synchronizer
  if (!this->sync_value_.empty()) {
    uint8_t polarity = (this->preamble_polarity_ == 0xAA) ? PREAMBLE_AA : PREAMBLE_55;
    uint8_t size = this->sync_value_.size() - 1;
    this->write_register_(REG_SYNC_CONFIG, SYNC_ON | polarity | size);
    for (uint32_t i = 0; i < this->sync_value_.size(); i++) {
      this->write_register_(REG_SYNC_VALUE1 + i, this->sync_value_[i]);
    }
  } else {
    this->write_register_(REG_SYNC_CONFIG, SYNC_OFF);
  }

  // config preamble detector
  if (this->preamble_size_ > 0 && this->preamble_size_ < 4) {
    uint8_t size = (this->preamble_size_ - 1) << PREAMBLE_BYTES_SHIFT;
    uint8_t errors = this->preamble_errors_;
    this->write_register_(REG_PREAMBLE_DETECT, PREAMBLE_DETECTOR_ON | size | errors);
  } else {
    this->write_register_(REG_PREAMBLE_DETECT, PREAMBLE_DETECTOR_OFF);
  }
  this->write_register_(REG_PREAMBLE_MSB, 0);
  this->write_register_(REG_PREAMBLE_LSB, this->preamble_size_);

  // config sync generation and setup ook threshold
  uint8_t bitsync = this->bitsync_ ? BIT_SYNC_ON : BIT_SYNC_OFF;
  this->write_register_(REG_OOK_PEAK, bitsync | OOK_THRESH_STEP_0_5 | OOK_THRESH_PEAK);
  this->write_register_(REG_OOK_AVG, OOK_AVG_RESERVED | OOK_THRESH_DEC_1_8);

  // set rx floor
  this->write_register_(REG_OOK_FIX, 256 + int(this->rx_floor_ * 2.0));
  this->write_register_(REG_RSSI_THRESH, std::abs(int(this->rx_floor_ * 2.0)));

  // clear irq flag
  this->store_.dio0_irq = false;

  // enable standby mode
  this->set_mode_standby();

  // run image cal
  this->write_register_(REG_IMAGE_CAL, AUTO_IMAGE_CAL_ON | IMAGE_CAL_START | TEMP_THRESHOLD_10C);
  delay(10);

  // enable rx mode
  if (this->rx_start_) {
    this->set_mode_rx();
  }
}

void SX127x::transmit_packet(const std::vector<uint8_t> &packet) {
  if (packet.size() != this->payload_length_) {
    ESP_LOGE(TAG, "Packet size does not match payload length");
    return;
  }
  this->set_mode_standby();
  this->write_fifo_(packet);
  this->set_mode_tx();
  while (!this->store_.dio0_irq) {
    // do nothing
  }
  this->store_.dio0_irq = false;
  if (this->rx_start_) {
    this->set_mode_rx();
  } else {
    this->set_mode_standby();
  }
}

void SX127x::loop() {
  if (this->store_.dio0_irq && this->payload_length_ > 0) {
    std::vector<uint8_t> packet(this->payload_length_);
    this->store_.dio0_irq = false;
    this->read_fifo_(packet);
    this->write_register_(REG_RX_CONFIG, RESTART_PLL_LOCK | this->rx_config_);
    this->packet_trigger_->trigger(packet);
  }
}

void SX127x::set_mode_standby() {
  this->write_register_(REG_OP_MODE, this->modulation_ | MODE_STDBY);
  delay(1);
}

void SX127x::set_mode_rx() {
  this->write_register_(REG_OP_MODE, this->modulation_ | MODE_RX_FS);
  delay(1);
  this->write_register_(REG_OP_MODE, this->modulation_ | MODE_RX);
  delay(1);
}

void SX127x::set_mode_tx() {
  this->write_register_(REG_OP_MODE, this->modulation_ | MODE_TX_FS);
  delay(1);
  this->write_register_(REG_OP_MODE, this->modulation_ | MODE_TX);
  delay(1);
}

void SX127x::dump_config() {
  static const uint16_t RAMP_LUT[16] = {3400, 2000, 1000, 500, 250, 125, 100, 62, 50, 40, 31, 25, 20, 15, 12, 10};
  uint32_t rx_bw_mant = 16 + (this->rx_bandwidth_ >> 3) * 4;
  uint32_t rx_bw_exp = this->rx_bandwidth_ & 0x7;
  float rx_bw = (float) FXOSC / (rx_bw_mant * (1 << (rx_bw_exp + 2)));
  ESP_LOGCONFIG(TAG, "SX127x:");
  LOG_PIN("  NSS Pin: ", this->cs_);
  LOG_PIN("  RST Pin: ", this->rst_pin_);
  LOG_PIN("  DIO0 Pin: ", this->dio0_pin_);
  ESP_LOGCONFIG(TAG, "  Frequency: %f MHz", (float) this->frequency_ / 1000000);
  ESP_LOGCONFIG(TAG, "  Modulation: %s", this->modulation_ == MOD_FSK ? "FSK" : "OOK");
  ESP_LOGCONFIG(TAG, "  Bitrate: %" PRIu32 "b/s", this->bitrate_);
  ESP_LOGCONFIG(TAG, "  Bitsync: %s", TRUEFALSE(this->bitsync_));
  ESP_LOGCONFIG(TAG, "  Rx Bandwidth: %.1f kHz", (float) rx_bw / 1000);
  ESP_LOGCONFIG(TAG, "  Rx Start: %s", TRUEFALSE(this->rx_start_));
  ESP_LOGCONFIG(TAG, "  Rx Floor: %.1f dBm", this->rx_floor_);
  ESP_LOGCONFIG(TAG, "  Payload Length: %" PRIu32, this->payload_length_);
  ESP_LOGCONFIG(TAG, "  Preamble Polarity: 0x%X", this->preamble_polarity_);
  ESP_LOGCONFIG(TAG, "  Preamble Size: %" PRIu8, this->preamble_size_);
  ESP_LOGCONFIG(TAG, "  Preamble Errors: %" PRIu8, this->preamble_errors_);
  if (!this->sync_value_.empty()) {
    ESP_LOGCONFIG(TAG, "  Sync Value: 0x%s", format_hex(this->sync_value_).c_str());
  }
  if (this->modulation_ == MOD_FSK) {
    static const char *shaping_lut[4] = {"NONE", "GAUSSIAN_BT_1_0", "GAUSSIAN_BT_0_5", "GAUSSIAN_BT_0_3"};
    ESP_LOGCONFIG(TAG, "  Shaping: %s", shaping_lut[this->shaping_ >> SHAPING_SHIFT]);
  } else {
    static const char *shaping_lut[4] = {"NONE", "CUTOFF_BR_X_1", "CUTOFF_BR_X_2", "ERROR"};
    ESP_LOGCONFIG(TAG, "  Shaping: %s", shaping_lut[this->shaping_ >> SHAPING_SHIFT]);
  }
  ESP_LOGCONFIG(TAG, "  PA Pin: %s", this->pa_pin_ == PA_PIN_BOOST ? "BOOST" : "RFO");
  ESP_LOGCONFIG(TAG, "  PA Power: %" PRIu32 " dBm", this->pa_power_);
  ESP_LOGCONFIG(TAG, "  FSK Fdev: %" PRIu32 " Hz", this->fsk_fdev_);
  ESP_LOGCONFIG(TAG, "  FSK Ramp: %" PRIu16 " us", RAMP_LUT[this->fsk_ramp_]);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Configuring SX127x failed");
  }
}

}  // namespace sx127x
}  // namespace esphome
