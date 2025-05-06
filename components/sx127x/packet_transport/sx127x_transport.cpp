#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/components/network/util.h"
#include "sx127x_transport.h"

namespace esphome {
namespace sx127x {

static const char *const TAG = "sx127x_transport";

void SX127xTransport::update() {
  PacketTransport::update();
  this->updated_ = true;
  this->resend_data_ = true;
}

void SX127xTransport::send_packet(std::vector<uint8_t> &buf) const { this->parent_->transmit_packet(buf); }

void SX127xTransport::on_packet(const std::vector<uint8_t> &packet, float rssi, float snr) {
  std::vector<uint8_t> temp = packet;
  this->process_(temp);
}

}  // namespace sx127x
}  // namespace esphome
