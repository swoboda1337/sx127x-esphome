ESPHome SX127x driver.

SX127x component configures SX1276, SX1277, SX1278 or SX1279 hardware for use in ESPHome. 

Tested with the LILYGO LoRa32 V2.1_1.6 board.

Note the small 433 MHz antennas that come with these boards work fine but are not ideal. The antenna gain is really poor and it’s too close to the Wi-Fi antenna which can cause glitches. A proper antenna like the Siretta Tango 9 has better range and doesn’t glitch when Wi-Fi transmits.

Receiver example to use in ESPHome, timing data is read by remote receiver from GPIO32:
    
    external_components:
      - source:
          type: git
          url: https://github.com/swoboda1337/sx127x-esphome
          ref: main
        refresh: 1d
   
    spi:
      clk_pin: GPIO5
      mosi_pin: GPIO27
      miso_pin: GPIO19
    
    sx127x:
      nss_pin: GPIO18
      rst_pin: GPIO23
      frequency: 433920000
      modulation: OOK
      rx_floor: -90
      rx_bandwidth: 50_0kHz

    remote_receiver:
      pin: GPIO32
      filter: 255us
      idle: 2000us
      buffer_size: 100000b
      memory_blocks: 8
      dump: raw

Transmitter example to use in ESPHome, timing data is sent on GPIO32 by remote transmitter (ESPHome needs an API change so the transmitter start/stop is handled automatically):

    external_components:
      - source:
          type: git
          url: https://github.com/swoboda1337/sx127x-esphome
          ref: main
        refresh: 1d

    spi:
      clk_pin: GPIO5
      mosi_pin: GPIO27
      miso_pin: GPIO19

    sx127x:
      id: sx127x_id
      nss_pin: GPIO18
      rst_pin: GPIO23
      frequency: 433920000
      modulation: OOK
      pa_pin: PA_BOOST
      pa_power: 17
      rx_start: false

    remote_transmitter:
      id: remote_transmitter_id
      pin: GPIO32
      carrier_duty_percent: 100%

    interval:
      - interval: 18sec
        then:
          - lambda: |-
              esphome::remote_base::RawTimings timings = {601, -613, 601, -613, 601, -613, 601, -613, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 405, -209, 405, -209, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209};
              auto call = id(remote_transmitter_id).transmit();
              call.get_data()->set_data(timings);
              call.set_send_times(2);
              id(sx127x_id).set_mode_tx();
              call.perform();
              id(sx127x_id).set_mode_standby();
