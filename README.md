ESPHome SX127x driver.

SX127x component configures SX1276, SX1277, SX1278 or SX1279 hardware for use in ESPHome. 

Tested with the LILYGO LoRa32 V2.1_1.6 / T3 V1.6.1 board. Note the small 433 MHz antennas that come with these boards work fine but are not ideal. The antenna gain is really poor and it’s too close to the Wi-Fi antenna which can cause glitches. A proper antenna like the Siretta Tango 9 has better range and doesn’t glitch when Wi-Fi transmits.

When receiving OOK data rx_floor should be set appropriately for your environment / device / antenna. If the floor is set too high (ie closer to 0) the radio will ignore everything. If the floor is set too low (ie closer to -128) then noise will overwhelm remote receiver. To calibrate this it is recommended to set the log level to verbose and the remote receiver filter to 3us. Play around with the floor, -90 is a good starting point, it will be clear from the logs when too much noise is getting through. 

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

Transmitter example to use in ESPHome, timing data is sent on GPIO32 by remote transmitter. ESPHome needs an API change so the transmitter start/stop is handled automatically, for now the mode change has to be called manually via lambda:

    sx127x:
      id: sx127x_id
      nss_pin: GPIO18
      rst_pin: GPIO23
      frequency: 433920000
      modulation: OOK
      pa_pin: BOOST
      pa_power: 17
      rx_start: false

    remote_transmitter:
      id: remote_transmitter_id
      pin: GPIO32
      carrier_duty_percent: 100%

    interval:
      - interval: 20sec
        then:
          - lambda: |-
              esphome::remote_base::RawTimings timings = {601, -613, 601, -613, 601, -613, 601, -613, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 405, -209, 405, -209, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209};
              auto call = id(remote_transmitter_id).transmit();
              call.get_data()->set_data(timings);
              call.set_send_times(2);
              id(sx127x_id).set_mode_tx();
              call.perform();
              id(sx127x_id).set_mode_standby();
      - interval: 30sec
        then:
          - lambda: |-
             id(sx127x_id)->set_mode_tx();
          - remote_transmitter.transmit_raw:
              code: [601, -613, 601, -613, 601, -613, 601, -613, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 405, -209, 405, -209, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209]
              repeat:
                times: 2
                wait_time: 1000us
          - lambda: |-
             id(sx127x_id)->set_mode_standby();

Example of reconfiguring radio at runtime. Any public function in the sx127x component can be called via lambda (see sx127x.h). Note settings are only applied after calling configure (except mode changes which are applied immediately): 

    sx127x:
      id: sx127x_id
      nss_pin: GPIO18
      rst_pin: GPIO23
      frequency: 433920000
      modulation: OOK
      pa_pin: BOOST
      pa_power: 17
      rx_start: false

    remote_transmitter:
      id: remote_transmitter_id
      pin: GPIO32
      carrier_duty_percent: 100%

    interval:
      - interval: 30sec
        then:
          - lambda: |-
             id(sx127x_id)->set_frequency(433920000);
             id(sx127x_id)->set_rx_bandwidth(sx127x::RX_BW_50_0);
             id(sx127x_id)->set_modulation(sx127x::MOD_OOK);
             id(sx127x_id)->configure();
             id(sx127x_id)->set_mode_tx();
          - remote_transmitter.transmit_raw:
              code: [601, -613, 601, -613, 601, -613, 601, -613, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 405, -209, 405, -209, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 405, -209]
              repeat:
                times: 3
                wait_time: 1000us
          - lambda: |-
             id(sx127x_id)->set_mode_standby();

Example of rx and tx on the same radio and the same gpio. Note remote transmitter setup needs to be called again before transmitting after a gpio mode change. Hopefully this work around can be removed in the future when the ESPHome platform library gets updated:

    sx127x:
      id: sx127x_id
      nss_pin: GPIO18
      rst_pin: GPIO23
      pa_pin: BOOST
      pa_power: 12
      frequency: 433920000
      rx_bandwidth: 50_0kHz
      rx_floor: -85
      rx_start: true
      modulation: OOK

    remote_transmitter:
      id: tx_id
      pin:
        id: tx_gpio_id
        number: GPIO32
        allow_other_uses: true
      carrier_duty_percent: 100%

    remote_receiver:
      id: rx_id
      pin:
        id: rx_gpio_id
        number: GPIO32
        allow_other_uses: true
      filter: 150us
      idle: 800us
      dump: raw

    interval:
      - interval: 20sec
        then:
          - lambda: |-
             id(sx127x_id)->set_mode_standby();
             id(tx_gpio_id)->pin_mode(gpio::FLAG_OUTPUT);
             id(tx_id)->setup();
             id(sx127x_id)->set_mode_tx();
          - remote_transmitter.transmit_raw:
              code: [614, -614, 600, -614, 614, -614, 601, -614, 405, -209, 405, -209, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 405, -209, 209, -405, 405, -209, 209, -405, 405, -209, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 405, -209, 405, -209, 405, -209, 405, -209, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 209, -405, 209, -405, 209, -405, 209, -405, 405, -209, 405, -209, 405, -209, 405, -209, 405, -209, 405, -209, 405, -209, 405, -209, 209, -405, 209, -405, 405, -209, 405, -209, 209, -405, 209, -405, 405, -209, 405, -209, 405, -209, 209, -405, 209, -405, 405, -209, 209, -405, 405, -209, 209, -405, 405, -209, 209, -405, 405, -209, 209, -405]
              repeat:
                times: 3
                wait_time: 1000us
          - lambda: |-
             id(sx127x_id)->set_mode_standby();
             id(rx_gpio_id)->pin_mode(gpio::FLAG_INPUT);
             id(sx127x_id)->set_mode_rx();
