ESPHome SX127x driver. Tested with the LILYGO LoRa32 V2.1_1.6 board.

SX127x component configures SX1276, SX1277, SX1278 or SX1279 hardware and provides demodulated data on a GPIO for use in ESPHome. 

Example yaml to use in esphome device config:
    
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
      bandwidth: 50_0kHz
      modulation: OOK

    remote_receiver:
      pin: GPIO32
      filter: 255us
      idle: 2000us
      ook_floor: -90
      buffer_size: 100000b
      memory_blocks: 8
