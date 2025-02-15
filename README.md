ESPHome SX127x driver.

SX127x component configures SX1276, SX1277, SX1278 or SX1279 hardware for use in ESPHome. Tested with the LILYGO LoRa32 V2.1_1.6 / T3 V1.6.1 board.

Docs can be found here:

https://deploy-preview-4278--esphome.netlify.app/components/sx127x

ESPHome PR:

https://github.com/esphome/esphome/pull/7490

To use the PR:

	external_components:
	  - source: github://pr#7490
	    components: [ sx127x ]

To use this repo:

	external_components:
	  - source: github://swoboda1337/sx127x-esphome@main
	    components: [ sx127x ]
