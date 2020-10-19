# RC_TX_nRF24L01_Telemetry_LED
It is a simple surface 4-5 channel RC transmitter for direct control of the nRF24L01 transceiver on the Arduino with ATmega328P for cars and boats.
It contains the telemetry of the monitored RX voltage by means of a flashing LED indication.

This RC transmitter works with RC receiver from my repository [RC_RX_nRF24L01_Telemetry_Motor_Driver_Servo](https://github.com/stanekTM/RC_RX_nRF24L01_Telemetry_Motor_Driver_Servo)

## Function:
### Calibration:
* Hold calibration button, switch transmitter TX on, still holding calibration button move all controls to extremes including auxilliary pots.
* Center all controls and aux pots.
* Release calibration button.
* Do not switch transmitter TX off during calibration, the calibration values cant be saved if the set has been switched off!
#
### Servo reversing:
* By holding joysticks over on power up (saved to eeprom).

## Arduino pins:
* A0 - pot 1
* A1 - pot 2
* A2 - pot 3
* A3 - pot 4
* A4 - pot 5 (only version without calibration)
# 
* D2 - LED
* D4 - calibration button (calibration version only)
* A7 - input TX battery
#
nRF24L01:
* D9  - CE
* D10 - CSN
* D11 - MOSI
* D12 - MISO
* D13 - SCK

## Code:
No semi-automatic calibration, settings are made in code
[RC_TX_nRF24L01_Telemetry_LED](https://github.com/stanekTM/RC_TX_nRF24L01_Telemetry_LED/blob/master/RC_TX_nRF24L01_Telemetry_LED/RC_TX_nRF24L01_Telemetry_LED.ino)

Semi-automatic calibration
[RC_TX_nRF24L01_Telemetry_LED_Calibration](https://github.com/stanekTM/RC_TX_nRF24L01_Telemetry_LED/blob/master/RC_TX_nRF24L01_Telemetry_LED_Calibration/RC_TX_nRF24L01_Telemetry_LED_Calibration.ino)

## Used libraries:
* <SPI.h>  Arduino standard library
* <RF24.h> https://github.com/nRF24/RF24
#
George StanekTM
