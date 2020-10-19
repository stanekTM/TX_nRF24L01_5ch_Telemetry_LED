# RC_TX_nRF24L01_Telemetry_LED
It is a simple surface 5 channel RC transmitter for direct control of the nRF24L01 transceiver on the Arduino with ATmega328P for cars and boats.
It contains the telemetry of the monitored RX voltage by means of a flashing LED indication.

This RC transmitter works with RC receiver from my repository [RC_RX_nRF24L01_Telemetry_Motor_Driver_Servo](https://github.com/stanekTM/RC_RX_nRF24L01_Telemetry_Motor_Driver_Servo)

## Function:
### Calibration:
* Hold calibration button (pin D4), switch transmitter TX on, still holding calibration button move all controls to extremes including auxilliary pots.
* Center all controls and aux pots.
* Release calibration button.
* Do not switch transmitter TX off during calibration, the calibration values cant be saved if the set has been switched off!
#
### Servo reversing:
* By holding joysticks over on power up (saved to eeprom).

## Arduino pins:
* A0 - joystick 1
* A1 - joystick 2
* A2 - potentiometer 3
* A3 - potentiometer 4
* A4 - potentiometer 5
# 
* D2 - LED
* A7 - input TX battery
#
nRF24L01:
* D9  - CE
* D10 - CSN
* D11 - MOSI
* D12 - MISO
* D13 - SCK

## Code:
[RC_TX_nRF24L01_Telemetry_LED](https://github.com/stanekTM/RC_TX_nRF24L01_Telemetry_LED/blob/master/RC_TX_nRF24L01_Telemetry_LED/RC_TX_nRF24L01_Telemetry_LED.ino)

[RC_TX_nRF24L01_Telemetry_LED_Calibration](https://github.com/stanekTM/RC_TX_nRF24L01_Telemetry_LED/blob/master/RC_TX_nRF24L01_Telemetry_LED_Calibration/RC_TX_nRF24L01_Telemetry_LED_Calibration.ino)

## Used libraries:
* <SPI.h>  Arduino standard library
* <RF24.h> https://github.com/nRF24/RF24
#
George StanekTM
