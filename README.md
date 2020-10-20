# RC_TX_nRF24L01_Telemetry_LED
It is a simple surface 5 channel RC transmitter for direct control of the nRF24L01 transceiver on the Arduino with ATmega328P for cars and boats.
It contains the telemetry of the monitored RX voltage by means of a flashing LED indication.

Thanks to "Phil_G" http://www.singlechannel.co.uk for the calibration and reverse routine I used in the code.

This RC transmitter works with RC receiver from my repository [RC_RX_nRF24L01_Telemetry_Motor_Driver_Servo](https://github.com/stanekTM/RC_RX_nRF24L01_Telemetry_Motor_Driver_Servo)

## Function:
### Calibration:
* Hold calibration button, switch transmitter TX on, still holding calibration button move all controls to extremes including auxilliary pots.
* Center all controls and aux pots.
* Release calibration button (saved to eeprom).
#
### Servo reversing:
* To reverse the desired channel, hold the joystick in the end position and turn on the TX transmitter (saved to eeprom).

## Arduino pins:
* A0 - pot 1
* A1 - pot 2
* A2 - pot 3
* A3 - pot 4
* A4 - pot 5
# 
* D2 - LED
* D4 - calibration button
* A7 - input TX battery
#
nRF24L01:
* D9  - CE
* D10 - CSN
* D11 - MOSI
* D12 - MISO
* D13 - SCK

## Code:
[RC_TX_nRF24L01_Telemetry_LED](https://github.com/stanekTM/RC_TX_nRF24L01_Telemetry_LED/blob/master/RC_TX_nRF24L01_Telemetry_LED.ino)

## Used libraries:
* <SPI.h>  Arduino standard library
* <RF24.h> https://github.com/nRF24/RF24
#
George StanekTM
