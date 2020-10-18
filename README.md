# RC_TX_nRF24L01_Telemetry_LED
It is a simple surface 5 channel RC transmitter for direct control of the nRF24L01 transceiver on the Arduino with ATmega328P for cars and boats.
It contains the telemetry of the monitored RX voltage by means of a flashing LED indication.

This RC transmitter works with RC receiver from my repository [RC_RX_nRF24L01_Telemetry_Motor_Driver_Servo](https://github.com/stanekTM/RC_RX_nRF24L01_Telemetry_Motor_Driver_Servo)

### Arduino pins:
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

### Code:
[RC_TX_nRF24L01_Telemetry_LED](https://github.com/stanekTM/RC_TX_nRF24L01_Telemetry_LED/blob/master/RC_TX_nRF24L01_Telemetry_LED.ino)

### Used libraries:
* <SPI.h>  Arduino standard library
* <RF24.h> https://github.com/nRF24/RF24
#
George StanekTM
