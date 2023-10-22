# RC TX nRF24L01 5ch
Surface 5 channel RC transmitter nRF24L01 with ATmega328P/16Mhz, Arduino Nano or Pro Mini for cars, boats and robots.
It contains the telemetry of the monitored RX voltage by means of a flashing LED indication.

Thanks to "Phil_G" http://www.singlechannel.co.uk for the calibration and reverse routine I used in the code.

This RC transmitter works with RC receiver from my repository [RX_nRF24L01_Telemetry_Motor_Servo](https://github.com/stanekTM/RX_nRF24L01_Telemetry_Motor_Servo)

## Function
* Normal mode = LED TX is lit
* TX battery voltage 1S LiPo (4.2V) < TX monitored voltage = TX LED flash at a interval of 0.2s
* RX battery voltage 1S LiPo (4.2V) < RX monitored voltage = TX, RX LEDs flash at a interval of 0.5s
* RX receiver off or signal loss = TX LED flash at a interval of 0.1s
### Calibration:
* Hold calibration button, switch transmitter TX on, still holding calibration button move all controls to extremes including auxilliary pots.
* Center all controls and aux pots.
* Release calibration button (saved to eeprom).
### Servo reversing:
* To reverse the desired channel, hold the joystick in the end position and turn on the TX transmitter (saved to eeprom).

## Arduino pins
```
A0 - pot 1
A1 - pot 2
A2 - pot 3
A3 - pot 4
A4 - pot 5

D2 - LED
D4 - calibration button (I had to add a 10k resistor -> VCC even when the internal INPUT_PULLUP is activated)
A7 - input TX battery

nRF24L01:
D9  - CE
D10 - CSN
D11 - MOSI
D12 - MISO
D13 - SCK
```

## Used libraries
* <RF24.h>   https://github.com/nRF24/RF24
* <EEPROM.h> Arduino standard library
* <SPI.h>    Arduino standard library
