
#include <SPI.h>      //https://github.com/arduino/ArduinoCore-avr/tree/master/libraries/SPI
#include <nRF24L01.h> //https://github.com/nRF24/RF24 
#include <RF24.h>     //https://github.com/nRF24/RF24

RF24 radio(9, 10); //set CE and CSN pins

const byte addresses[][6] = {"tx001", "rx002"};

boolean buttonState = 0;

//**************************************************************************************************************************
//structure size max 32 bytes **********************************************************************************************
//**************************************************************************************************************************
struct tx_data
{
  byte ch1;
  byte ch2;
  byte ch3;
  byte ch4;
  byte ch5;
  byte ch6;
  byte ch7;
  byte ch8;
};

tx_data rc_data; //Create a variable with the above structure

//**************************************************************************************************************************
//reset values ​​(min = 0, mid = 127, max = 255) *****************************************************************************
//**************************************************************************************************************************
void resetData()
{
  rc_data.ch1 = 127;     
  rc_data.ch2 = 127;
  rc_data.ch3 = 127;
  rc_data.ch4 = 127;
  rc_data.ch5 = 0;
  rc_data.ch6 = 0;
  rc_data.ch7 = 127;
  rc_data.ch8 = 127;
}

//**************************************************************************************************************************
//**************************************************************************************************************************
//**************************************************************************************************************************
void inputDriver()
{
/*
 * Read all analog inputs and map them to one byte value
 * Normal:    rc_data.ch1 = map( analogRead(A0), 0, 1023, 0, 255);
 * Reversed:  rc_data.ch1 = map( analogRead(A0), 0, 1023, 255, 0);
 * Convert the analog read value from 0 to 1023 into a byte value from 0 to 255
 */ 
  rc_data.ch1 = map(analogRead(A0), 0, 1023, 0, 255);
  rc_data.ch2 = map(analogRead(A1), 0, 1023, 0, 255);
  rc_data.ch3 = map(analogRead(A2), 0, 1023, 0, 255);
  rc_data.ch4 = map(analogRead(A3), 0, 1023, 0, 255);
  rc_data.ch5 = digitalRead(2);
  rc_data.ch6 = digitalRead(3);
  rc_data.ch7 = map(analogRead(A4), 333, 691, 7, 255); //Hitec Ranger throttle 333, 691, 7, 255
  rc_data.ch8 = map(analogRead(A5), 330, 694, 4, 255); //Hitec Ranger steering 330, 694, 4, 255
}

//**************************************************************************************************************************
//initial main settings ****************************************************************************************************
//**************************************************************************************************************************
void setup()
{
  Serial.begin(9600);

  pinMode(4, OUTPUT);

  resetData(); //reset each channel value
  
  //define the radio communication
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);  

  radio.openWritingPipe(addresses[1]);    // rx002
  radio.openReadingPipe(1, addresses[0]); // tx001
}

//**************************************************************************************************************************
//program loop *************************************************************************************************************
//**************************************************************************************************************************
void loop()
{
  delay(5); //5
  radio.stopListening();                  //set the module as transmitter
  radio.write(&rc_data, sizeof(tx_data)); //send the whole data from the structure to the receiver

  delay(5);
  radio.startListening(); //set the module as receiver
  while (!radio.available());
  radio.read(&buttonState, sizeof(buttonState));
  if (buttonState == HIGH)
  {
    digitalWrite(4, LOW);
  }
  else
  {
    digitalWrite(4, HIGH);
  }

  inputDriver();

//  Serial.println(rc_data.ch7); //print value ​​on a serial monitor
} //end program loop
