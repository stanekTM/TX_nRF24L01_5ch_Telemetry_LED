
#include <SPI.h>      //https://github.com/arduino/ArduinoCore-avr/tree/master/libraries/SPI
#include <nRF24L01.h> //https://github.com/nRF24/RF24 
#include <RF24.h>     //https://github.com/nRF24/RF24

//pins for driver
#define driv1    A0
#define driv2    A1
#define driv3    A2
#define driv4    A3
#define driv5    2
#define driv6    3
#define driv7    A4
#define driv8    A5

//LED telemetry RX vcc/RF data off
#define ledTX    4

//pins for nRF24L01
#define CE       9
#define CSN      10
//***** MOSI     11
//***** MISO     12
//***** SCK      13

RF24 radio(CE, CSN); //set CE and CSN pins

const byte addresses[][6] = {"tx001", "rx002"};

//************************************************************************************************************************************************************************
//structure size max 32 bytes ********************************************************************************************************************************************
//************************************************************************************************************************************************************************
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

//************************************************************************************************************************************************************************
//this struct defines data, which are embedded inside the ACK payload ****************************************************************************************************
//************************************************************************************************************************************************************************
struct ackPayload
{
  float RXvcc; //led telemetry
};
ackPayload payload;

//************************************************************************************************************************************************************************
//reset values ​​(min = 0, mid = 127, max = 255) ***************************************************************************************************************************
//************************************************************************************************************************************************************************
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

//************************************************************************************************************************************************************************
//inputs of control joysticks and switches *******************************************************************************************************************************
//************************************************************************************************************************************************************************
void inputDriver()
{
/*
 * Read all analog inputs and map them to one byte value
 * Normal:    rc_data.ch1 = map(analogRead(A0), 0, 1023, 0, 255);
 * Reversed:  rc_data.ch1 = map(analogRead(A0), 0, 1023, 255, 0);
 * Convert the analog read value from 0 to 1023 into a byte value from 0 to 255
 */ 
  rc_data.ch1 = map(analogRead(driv1),  0, 1023, 0, 255);
  rc_data.ch2 = map(analogRead(driv2),  0, 1023, 0, 255);
  rc_data.ch3 = map(analogRead(driv3),  0, 1023, 0, 255);
  rc_data.ch4 = map(analogRead(driv4),  0, 1023, 0, 255);
  rc_data.ch5 =    digitalRead(driv5);
  rc_data.ch6 =    digitalRead(driv6);
//steering, throttle --------------------------------------------------------------------------- 
  rc_data.ch7 = map(analogRead(driv7), 333, 690, 0, 255); //steering (333, 690) Hitec Ranger AM
  rc_data.ch8 = map(analogRead(driv8), 333, 690, 0, 255); //throttle (333, 690) Hitec Ranger AM 

//  Serial.println(rc_data.ch8); //print value ​​on a serial monitor  
}

//************************************************************************************************************************************************************************
//initial main settings **************************************************************************************************************************************************
//************************************************************************************************************************************************************************
void setup()
{ 
  Serial.begin(9600);

  pinMode(ledTX, OUTPUT); //LED telemetry RX vcc/RF data off

  resetData(); //reset each channel value
  
  //define the radio communication
  radio.begin();
  
  radio.setAutoAck(1);             //ensure autoACK is enabled (address 1, tx001)
  radio.enableAckPayload();        //enable custom ack payloads on the acknowledge packets
  radio.enableDynamicPayloads();   //enable dynamically-sized payloads
  radio.setRetries(5, 5);          //set the number and delay of retries on failed submit (max. 15 x 250us delay (blocking !), max. 15 retries)

  radio.setChannel(76);            //which RF channel to communicate on (0-125, default 76) 
  radio.setDataRate(RF24_250KBPS); //RF24_250KBPS (fails for units without +), RF24_1MBPS, RF24_2MBPS
  radio.setPALevel(RF24_PA_MIN);   //RF24_PA_MIN (-18dBm), RF24_PA_LOW (-12dBm), RF24_PA_HIGH (-6dbm), RF24_PA_MAX (0dBm)

  radio.stopListening();           //set the module as transmitter. Stop listening for incoming messages, and switch to transmit mode
  
  radio.openWritingPipe(addresses[1]);    //(address 2, rx002) open a pipe for writing via byte array. Call "stopListening" first
  radio.openReadingPipe(1, addresses[0]); //(address 1, tx001) open all the required reading pipes, and then call "startListening"
                                          //which number pipe to open (0-5)
                                          //the 24, 32 or 40 bit address of the pipe to open
}

//************************************************************************************************************************************************************************
//program loop ***********************************************************************************************************************************************************
//************************************************************************************************************************************************************************
void loop()
{ 
  receive_time();
  send_and_receive_data();
                                                            
  inputDriver();

} //end program loop

//************************************************************************************************************************************************************************
//get time and reset data ************************************************************************************************************************************************
//************************************************************************************************************************************************************************
unsigned long lastReceiveTime = 0;

void receive_time()
{
  //check whether we keep receving data, or we have a connection between the two modules
  if(millis() >= lastReceiveTime + 1000) //1000 (1second)
  {
    RFoff_indication();
  }
}

//************************************************************************************************************************************************************************
//send and receive data **************************************************************************************************************************************************
//************************************************************************************************************************************************************************
void send_and_receive_data()
{
  if (radio.write(&rc_data, sizeof(tx_data)))   //"write" send all data from the structure and check if the transfer was successful
                                                //"rc_data" pointer to the data to be sent
                                                //"tx_data" number of bytes to be sent
    {                                                                                      
    if (radio.isAckPayloadAvailable())          //determine if an ack payload was received in the most recent call to "write". The regular "available" can also be used
    {
      radio.read(&payload, sizeof(ackPayload)); //"read" retrieve the ack payload
                                                //"payload" pointer to a buffer where the data should be written
                                                //"ackPayload" maximum number of bytes to read into the buffer
      lastReceiveTime = millis();               //at this moment we have received the data
      RFon_indication();                                          
    }                              
  } 
}

//************************************************************************************************************************************************************************
//after receiving the RF data, it activates telemetry with the detection of the set undervoltage RX by means of the LED flashing indication ******************************
//************************************************************************************************************************************************************************
unsigned long ledTime = 0;
int ledState;

void RFon_indication()
{
  if (millis() >= ledTime + 500) //1000 (1second)
  {
    ledTime = millis();
    
    if (ledState >= !payload.RXvcc + HIGH)
    {
      ledState = LOW;
    }
    else
    {
      ledState = HIGH;
    }   
    digitalWrite(ledTX, ledState);
      
//    digitalWrite(ledTX, payload.RXvcc); //LED indication without flashing
  }
}

//************************************************************************************************************************************************************************
//when TX is switched on and RX is switched off, or after the loss of RF data, the LED activates flashing ****************************************************************
//************************************************************************************************************************************************************************
void RFoff_indication()
{
  if (millis() >= ledTime + 100) //1000 (1second)
  {
    ledTime = millis();
    
    if (ledState)
    {
      ledState = LOW;
    }
    else
    {
      ledState = HIGH;
    }   
    digitalWrite(ledTX, ledState);
  }
}
  
