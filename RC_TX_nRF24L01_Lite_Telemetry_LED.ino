
#include <RF24.h>     //https://github.com/nRF24/RF24
#include <nRF24L01.h> //https://github.com/nRF24/RF24
#include <SPI.h>      //https://github.com/arduino/ArduinoCore-avr/tree/master/libraries/SPI 

//free pins
//pin            3
//pin            4
//pin            5
//pin            6
//pin            7
//pin            8
//pin            A5
//pin            A6

//pins for driver
#define driv1    A0
#define driv2    A1
#define driv3    A2
#define driv4    A3
#define driv5    A4

//LED RX, TX battery and RF on/off
#define led      2

//input TX battery
#define inTxBat  A7

//pins for nRF24L01
#define CE       9
#define CSN      10

//hardware SPI
//----- MOSI     11 
//----- MISO     12 
//----- SCK      13 

RF24 radio(CE, CSN); //setup CE and CSN pins

const byte addresses[][6] = {"tx001", "rx002"};

//************************************************************************************************************************************************************************
//this structure defines the sent data in bytes (structure size max. 32 bytes), values ​​(min = 1000us, mid = 1500us, max = 2000us) ****************************************
//************************************************************************************************************************************************************************
struct packet
{
  unsigned int steering = 1500;
  unsigned int throttle = 1500;
  unsigned int ch3      = 1500;
  unsigned int ch4      = 1500;
  unsigned int ch5      = 1500;
};
packet rc_data; //create a variable with the above structure

//************************************************************************************************************************************************************************
//this struct defines data, which are embedded inside the ACK payload ****************************************************************************************************
//************************************************************************************************************************************************************************
struct ackPayload
{
  float RxBat;
};
ackPayload payload;

//************************************************************************************************************************************************************************
//inputs of control joysticks and switches *******************************************************************************************************************************
//************************************************************************************************************************************************************************
int valmin = 342; //joystick path compensation for 1000us to 2000us Hitec Ranger 2AM
int valmax = 1023 - valmin;

void inputDriver()
{
/*
 * Read all analog inputs and map them to one byte value
 * Normal:    rc_data.ch1 = map(analogRead(A0), 0, 1023, 1000, 2000);
 * Reversed:  rc_data.ch1 = map(analogRead(A0), 0, 1023, 2000, 1000);
 * Convert the analog read value from 0 to 1023 into a byte value from 1000us to 2000us
 */ 

  rc_data.steering = map(analogRead(driv1), valmin, valmax, 1000, 2000);
  rc_data.steering = constrain(rc_data.steering, 1000, 2000);
  
  rc_data.throttle = map(analogRead(driv2), valmin, valmax, 1000, 2000);
  rc_data.throttle = constrain(rc_data.throttle, 1000, 2000);
 
  rc_data.ch3 = map(analogRead(driv3), 0, 1023, 1000, 2000);
  rc_data.ch3 = constrain(rc_data.ch3, 1000, 2000); 
  rc_data.ch4 = map(analogRead(driv4), 0, 1023, 1000, 2000);
  rc_data.ch4 = constrain(rc_data.ch4, 1000, 2000);
  rc_data.ch5 = map(analogRead(driv5), 0, 1023, 1000, 2000);
  rc_data.ch5 = constrain(rc_data.ch5, 1000, 2000);

//  Serial.println(rc_data.throttle); //print value ​​on a serial monitor  
}

//************************************************************************************************************************************************************************
//initial main settings **************************************************************************************************************************************************
//************************************************************************************************************************************************************************
void setup()
{ 
  Serial.begin(9600);

  pinMode(led, OUTPUT); //LED RX, TX battery and RF on/off
  pinMode(inTxBat, INPUT); //input TX battery
  
  //define the radio communication
  radio.begin();
  
  radio.setAutoAck(true);          //ensure autoACK is enabled (default true)
  radio.enableAckPayload();        //enable custom ack payloads on the acknowledge packets
  radio.enableDynamicPayloads();   //enable dynamically-sized payloads
  radio.setRetries(5, 5);          //set the number and delay of retries on failed submit (max. 15 x 250us delay (blocking !), max. 15 retries)

  radio.setChannel(76);            //which RF channel to communicate on (0-125, 2.4Ghz + default 76 = 2.476Ghz) 
  radio.setDataRate(RF24_250KBPS); //RF24_250KBPS (fails for units without +), RF24_1MBPS, RF24_2MBPS
  radio.setPALevel(RF24_PA_MIN);   //RF24_PA_MIN (-18dBm), RF24_PA_LOW (-12dBm), RF24_PA_HIGH (-6dbm), RF24_PA_MAX (0dBm)

  radio.stopListening();           //set the module as transmitter. Stop listening for incoming messages, and switch to transmit mode
  
  radio.openWritingPipe(addresses[1]);    //open a pipe for writing via byte array. Call "stopListening" first
  radio.openReadingPipe(1, addresses[0]); //open all the required reading pipes
}

//************************************************************************************************************************************************************************
//program loop ***********************************************************************************************************************************************************
//************************************************************************************************************************************************************************
void loop()
{ 
  receive_time();
  send_and_receive_data();
                                                            
  inputDriver();

  battery_voltage();

} //end program loop

//************************************************************************************************************************************************************************
//after losing RF data or turning off the RX, gain time and the LED activates flashing ***********************************************************************************
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
  if (radio.write(&rc_data, sizeof(packet)))    //"write" send all data from the structure and check if the transfer was successful
                                                //"rc_data" pointer to the data to be sent
                                                //"packet" number of bytes to be sent
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
//input TX battery with undervoltage detection ***************************************************************************************************************************
//************************************************************************************************************************************************************************
float TxBat = 0;

void battery_voltage()
{
  //---------------------------- TX battery ----- monitored voltage
  TxBat = analogRead(inTxBat) * (4.5 / 1023.0) <= 3.3;
  
  if (TxBat)
  {
    TxBat_indication();
  }

//  Serial.println(TxBat); //print value ​​on a serial monitor 
}

//************************************************************************************************************************************************************************
//after receiving the RF data, it activates of the monitored RX battery by means of a flashing LED indication ************************************************************
//************************************************************************************************************************************************************************
int ledState;
unsigned long ledTime = 0;

void RFon_indication()
{
  if (millis() >= ledTime + 500) //1000 (1second)
  {
    ledTime = millis();
    
    if (ledState >= !payload.RxBat + HIGH)
    {
      ledState = LOW;
    }
    else
    {
      ledState = HIGH;
    }   
    digitalWrite(led, ledState);
      
//    digitalWrite(led, payload.RxBat); //LED indication without flashing
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
    digitalWrite(led, ledState);
  }
}

//************************************************************************************************************************************************************************
//specific flashing of the TX transmitter battery voltage drop LED *******************************************************************************************************
//************************************************************************************************************************************************************************
void TxBat_indication()
{
  if (millis() >= ledTime + 200) //1000 (1second)
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
    digitalWrite(led, ledState);
  }
}
    
