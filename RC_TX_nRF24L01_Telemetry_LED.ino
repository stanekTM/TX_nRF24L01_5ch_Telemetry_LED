/*
 Thanks to "Phil_G" http://www.singlechannel.co.uk philg@talk21.com for the calibration and reverse routine I used in the code.
 Calibration:
 Hold calibration button, switch transmitter TX on, still holding calibration button move all controls to extremes including auxilliary pots. 
 Center all controls and aux pots.
 Release calibration button (saved to eeprom).
 Servo reversing:
 To reverse the desired channel, hold the joystick in the end position and turn on the TX transmitter (saved to eeprom).
*/
//************************************************************************************************************************************************************************
//************************************************************************************************************************************************************************
//************************************************************************************************************************************************************************
#include <SPI.h>      // Arduino standard library
#include <EEPROM.h>   // Arduino standard library
#include <RF24.h>     // https://github.com/nRF24/RF24
#include <nRF24L01.h>

// Config PPM settings
#define servoMid    1500
#define servoMin    1000
#define servoMax    2000
#define epa_p       500
#define epa_n      -500

//free pins
//pin                3
//pin                5
//pin                6
//pin                7
//pin                8
//pin                A5
//pin                A6 

//pins for pots, joysticks
//pot1               A0
//pot2               A1
//pot3               A2
//pot4               A3
//pot5               A4

//LED RX, TX battery and RF on/off
#define led          2

//calibration button
#define button_calib 4

//input TX battery
#define inTXbatt     A7

//pins for nRF24L01
#define CE           9
#define CSN          10

//hardware SPI
//----- MOSI         11 
//----- MISO         12 
//----- SCK          13 

RF24 radio(CE, CSN); //setup CE and CSN pins

const byte addresses[][6] = {"tx001", "rx002"};

//************************************************************************************************************************************************************************
//this structure defines the sent data in bytes (structure size max. 32 bytes), values ​​(servoMin = 1000us, servoMid = 1500us, servoMax = 2000us) *************************
//************************************************************************************************************************************************************************
struct packet
{
  unsigned int steering = servoMid;
  unsigned int throttle = servoMid;
  unsigned int ch3      = servoMid;
  unsigned int ch4      = servoMid;
  unsigned int ch5      = servoMid;
};
packet rc_data; //create a variable with the above structure

//************************************************************************************************************************************************************************
//this struct defines data, which are embedded inside the ACK payload ****************************************************************************************************
//************************************************************************************************************************************************************************
struct ackPayload
{
  float RXbatt;
};
ackPayload payload;

//************************************************************************************************************************************************************************
//read pots, joysticks ***************************************************************************************************************************************************
//************************************************************************************************************************************************************************
int raw, ch, calibrated = 1, pot_calib_min[] = {0, 0, 0, 0, 0}, pot_calib_max[] = {1023, 1023, 1023, 1023, 1023}, pot_calib_mid[] = {512, 512, 512, 512, 512};
int ppm[] = {1500, 1500, 1500, 1500, 1500};
byte reverse[] = {0, 0, 0, 0, 0};

void read_pots()
{
  rc_data.steering = ppm[0]; //A0
  rc_data.throttle = ppm[1]; //A1
  rc_data.ch3      = ppm[2]; //A2
  rc_data.ch4      = ppm[3]; //A3
  rc_data.ch5      = ppm[4]; //A4

  Serial.println(rc_data.steering); //print value ​​on a serial monitor  

  for (ch = 0; ch < 5; ch++)
  {
    raw = analogRead(ch);
    if (raw > pot_calib_mid[ch])
    ppm[ch] = map(raw, pot_calib_mid[ch], pot_calib_min[ch], 0, epa_p);
    else
    ppm[ch] = map(raw, pot_calib_max[ch], pot_calib_mid[ch], epa_n, 0);
  }
 
  // format the frame
  for (ch = 0; ch < 5; ch++)
  {
    ppm[ch] += servoMid;
    ppm[ch] = constrain(ppm[ch], servoMin, servoMax);
    if (reverse[ch] == 1) ppm[ch] = 3000 - ppm[ch];
  }
}

//************************************************************************************************************************************************************************
//calibrate pots, joysticks **********************************************************************************************************************************************
//************************************************************************************************************************************************************************
void calibrate_pots()
{ 
  while (digitalRead(button_calib) == 0)
  {
    calibrated = 0;
    for (int pot = 0; pot < 5; ++pot)
    {
      raw = analogRead(pot);
      if (raw > pot_calib_min[pot]) pot_calib_min[pot] = raw;
      if (raw < pot_calib_max[pot]) pot_calib_max[pot] = raw;
      pot_calib_mid[pot] = raw;  // save neutral pots, joysticks as button is released
    }
  }  //calibrate button released

  if (calibrated == 0)
  {
    for (ch = 0; ch < 5; ch++)
    {
      EEPROMWriteInt(ch * 6,     pot_calib_max[ch]); // eeprom locations  0,  6, 12, 18 (decimal)
      EEPROMWriteInt(ch * 6 + 2, pot_calib_mid[ch]); // eeprom locations  2,  8, 14, 20 (decimal)
      EEPROMWriteInt(ch * 6 + 4, pot_calib_min[ch]); // eeprom locations  4, 10, 16, 22 (decimal)
    }
    calibrated = 1;
//    digitalWrite(led, HIGH);
//    delay(30);
//    digitalWrite(led, LOW);
  }
  
  for (ch = 0; ch < 5; ch++)
  {
    pot_calib_max[ch] = EEPROMReadInt(ch * 6);     // eeprom locations  0,  6, 12, 18 (decimal)
    pot_calib_mid[ch] = EEPROMReadInt(ch * 6 + 2); // eeprom locations  2,  8, 14, 20 (decimal)
    pot_calib_min[ch] = EEPROMReadInt(ch * 6 + 4); // eeprom locations  4, 10, 16, 22 (decimal)
    reverse[ch] = EEPROM.read(ch + 24) & 1;        // eeprom locations 24, 25, 26, 27 (decimal)
  }
  
  // check for reversing, stick over on power-up
  for (ch = 0; ch < 5; ch++)
  {
    ppm[ch] = map(analogRead(ch), pot_calib_max[ch], pot_calib_min[ch], epa_n, epa_p);
    if (ppm[ch] > epa_p - 50 || ppm[ch] < epa_n + 50)
    {
      reverse[ch] ^= B00000001;
      EEPROM.write(24 + ch, reverse[ch]);
    }
  }
}

//************************************************************************************************************************************************************************
//this function will write a 2 byte integer to the eeprom at the specified address and address + 1 ***********************************************************************
//************************************************************************************************************************************************************************
void EEPROMWriteInt(int p_address, int p_value)
{
  byte lowByte = p_value % 256;
  byte highByte = p_value / 256;
  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

//************************************************************************************************************************************************************************
//this function will read a 2 byte integer from the eeprom at the specified address and address + 1
//************************************************************************************************************************************************************************
unsigned int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);
  return lowByte + highByte * 256;
}

//************************************************************************************************************************************************************************
//initial main settings **************************************************************************************************************************************************
//************************************************************************************************************************************************************************
void setup()
{ 
  Serial.begin(9600);

  calibrate_pots();

  pinMode(led, OUTPUT);
  pinMode(inTXbatt, INPUT);
  pinMode(button_calib, INPUT_PULLUP);
  
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
                                                            
  read_pots();
  
  battery_voltage();

} //end program loop

//************************************************************************************************************************************************************************
//after losing RF data or turning off the RX, gain time and the LED flashing *********************************************************************************************
//************************************************************************************************************************************************************************
unsigned long lastRxTime = 0;

void receive_time()
{
  if(millis() >= lastRxTime + 1000) //1000 (1second)
  {
    RFoff_indication();
  }
}

//************************************************************************************************************************************************************************
//send and receive data **************************************************************************************************************************************************
//************************************************************************************************************************************************************************
void send_and_receive_data()
{
  if (radio.write(&rc_data, sizeof(packet)))
  {
    if (radio.isAckPayloadAvailable())
    {
      radio.read(&payload, sizeof(ackPayload));
      
      lastRxTime = millis(); //at this moment we have received the data 
      RXbatt_indication();                                     
    }                              
  } 
}

//************************************************************************************************************************************************************************
//input measurement TX battery voltage 1S LiPo (4.2V) < 3.3V = LED flash at a interval of 200ms. Battery OK = LED TX is lit **********************************************
//************************************************************************************************************************************************************************
unsigned long ledTime = 0;
int ledState, detect;
float TXbatt;

void battery_voltage()
{ 
  //----------------------------- TX battery --
  TXbatt = analogRead(inTXbatt) * (4.2 / 1023);

  //--------------- monitored voltage
  detect = TXbatt <= 3.3;

  if (detect)
  {
    TXbatt_indication();
  }
  
//  Serial.println(TXbatt); //print value ​​on a serial monitor 
}

//-----------------------------------------------------------
void TXbatt_indication()
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

//************************************************************************************************************************************************************************
//after receiving RF data, the monitored RX battery is activated *********************************************************************************************************
//RX battery voltage 1S LiPo (4.2V) < 3.3V = LEDs TX, RX flash at a interval of 500ms. Battery OK = LEDs TX, RX is lit ***************************************************
//************************************************************************************************************************************************************************
void RXbatt_indication()
{ 
  //------------------------ monitored voltage
  detect = payload.RXbatt <= 3.3;
  
  if (millis() >= ledTime + 500) //1000 (1second)
  {
    ledTime = millis();
    
    if (ledState >= !detect + HIGH)
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
//when TX is switched on and RX is switched off, or after the loss of RF data = LED TX flash at a interval of 100 ms. Normal mode = LED TX is lit ************************
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
 
