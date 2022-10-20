
//************************************************************************************************************************************************************************
// Add communication nRF24L01P. Fixed RF channel, fixed address.
// Support for Arduino-based receivers and RF24 libraries from this repository https://github.com/stanekTM/RX_nRF24L01_Telemetry_Motor_Servo
// Thanks to "Phil_G" http://www.singlechannel.co.uk philg@talk21.com for the calibration and reverse routine I used in the code.
// Calibration:
// Hold calibration button, switch transmitter TX on, still holding calibration button move all controls to extremes including auxilliary pots.
// Center all controls and aux pots.
// Release calibration button (saved to eeprom).
// Servo reversing:
// To reverse the desired channel, hold the joystick in the end position and turn on the TX transmitter (saved to eeprom).
//************************************************************************************************************************************************************************


#include <RF24.h>     // https://github.com/nRF24/RF24
#include <SPI.h>      // Arduino standard library
#include <EEPROM.h>   // Arduino standard library


//setting RF channels address (5 bytes number or character)
const byte address[] = "jirka";

//RF communication channel settings (0-125, 2.4Ghz + 76 = 2.476Ghz)
#define RADIO_CHANNEL         76

//TX battery voltage settings
#define TX_BATTERY_VOLTAGE    4.2
#define TX_MONITORED_VOLTAGE  3.3

//RX voltage monitoring settings
#define RX_MONITORED_VOLTAGE  3.49

//setting the control range value
#define MIN_CONTROL_VAL       1000
#define MID_CONTROL_VAL       1500
#define MAX_CONTROL_VAL       2000
#define EPA_POSITIVE          500
#define EPA_NEGATIVE         -500

//free pins
//pin                     0
//pin                     1
//pin                     3
//pin                     5
//pin                     6
//pin                     7
//pin                     8
//pin                     A5
//pin                     A6

//pins for pots, joysticks
//pot1                    A0
//pot2                    A1
//pot3                    A2
//pot4                    A3
//pot5                    A4

//LED RX, TX battery and RF on/off
#define PIN_LED           2

//calibration button (I had to add a 10k resistor -> VCC even when the internal INPUT_PULLUP is activated)
#define PIN_BUTTON_CALIB  4

//input TX battery
#define PIN_TX_BATTERY    A7

//pins for nRF24L01
#define PIN_CE            9
#define PIN_CSN           10

//hardware SPI
//----- MOSI              11 
//----- MISO              12 
//----- SCK               13 

//setting of CE and CSN pins
RF24 radio(PIN_CE, PIN_CSN);

//************************************************************************************************************************************************************************
//this structure defines the sent data in bytes **************************************************************************************************************************
//************************************************************************************************************************************************************************
struct rc_packet_size
{
  unsigned int ch1;
  unsigned int ch2;
  unsigned int ch3;
  unsigned int ch4;
  unsigned int ch5;
};
rc_packet_size rc_packet; //create a variable with the above structure

//************************************************************************************************************************************************************************
//this struct defines data, which are embedded inside the ACK payload ****************************************************************************************************
//************************************************************************************************************************************************************************
struct telemetry_packet_size
{
  byte rssi;     //not used yet
  float RX_batt_A1;
  float RX_batt_A2; //not used yet
};
telemetry_packet_size telemetry_packet;

//************************************************************************************************************************************************************************
//read pots, joysticks ***************************************************************************************************************************************************
//************************************************************************************************************************************************************************
int raw_pots, ch, calibrated = 1, pot_calib_min[] = {0, 0, 0, 0, 0}, pot_calib_max[] = {1023, 1023, 1023, 1023, 1023}, pot_calib_mid[] = {512, 512, 512, 512, 512};
int pots_value[] = {1500, 1500, 1500, 1500, 1500};
byte reverse[] = {0, 0, 0, 0, 0};

void read_pots()
{
  for (ch = 0; ch < 5; ch++)
  {
    raw_pots = analogRead(ch);
    if (raw_pots > pot_calib_mid[ch])
    pots_value[ch] = map(raw_pots, pot_calib_mid[ch], pot_calib_min[ch], 0, EPA_POSITIVE);
    else
    pots_value[ch] = map(raw_pots, pot_calib_max[ch], pot_calib_mid[ch], EPA_NEGATIVE, 0);
  }
  
  // format the frame
  for (ch = 0; ch < 5; ch++)
  {
    pots_value[ch] += MID_CONTROL_VAL;
    pots_value[ch] = constrain(pots_value[ch], MIN_CONTROL_VAL, MAX_CONTROL_VAL);
    if (reverse[ch] == 1) pots_value[ch] = 3000 - pots_value[ch];
  }
  
  rc_packet.ch1 = pots_value[0]; //A0
  rc_packet.ch2 = pots_value[1]; //A1
  rc_packet.ch3 = pots_value[2]; //A2
  rc_packet.ch4 = pots_value[3]; //A3
  rc_packet.ch5 = pots_value[4]; //A4
  
  //Serial.println(rc_packet.ch1); //print value on a serial monitor
}

//************************************************************************************************************************************************************************
//calibrate pots, joysticks **********************************************************************************************************************************************
//************************************************************************************************************************************************************************
void calibrate_pots()
{
  while (digitalRead(PIN_BUTTON_CALIB) == 0)
  {
    calibrated = 0;
    for (int pot = 0; pot < 5; ++pot)
    {
      raw_pots = analogRead(pot);
      if (raw_pots > pot_calib_min[pot]) pot_calib_min[pot] = raw_pots;
      if (raw_pots < pot_calib_max[pot]) pot_calib_max[pot] = raw_pots;
      pot_calib_mid[pot] = raw_pots;  //save neutral pots, joysticks as button is released
    }
  }   //calibrate button released
  
  if (calibrated == 0)
  {
    for (ch = 0; ch < 5; ch++)
    {
      EEPROMWriteInt(ch * 6,     pot_calib_max[ch]); //eeprom locations  0,  6, 12, 18 (decimal)
      EEPROMWriteInt(ch * 6 + 2, pot_calib_mid[ch]); //eeprom locations  2,  8, 14, 20 (decimal)
      EEPROMWriteInt(ch * 6 + 4, pot_calib_min[ch]); //eeprom locations  4, 10, 16, 22 (decimal)
    }
    calibrated = 1;
  }
  
  for (ch = 0; ch < 5; ch++)
  {
    pot_calib_max[ch] = EEPROMReadInt(ch * 6);     //eeprom locations  0,  6, 12, 18 (decimal)
    pot_calib_mid[ch] = EEPROMReadInt(ch * 6 + 2); //eeprom locations  2,  8, 14, 20 (decimal)
    pot_calib_min[ch] = EEPROMReadInt(ch * 6 + 4); //eeprom locations  4, 10, 16, 22 (decimal)
    reverse[ch] = EEPROM.read(ch + 30) & 1;        //eeprom locations 30, 31, 32, 33 (decimal), ch * 6 = 30
  }
  
  //check for reversing, stick over on power-up
  for (ch = 0; ch < 5; ch++)
  {
    pots_value[ch] = map(analogRead(ch), pot_calib_max[ch], pot_calib_min[ch], EPA_NEGATIVE, EPA_POSITIVE);
    if (pots_value[ch] > EPA_POSITIVE - 50 || pots_value[ch] < EPA_NEGATIVE + 50)
    {
      reverse[ch] ^= B00000001;
      EEPROM.write(30 + ch, reverse[ch]); //ch * 6 = 30
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
//this function will read a 2 byte integer from the eeprom at the specified address and address + 1 **********************************************************************
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
const byte invert_address = ~address[5]; //invert bits for reading so that telemetry packets have a different address

void setup()
{
  //Serial.begin(9600); //print value on a serial monitor
  
  calibrate_pots();
  
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TX_BATTERY, INPUT);
  pinMode(PIN_BUTTON_CALIB, INPUT_PULLUP);
  
  //define the radio communication
  radio.begin();
  radio.setAutoAck(true);          //ensure autoACK is enabled (default true)
  radio.enableAckPayload();        //enable Ack dynamic payloads. This only works on pipes 0&1 by default
  radio.enableDynamicPayloads();   //enable dynamic payloads on all pipes
  radio.setRetries(5, 5);          //set the number and delay of retries on failed submit (max. 15 x 250us delay (blocking !), max. 15 retries)
  
  radio.setChannel(RADIO_CHANNEL); //which RF channel to communicate on (0-125, 2.4Ghz + 76 = 2.476Ghz)
  radio.setDataRate(RF24_250KBPS); //RF24_250KBPS (fails for units without +), RF24_1MBPS, RF24_2MBPS
  radio.setPALevel(RF24_PA_MIN);   //RF24_PA_MIN (-18dBm), RF24_PA_LOW (-12dBm), RF24_PA_HIGH (-6dbm), RF24_PA_MAX (0dBm)
  
  radio.stopListening();           //set the module as transmitter. Stop listening for incoming messages, and switch to transmit mode
  
  radio.openWritingPipe(address);           //open the writing pipe0 (RX_ADDR_P0 + TX_ADDR), but first call "stopListening"
  radio.openReadingPipe(1, invert_address); //open the reading pipe1 (RX_ADDR_P1)
}

//************************************************************************************************************************************************************************
//program loop ***********************************************************************************************************************************************************
//************************************************************************************************************************************************************************
void loop()
{
  receive_time();
  send_and_receive_data();
  read_pots();
  TX_batt_check();
}

//************************************************************************************************************************************************************************
//after losing RF data or turning off the RX, gain time and the LED flashing *********************************************************************************************
//************************************************************************************************************************************************************************
unsigned long lastRxTime = 0;

void receive_time()
{
  if(millis() >= lastRxTime + 1000) //1s
  {
    RFoff_check();
  }
}

//************************************************************************************************************************************************************************
//send and receive data **************************************************************************************************************************************************
//************************************************************************************************************************************************************************
void send_and_receive_data()
{
  if (radio.write(&rc_packet, sizeof(rc_packet_size)))
  {
    if (radio.isAckPayloadAvailable())
    {
      radio.read(&telemetry_packet, sizeof(telemetry_packet_size));
      
      RX_batt_check();
      lastRxTime = millis(); //at this moment we have received the data
    }
  }
}

//************************************************************************************************************************************************************************
//input measurement TX_BATTERY_VOLTAGE < TX_MONITORED_VOLTAGE = LED flash at a interval of 0.2s. Battery OK = LED TX is lit **********************************************
//************************************************************************************************************************************************************************
float val_TX_battery;
unsigned long ledTime = 0;
bool ledState;

void TX_batt_check()
{
  val_TX_battery = analogRead(PIN_TX_BATTERY) * (TX_BATTERY_VOLTAGE / 1023);
  
  if (val_TX_battery <= TX_MONITORED_VOLTAGE)
  {
    if (millis() >= ledTime + 200)
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
      digitalWrite(PIN_LED, ledState);
    }
  }
  //Serial.println(val_TX_battery); //print value on a serial monitor
}

//************************************************************************************************************************************************************************
//after receiving RF data, the monitored RX battery is activated *********************************************************************************************************
//RX battery voltage(payload.RXbatt) < RX_MONITORED_VOLTAGE = LEDs TX, RX flash at a interval of 0.5s. Battery OK = LEDs TX, RX is lit ***********************************
//************************************************************************************************************************************************************************
bool detect;

void RX_batt_check()
{
  detect = telemetry_packet.RX_batt_A1 <= RX_MONITORED_VOLTAGE;
  
  if (millis() >= ledTime + 500)
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
    digitalWrite(PIN_LED, ledState);
  }
  //Serial.println(telemetry_packet.RX_batt_A1); //print value ​​on a serial monitor
}

//************************************************************************************************************************************************************************
//when TX is switched on and RX is switched off, or after the loss of RF data = LED TX flash at a interval of 0.1s. Normal mode = LED TX is lit **************************
//************************************************************************************************************************************************************************
void RFoff_check()
{
  if (millis() >= ledTime + 100)
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
    digitalWrite(PIN_LED, ledState);
  }
}
 
