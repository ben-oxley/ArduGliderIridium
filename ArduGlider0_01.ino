#define MAVLINK10
#define DEBUG

#include <FastSerial.h>
#include <SoftwareSerial.h>
#include <GCS_MAVLink.h>
#include "parameter.h"
#include <avr/pgmspace.h>
#include "RFM22.h"
#include "CommCtrlrConfig.h"
#include "Iridium9602.h"
#include <stdlib.h>
#include <SPI.h>

#undef PSTR 
#define PSTR(s) (__extension__({static const char __c[] PROGMEM = (s); &__c[0];})) 

#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 

#define SERIAL_BAUD 57600
FastSerialPort0(Serial);

#define APM_PARAMS 37   //Check in Params.pde that the set of APM parameters are numberd 0 to this number -1
#define ACM_PARAMS 48   //Check in Params.pde that the set of ACM parameters are number 0 to this number - 1

// data streams active and rates
#define MAV_DATA_STREAM_POSITION_ACTIVE 1
#define MAV_DATA_STREAM_RAW_SENSORS_ACTIVE 1
#define MAV_DATA_STREAM_EXTENDED_STATUS_ACTIVE 1
#define MAV_DATA_STREAM_RAW_CONTROLLER_ACTIVE 1
#define MAV_DATA_STREAM_EXTRA1_ACTIVE 1

// update rate is times per second (hz)
#define MAV_DATA_STREAM_POSITION_RATE 1
#define MAV_DATA_STREAM_RAW_SENSORS_RATE 1
#define MAV_DATA_STREAM_EXTENDED_STATUS_RATE 1
#define MAV_DATA_STREAM_RAW_CONTROLLER_RATE 1
#define MAV_DATA_STREAM_EXTRA1_RATE 1

#define GET_PARAMS_TIMEOUT 200 //(20 seconds)

#define RELEASE 2
#define ROCKBLOCK_ON 4
#define ARDUPILOT_ON 3
#define RB_RTS 5
#define RB_CTS 6
#define RB_SLEEP 7
#define RADIO_EN 8
#define TELEMETRY 9
#define REG_N_SHDN 10
#define RB_MOSI 11
#define RB_MISO 12
#define RB_SCK 13
#define IRIDIUM_SERIAL_PORT satPort
SoftwareSerial IRIDIUM_SERIAL_PORT(A2,A3);
#define toRad(x) (x*PI)/180.0
#define toDeg(x) (x*180.0)/PI

float offset = 0;
// flight data
byte relAltitude = 0; // if =1 use relative altitude for display instead of GPS alt above sea level
int altitude=0;
float pitch=0;
float roll=0;
float yaw=0;
float longitude=0;
float latitude=0;
float velocity = 0;
int numSats=0;
float battery=0;
int currentSMode=0;
int currentNMode=0;
int gpsfix=0;


//SoftwareSerial TelemetryRx(A1,A2);

// Check for 3 valid heartbeats and shut down wrong Mavlink type
byte param_rec[7];
byte waitingAck=0;
parameter editParm;
byte numGoodHeartbeats = 0;
byte paramsRecv=0;
byte beat=0;
byte droneType = 1;  // 0 = Not established, 1 = APM, 2 = ACM  - Normally read from Mavlink heartbeat, but ACM returns 0, not 2
byte autoPilot = 3;  // This should be 3 for ArdupilotMeg
uint8_t received_sysid=0;   ///< ID of heartbeat sender
uint8_t received_compid=0;  // component id of heartbeat sender
byte TOTAL_PARAMS = APM_PARAMS; // default total params to ACM set up

// Saved time (in msecs) for the last byte read from the serial port
long lastByteTime = 0;
uint8_t byt[15]; // Saved received bytes
uint8_t byt_Counter = 0;
uint8_t b_ct;
int byte_per_half_sec = 0;
int last_byte_per = 0;
int timeOut = 0;
unsigned long timer;

// Saved time (in msecs) for the last message byte received while not in Mavlink idle
// 0 = in idle - no message in process
long timeLastByte = 0;
long maxByteTime = 0;

// Wrong Mavlink Version detector
byte wrongMavlinkState = 0; // Check incoming serial seperately from Mavlink library - in case we have wrong Mavlink source 
                           // State 0 means Mavlink header not yet detected
                           // if State = 5 we are receiving the wrong Mavlink format
byte packetStartByte = 0;  // first byte detected used to store which wrong Mavlink was detected 0x55 Mavlink 0.9, 0xFE Mavlink 1.0

Iridium9602 iridium = Iridium9602(IRIDIUM_SERIAL_PORT);

//Setup radio on SPI with NSEL on pin 8
rfm22 radio1(8);

// the setup routine runs once when you press reset:
void setup() { 
  Serial.begin(SERIAL_BAUD);  
  initialisePins(); // initialize the digital pin as an output.
  delay(10); 
  digitalWrite(REG_N_SHDN, HIGH);
  pMosfetOn(ARDUPILOT_ON);
}
// the loop routine runs over and over again forever:
void loop() {
 digitalWrite(SCK,HIGH); 
 delay(100);
 digitalWrite(SCK,LOW);
 delay(100);
 recieveTelem();
 checkRelease();
}

void pMosfetOn(int pin) {
  pinMode(pin,INPUT);
}

void initialisePins() {
  pinMode(RELEASE, OUTPUT);
  digitalWrite(RELEASE, LOW);
  pinMode(ROCKBLOCK_ON, OUTPUT);
  digitalWrite(ROCKBLOCK_ON, LOW);
  pinMode(ARDUPILOT_ON, OUTPUT);
  digitalWrite(ARDUPILOT_ON, LOW);
  pinMode(REG_N_SHDN, OUTPUT);
  digitalWrite(REG_N_SHDN, LOW);
  pinMode(SCK, OUTPUT);
}

int recieveTelem() {
  gcs_update();
#ifdef DEBUG
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print(" Altitude: ");
  Serial.print(altitude);
  Serial.print(" Latitude: ");
  Serial.print(latitude*1000);
  Serial.print(" Longitude: ");
  Serial.println(longitude*1000);
#endif
}

void checkRelease() {
  if (getInputPWM(A1) > 1500) { 
    pMosfetOn(RELEASE);
  }
}

int getInputPWM(int pin) {
  unsigned int pulseWidth, pulseWidthAvg;
  for(int i=1;i<=10;i++) {
    pulseWidth = pulseIn(pin,HIGH,50);
    if (pulseWidth = 0) return 0;
    pulseWidthAvg = (pulseWidthAvg*(i-1)+pulseWidth)/i;
  }
  return pulseWidthAvg;
}

void setupRadio(){
 
  //digitalWrite(5, LOW); //If this pin is connected to the SDN on the radio we want to pull it low first
 
  delay(1000);
 
  rfm22::initSPI();
 
  radio1.init();
 
  radio1.write(0x71, 0x00); // unmodulated carrier
 
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
  radio1.write(0x0b,0x12);
  radio1.write(0x0c,0x15);
 
  radio1.setFrequency(434.201);
 
  //Quick test
  radio1.write(0x07, 0x08); // turn tx on
  delay(1000);
  radio1.write(0x07, 0x01); // turn tx off
 
}

// RTTY Functions - from RJHARRISON's AVR Code
void rtty_txstring (char * string)
{
 
	/* Simple function to sent a char at a time to 
	** rtty_txbyte function. 
	** NB Each char is one byte (8 Bits)
	*/
	char c;
	c = *string++;
	while ( c != '\0')
	{
		rtty_txbyte (c);
		c = *string++;
	}
}
 
void rtty_txbyte (char c)
{
	/* Simple function to sent each bit of a char to 
	** rtty_txbit function. 
	** NB The bits are sent Least Significant Bit first
	**
	** All chars should be preceded with a 0 and 
	** proceded with a 1. 0 = Start bit; 1 = Stop bit
	**
	** ASCII_BIT = 7 or 8 for ASCII-7 / ASCII-8
	*/
	int i;
	rtty_txbit (0); // Start bit
	// Send bits for for char LSB first	
	for (i=0;i<8;i++)
	{
		if (c & 1) rtty_txbit(1); 
			else rtty_txbit(0);	
		c = c >> 1;
	}
	rtty_txbit (1); // Stop bit
        rtty_txbit (1); // Stop bit
}
 
void rtty_txbit (int bit)
{
		if (bit)
		{
		  // high
                  radio1.setFrequency(434.2010);
		}
		else
		{
		  // low
                  radio1.setFrequency(434.2015);
		}
                delayMicroseconds(19500); // 10000 = 100 BAUD 20150
 
}
 
void rtty_tx(){
      char superbuffer[80];
      radio1.write(0x07, 0x08); // turn tx on
      delay(5000);
      rtty_txstring("$$$$");
      rtty_txstring(superbuffer);
      radio1.write(0x07, 0x01); // turn tx off
}
