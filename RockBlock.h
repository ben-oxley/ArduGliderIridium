/**
  * RockBlock.h
  *
  * Part of the Kraken Ocean Drifter
  * http://poseidon.sgsphysics.co.uk
  *
  * Daniel Saul
  * (C) SGS Poseidon Project 2013
  */

#ifndef ROCKBLOCK_H
#define ROCKBLOCK_H

#include "Arduino.h"
#include <SoftwareSerial.h>
//#include "debug.h"
#include <string.h>

const uint8_t RB_RX = A2;     // Serial RX
const uint8_t RB_TX = A3;     // Serial TX
const uint8_t RB_CTS = 6;    
const uint8_t RB_RTS = 5;
const uint8_t RB_SLEEP = 7;  // POWER ON/OFF 
const uint8_t RB_RING = A0;   // RING INDICATOR
const uint8_t RB_NET = A1;    // NETWORK AVAILABLE
//#define RELEASE 2
//#define ROCKBLOCK_ON 4
//#define ARDUPILOT_ON 3
//#define RB_RTS 5
//#define RB_CTS 6
//#define RB_SLEEP 7
//#define RADIO_EN 8
//#define TELEMETRY 9
//#define REG_N_SHDN 10
//#define RB_MOSI 11
//#define RB_MISO 12
//#define RB_SCK 13
const uint8_t minimumSignalRequired = 2;
const uint16_t maxTelemetryLength = 340;
const uint16_t responseLost = 30000;


void rockblock_init();
void rockblock_on();
void rockblock_off();

bool initiateSession();
bool loadMessage(unsigned char *msg, int length);
int readMessage(unsigned char *msg, int maxLength);

void parseSBDIX();

bool sendCommandandExpectPrefix(const char * command, const char * response, unsigned long timeout);
void sendCommand(const char * command);
bool expectResponse(const char * response, unsigned long timeout);

bool receiveCmdCRLF(unsigned long timeout);
void clearReceivedCmd();

int checkSignal();
bool isSatAvailable();

bool messageSent();
int messagesToReceive();
int messagesWaitingOnNetwork();
bool messageAvailableToRead();

#endif
