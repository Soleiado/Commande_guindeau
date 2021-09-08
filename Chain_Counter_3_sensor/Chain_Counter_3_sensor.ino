#include <EEPROM.h>
#include <SPI.h>
#include "RF24.h"

//-----------------------------------

#define DEBOUNCE_DELAY    20

#define NMEA_INPUT_BUFFER 100

#define pinCounter    2  // #04 - PD2 : Reed switch : windlass sensor
#define pinRelayUp    7  // #13 - PD7
#define pinRelayDown  8  // #14 - PB0
#define pinLedUp      6  // #12 - PD6

#define RelayUPStop()     PORTD &= B01111111
#define RelayDOWNStop()   PORTB &= B11111110
#define LedStop()         PORTD &= B10111111

#define RelayUPStart()    PORTD |= B10000000
#define RelayDOWNStart()  PORTB |= B00000001
#define LedStart()        PORTD |= B01000000

#define CounterState()    ((PIND & B00000100) >> 2) 

//-----------------------------------
// Protocol
//-----------------------------------

// packet type
#define PROT_TYPE_NONE    0
#define PROT_TYPE_CMD     1
#define PROT_TYPE_NAVDATA 2
#define PROT_TYPE_RESET   3
#define PROT_TYPE_ACK     255

// Command type
#define CMD_NONE          0
#define CMD_CHAIN_UP      1
#define CMD_CHAIN_DOWN    2
#define CMD_CHAIN_STOP    3

// Chain direction
#define DIR_CHAIN_UP      0
#define DIR_CHAIN_DOWN    2
#define DIR_CHAIN_STOP    1

//-----------------------------------

typedef struct _Proto {
  byte type;
  union  {
    byte command;
    struct {
      byte  direction;
      int   chain;
      float depth;
      float speed;
      // Wind data
      byte  windS; // Wind speed
      int   windA; // Wind angle
      char  windR; // Reference: (T)rue, (R)elative
      char  windU; // Unit: K - Km/h, N - Knot, M - m/s
    } NavData;
  } c;
} Proto;

//-----------------------------------

int            pinUp             = A0; // #23
int            pinDown           = A1; // #24

int            counter          = 0;
int            oldCounter       = 0;
int            lastStateCounter = 0;
int            oldCommand       = CMD_CHAIN_STOP;

float          depth         = 0.0;
float          speed         = 0.0;
float          windS         = 0.0;
float          windA         = 0.0;
char           windR         = 'R';
char           windU         = 'N';
int            oldDirection  = 0;
unsigned long  lastTimeSaved = 0;

Proto          protoSend;
Proto          protoReceive;

RF24           radio(9,10); // (#15, #16), CE, CSN
byte           addresses[][6] = {"Sensr","Displ"};

//------------------------------------
// Arduino initialisation
//------------------------------------

void setup()
{
  analogReference(EXTERNAL);
  /*
  pinMode(pinCounter,   INPUT);
  pinMode(pinRelayUp,   OUTPUT);
  pinMode(pinRelayDown, OUTPUT);
  pinMode(pinLedUp,     OUTPUT);
  */
  DDRD = B11000000;
  DDRB |= B00000001;
  /*
  digitalWrite(pinRelayUp,   LOW);
  digitalWrite(pinRelayDown, LOW);
  */
  RelayUPStop();
  RelayDOWNStop();

  //-------------------------------------
  // Read parameters for EEprom
  //-------------------------------------
  byte* p = (byte*) &counter;
  for (int i = 0; i < sizeof(counter); i++)
    *p++ = EEPROM.read(i);

  memset(&protoSend,    0, sizeof(Proto));
  memset(&protoReceive, 0, sizeof(Proto));

  //-------------------------------------
  // nrf24l01+ setup
  //-------------------------------------
  radio.begin();
  radio.setRetries(5,15);
  radio.setAutoAck(1);
  radio.setCRCLength(RF24_CRC_8);
  radio.setPayloadSize(sizeof(Proto));
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(118);
  //
  // Open pipes to other nodes for communication
  //
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1,addresses[0]);
  //
  // Start listening
  //
  radio.startListening();

  //----------------------------------------
  // Serial setup: nmea input
  //----------------------------------------

  Serial.begin(4800);
}

//------------------------------------
// Communication
//------------------------------------

void protoXchange()
{
  static unsigned long startWaitingAt = 0;

  if (startWaitingAt == 0 || startWaitingAt > millis()) startWaitingAt = millis();
  
  if (radio.available()) {
    radio.read((byte *)&protoReceive, sizeof(Proto));
    Serial.println("Data");
    startWaitingAt = 0;
    radio.stopListening();
    radio.write((byte *)&protoSend, sizeof(Proto));
    radio.startListening();
  } else if (millis() - startWaitingAt > 1000) {
    memset(&protoReceive, 0, sizeof(Proto));
    startWaitingAt = 0;
  }
}

//------------------------------------
// Extract CSV field
//------------------------------------

char* extractCSVField(char** ppTmp)
{
  char*  pField = *ppTmp;
  char*  pTmp   = *ppTmp;
  
  while (*pTmp != ',' && *pTmp != '\0') pTmp++;

  if (*pTmp != '\0') *pTmp++  = '\0';

  *ppTmp = pTmp;
  
  return pField;
}

//------------------------------------
// Get navigation data from NMEA input
//------------------------------------

boolean getNmeaValue(
  char*   pstNmeaBuffer)
{
  char*   pTmp   = &pstNmeaBuffer[2]; // first 2 char identify the talker, 
  char*   pField = pTmp;

  // Get the sentence identifier
  pField = extractCSVField(&pTmp);
  
  if (!strcmp(pField, "DPT")) {
    // Depth
    // $--DPT,x.x,x.x*hh
    // 1) Depth, meters
    // 2) Offset from transducer;
    //    positive means distance from transducer to water line,
    //    negative means distance from transducer to keel
    // 3) Checksum
    
    pField = extractCSVField(&pTmp); // 1) Depth, meter
    if (*pField != '\0') depth = atof(pField);

    pField = extractCSVField(&pTmp); // 2) offset, meter
    if (*pField != '\0') depth += atof(pField);

    depth = depth / 0.304;

  } else if (!strcmp(pField, "DBT")) {
    // Depth below transducer
    // $--DBT,x.x,f,x.x,M,x.x,F*hh
    // 1) Depth, feet
    // 2) f = feet
    // 3) Depth, meters
    // 4) M = meters
    // 5) Depth, Fathoms
    // 6) F = Fathoms
    // 7) Checksum
    
    pField = extractCSVField(&pTmp); // 1) Depth, feet
    if (*pField != '\0') depth = atof(pField);

  } else if (!strcmp(pField, "VHW")) {
    // water speed and heading
    // $--VHW,x.x,T,x.x,M,x.x,N,x.x,K*hh
    // 1) Degress True
    // 2) T = True
    // 3) Degrees Magnetic
    // 4) M = Magnetic
    // 5) Knots (speed of vessel relative to the water)
    // 6) N = Knots
    // 7) Kilometers (speed of vessel relative to the water)
    // 8) K = Kilometres
    // 9) Checksum
    
    pField = extractCSVField(&pTmp); // 1) Degress True
    pField = extractCSVField(&pTmp); // 2) T = True
    pField = extractCSVField(&pTmp); // 3) Degrees Magnetic
    pField = extractCSVField(&pTmp); // 4) M = Magnetic
    pField = extractCSVField(&pTmp); // 5) Knots (speed of vessel relative to the water)
    if (*pField != '\0') speed = atof(pField);
  } else if (!strcmp(pField, "MWV")) {
    // MWV - Wind Speed and Angle
    // $--MWV,x.x,a,x.x,a,a*hh
    // 1) Wind Angle, 0 to 360 degrees
    // 2) Reference, R = Relative, T = True
    // 3) Wind Speed
    // 4) Wind Speed Units, K/M/N
    // 5) Status, A = Data Valid
    // 6) Checksum

    pField = extractCSVField(&pTmp); // 1) Wind Angle, 0 to 360 degrees
    if (*pField != '\0') windA = round(atof(pField));
    pField = extractCSVField(&pTmp); // 2) Reference, R = Relative, T = True
    windR = pField[0];   
    pField = extractCSVField(&pTmp); // 3) Wind Speed
    if (*pField != '\0') windS = round(atof(pField));
    pField = extractCSVField(&pTmp); // 4) Wind Speed Units, K/M/N
    windU = pField[0];   
  }
  
  return false;
}

//------------------------------------
// Get navigation data from NMEA input
//------------------------------------

boolean getNavigationData()
{
  static char    stNmeaBuffer[NMEA_INPUT_BUFFER+1];
  static int     iPos         = 0;
  static boolean bNmeaStarted = false;
  static boolean bNmeaEnded   = false;
  boolean        bReturn      = false;
  long           lStartTime   = millis();

  while (Serial.available() && (millis() - lStartTime) < 75) {
    // get the new byte:
    char inChar = (char) Serial.read();
 
    // check if NMEA start char
    if (inChar == '$') {
      bNmeaStarted = true;
      memset(stNmeaBuffer, '\0', NMEA_INPUT_BUFFER+1);
      iPos = 0;
      continue;
    }
    
    if (bNmeaStarted) {
      // add it to the inputString:
      if (inChar == '\n') {
        bNmeaEnded = true;
        break;
      } else {
        stNmeaBuffer[iPos++] = inChar;
        
        if (iPos >= NMEA_INPUT_BUFFER) {
          bNmeaStarted = false;
        }
      }
    } 
  }

  if (bNmeaEnded) {
    bNmeaEnded   = false;
    bNmeaStarted = false;
    iPos         = 0;
    bReturn      = getNmeaValue(stNmeaBuffer);
  }
  
  return bReturn;
}

//----------------------------------------
// get & exec command from remote/display
//----------------------------------------

int getExecCommand(
  int         oldCommand)
{
  int         command          = oldCommand; 

  if (protoReceive.type == PROT_TYPE_CMD) {
    switch (protoReceive.c.command) {
      case CMD_CHAIN_UP: // Up
        command = CMD_CHAIN_UP;
        break;
      case CMD_CHAIN_DOWN: // Down
        command = CMD_CHAIN_DOWN;
        break;
      case CMD_CHAIN_STOP: // Stop            
      default: 
        command = CMD_CHAIN_STOP;
        break;
    }
  } else if (protoReceive.type == PROT_TYPE_RESET) {
    counter = 0;
    command = CMD_CHAIN_STOP;
  } else {
    command = CMD_CHAIN_STOP;
  }

  if (oldCommand != command || command == CMD_CHAIN_STOP) {
    RelayDOWNStop();  // digitalWrite(pinRelayDown, LOW);
    RelayUPStop();    // digitalWrite(pinRelayUp,   LOW);

    if (command == CMD_CHAIN_UP) {
      //digitalWrite(pinRelayUp,   HIGH); 
      RelayUPStart();
    } else if (command == CMD_CHAIN_DOWN) {
      //digitalWrite(pinRelayDown, HIGH); 
      RelayDOWNStart();
    }
  }
  
  return command;
}

//------------------------------------
// Check windlass rotation direction
//------------------------------------

int getWindlassDirection()
{
  if (analogRead(pinUp) > 300) { 
    return DIR_CHAIN_UP - 1;
  } else if (analogRead(pinDown) > 300)
    return DIR_CHAIN_DOWN - 1;

  return DIR_CHAIN_STOP - 1;
}

//------------------------------------
//  Get the chain counter sensor value
//------------------------------------

int getChainCounter()
{
  int iStateCounter                 = lastStateCounter;
  int iStateRead                    = CounterState();
  int iPulse                        = 0;
  static unsigned long debounceTime = 0;  // the last time the output pin was toggled

  if (iStateRead != lastStateCounter && debounceTime == 0) {
    // reset the debouncing timer
    debounceTime = millis();
  } 
  
  if ((millis() - debounceTime) > DEBOUNCE_DELAY && debounceTime != 0) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    iStateCounter = iStateRead;
    debounceTime = 0;
  }
  
  if (iStateCounter != lastStateCounter) {
    iPulse           = iStateCounter;
    lastStateCounter = iStateCounter;
  } 
  
  return iPulse;
}

//------------------------------------
// Main Loop
//------------------------------------

void loop() {
  int                   direction     = 0;
  int                   pulse         = 0;

  //------------------------------------------
  // Get & execute command from remote/display
  //------------------------------------------
  oldCommand = getExecCommand(oldCommand);
  
  //--------------------------------------
  // Check windlass rotation direction
  //--------------------------------------
  direction = getWindlassDirection();
  
  //--------------------------------------
  // Get the chain counter sensor value
  //--------------------------------------
  pulse = getChainCounter();
  
  //--------------------------------------
  // Do the math :)
  //--------------------------------------
  if (pulse) {
    if (!direction)
      counter += 1;
    else
      counter += direction ;
    counter = counter > 0 ? counter : 0;
  }    
  
  //--------------------------------------
  // Save the value
  //--------------------------------------
  if (millis() - lastTimeSaved > 10000L && counter != oldCounter) {
     // Save counter
     byte* p = (byte*) &counter;
     for (int i = 0; i < sizeof(counter); i++)
       EEPROM.write(i, *p++);
     lastTimeSaved = millis();
  }

  oldCounter = counter;

  //----------------------------------------
  // Get navigation data from NMEA input
  //----------------------------------------
  getNavigationData();

  //----------------------------------------
  // Prepare data to send
  //----------------------------------------
  protoSend.type                = PROT_TYPE_NAVDATA;
  protoSend.c.NavData.chain     = counter;
  protoSend.c.NavData.direction = direction + 1;
  protoSend.c.NavData.depth     = depth;
  protoSend.c.NavData.speed     = speed;
  protoSend.c.NavData.windS     = windS;
  protoSend.c.NavData.windA     = windA;
  protoSend.c.NavData.windR     = windR;
  protoSend.c.NavData.windU     = windU;

  protoXchange();  
}
