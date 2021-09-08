#include <EEPROM.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include "RF24.h"
#include "SymbolMono18pt7b.h"

//-----------------------------------
// Communication protocol
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
// Mode definition
//-----------------------------------
#define MODE_NAV_DISPLAY   1
#define MODE_SETUP         2
#define MODE_CHAIN_COUNTER 3
#define MODE_AUTO_PILOT    4 // not implemented ...
#define MODE_CHAIN_SETUP   5
#define MODE_SETTING_UP    128

//-----------------------------------
// Type definition
//-----------------------------------

// Parameter
typedef struct _Parameter {
  byte  unit;
  byte  nbLinkPerTurn;
  byte  lenOfLink;
} Parameter;

// Menu & Options
typedef struct _Option {
  const char*         optionName;
  const char*         subMenu;
  byte                flag;
  int                 action;
} Option;

typedef struct _Menu {
  const char*         title;
  byte                nbOptions;
  Option*             option;
  struct _Menu*       previous;
} Menu;

// Communication Protocol: packet structure
typedef struct _Proto {
  byte type;
  union {
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
//  Menu & options
//-----------------------------------
static Option setupOptionUnit[] = {
  {"Imperial", NULL, 0, 11},
  {"Metric",   NULL, 0, 12},
  {"Exit",     NULL, 0, 0},
};

static Menu setupMenuUnit = {
  "Unit", 3, setupOptionUnit, NULL
};

//-----------------------------------

static Option setupOptionNbLink[] = {
  {"4",    NULL, 0, 221},
  {"5",    NULL, 0, 222},
  {"6",    NULL, 0, 223},
  {"Exit", NULL, 0, 0},
};

static Menu setupMenuNbLink = {
  "setup", 4, setupOptionNbLink, NULL
};

//-----------------------------------

static Option setupOptionChainSize[] = {
  {"G4 1/4 HT",  NULL, 0, 211},
  {"G4 5/16 HT", NULL, 0, 212},
  {"G4 3/8 HT",  NULL, 0, 213},
  {"Exit",       NULL, 0, 0},
};

static Menu setupMenuChainSize = {
  "setup", 4, setupOptionChainSize, NULL
};

//-----------------------------------

static Option setupOptionChain[] = {
  {"Chain Size",   (char*)&setupMenuChainSize, 0, 21},
  {"Nb Link",      (char*)&setupMenuNbLink,    0, 22},
  {"Reset Chain",  NULL,                       0, 23},
  {"Exit",         NULL,                       0, 0},
};

static Menu setupMenuChain = {
  "setup", 4, setupOptionChain, NULL
};

//-----------------------------------

static Option setupOptionMode[] = {
  {"Nav data",  NULL, 0, 31},
  {"Windlass",  NULL, 0, 32},
  {"Exit",      NULL, 0, 0},
};

static Menu setupMenuMode = {
  "Mode", 3, setupOptionMode, NULL
};

//-----------------------------------

static Option setupOptionMain[] = {
  {"Unit",        (char*)&setupMenuUnit,  0, 1},
  {"Chain setup", (char*)&setupMenuChain, 0, 2},
  {"Mode",        (char*)&setupMenuMode,  0, 3},
  {"Exit",        NULL,                   0, 0},
};

static Menu setupMenuMain = {
  "setup", 4, setupOptionMain, NULL
};

//-----------------------------------

Menu*             menu        = &setupMenuMain;
Menu*             currentMenu = menu;

Parameter         parameter;

Adafruit_PCD8544  display = Adafruit_PCD8544(8 /*14 - SCLK*/, 7 /*13 - DIN*/, 6 /* 12 - D/C*/, 5 /* 11 - CS*/, 4 /* 6 - RST*/);

#define           pinUp   A2
#define           pinDown A1

byte              displayMode = MODE_NAV_DISPLAY;
byte              oldMode     = MODE_NAV_DISPLAY;

float             chainMultiplier = 0.0;
byte              chainDirection;
float             chainLenght;
float             depth = 0.0;
float             speed = 0.0;
int               windS = 0;
int               windA = 0;
char              windF = '_';
char              windR = 'R';
char              windU = 'N';

char              output[41];
unsigned long     setupTime;

byte              stateUp      = LOW;
byte              stateDown    = LOW;
byte              oldStateUp   = LOW;
byte              oldStateDown = LOW;

RF24              radio(9, 10);
byte              addresses[][6] = {"Sensr","Displ"};
Proto             protoSend;
Proto             protoReceive;

//------------------------------------
// Communication
//------------------------------------

void protoXchange()
{
  static unsigned long lastSentTime = 0;

  if (lastSentTime > millis()) lastSentTime = millis();
   
  if (millis() - lastSentTime > 100) {
    radio.stopListening();
    radio.write((byte *)&protoSend, sizeof(Proto));
    radio.startListening();
    delay(10);
    if (radio.available())
      radio.read((byte *)&protoReceive, sizeof(Proto));
    
    lastSentTime = millis();
  }    
}

//------------------------------------
// Read data from remote sensor
//------------------------------------

void getData()
{
    if (protoReceive.type == PROT_TYPE_NAVDATA) {
      chainLenght    = (float) (protoReceive.c.NavData.chain * chainMultiplier);
      chainDirection = protoReceive.c.NavData.direction;
      depth          = protoReceive.c.NavData.depth;
      speed          = protoReceive.c.NavData.speed;
      windS          = protoReceive.c.NavData.windS;
      windA          = protoReceive.c.NavData.windA;
      windR          = protoReceive.c.NavData.windR;
      windU          = protoReceive.c.NavData.windU;
    } else {
      chainLenght    = 0.0;
      chainDirection = DIR_CHAIN_STOP;
      depth          = 0;
      speed          = 0;
      windS          = 0;
      windA          = 0;
      windR          = 0;
      windU          = 0;
    }
  
	  if (windA > 180) {
       windA = 360 - windA;
	     windF = 'P'; // Babord - Port
    } else 
       windF = 'S'; // Tribord - Starboard
}

//------------------------------------
// Display data
//------------------------------------

void modeDisplay(const char* title)
{
  getData();
  
  display.clearDisplay();

  // Tile outline
  display.setTextColor(BLACK);
  display.drawLine(0, 9, display.width()-1, 9, BLACK);
  display.drawLine(display.width()/2, 9, display.width()/2, display.height(), BLACK);
  display.drawLine(0, 27, display.width()-1, 27, BLACK);

  // Tile: Title
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK); // 'inverted' text
  display.print(title);
  display.setTextColor(BLACK);

  // Tile 1: depth
  display.setCursor(0, 11);
  display.print("depth");
  display.setCursor(0, 19);
  if (parameter.unit == 0) // Imperial
    sprintf(output, "%5dft", (int)round(depth));
  else
    sprintf(output, "%4d.%1dm", (int) (depth*0.304), (int) (depth*3.04)%10);
  display.print(output);

  // Tile 2: speed
  display.setCursor(display.width()/2 + 2, 11);
  display.print("speed");
  display.setCursor(display.width()/2 + 2, 19);
  sprintf(output, "%2d.%1dkt", (int)speed, (int)(speed*10)%10);
  display.print(output);

  // Tile 3: Chain direction & Length
  sprintf(output, " ");
  if (chainDirection == DIR_CHAIN_UP) 
    sprintf(output, "%c", (char)0);
  else if (chainDirection == DIR_CHAIN_DOWN) 
    sprintf(output, "%c", (char)1); 
    
  display.setCursor(0, display.height()-3);
  display.setFont(&SymbolMono18pt7b);  
  display.print(output);
  display.setFont();  

  // Chain length
  display.setCursor(11, 29);
  display.print("Chain");
  display.setCursor(11, 37);
  if (parameter.unit == 0) // Imperial
    sprintf(output, "%3dft", (int) round(chainLenght));
  else
    sprintf(output, "%2d.%1dm", (int) (chainLenght*0.304), (int) (chainLenght*3.04)%10);
  display.print(output);

  // Tile 4: Wind
  display.setCursor(display.width()/2 + 2, 29);
  sprintf(output, " %3d %c", windA, windF);
  display.print(output);
  display.setCursor(display.width()/2 + 2, 37);
  sprintf(output, "%c%3d%s", windR, windS, (windU == 'N' ? "Kt" : ((windU == 'K' ? "kh" : "ms"))));
  display.print(output);

  display.display();
}

//------------------------------------
// Change mode
//------------------------------------

int setMode()
{
  byte newMode = displayMode;

  if (stateUp == HIGH && stateDown == HIGH) {
    if ((newMode & MODE_SETTING_UP) == MODE_SETTING_UP) {
      if (millis() - setupTime > 3000L) {
        newMode = MODE_SETUP;
      }
    } else if (newMode != MODE_SETUP) {
      oldMode =  displayMode;
      newMode |= MODE_SETTING_UP;
      setupTime = millis();
    }
  } else {
    if ((newMode & MODE_SETTING_UP) == MODE_SETTING_UP) {
      newMode ^= MODE_SETTING_UP;
    }
  }

  displayMode = newMode;
}

//------------------------------------
// Show menu
//------------------------------------

void showMenu(
  int   option)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0,0);

  if (currentMenu != NULL) {
    display.println(currentMenu->title);
    display.drawLine(0, 9, display.width()-1, 9, BLACK);

    display.setCursor(0, 11);

    for (int i=0; i < currentMenu->nbOptions; i++) {
      if (i == option)
         display.setTextColor(WHITE, BLACK); // 'inverted' text

      display.print(currentMenu->option[i].optionName);

      if (i == option)
         display.setTextColor(BLACK);

      if (currentMenu->option[i].flag == 1)
         display.print(" *");

      display.println();
    }

    display.display();
  }
}

//--------------------------------------
// Do action associated with menu option
//--------------------------------------

void doAction(int action, int& option)
{
  switch (action) {
    case 0:
      option = 0;
      if (currentMenu->previous == NULL)
         displayMode = oldMode;
      else
         currentMenu = currentMenu->previous;
      break;
    case 2:
      displayMode = MODE_CHAIN_SETUP;
      break;
    case 11: // mode imperial
    case 12: // mode metric
      setupOptionUnit[0].flag      = 0;
      setupOptionUnit[1].flag      = 0;
      setupOptionUnit[option].flag = 1;
      parameter.unit               = option;
      break;
    case 211: // 1/4
    case 212: // 5/16
    case 213: // 3/8
      setupOptionChainSize[0].flag      = 0;
      setupOptionChainSize[1].flag      = 0;
      setupOptionChainSize[2].flag      = 0;
      setupOptionChainSize[option].flag = 1;
      parameter.lenOfLink               = option;
      break;
    case 221: // 4 links
    case 222: // 5 links
    case 223: // 6 links
      setupOptionNbLink[0].flag      = 0;
      setupOptionNbLink[1].flag      = 0;
      setupOptionNbLink[2].flag      = 0;
      setupOptionNbLink[option].flag = 1;
      parameter.nbLinkPerTurn        = option;
      break;
    case 23:
      //------------------------
      // Prepare command to send
      //------------------------
      memset(&protoSend, 0, sizeof(Proto));
      protoSend.type = PROT_TYPE_RESET;
    case 31:
      displayMode = MODE_NAV_DISPLAY;
      option = 0;
      break;
    case 32:
      displayMode = MODE_CHAIN_COUNTER;
      delay(500);
      option = 0;
      break;
    default:
      break;
  }

  // Save Parameters
  byte* p = (byte*)&parameter;
  for (int i = 0; i < sizeof(Parameter); i++) EEPROM.write(i, *p++);

  chainMultiplier = (float) (parameter.nbLinkPerTurn + 4);
  
  switch(parameter.lenOfLink) {
    case 0: // 1/4
      chainMultiplier = (float) chainMultiplier / 14.2; // 14.2 links per foot
      break;
    case 1: // 5/16
      chainMultiplier = (float) chainMultiplier / 11.7; // 11.7 links per foot
      break;
    case 2: // 3/8
      chainMultiplier = (float) chainMultiplier / 9.8;  // 9.8 links per foot
      break;
  }
}

//------------------------------------
// Configuration
//------------------------------------

void modeSetup()
{
  static int option = 0;

  if (oldStateUp != stateUp || oldStateDown != stateDown) {
    if (stateUp != stateDown && stateUp == HIGH && oldStateUp == LOW) {
      option++;
      option = option % currentMenu->nbOptions;
    } else if (stateUp != stateDown && stateDown == HIGH && oldStateDown == LOW) {
      if (currentMenu->option[option].subMenu != NULL) {
        currentMenu = (Menu*) currentMenu->option[option].subMenu;
        option = 0;
      } else {
        doAction(currentMenu->option[option].action, option);
      }
    }
  }

  if (displayMode == MODE_SETUP)
    showMenu(option);
}

//------------------------------------
// Windlass
//------------------------------------

void modeChainCounter()
{
  protoSend.type = PROT_TYPE_CMD;
  if (stateUp == HIGH)
    protoSend.c.command = CMD_CHAIN_UP;
  else if (stateDown == HIGH)
    protoSend.c.command = CMD_CHAIN_DOWN;
  else
    protoSend.c.command = CMD_CHAIN_STOP;
}

//------------------------------------
// Arduino initialisation
//------------------------------------
void setup()
{
  //-------------------------------------
  // Display initialisation
  //-------------------------------------
  display.begin();
  display.setContrast(40);
  display.drawBitmap(0, 0, ancre_bmp, 84, 48, BLACK);
  display.display(); // show splashscreen

  //-------------------------------------
  // Read parameters for EEprom
  //-------------------------------------
  byte* p = (byte*) &parameter;
  for (int i = 0; i < sizeof(Parameter); i++)
    *p++ = EEPROM.read(i);

  if (parameter.unit > 1)          parameter.unit = 0;
  if (parameter.nbLinkPerTurn > 2) parameter.nbLinkPerTurn = 0;
  if (parameter.lenOfLink > 2)     parameter.lenOfLink = 0;

  chainMultiplier = (float) (parameter.nbLinkPerTurn + 4);
  
  switch(parameter.lenOfLink) {
    case 0: // 1/4
      chainMultiplier = (float) chainMultiplier / 14.2; // 14.2 links per foot
      break;
    case 1: // 5/16
      chainMultiplier = (float) chainMultiplier / 11.7; // 11.7 links per foot
      break;
    case 2: // 3/8
      chainMultiplier = (float) chainMultiplier / 9.8;  // 9.8 links per foot
      break;
  }

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
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  //
  // Start listening
  //
  radio.startListening();

  //------------------------------------------
  // Menu setup
  //------------------------------------------
  setupMenuMain.previous      = NULL;
  setupMenuMode.previous      = &setupMenuMain;
  setupMenuChain.previous     = &setupMenuMain;
  setupMenuUnit.previous      = &setupMenuMain;
  setupMenuChainSize.previous = &setupMenuChain;
  setupMenuNbLink.previous    = &setupMenuChain;

  //------------------------------------------
  // Option setup
  //------------------------------------------
  setupOptionUnit[parameter.unit].flag            = 1;
  setupOptionNbLink[parameter.nbLinkPerTurn].flag = 1;
  setupOptionChainSize[parameter.lenOfLink].flag  = 1;

  //------------------------------------------
  // Finish setup
  //------------------------------------------
  delay(2000);
  display.clearDisplay();   // clears the screen and buffer

  memset(&protoSend,    0, sizeof(Proto));
  memset(&protoReceive, 0, sizeof(Proto));
}

//------------------------------------
// Main Loop
//------------------------------------

void loop()
{
  stateUp   = analogRead(pinUp) > 300;
  stateDown = analogRead(pinDown) > 300;

  //------------------------
  setMode();
  //------------------------
  
  switch (displayMode) {
    case MODE_NAV_DISPLAY:
      modeDisplay("Nav Data");
      break;
    case MODE_SETUP:
      modeSetup();
      break;
    case MODE_CHAIN_COUNTER:
      modeChainCounter();
      modeDisplay("Windlass");
      break;
  }

  oldStateUp   = stateUp;
  oldStateDown = stateDown;

  //------------------------
  protoXchange();
}
