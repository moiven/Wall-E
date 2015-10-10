#include <NESpad.h>
#include <SPI.h>
#include "RF24.h"
#include <LiquidCrystal.h>

/**
This is the code for controlling the robot wirelessly. The controller operates using an arduino Uno.
The direction and button control comes from an NES game controller. An lcd display will display
the status of the robot and status of the controller. The controller operates the robot via a wireless 2.4Gh
signal coming from the RF24L01 transceiver.
**/

/*
********The NES PAD*******
* NES strobe pin -> pin 2
* NES clock pin -> pin 3
* NES data pin -> pin 4
*
***The RF24L01 transmitter***
* vcc pin -> 3.3v
* CSN pin -> pin 10
* MOSI pin -> pin 11
* IRQ pin -> pin 8
* GND pin -> pin gnd
* CE pin -> pin 9
* SCK pin -> pin 13
* MISO pin -> pin 12
*
*****The LCD display*****
* LCD pin 1 -> ground
* LCD pin 2 -> 5V
* LCD pin 3 -> 10k pot
* LCD pin 4 -> pin A1
* LCD pin 5 -> ground
* LCD pin 6 -> pin A2
* LCD pin 9 -> pin 5
* LCD pin 11 -> pin A0
* LCD pin 12 -> pin 6
* LCD pin 13 -> pin 7
* LCD pin 14 -> pin A1
* LCD pin 15 -> 5V
* LCD pin 16 -> gnd
*/

//controller send commands
#define UP          1
#define DOWN        2
#define LEFT        3
#define RIGHT       4
#define A           5
#define B           6
#define START       7
#define SELECT      8

#define PRESSED     1

static String history;

//Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);

//initialize the NES library with the numbers of the interface pins
NESpad nintendo = NESpad(2,3,4); //(strobe, clock, data);

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(A1, A2, 5, A0, 6, 7);

byte addresses[][6] = {"8Node","9Node"};
byte state = 0; // Buttons state (it ends up looking like a bitmap with 1's for buttons that are pressed and 0's for unpressed)

void setup() 
{  
  lcd.begin(16, 2); 
  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  
  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  
  // Start the radio listening for data
  radio.stopListening();
  //notify when the arduino is ready
  lcd.print("Ready");
}

//send command over wireless
//print to lcd screen
void sendCommand(int command, String message)
{
  //send and print command
  //but print "fails" if it fails
  if(!radio.write(&command, sizeof(int)))
    message = "failed";
  if(history != message)
  {
    lcd.clear();
    lcd.print(message);
    history = message;
  }
}
//read radio transmission
/*void readRadio()
{
  int num;
  float omega;
  int gyroXTemp;
  int gyroZTemp;
  
  //start listening for any data
  radio.startListening();
  if(radio.available())
  {
    radio.read(&num, sizeof(int));
    //radio.read(&omega, sizeof(float));
    //radio.read(&gyroXTemp, sizeof(int));
    //radio.read(&gyroZTemp, sizeof(int));
  }
  //resume not listening
  radio.stopListening();
  //print the data to the lcd
  //lcd.clear();
  //move cursor to column 0, row 1
  lcd.setCursor(0,1);
  lcd.print(num);
  //lcd.print(omega);
  //lcd.print(gyroXTemp);
  //lcd.print(gyroZTemp);
}*/
void loop() 
{  
  //readRadio();
  state = nintendo.buttons();

  // if the button is pressed right now and it's last state was UNPRESSED...
  if(state & NES_A) 
    sendCommand(A, "A");            //send command A
  if(state & NES_B)
    sendCommand(B, "B");            //send command B
  if(state & NES_UP) 
    sendCommand(UP, "UP");          //send command UP
  if(state & NES_LEFT)
    sendCommand(LEFT, "LEFT");      //send command LEFT
  if(state & NES_RIGHT)
    sendCommand(RIGHT, "RIGHT");    //send command RIGHT
  if((state & NES_DOWN))
    sendCommand(DOWN, "DOWN");      //send command DOWN
  if(state & NES_SELECT)
    sendCommand(SELECT, "SELECT");  //send command SELECT
  if(state & NES_START)
    sendCommand(START, "START");    //send command START
}
