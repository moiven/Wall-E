//Include the RF24L01 wireless library
#include <SPI.h>
#include "RF24.h"
//Include an Encoder library, that let's us easily read the encoders
#include <Encoder.h>
#include <Wire.h>
//#include <printf.h>
//the APDS9960 library by sparkfun
#include <SparkFun_APDS9960.h>

//Define pin locations with names
#define enable      4      //Motor Enable for Motor 1
#define m1dir       7      //Motor Direction for Motor 1
#define m2dir       8      // Motor direction for Motor 2       
#define m1speed     9      //Motor Speed for Motor 1
#define m2speed     10      //Motor Speed for Motor 2
#define encoderA_M1 2     //Encoder 1 for Motor 1
#define encoderB_M1 3     //Encoder 2 for Motor 1
#define encoderA_M2 18     //Encoder 1 for Motor 2
#define encoderB_M2 19     //Encoder 2 for Motor 2
#define RGBinterrupt  41
#define gyroX       0
#define gyroZ       1
#define trigPin 30    //Trig
#define echoPin 31    //Echo

//controller recieve commands
#define UP          1
#define DOWN        2
#define LEFT        3
#define RIGHT       4
#define A           5
#define B           6
#define START       7
#define SELECT      8
#define STOP        9
#define ERR         10
#define SLOWUP      11

// Define a variable encoderNew, as an instantaneous encoder reading
#define encoderNew myEnc.read()
#define gyroXNew analogRead( gyroX )
#define gyroZNew analogRead(gyroZ )

//use the APDS9960 Library from sparkfun
SparkFun_APDS9960 apds = SparkFun_APDS9960();

//Use both encoders
Encoder myEnc(encoderA_M1, encoderB_M1);

//Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 40 & 53
RF24 radio(40,53);  //for the mega

//Define variables necessary for our speed calcs
unsigned long t, milliOld;
unsigned long encoderOld = 0;
unsigned long encoderTemp = 0;
float omega =0;
float omega2 =0;
float ts = 0.0;
int gyroXTemp = 0;
int gyroZTemp = 0;
bool stopCompletely = false;
bool autonomous = false;
bool forwardDisable = false;


//globals for the line sensor
uint16_t ambient_light = 0;
uint8_t proximity_data = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;

//vars for US Ranger
long duration, cm;
/**************************************Setup***************dri**********************/
void setup() {
  //Serial.begin(9600);
  //printf_begin();
  //Set the enable pin high (necessary for driving the motor)
  pinMode(enable, OUTPUT);
  digitalWrite(enable, HIGH);
  pinMode(m1dir, OUTPUT);
  pinMode(m2dir, OUTPUT);
  pinMode(m1speed, OUTPUT);
  pinMode(m2speed, OUTPUT);

  //Define gyroscope pins
  pinMode( gyroX, INPUT);
  pinMode( gyroZ, INPUT);

  //The address is the same for both transievers
  //byte addresses[][6] = {"8abde"};
  //byte addresses[][6] = {"8Node","9Node"};
  
  radio.begin();
  radio.setChannel(57); //Sets frequency of channel to 2400+x Hz, if x=107 Hz=2507, max = 2525Hz
  radio.setPALevel(RF24_PA_MAX); //sets broadcast amplitude, default=max
  //radio.openReadingPipe(1,addresses[0]);
  radio.openReadingPipe(1,121501163909);
  radio.startListening();

  //radio.maskIRQ(1,1,0); //configure the interrupt on the comm board to switch LOW on packet acceptance
  //starting the radio
  /*
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  
  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1,addresses[0]);
  
  // Start the radio listening for data
  radio.startListening();

  */
  //initialize the APDS-9960
  apds.init();
  //start running the APDS-9960 light sensor (no interrupts)
  apds.enableLightSensor(true);

  //initialize US Ranger
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Wait for initialization and calibration to finish
  delay(500);
}
/*******************************drive motors**********************************/
//function to drive both motors from given parameters
void driveMotor(int m1_speed, int m2_speed, bool m1_dir, bool m2_dir)
{
  //dir is the direction the wheels spin
  digitalWrite(m1dir, m1_dir);
  digitalWrite(m2dir, m2_dir);
  //the speed is from 0 (stopped) to 255 (max)
  analogWrite(m1speed, m1_speed);
  analogWrite(m2speed, m2_speed);
}
/*********************************read radio data*******************************/
//read radio transmission from the controller
int readCommand()
{
  int command;
  static unsigned long tread ;
  //read command from the controller if available
  if(radio.available())
  {
    radio.read(&command, sizeof(int));
    tread = millis();
  }
  //add a timeout
  else if((millis() - tread) > 50)
    command = STOP;
  return command;
}
/*********************************Send radio data********************************/
//data to send back to the controller
//this will be printed on controller's lcd
/*void sendRadio (float omega, int gyroXTemp, int gyroZTemp)
{
  int num = 5;
  //stop listening so it can send
  radio.stopListening();
  //send data
  //radio.write(&omega, sizeof(float));
  //radio.write(&gyroXTemp, sizeof(int));
  //radio.write(&gyroZTemp, sizeof(int));
  radio.write(&num, sizeof(int));
  //resume listening
  radio.startListening();
}*/
/***********************************Line tracing***********************************/
//the robot will read light values and perform commands based on the line color
void lineTracker(int& command, bool& stopCompletely)
{
  //check if there is an ambient light reading
  //if it fails it exits the funtion
  if(!apds.readAmbientLight(ambient_light) || !apds.readRedLight(red_light) || !apds.readGreenLight(green_light) || !apds.readBlueLight(blue_light))
    return;
  //change nothing if A or B are chosen
  if(command == A || command == B)
    return;
  //stop the robot if it detects a line
  //else overwrite the command to up
  //if(green_light < 500 || stopCompletely)
  if(digitalRead(RGBinterrupt)==0)
  {
    //Serial.println(stopCompletely);
    stopCompletely = true;
    command = STOP;
  }
  else
  //    command = SLOWUP;
    command = UP;
} 
/**************************************The loop************************************/
//This loop is designed to send a command to the motor every iteration
// If it is before 1second, it sends o (off), afterwards it send 255(on).
//Every 1ms it will take readings and calculate speed, then print out values
void loop()
{
  //radio.printDetails();
  //get the command from the controller
  int command = readCommand();
  
  //US Ranger ping for distance
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 10000);
  cm = (duration/2) / 29.1;
  
  if (cm < 15 && cm > 0)
    forwardDisable=1;
  else
    forwardDisable=0;
 
  //the lineTracker will overwrite commands other than A or B
  //if autonomous is enabled (autonomous = true)
  if(autonomous)
    lineTracker(command, stopCompletely);
  
  //get the command from the controller
  //give robot commands
    switch(command)
    {
      case UP:  //Drive FORWARD
        //Serial.println(forwardDisable);
        if(forwardDisable==0)
          driveMotor(255, 230, LOW, LOW);
        else
          driveMotor(0, 0, LOW, HIGH);
        break;
      case SLOWUP:  //Drive FORWARD
        if(forwardDisable==0)
          driveMotor(115, 100, LOW, LOW); //150
        else
          driveMotor(0, 0, LOW, HIGH);
        break;
      case DOWN:  //Drive BACKWARDS
        driveMotor(255, 230, HIGH, HIGH);
        break;
      case LEFT:  //Turn LEFT
        driveMotor(150, 150, HIGH, LOW);
        break;
      case RIGHT:  //Turn RIGHT
        driveMotor(150, 150, LOW, HIGH);
        break;
      case A:  //A BUTTON
        // activate the lineTracker function
        autonomous = true;
        apds.clearAmbientLightInt();
        break;
      case B:  //B BUTTON
        //deactivate the lineTracker function
        stopCompletely = false;
        autonomous = false;
        break;
      case START:  //Start button
        //do something
        break;
      case SELECT:  //Select button
        //do something
        break;
      case STOP:  //Stop the robot driving
        driveMotor(0, 0, LOW, HIGH);
        break;
      case ERR:  //Stop the robot and report error
        driveMotor(0, 0, LOW, HIGH);
        break;
    }
}
