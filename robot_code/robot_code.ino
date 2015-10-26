//Include the RF24L01 wireless library
#include <SPI.h>
#include "RF24.h"
//Include an Encoder library, that let's us easily read the encoders
#include <Encoder.h>
#include <Wire.h>
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
#define gyroX       0
#define gyroZ       1
#define TRIG_PIN    30    //Trig
#define ECHO_PIN    31    //Echo

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

//The address is the same for both transevers
byte addresses[][6] = {"8Node","9Node"};

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
//variables for the rangerInterrupt
volatile unsigned long int_time, out_time;
volatile bool st = false;


//globals for the line sensor
uint16_t ambient_light = 0;
uint8_t proximity_data = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;

//vars for US Ranger
long duration, cm;
/**************************************Setup*************************************/
void setup() {
//  Serial.begin(9600);
  // Set the enable pin high (necessary for driving the motor)
  pinMode(enable, OUTPUT);
  digitalWrite(enable, HIGH);
  pinMode(m1dir, OUTPUT);
  pinMode(m2dir, OUTPUT);
  pinMode(m1speed, OUTPUT);
  pinMode(m2speed, OUTPUT);
  
  //initialize ultrasonic ranger
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  //Define gyroscope pins
  pinMode( gyroX, INPUT);
  pinMode( gyroZ, INPUT);
  
  //initializing an interrupt function called 'rangerInterrupt'
  //interrupt rutine starts on a changing signal (low to high or high to low)
/*  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), rangerInterrupt, CHANGE);  */

  //starting the radio
  radio.begin();
  
  //initialize the APDS-9960
  apds.init();
  //start running the APDS-9960 light sensor (no interrupts)
  apds.enableLightSensor(false);
  
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  
  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1,addresses[0]);
  
  // Start the radio listening for data
  radio.startListening();
  
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
  else if((millis() - tread) > 100)
    command = STOP;
  return command;
}
/*********************************Send radio data********************************/
//data to send back to the controller
//this will be printed on controller's lcd
/*void sendRadio (float omega, int gyroXTemp, int gyroZTemp)
{
  //int num = 5;
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
//if it discovers a red line it will overwrite the command to stop
//the lineTracker will disable all commands other than A or B
void lineTracker(int& command, bool& stopCompletely)
{
  //check if there is an ambient light reading
  //if it fails it exits the funtion
  if(!apds.readAmbientLight(ambient_light) || !apds.readRedLight(red_light) || !apds.readGreenLight(green_light) || !apds.readBlueLight(blue_light))
    return;
  //check if A or B button is pressed
  if(command == A || command == B)
    return;
  //stop the robot if it detects a line
  //else overwrite the command to up
  if(green_light < 1500 || stopCompletely)
  {
    stopCompletely = true;
    command = STOP;
  }
  else
    command = UP;
}
/**************************************Ultrasonic Ranger*****************************/
//Send a pulse to activate the usRanger
//use interrupts to get the return time
//will overwrite the command to stop if return time is roughly < 20cm
void usRanger(int& command)
{
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(7);          //delay stability for the interrupt
  if(out_time - int_time < 1200)
    command = STOP;
}
/*************************************Ranger ISR***********************************/
//records the time micros of the initial low to high pulse
//and the ending high to low pulse
/*void rangerInterrupt()
{
  if(!st)
    int_time = micros();
  else if(st)
     out_time = micros();
  st = !st;
}*/
/**************************************The loop************************************/
//This loop is designed to send a command to the motor every iteration
// If it is before 1second, it sends o (off), afterwards it send 255(on).
//Every 1ms it will take readings and calculate speed, then print out values
void loop()
{
  //get the command from the controller
  int command = readCommand();
  
  //lineTracker function call if autonomous is set to true
  if(autonomous)
    lineTracker(command, stopCompletely);
    
  //usRanger function call
/*  usRanger(command);  */
  
  //give robot commands
  switch(command)
  {
    case UP:  //Drive FORWARD
      driveMotor(250, 250, LOW, HIGH);
      break;
    case DOWN:  //Drive BACKWARDS
      driveMotor(250, 200, HIGH, HIGH);
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
