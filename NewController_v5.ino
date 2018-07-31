
/****************************************************************************************************************************************************************************************
*
* ZEISS AXIOVERT 200 MICROSCOPE STAGE CONTROLLER ARDUINO PROGRAMfind
* WRITTEN DURING 2102 BY GADI AFEK & ALON OYLER YANIV @ OLEG KRICHEVSKY'S LAB IN BGU PHYSICS DEPT.
* SEE ATTACHED TXT FILE FOR ADDITIONAL PINNOUT INFORMATION
*
*  This program uses the following hardware:
*
*  1. Arduino mega2560 board
*  2. DFRobot 1A motor shield
*  3. Zeiss joystick (by Megatron)
*
*  It recieves input from the joystick and translates it to voltage to the motors of a Zeiss 
*  Axoivert 200 microscope stage. 
*
*  It also reads data from the motors' limit switches and encoders (for this we use interrupts).
*
* The program has a capability of remembering 10 setpoints and going accurately to any given relative coordinate.
*
*  Furthermore it reads and prints out relevant data using the serial monitor.
*
*  Notice that there's a pin conflict between the joystick and this motor shield - 
*  pins 4,5,6 are used by the joystick for the buttons and by the motor shield 
*  for input to the motors. This is averted here by rerouting the joystick buttons
*  to pins 8-11
*
*  Motor 2 is defined as the x axis and Motor 1 is defined as the y axis
*
* 24.12.2014 New LED control: single PWM + 3 bit MUX
*****************************************************************************************************************************************************************************************/


#include <PWM.h>

enum StateType{idle,
       controlled_motion,
     fine_motion};

StateType SystemState = idle;

const byte additional_GND_PIN = 47;

const byte PWM_X  =  11;  //Motor X PWM control - Atmega pin PB5 (OC1A/PCINT5)
const byte PWM_Y =  3;  //Motor Y PWM control / Atmega pin PE5 (OC3C/INT5)
const byte DIR_X = 13;  //Motor X direction control
const byte DIR_Y = 12;  //Motor Y direction control
const byte BRK_X = 8;  //Motor X brake control
const byte BRK_Y = 9;  //Motor Y brake control

boolean rightX_ENC_state = LOW; //HIGH; // whats that?
boolean rightY_ENC_state = LOW; //HIGH; // whats that?



const byte FINE_MOTION_XY_SWITCH = 31;

const byte PIN_ANALOG_X = A10; //Joystick control
const byte PIN_ANALOG_Y = A14;

const byte ENCODER_X_A = 21; //Sets interrupt  pins 21 (interrupt 2) 
const byte INTRPT_ENC_X = 2; // interrupt 2

const byte ENCODER_Y_A = 20;// and 20 (interrupt 3) as encoder A inputs for both axes
const byte INTRPT_ENC_Y = 3; // interrupt 3

const byte ENCODER_X_B = 24; //encoder B s on digital pins
const byte ENCODER_Y_B = 22;

const byte FINE_ENCODER_A = 19; //Sets interrupt  pins 19 (interrupt 4) to work with fine motion wheel
const byte INTRPT_FINE_WHEEL_ENC = 4; // interrupt 4
const byte FINE_ENCODER_B = 30; // fine motion wheel encoder B digital pin

const byte LIMIT_SWITCH_S1_X = 28; //Sets pins 24-27 (arbitrary) as limit switch controls
const byte LIMIT_SWITCH_S2_X = 29;
const byte LIMIT_SWITCH_S3_Y = 27; 
const byte LIMIT_SWITCH_S4_Y = 26;

int PWM_FW = 7;  //  Filter wheel motor enable (no PWM right now: perhaps not needed). Set to H for motion and for fast stop. For slow stop set to L. See http://adhocnode.com/motor-control/
int  DIR_FW = 5;  //  set  to H for forward motion and L otherwise
int  BRK_FW = 6;  //  set to H for fast break
int FW_InterruptNo = 0; // the number of the interrupt 
int FW_InterruptPin = 2; // the number of the pin for FW interrupt 

//int WLShutter_EN = 48; // White light shutter: motor
//int WLShutter_IN2 = 46; // 
//int WLShutter_IN1 = 44;
//int WLShutterDelay = 20; //Delay in ms to switch off the shutter motor

//const byte LED1_PWMpin = 44;
// const byte LED2_PWMpin = 45;
//const byte LED3_PWMpin = 46; 
const byte LED_PWMpin = 46;
const byte LED_MUX[3] = {47 , 48,  49}; //pins of the multiplexor picking the LED on the wheel

const byte LED_temp = 44; // LED outside the microscope
//const byte LED_MUX2 = 49;


const byte ShutterBlackPin = 36;
const byte ShutterRedPin = 37;


volatile long int x_position = 0; //Global, relative coordinates
volatile long int y_position = 0;
long int x_set = 0; // set point
long int y_set = 0;
volatile boolean arrived_to_x_set = true;
volatile boolean arrived_to_y_set = true;

boolean debugMode = false;
long int StartTime = 0;  //used for degugging
long int CurrentTime = 0; //used for debugging
long int DelayTime = 100; // in ms used for debugging
long int TimeOut = 10000; // time out in ms for controlled motion: if has not arrived then stop
long int MovementStartTime = 0;
long int MovementCurrentTime = 0;



// things below to check
volatile byte right_or_left = 1;  //Direction indicators
volatile byte up_or_down = 1;


boolean B_set = (boolean) digitalRead(FINE_ENCODER_B); //Preliminary status of wheel encoder's B channel

long int setpoint[10] [2] = {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}}; //Setpoint array, allowing to remember 10 setpoints

//int x = 0, y = 0; //For joystick input



//String command = ""; //For serial input

/****************************************************************************************************************************************************************************************
* Controllable parameters for the fine tuning of the program
*****************************************************************************************************************************************************************************************/

const int X_MAX_SPEED = 125; //Maximal PWM value for joystick, x axis
const int Y_MAX_SPEED = 135; //Maximal PWM value for joystick, y axis
const int THRESHOLD = 30; //Threshold value for motion from joystick
int MotorX_PWM_Freq = 50; // pwm frequency in Hz
int MotorY_PWM_Freq = 50; // pwm frequency in Hz put 50 or 100

const int CONVERSION = 1000; //Convertion between ecnoder pulses (as depicted in x_position and y_position) and mm
const int X_OVERSHOOT = 0; //Overshoot in pulses for coarse gotoxy function - x axis
const int Y_OVERSHOOT = 0; //Overshoot in pulses for coarse gotoxy function - y axis
const int DELAY_AFTER_MOTOR = 100; //Delay in ms after motor operation
const int FINE_DELAY = 10; //Delay in ms for fine motion buttons
const double X_RESPONSE_POWER = 1; //Power law for renormalization - x axis
const double Y_RESPONSE_POWER = 1; //Power law for renormalization - y axis
const int PROP_FACTOR_X = 1; //Proportionalliny factor for goto functions
const int PROP_FACTOR_Y = 2; //Proportionalliny factor for goto functions

// velocity range changes
int GOTO_SPEED = 50; //Default speed for automatic motor functions (up to 255)
int range1 = 150;
int mediumSpeed = 10;
int range2 = 20;
int lowSpeed = 5;
volatile boolean changeSpeedX = false;
volatile boolean changeSpeedY = false;

int Vx;  //speed from joystick
int Vy;

// fine wheel motion
int PulseTime = 2; // in ms
int FineSpeed = 100;
volatile int fine_sign = 0; // the direction of fine motion
volatile byte fine_axis = 0; //Indicator for fine motion. 0 for X, 1 for Y



/****************************************************************************************************************************************************************************************
* A  helpful little sign function
*****************************************************************************************************************************************************************************************/

int sign(int num)
{
  if (num == 0) return 0;
  return (num / abs(num));
}

int pickVelocity(long int deviation) {
  if (deviation == 0) {
    return 0;
  }
  else if (abs(deviation) <= range2) {
    return lowSpeed;
  }
  else if (abs(deviation) <= range1) {
    return mediumSpeed;
  } 
  else {
    return GOTO_SPEED;
  }
  
}
/****************************************************************************************************************************************************************************************
* checkLimitSwitch function - Checks whether limit switches were activated. Gives 1 if, in the x axis,
*                                                             S1 is activated and left motion is requested OR S2 activated and right 
*                                                             motion is requested. Respectively for S3 and S4 on the y axis.
*****************************************************************************************************************************************************************************************/

byte checkLimitSwitches(byte x_or_y)
{
  byte right_and_S1 = (!digitalRead(LIMIT_SWITCH_S1_X) && (right_or_left));
  byte left_and_S2 = (!digitalRead(LIMIT_SWITCH_S2_X) && (!right_or_left));
  byte up_and_S4 = ((!digitalRead(LIMIT_SWITCH_S4_Y)) && (up_or_down));
  byte down_and_S3 = (!digitalRead(LIMIT_SWITCH_S3_Y) && (!up_or_down));
    
  if (x_or_y && (right_and_S1 || left_and_S2)) return 1;
  if ((! x_or_y) && (down_and_S3 || up_and_S4))  return 1;
    
  return 0;
}

/****************************************************************************************************************************************************************************************
* Individual motor control functions
*****************************************************************************************************************************************************************************************/

void RunMotor(int Vel, byte PWMpin, byte DIRpin, byte BRKpin)
{
  digitalWrite (BRKpin, (Vel == 0));
  digitalWrite (DIRpin, (Vel > 0));
  pwmWrite (PWMpin, abs(Vel));
}

void RunXMotor(int Vel)
{
  RunMotor(Vel, PWM_X, DIR_X, BRK_X);
}

void RunYMotor(int Vel)
{
  RunMotor(Vel, PWM_Y, DIR_Y, BRK_Y);
}


/****************************************************************************************************************************************************************************************
* Filter wheel motor control
*****************************************************************************************************************************************************************************************/

void FWMotor(int drct)
{
  switch (drct)
  {
    case 1 :
        // move clockwise
       digitalWrite (DIR_FW, HIGH);
       digitalWrite (BRK_FW, LOW);
       digitalWrite (PWM_FW, HIGH);
       break;
       
    case -1 :
        // move counter clockwise
       digitalWrite (DIR_FW, LOW);
       digitalWrite (BRK_FW, LOW);
       digitalWrite (PWM_FW, HIGH);
       break;
       
   case 0 :
        // slow stop: natural motion
       digitalWrite (DIR_FW, HIGH);
       digitalWrite (BRK_FW, LOW);
       digitalWrite (PWM_FW, LOW);
       Serial.println("OK"); // patching
       break;
       
   case -2 :
        // fast stop: active
       digitalWrite (BRK_FW, HIGH);   
       //digitalWrite (PWM_FW, LOW);
       break;
  }
  
}  

   
/****************************************************************************************************************************************************************************************
* displayMenu function - displays main menu
*****************************************************************************************************************************************************************************************/

void displayMenu ()
{
  Serial.println("THESE ARE THE FUNCTIONS AT YOUR DISPOSAL. NO SPACES PLEASE!:");
  Serial.println();
  Serial.println("setzero - SET CURRENT AS ZERO");
  //Serial.println("setpoint(N,x,y) - SET AS SETPOINT NUMBER N BETWEEN 0 AND 9. (x,y) ARE OPTIONAL. IF THEY DO NOT APPEAR THEN SAVE CURRENT POINT");
 // Serial.println("gotosetpoint(N) - GO TO SETPOINT N");
  Serial.println("goto x,y - GO TO COORDINATES (x,y) IN MICRONS");
  Serial.println("getxy - GET CURRENT (x,y) VALUES");
  Serial.println("home - GOTO (0,0)");
  Serial.println("SETSPEED V - set the maximal speed of motion");
  Serial.println("menu - DISPLAY MAIN MENU");
  Serial.println();
}

/****************************************************************************************************************************************************************************************
* SetMotorPWMfreq function - sets PWM frequency of the channels controlling the stage motors. The frequency should be low enough so that motors can respond to low values of speed
*****************************************************************************************************************************************************************************************/


void SetMotorPWMfreq()
{
   bool success = SetPinFrequencySafe(PWM_X, MotorX_PWM_Freq);
   if(!success) {
    Serial.println("Motor X frequency has not changed!");    
   } 
   success = SetPinFrequencySafe(PWM_Y, MotorY_PWM_Freq);
   if(!success) {
    Serial.println("Motor Y frequency has not changed!");    
   } 
}

int32_t GetPWMfreq(byte PWMpin)
{
  byte TimerNo = digitalPinToTimer(PWMpin);
  switch (TimerNo) {
    case 1:
    case 2: 
      Serial.println("You should not use timer 0");     //timer 0
      return 0;
      break;
      
    case 3:
    case 4:
    case 5:     //timer 1 (A, B, C PWM channels)
      return Timer1_GetFrequency();
      break;
      
    case 6:
    case 7:
    case 8:     //timer 2 (A, B, C PWM channels)
      return Timer2_GetFrequency();
      break;
      
    case 9:
    case 10:
    case 11:     //timer 3 (A, B, C PWM channels)
      return Timer3_GetFrequency();
      break;
      
    case 12:
    case 13:
    case 14:     //timer 4 (A, B, C PWM channels)
      return Timer4_GetFrequency();
      break;
      
    case 15:
    case 16:
    case 17:     //timer 5 (A, B, C PWM channels)
      return Timer4_GetFrequency();
      break;
    
  }
  //Serial.println(TimerNo);
}
/****************************************************************************************************************************************************************************************
* Gotoxy function - uses coarseGotoxy and fineGotoxy and prints out information on serial monitor. Comments are for compatibility with LV
*****************************************************************************************************************************************************************************************/

void gotoxy(long int X, long int Y)
{
    x_set = X;
    y_set = Y;
    arrived_to_x_set = (x_set == x_position);
    arrived_to_y_set = (y_set == y_position);
    RunXMotor(pickVelocity(x_set - x_position)*sign(x_set - x_position));
    RunYMotor(pickVelocity(y_set - y_position)*sign(y_set - y_position));
    MovementStartTime =  millis();
    if (debugMode){
      StartTime =  millis();
    } 
}



/****************************************************************************************************************************************************************************************
* Thresholding function - THRESHOLD strip. 
*****************************************************************************************************************************************************************************************/

int renormalizeAndThreshold (int input_parameter, int upper_limit, double power)
{  
  input_parameter = (input_parameter / 2) - 256; //Transform from 0 : 1023 to -255 : 255
  if (input_parameter == -256) input_parameter = -255;
  
  double fraction = (double(abs(input_parameter)) / double(255));
  
  if (abs(input_parameter) < THRESHOLD) return 0; //Threshold strip
  
  return (int) (sign(input_parameter) * (upper_limit * pow(fraction,power))); //Power law
}


/****************************************************************************************************************************************************************************************
* Get serial input function - gets input from serial port
*****************************************************************************************************************************************************************************************/

void getSerialInput()   {
  long x_temp = 0, y_temp = 0, multiplicator = 1;
  int i = 0;
  String command = "";
  
  while (Serial.available() > 0) {       //read input
  char in_char = (char) Serial.read(); 
  command += in_char;
  delay(10);
  }
  
  if (command.equalsIgnoreCase("setzero"))  {//Reset coordinates to zero
    x_position = 0;
    y_position = 0;
    Serial.println("OK"); //Notify that action is completed
  }
  else if (command.substring(0,5).equalsIgnoreCase("goto "))  { // goto specified comma separated coordinates
    int comaPos = command.indexOf(",");  
    String CoordStr = command.substring(5, comaPos);
    long int X = CoordStr.toInt();
    CoordStr = command.substring(comaPos+1);
    long int Y = CoordStr.toInt();  
    SystemState =  controlled_motion;
    gotoxy(X,Y); 
    //Serial.println("OK"); //Notify that action is completed
  }
  else if (command.substring(0,3).equalsIgnoreCase("go "))  { // goto specified comma separated coordinates
    int comaPos = command.indexOf(",");  
    String CoordStr = command.substring(3, comaPos);
    long int X = CoordStr.toInt();
    CoordStr = command.substring(comaPos+1);
    long int Y = CoordStr.toInt();  
    SystemState =  controlled_motion;
    gotoxy(x_position + X, y_position + Y); 
  }
  else if (command.equalsIgnoreCase("getxy")) {   //get coordinates
   // Serial.println("CURRENT COORDINATES ARE:");
    Serial.println(x_position);
    Serial.println(y_position);
    Serial.println("OK"); //Notify that action is completed
  }
  else if (command.equalsIgnoreCase("home")) {
    gotoxy(0,0);
    //Serial.println("OK"); //Notify that action is completed
  }
  else if (command.substring(0,9).equalsIgnoreCase("setspeed ")) { // set goto speed   
    long int tempSpeed = command.substring(9).toInt();  
    if ((tempSpeed >= 0) && (tempSpeed <= 255)) {
      GOTO_SPEED = (int)tempSpeed;
     }
    else  {
      Serial.println("Speed out of limits (0 to 255)");
    }
  }
  else if (command.substring(0,13).equalsIgnoreCase("SetPWMfreq X "))  {    // PWM frequency in Hz; for 16 bit timer from 1Hz to 2MHz
    MotorX_PWM_Freq = command.substring(13).toInt();
    SetMotorPWMfreq();
    Serial.println(GetPWMfreq(PWM_X));
  }
   else if (command.substring(0,13).equalsIgnoreCase("SetPWMfreq Y "))  {    // PWM frequency in Hz; for 16 bit timer from 1Hz to 2MHz
    MotorY_PWM_Freq = command.substring(13).toInt();
    SetMotorPWMfreq();
    Serial.println(GetPWMfreq(PWM_Y));
  }
  else if (command.substring(0,10).equalsIgnoreCase("SetRange1 "))  {   // speed change range
    range1 = command.substring(10).toInt();
    Serial.println(range1);
  }
  else if (command.substring(0,10).equalsIgnoreCase("SetRange2 "))  {   // speed change range
    range2 = command.substring(10).toInt();
    Serial.println(range1);
  }
  else if (command.substring(0,12).equalsIgnoreCase("SetLowSpeed "))  {   // speed change range
    lowSpeed = command.substring(12).toInt();
    Serial.println(lowSpeed);
  }
  else if (command.substring(0,12).equalsIgnoreCase("SetMediumSpeed "))  {   // speed change range
    lowSpeed = command.substring(12).toInt();
    Serial.println(mediumSpeed);
  }
  else if (command.substring(0,11).equalsIgnoreCase("GetJoystick"))  {   // speed change range
    Serial.println(Vx);
    Serial.println(Vy);
  }
  else if (command.equalsIgnoreCase("STOP")) {  // stop motion
     digitalWrite (BRK_X, HIGH);
     digitalWrite (BRK_Y, HIGH);
  }
  else if (command.equalsIgnoreCase("TESTMotors")) {  // test motor motion
    int velocity = 5;
    long int x_test = 0;
    StartTime = micros();  // check timing of the commands in Encoder routines
    right_or_left = (digitalRead(ENCODER_X_B) == rightX_ENC_state);
    x_test += (right_or_left) ? +1 : -1;
    if (x_position == x_set) {
      digitalWrite (BRK_X, HIGH);
      //RunYMotor(0);
     // Serial.println("OK");
    }
   
    CurrentTime = micros();
    Serial.println(CurrentTime - StartTime);
    
    StartTime = micros();  // check timing of the commands in Encoder routines
    right_or_left = (digitalRead(ENCODER_X_B) == rightX_ENC_state);
    x_test += (right_or_left) ? +1 : -1;
    if (x_position == x_set) {
      digitalWrite (BRK_X, HIGH);
    }
    
    if (abs(x_position - x_set) < 100) {
     velocity = 10;
    } 
    CurrentTime = micros();
    Serial.println(CurrentTime - StartTime);
    
  }
  else if (command.substring(0,11).equalsIgnoreCase("GetPWMfreq ")) {  // Get PWM frequency in Hz; parameter PWM pin number 
    Serial.println(GetPWMfreq(command.substring(11).toInt()));
  }
  else if (command.equalsIgnoreCase("DEBUG ON"))  { // debug mode on : extended output
    debugMode = true;
  }
  else if (command.equalsIgnoreCase("DEBUG OFF")) { // debug mode on : extended output
    debugMode = false;
  }  
  else if (command.substring(0,15).equalsIgnoreCase("SetDelayTimeMs ")) { // PWM frequency in Hz; for 16 bit timer from 1Hz to 2MHz
    DelayTime = command.substring(15).toInt();
  }
  else if (command.equalsIgnoreCase("menu")) {  //display main menu
    displayMenu();
  }
  else if (command.equalsIgnoreCase("FWCLK")) { // rotate filter wheel clockwise
     FWMotor(1);
  }
  else if (command.equalsIgnoreCase("FWCNT")) { //rotate filter wheel counter clockwise
     FWMotor(-1);
  }
  else if (command.equalsIgnoreCase("FWSTOP")) { //slow stopping of filter wheel
     FWMotor(0);
  }
  else if (command.equalsIgnoreCase("FWBREAK"))  {//fast active stopping of filter wheel
     FWMotor(-2);
  }
  else if (command.equalsIgnoreCase("WLShutterClose"))  { //close white light shutter
          digitalWrite(ShutterRedPin, LOW);
          Serial.println("OK");
  }  
   else if (command.equalsIgnoreCase("WLShutterOpen"))  {//open white light shutter
          digitalWrite(ShutterRedPin, HIGH);
          Serial.println("OK");
  } 
  else if (command.substring(0,9).equalsIgnoreCase("pwmWrite "))  { // switch on LED specified comma separated pwm line number and power
    int comaPos = command.indexOf(",");  
    String PWMStr = command.substring(9, comaPos);
    byte PWMline = PWMStr.toInt();
    // MUXing
    for (int muxPin = 0;  muxPin<3; muxPin++) {
      digitalWrite(LED_MUX[muxPin], bitRead(PWMline, muxPin));
    }
    
    PWMStr = command.substring(comaPos+1);
    long int PWMpower = PWMStr.toInt();  
    pwmWrite(LED_PWMpin, PWMpower); 
    Serial.println("OK"); //Notify that action is completed
  }
  else if  (command.substring(0,8).equalsIgnoreCase("LEDtemp "))  { // switch on the LED outside the microscope
    long int PWMpower = command.substring(8).toInt();
    pwmWrite(LED_temp, PWMpower); 
    Serial.println("OK"); //Notify that action is completed
  }
  else if  (command.equalsIgnoreCase("LED ON"))  { // switch on the LED outside the microscope
    digitalWrite(LED_PWMpin, HIGH); 
    Serial.println("OK"); //Notify that action is completed
  }
  else if  (command.equalsIgnoreCase("LED OFF"))  { // switch on the LED outside the microscope
    digitalWrite(LED_PWMpin, LOW); 
    Serial.println("OK"); //Notify that action is completed
  }
  else Serial.println("WRONG INPUT");
  //Serial.println();

  command = "";
 }

/****************************************************************************************************************************************************************************************
* Initial setup
*****************************************************************************************************************************************************************************************/

void setup() 
{ 
    Serial.begin(9600);  //Activate serial monitor
   displayMenu();
  
  pinMode(DIR_X, OUTPUT);  //motor inputs
  pinMode(DIR_Y, OUTPUT);  //motor inputs
  pinMode(BRK_X,OUTPUT);  //motor inputs
  pinMode(BRK_Y,OUTPUT);  //motor inputs
 
  pinMode(ENCODER_X_A,INPUT_PULLUP);  //Encoder inputs
  pinMode(ENCODER_X_B,INPUT_PULLUP);  //Encoder inputs
  pinMode(ENCODER_Y_A,INPUT_PULLUP);  //Encoder inputs
  pinMode(ENCODER_Y_B,INPUT_PULLUP);  //Encoder inputs
   
  pinMode(LIMIT_SWITCH_S1_X,INPUT_PULLUP);  //Limit switches
  pinMode(LIMIT_SWITCH_S2_X, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_S3_Y, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_S4_Y,INPUT_PULLUP);

  
  pinMode(FINE_MOTION_XY_SWITCH, INPUT_PULLUP);  //Fine x-y motion switch;
  pinMode(FINE_ENCODER_A ,INPUT_PULLUP); 
  pinMode(FINE_ENCODER_B ,INPUT_PULLUP); 
  
  pinMode(PIN_ANALOG_X, INPUT_PULLUP);
  pinMode(PIN_ANALOG_Y, INPUT_PULLUP);
  
  //set filter wheel motor control pins to output mode
    pinMode(PWM_FW, OUTPUT); 
    pinMode(DIR_FW, OUTPUT);
    pinMode(BRK_FW, OUTPUT);  
    
    pinMode(ShutterBlackPin, OUTPUT);
    pinMode(ShutterRedPin, OUTPUT);
    digitalWrite( ShutterBlackPin, LOW);
    digitalWrite( ShutterRedPin, HIGH);
    
    pinMode(LED_MUX[0], OUTPUT);
    pinMode(LED_MUX[1], OUTPUT);
    pinMode(LED_MUX[2], OUTPUT);
    pwmWrite(LED_PWMpin, 255);
    
    pinMode(LED_temp,  OUTPUT);
    pwmWrite(LED_temp, 255);
    
    // additional ground pin
    pinMode(additional_GND_PIN, OUTPUT);
    digitalWrite( additional_GND_PIN, LOW);
  
  InitTimersSafe(); // see Arduino PWM frequency library http://forum.arduino.cc/index.php?PHPSESSID=1sasrj8o8jd2bhfhcirknq9oa7&topic=117425.0
  SetMotorPWMfreq();
  
  attachInterrupt(INTRPT_ENC_X, doEncoderX, RISING); //Define interrupts on counters 2,3 for x,y axes respectively upon rise in waveform
  attachInterrupt(INTRPT_ENC_Y, doEncoderY, RISING);
  attachInterrupt(INTRPT_FINE_WHEEL_ENC, fineMotionA, RISING); //Define interrupts on counters 4,5 for x,y axes respectively upon cange in waveform
  //attachInterrupt(5, fineMotionB, CHANGE);
  attachInterrupt(FW_InterruptNo, FW_InterruptRoutine, FALLING);

} 

/****************************************************************************************************************************************************************************************
* Main Program
*****************************************************************************************************************************************************************************************/

void loop()
{
 //int Vx;
 //int Vy;
 long int deviation;
  
  if (Serial.available() > 0) getSerialInput();//Get information from serial monitor
 
 if (SystemState == idle) { 
  Vx = analogRead(PIN_ANALOG_X); //Read data from joystick
  Vx = renormalizeAndThreshold (Vx, X_MAX_SPEED, X_RESPONSE_POWER); //Thresholding X using renormalizeAndThreshold

  Vy = analogRead(PIN_ANALOG_Y);
  Vy = renormalizeAndThreshold (Vy, Y_MAX_SPEED, Y_RESPONSE_POWER); //Thresholding Y using renormalizeAndThreshold
  
  RunXMotor(Vx); //Send info to motors
  RunYMotor(Vy);
 
 }
 else if (SystemState == controlled_motion ) {    // controlled motion
   
   if (changeSpeedX) {
     deviation = x_position - x_set;
     Vx = pickVelocity(deviation);
     pwmWrite(PWM_X, Vx);
     
     changeSpeedX = false;
     //Serial.println(deviation);
   }
   if (changeSpeedY) {
     deviation = y_position - y_set;
     Vy = pickVelocity(deviation);
     pwmWrite(PWM_Y, Vy);
     changeSpeedY = false;
     //Serial.println(deviation);
   } 
   
   MovementCurrentTime = millis();
   if (   (arrived_to_x_set && arrived_to_y_set)  ||   ( ( MovementCurrentTime-MovementStartTime) > TimeOut) ) {
     RunXMotor(0); 
     RunYMotor(0);
     //Serial.println(x_position);
     //Serial.println(y_position);
     Serial.println("OK");
     SystemState = idle;     
   }
 }
 else {                        // fine wheel
     if (fine_axis)  { // fine motion on Y
        RunYMotor(fine_sign*FineSpeed);
        delay(PulseTime);
        RunYMotor(0);         
     }
     else {     // fine motion on X
        RunXMotor(fine_sign*FineSpeed);
        delay(PulseTime);
        RunXMotor(0);         
     }
     SystemState = idle;
   }
}
 //if (x != 0) //Compensate for inertia effects
//  {
//    right_or_left = (x > 0);
//    x_last_sign = sign(x);
//  }
//  else right_or_left = (x_last_sign > 0);
//  
//  if (y != 0)
//  {
//    up_or_down = (y > 0);
//    y_last_sign = sign(y);
//  }
//  else up_or_down = (y_last_sign > 0);

  //RunYMotor(abs(Vy)); //Send info to motors
  //RunXMotor(abs(Vx));
  
  //if (fine_x_y_or_stop == 1) pulsedMotor(3,1);
  //if (fine_x_y_or_stop == 2) pulsedMotor(3,0);
  //fine_x_y_or_stop = 0;
  
  
  
  
//  if ((right_or_left && (x_position >= x_set)) || ((!right_or_left) && (x_position <= x_set)) || checkLimitSwitches(1)) {
//    RunXMotor(0);    
//  }
//  
//   if ((up_or_down && (y_position >= y_set)) || ((!up_or_down) && (y_position <= y_set)) || checkLimitSwitches(0)) {
//    RunYMotor(0);
//  }



/****************************************************************************************************************************************************************************************
* Encoder interrupt service routines - coordinate counting
*****************************************************************************************************************************************************************************************/

void doEncoderX()
{
  right_or_left = (digitalRead(ENCODER_X_B) == rightX_ENC_state);
  x_position += (right_or_left) ? +1 : -1; // and adjust X counter: + if going right and - if opposite
   if (x_position == x_set) {
      arrived_to_x_set = true;
      digitalWrite (BRK_X, HIGH);
  }
  
  if ((right_or_left && ( ((x_set - x_position)==range1) || ((x_set - x_position)==range2) )) || ((!right_or_left) && (((x_position - x_set)==range1) || ((x_position - x_set)==range2) ))) {
    changeSpeedX = true;
  }
   //Serial.println("OK_doEncX");
}

void doEncoderY()
{
  up_or_down =(digitalRead(ENCODER_Y_B) == rightY_ENC_state);
  y_position += (up_or_down) ? +1 : -1; // and adjust Y counter: + if going up and -  if opposite
  if (y_position == y_set) {
     arrived_to_y_set = true;
     digitalWrite (BRK_Y, HIGH);
  }
  if ((up_or_down && (((y_set - y_position)==range1) || ((y_set - y_position)==range2)) ) || ((!up_or_down) && (((y_position - y_set)==range1) || ((y_position - y_set)==range2)))) {
    changeSpeedY = true;
  }
  //Serial.println("OK_doEncY");
}

/****************************************************************************************************************************************************************************************
* Encoder interrupt service routines - fine motion wheel
*****************************************************************************************************************************************************************************************/

void fineMotionA()
{
  SystemState = fine_motion;
  B_set = (boolean) digitalRead(FINE_ENCODER_B);
  if (digitalRead(FINE_MOTION_XY_SWITCH)) //If we want to go in the X axis
  {
    //right_or_left = !B_set; //If A and B are different we're goint right. If they're the same we're going left
    fine_sign = (B_set) ? -1 : 1;
    fine_axis = 0;    // move on x
  }
  else //If we want to go in the Y axis
  {
   // up_or_down = !B_set; //If A and B are different we're goint up. If they're the same we're going down
    fine_sign = (B_set) ? -1 : 1;
   fine_axis = 1; // move on Y
  }
  //Serial.println("OK_FineMotion");
}

/****************************************************************************************************************************************************************************************
* Filter wheel interrupt
*****************************************************************************************************************************************************************************************/

void FW_InterruptRoutine()
{
   FWMotor(-2); //Break fast
  //FWMotor(0); //then stop
  Serial.println("OK");
}

//void fineMotionB()
//{
//  B_set = !B_set; //Change in channel B's status
//}
