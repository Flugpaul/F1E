/**************************************************************************/
/*
 F1E-Steering with Sensor Adafruit BNO055 (Orientation-Fusion in sensor) 
 and Seeduino Xiao
 Moves the servo if there is a deviation to desired/wanted direction 
 tested with UNO and NANO Boards
 written for ARDUINO
 Paul Seren / Germany / 07.01.2022 Ver. 0.9
*/
/**************************************************************************/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Lib. for OLED Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include <Servo.h> // use of the PWM-library
//#include <Adafruit_TiCoServo.h> 

// Set the delay between fresh samples /
#define BNO055_SAMPLERATE_DELAY_MS (100)
// Check I2C device address from BNO055 and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Variables for sensor evaluation
float roll, pitch, heading;
int delayTime=500;

char * RevText = "F1E-Control - Rev.0.9";
// Variables for some conversion stuff

// Variables for direction change / F1E_Servo_Move ()

int current_Bearing = 180;                          // INT-Variable for current heading
int wanted_Bearing = 180;                           // INT-Variable for desired heading
int delta_Bearing;                                  // INT-(+-)Variable for deviation from desired heading

// Servo Variables

int servoPinF1E     =  9;    // Servo Pin for <Adafruit_TiCoServo.h> library to avoid servo jitters https://forums.adafruit.com/viewtopic.php?f=22&t=96847 
                          // https://learn.adafruit.com/neopixels-and-servos/the-ticoservo-library
int minGrad      =  45;   // minimum servo position  45°
int maxGrad     =  135;   // maximum servo position  135°
int centerGrad  = 90;     // midle position Servos
int servoPinDT    =   10;   // Dummy Servo for later use
int servoDirection = 1;   // Servo sign (+/-)

int servoPos;             // Servo position
int servoPosDT;             // Servo position for later use


// Servo-Trimmungs-Varianten
const int potiPin = A0;   // Analog-Input for Trimpoti
int sensor = 0;           // sensor-Value on Analog-Input (0 - 1023) 
int pwm_F1E = 0;              // calculated sensor-value for the PWM-Trim-Value (-maxTrim/2 bis + maxTrim/2)
int pwmOld = 0;           // PWM-value to remember for next loop
int maxTrim = 40;         // max. Trim-degrees

//Servo für ToCoServo.h   servoF1E; 
//Adafruit_TiCoServo servoF1E; 
// Adafruit_TiCoServo servoDT; 
//Servo für servo.h 
Servo servoF1E; 
Servo servoDT; 
// Button Variables
int SW1          =  6;    // Input button / digital pin:  input for desired heading and trimming 
int SW2          =  8;    // Input button / digital pin:  input for DT-Timer-Function for later use 
int SW3          =  7;     // Input button / digital pin: sign (+/-) for Servo direction. 

//PID-correction

int abs_delta_Bearing = 0; 
int abs_delta_Bearing_n1 = 0; // default value deviation one measurement before
int abs_delta_Bearing_n2 = 0; // default value deviation two measurements before
int PID_Input;
int PID_Output;

float sensor_P;
float sensor_I;
float sensor_D;

float PID_Kp = 1.0;
float PID_Ki = 0;
float PID_Kd = 0;

int PID_potiPin = A1; 

 
void setup() {

// Initialize OLED Display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(1,1);
  display.setTextSize(1);
  display.println(RevText);
  display.display();
  delay(1000);
  
  
 /* Initialise serial Com Board*/
 // SerialUSB.begin(9600);
 //Serial.begin(115200);
  //while (!Serial) yield();

  /* Initialise the internal LED for calibration Status servo */
  pinMode(LED_BUILTIN,OUTPUT);

  // Set the switch-Buttons
  pinMode(SW1,INPUT);           // define button type
  digitalWrite(SW1,HIGH);       // button default High
  pinMode(SW2,INPUT);           // define button type
  digitalWrite(SW2,HIGH);       // button default High
   pinMode(SW3,INPUT);           // define button type
  digitalWrite(SW3,HIGH);       // button default High
  
  /* Initialise the servo */
  servoF1E.attach(servoPinF1E);    // connect Servo F1E
  servoDT.attach(servoPinDT);    // connect Servo DT
  servoPos = centerGrad;        // move servo to default midle position

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    // SerialUSB.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      /* display.clearDisplay();
       display.println("No BNO055 detected");
       display.display();
        */
     Oprintln ("No BNO055 detected");
     while(1);
  }

    Oprintln ("BNO055 detected");
    delay (1000);
  /* 
  displaySensorDetails();
  displaySensorStatus();
  displayCalStatus();
  */
  servoF1E.write(minGrad);
 
  // Check calibration status in a loop. End setup only, when calibration is sufficient
  //Calibrate
  Oprintln ("Calibration started");
  while (Analyse_CalStatus())
  {
  //Serial.println("Not yet sufficiant calibrated");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(delayTime);
  digitalWrite(LED_BUILTIN, LOW);
  delay(delayTime);
  }
  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);
  Oprintln ("Calibration ended!");
  delay(1000);
  F1E_Get_PID_K ();
  
  if(!digitalRead(SW3))   
  {
  servoDirection = (-1*servoDirection);
  Oprintln ("Servo reverse" );
  delay (1000);
  } 
  
  Oprintln ("Fly a Max! ");
  delay (1000);
  
  F1E_Get_Current_Bearing ();
  wanted_Bearing = current_Bearing; 
  
  F1E_Get_Wanted_Trim ();
  pwmOld = pwm_F1E; 
  F1E_Servo_Move ();
  
}



void loop() {
  
  F1E_Get_Current_Bearing (); 
  F1E_Get_Wanted_Bearing (); 
  F1E_Servo_Move ();
  //DT_Interupt ();
}

int Oprintln(char * PrintText)
{
  display.clearDisplay();
  display.setCursor(5,10);
  display.println(PrintText);
  display.display(); 
}

/**************************************************************************/
/*
    Analyse the Calibration status 
*/
/**************************************************************************/
bool Analyse_CalStatus ()
  {
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // If the sensor is not sufficient calibrated, return 1. 
  if (gyro > 2 && mag > 2)  {return 0;}
  return 1;    
  }

/**************************************************************************/
/*
    Determine the desired bearing 
*/
/**************************************************************************/
void F1E_Get_PID_K ()
  {
 
    PID_potiPin = A1;
    sensor_P = analogRead(PID_potiPin);
    PID_Kp = sensor_P/1023;
    
    PID_potiPin = A2;
    sensor_I = analogRead(PID_potiPin);
    PID_Ki = sensor_I/10230;

    PID_potiPin = A3;
    sensor_D = analogRead(PID_potiPin);
    PID_Kd = sensor_D/(2*1023);
  /* display.clearDisplay();
  display.setCursor(5,10);
  display.print(PID_Kp);
  display.print(" | ");
  display.print(PID_Ki);
  display.print(" | ");
  display.print(PID_Kd);
  display.display(); 
  */
    //Serial.print("sensor_P = ");
    //Serial.print(sensor_P);
    //Serial.print("PID_Kp = ");
    //Serial.print(PID_Kp);
    //Serial.print("|| sensor_I = ");
    //Serial.print(sensor_I);
    //Serial.print("PID_Ki = ");
    //Serial.print(PID_Ki);
    //Serial.print("|| sensor_D = ");
    //Serial.print(sensor_D);
    //Serial.print("PID_Kd = ");
    //Serial.println(PID_Kd);
    
  }  

/**************************************************************************/
/*
    Move DT Servo / do an action (Dummy for later use) 
*/
/**************************************************************************/
void DT_Interupt ()
{
if(!digitalRead(SW2))   
  {
  DT_Servo_Move ();
  }
}

/**************************************************************************/
/*
    Determine the desired trim 
*/
/**************************************************************************/
void F1E_Get_Wanted_Trim ()
  {
    // Check if there is a change necessary for the middle position 
    sensor = analogRead(potiPin);
    pwm_F1E = map(sensor, 0, 1023, 0, maxTrim) - maxTrim/2;
    
    // new wanted trim
    centerGrad = centerGrad - (pwm_F1E-pwmOld);
    // adjust min and max Grad
    minGrad=minGrad-(pwm_F1E-pwmOld);
    maxGrad=maxGrad-(pwm_F1E-pwmOld);
    pwmOld = pwm_F1E; 

    return;
  }
/**************************************************************************/
/*
    Determine the desired bearing 
*/
/**************************************************************************/
void F1E_Get_Wanted_Bearing ()
  {
  if(!digitalRead(SW1)) 
    { 
    // Check if there is a change necessary for the middle position 
    F1E_Get_Wanted_Trim ();
    
    // new wanted bearing
    wanted_Bearing = current_Bearing;
    
   
    // Move servo position to the (new) middle position
    // servoPos = centerGrad;
    //Serial.print("Sensor = ");
    //Serial.print(sensor);
    //Serial.print("Centergrad = ");
    //Serial.print(centerGrad);
    //Serial.print(" pwm_F1E  = ");
    //Serial.println(pwm_F1E);
  
    //Serial.print("Direction button pressed - new direction and trim"); 
    //Serial.println();
    F1E_Get_PID_K ();
       
    }  
   return;
  }

/**************************************************************************/
/*
    Moves the serco if there is a deviation to desired/wanted direction 
*/
/**************************************************************************/
void F1E_PID ()
  {
  
  PID_Input = abs_delta_Bearing ; 
  // Output mit PID - Kp modifizieren 
  F1E_Get_PID_K ();
  PID_Output = PID_Kp * abs_delta_Bearing + PID_Ki * (abs_delta_Bearing + abs_delta_Bearing_n1 + abs_delta_Bearing_n2) + PID_Kd * (abs_delta_Bearing - abs_delta_Bearing_n1); 
  }
/**************************************************************************/
/*
    Moves the F1E-servo if there is a deviation to desired/wanted direction 
*/
/**************************************************************************/
void F1E_Servo_Move ()
  {
    delta_Bearing =  wanted_Bearing - current_Bearing; 
    //Modulo for beeing aware to 0/360° change of direction and calculation only the deviation/direction for corretion  
    //Serial.print("Modulo degrees change from desired direction: "); 

    //Remember to previous delta_bearing - they will be used for PID calculation
    abs_delta_Bearing_n2 = abs_delta_Bearing_n1;
    abs_delta_Bearing_n1 = abs_delta_Bearing;
    abs_delta_Bearing = ((delta_Bearing + 540) % 360) - 180; 

     
    F1E_PID();
    //servoPos = centerGrad - PID_Output;
    servoPos = centerGrad - (servoDirection*PID_Output);
    
     display.clearDisplay();
     display.setCursor(5,18);
     display.println(servoPos);
     display.setCursor(30,18);
     display.println(wanted_Bearing);
     display.setCursor(50,18  );
     display.println(current_Bearing);
     display.display(); 
     //delay (10);
    // Limitation of correction  
    if(servoPos > maxGrad) {servoPos = maxGrad;} 
    if(servoPos < minGrad) {servoPos = minGrad;} 
    
    // go to new Servo position
    servoF1E.write(servoPos);
    
   return;
  }
/**************************************************************************/
/*
    Moves the DT-servo if there is a deviation to desired/wanted direction 
*/
/**************************************************************************/
void DT_Servo_Move ()
  {
/*Aktuell noch Dummy Code um überhaupt was zu tun*/

       
    // go to new Servo position
    servoPosDT = 90;
    servoDT.write(servoPosDT);
    delay (1000);
    servoPosDT = 135;
    servoDT.write(servoPosDT);
    delay (1000);
    servoPosDT = 45;
    servoDT.write(servoPosDT);
    delay (1000);
  }


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  //Serial.println("------------------------------------");
  //Serial.print  ("Sensor:       "); //Serial.println(sensor.name);
  //Serial.print  ("Driver Ver:   "); //Serial.println(sensor.version);
  //Serial.print  ("Unique ID:    "); //Serial.println(sensor.sensor_id);
  //Serial.print  ("Max Value:    "); //Serial.print(sensor.max_value); //Serial.println(" xxx");
  //Serial.print  ("Min Value:    "); //Serial.print(sensor.min_value); //Serial.println(" xxx");
  //Serial.print  ("Resolution:   "); //Serial.print(sensor.resolution); //Serial.println(" xxx");
  //Serial.println("------------------------------------");
  //Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  //Serial.println("");
  //Serial.print("System Status: 0x");
  //Serial.println(system_status, HEX);
  //Serial.print("Self Test:     0x");
  //Serial.println(self_test_results, HEX);
  //Serial.print("System Error:  0x");
  //Serial.println(system_error, HEX);
  //Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  //Serial.print("\t");
  if (!system)
  {
    //Serial.print("! ");
  }

  /* Display the individual values */
  //Serial.print("Sys:");
  //Serial.print(system, DEC);
  //Serial.print(" G:");
  //Serial.print(gyro, DEC);
  //Serial.print(" A:");
  //Serial.print(accel, DEC);
  //Serial.print(" M:");
  //Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    determine current heading
*/
/**************************************************************************/
void F1E_Get_Current_Bearing ()
 {
 imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
 heading = euler.x();
 current_Bearing = heading; // Wandlung auf int

  /* Display the floating point data */
  //Serial.print("X: ");
  //Serial.print(euler.x());
  //Serial.print(" Y: ");
  //Serial.print(euler.y());
  //Serial.print(" Z: ");
  //Serial.print(euler.z());
  //Serial.print("\t\t");
  
// print the heading, pitch and roll    Serial.print("Orientation: ");
  //Serial.print(heading);
  //Serial.print(", ");
   //Serial.print(current_Bearing);
  //Serial.print(", ");
  //Serial.print(wanted_Bearing);
  //Serial.print(", ");
  //Serial.print(pitch);
  //Serial.print(", ");
  //Serial.println(roll);
}
