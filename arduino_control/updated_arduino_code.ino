//#include <NewPing.h>
//
//#define TRIGGER_PIN  10
//#define ECHO_PIN     9
//#define MAX_DISTANCE 10 // Maximum distance we want to measure (in centimeters).
//
//
//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
//
//void setup() {
//  Serial.begin(115200);
//}
//
//void loop() {
//  delay(1000);                    // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
//
//  int distance = sonar.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
//
//  Serial.print("Distance: ");
//  Serial.print(distance);
//  Serial.println("cm");
//}












// TODO:
// Add deadzone
// maybe use pulseInWait


// Arduino reads pin 13 and responds over Serial
#include "CytronMotorDriver.h"
#include <NewPing.h>

// Buffer to store cmd_vel data
const size_t BUF_SIZE = 32;
char cmd_vel_array[BUF_SIZE];

// Motor driver pins
uint8_t leftWheelDir = 23;
uint8_t rightWheelDir = 28;
uint8_t leftWheelPWM = 2;
uint8_t rightWheelPWM = 3;

// Configure the motor driver.
CytronMD rightMotor(PWM_DIR, rightWheelPWM, rightWheelDir); // PWM 2 = Pin 9, DIR 2 = Pin 10.
CytronMD leftMotor(PWM_DIR, leftWheelPWM, leftWheelDir);  // PWM 1 = Pin 3, DIR 1 = Pin 4.

// Maximum cmd_vel params
const float MAX_LINEAR_VEL = 0.5;   // Maximum linear velocity in m/s
const float MAX_ANGULAR_VEL = 1.0;  // Maximum angular velocity in rad/s

// Ultrasonic sensor pins
 const int trigPin1 = 6;
 const int echoPin1 = 7;
 const int maxDist1 = 20; //cm
 const int stopDist1 = 10;

const int trigPin2 = 8;
const int echoPin2 = 9;
const int maxDist2 = 20; //cm
const int stopDist2 = 4;

 const int trigPin3 = 10;
 const int echoPin3 = 11;
 const int maxDist3 = 20; //cm
 const int stopDist3 = 10;

// Setup ultrasonic sensor
// NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
 NewPing sonar1(trigPin1, echoPin1, maxDist1);
 NewPing sonar2(trigPin2, echoPin2, maxDist2);
 NewPing sonar3(trigPin3, echoPin3, maxDist3);

// FS i6X pins
double ch2=49;
double ch3=51;
double ch4=53;
int test = 50;
bool enableMotor = false;
int motorSpeedLeft = 0;
int motorSpeedRight = 0;

unsigned long last_cmd_vel_time = 0;
const unsigned long CMD_VEL_TIMEOUT = 200;  // ms


void setup() {
  leftMotor.setSpeed(0);  
  rightMotor.setSpeed(0); 
  pinMode(test, OUTPUT);

  pinMode(49,INPUT);
  pinMode(51,INPUT);
  pinMode(53,INPUT);

  Serial.begin(115200);
}

void motorRunStart() {
  // Get pulses from FS i6X receiver
  ch2 = pulseIn(49,HIGH);
  ch3 = pulseIn(51,HIGH);
  ch4 = pulseIn(53,HIGH);

  if ((ch2<1000)||(ch2>1600)||
      (ch4<1000)||(ch4>1600)) {
        
    if(!enableMotor) {
      Serial.println(enableMotor);
      motorSpeedLeft = 50;
      motorSpeedRight = 50;
      motorRunNormal();
      enableMotor = true;
    }
    else {
      motorSpeedLeft = 50;
      motorSpeedRight = 50;
      motorRunNormal();
      Serial.println(enableMotor);
    }
    
  }
  else {
    motorSpeedLeft = 0;
    motorSpeedRight = 0;
    motorRunNormal();
    enableMotor = false;
  }
}

void motorRunNormal() {
//  // Get pulses from FS i6X receiver
//  ch2 = pulseIn(49,HIGH);
//  ch3 = pulseIn(51,HIGH);
//  ch4 = pulseIn(53,HIGH);

  if(ch3 < 1600)
  {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
  }
  // Left motor slightly faster due to hardware issue
  else{
    // Forward
    if(ch2 > 1600)
    {   
      rightMotor.setSpeed(motorSpeedLeft);
      leftMotor.setSpeed(motorSpeedRight); 
    }

    // Backward
    else if(ch2 < 1000)
    {
      if (!obstacleInDirection("back")) {
        leftMotor.setSpeed(-motorSpeedLeft);  
        rightMotor.setSpeed(-motorSpeedRight);
      } else {
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
      }
    }
    
    // Left
    else if(ch4 < 1000)
    {
      if (!obstacleInDirection("left")) {
        leftMotor.setSpeed(-motorSpeedLeft);  
        rightMotor.setSpeed(motorSpeedRight);
      } else {
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
      }
    }
    
    // Right
    else if(ch4 > 1600)
    {
      if (!obstacleInDirection("right")) {
        leftMotor.setSpeed(motorSpeedLeft);  
        rightMotor.setSpeed(-motorSpeedRight);
      } else {
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
      }
    }

    else
    {
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0);
    }
  }
}

bool obstacleInDirection(String dir) {
  if (dir == "back") {
    int dist = sonar2.ping_cm();
    if (dist > 0 && dist < stopDist2) return true;
  }
  else if (dir == "left") {
    int dist = sonar1.ping_cm();
    if (dist > 0 && dist < stopDist1) return true;
  }
  else if (dir == "right") {
    int dist = sonar3.ping_cm();
    if (dist > 0 && dist < stopDist3) return true;
  }
  return false;
}

void loop() {
  // // Get pulses from FS i6X receiver
  // ch2 = pulseIn(49,HIGH);
  // ch3 = pulseIn(51,HIGH);
  // ch4 = pulseIn(53,HIGH);

  // Get sonar ping dist in cm
  // int distance = sonar1.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)

  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.println("cm");

  // int distance2 = sonar2.ping_cm();
  // if ((distance2 > 0) && (distance2 < 4)) {
  //       leftMotor.setSpeed(0);  
  //       rightMotor.setSpeed(0);
  //       digitalWrite(test, HIGH); 
  //     }

  // if(ch3 < (1430-deadzone))
  // {
  //   leftMotor.setSpeed(0);
  //   rightMotor.setSpeed(0);
  // }
  // else{
  //   if(ch2 > (1530+deadzone))
  //   {  
  //     leftMotor.setSpeed(45);  
  //     rightMotor.setSpeed(45);
  //   }

  //   // Backward
  //   else if(ch2 < (1430-deadzone))
  //   {  
  //     leftMotor.setSpeed(-45);  
  //     rightMotor.setSpeed(-45);
  //   }

  //   // Left
  //   else if(ch4 < (1430-deadzone))
  //   {  
  //     leftMotor.setSpeed(-45);  
  //     rightMotor.setSpeed(45);
  //   }

  //   // Right
  //   else if(ch4 > (1530+deadzone))
  //   {  
  //     leftMotor.setSpeed(45);  
  //     rightMotor.setSpeed(-45);
  //   }
  //   else
  //   {
  //     leftMotor.setSpeed(0);
  //     rightMotor.setSpeed(0);
  //   }
  // }

  // Read pulses once with timeout to avoid blocking forever
  ch2 = pulseIn(49, HIGH, 50000);  // 25ms timeout
  ch3 = pulseIn(51, HIGH, 50000);
  ch4 = pulseIn(53, HIGH, 50000);

  bool rc_active = false;

  // Use ch3 as enable switch: if ch3 is outside 1000-1600 range, RC control is active
  if (ch3 > 1600) {
    // Then check if ch2 or ch4 is commanding movement (outside 1000-1600 range)
    if (ch2 < 1000 || ch2 > 1600 || ch4 < 1000 || ch4 > 1600) {
      rc_active = true;
    }
  }

  if (rc_active) {
    motorRunStart();           // run RC override control
    last_cmd_vel_time = millis();  // reset cmd_vel timer so no fallback to cmd_vel
    return;
  }

  // check serial connection with Jetson
  else if (Serial && Serial.available()) {

    // get /cmd_vel topic from Jetson

    // read from serial until \n or 31 bytes
    size_t len = Serial.readBytesUntil('\n', cmd_vel_array, BUF_SIZE - 1);
    if (len == 0) {
      return;
    }
    // terminate array as string
    cmd_vel_array[len] = '\0'; 

    // sanity check to see if read data is VEL:
    if (strncmp(cmd_vel_array, "VEL:", 4) != 0) {
      return;
    }

    char *data_start = cmd_vel_array + 4;     // remove VEL: from serial input
    char *token = strtok(data_start, ",");    // split cmd_vel from ','
    if (token == NULL) return;
    float x = atof(token);                    // get x

    token = strtok(NULL, ",");                // get data after ','
    if (token == NULL) return;
    float z = atof(token);                    // get z

    // Cumulative movement of both motors
    float left_speed = x - z;
    float right_speed = x + z;

    Serial.println(left_speed);
    Serial.println(right_speed);
    int left_pwm = (int)(constrain(left_speed / MAX_LINEAR_VEL, -1.0, 1.0) * 50);
    int right_pwm = (int)(constrain(right_speed / MAX_LINEAR_VEL, -1.0, 1.0) * 50);
    Serial.println(left_pwm);
    Serial.println(right_pwm);

    // Get sonar ping dist in cm
    // int distance1 = sonar1.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
    // int distance2 = sonar2.ping_cm();
    // int distance3 = sonar3.ping_cm();

    //   Serial.print("ULT:");
    //   Serial.println(distance2);

    // Normal motor movement
    // Check ultrasonic sensors based on intended direction
    bool stop = false;
    
    if (left_pwm < 0 && right_pwm < 0 && obstacleInDirection("back")) {
      stop = true;
    } else if (left_pwm < 0 && right_pwm > 0 && obstacleInDirection("left")) {
      stop = true;
    } else if (left_pwm > 0 && right_pwm < 0 && obstacleInDirection("right")) {
      stop = true;
    }
    
    if (stop) {
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0);
      digitalWrite(test, HIGH);  // optional indicator
    } else {
      leftMotor.setSpeed(left_pwm);  
      rightMotor.setSpeed(right_pwm); 
      digitalWrite(test, LOW);
    }
 

    // Sanity check using us

    // if ((left_pwm < 0) && (right_pwm < 0)) {   //check backside
      // Get left us reading
      // int distance1 = sonar1.ping_cm();
      // Serial.print("ULT:");
      // Serial.println(distance1);
      
      // if ((distance1 > 0) && (distance1 < stopDist1)) {
      //   leftMotor.setSpeed(0);  
      //   rightMotor.setSpeed(0);
      //   digitalWrite(test, HIGH); 
      // }
    // }

    // if ((left_pwm < 0) && (right_pwm > 0)) {   //check left
      // Get left us reading
      // int distance2 = sonar2.ping_cm();
      // Serial.print("ULT:");
      // Serial.println(distance2);

      
      // if ((distance2 > 0) && (distance2 < stopDist2)) {
      //   leftMotor.setSpeed(0);  
      //   rightMotor.setSpeed(0);
      //   digitalWrite(test, HIGH); 
      // }
    // }

    //  if ((left_pwm > 0) && (right_pwm < 0)) {   //check left
      // Get left us reading
      // int distance3 = sonar3.ping_cm();
      // Serial.print("ULT:");
      // Serial.println(distance3);

      
      // if ((distance3 > 0) && (distance3 < stopDist3)) {
      //   leftMotor.setSpeed(0);  
      //   rightMotor.setSpeed(0);
      //   digitalWrite(test, HIGH); 
      
      // }
    // }





    // Serial.print("Left: "); Serial.println(left_pwm);
    // Serial.print("  Right: "); Serial.println(right_pwm);


    // clear buffers
    // Serial.flush();
    // ✅ Update the last time we received a command
    last_cmd_vel_time = millis();// ✅ Update the last time we received a command
    last_cmd_vel_time = millis();
  }

  else {
    // ✅ Check timeout
    if (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT) {
      motorRunStart();  // Fall back to RC only if no new cmd_vel for 200ms
    }
  }

  delay(50);
}
