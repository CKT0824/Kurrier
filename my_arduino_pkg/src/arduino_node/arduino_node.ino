#include <ros.h>
#include <std_msgs/UInt16.h>

// Define pins
const int rearLeftPWM = 2;
const int rearLeftDir = 3;
const int rearRightPWM = 4;
const int rearRightDir = 5;
const int frontRightPWM = 6;
const int frontRightDir = 7;
const int frontLeftPWM = 8;
const int frontLeftDir = 9;

// ROS Node handle
ros::NodeHandle nh;

// Motor control variables
int pwmValue;

// Motor control callback
void pwmCallback(const std_msgs::UInt16& msg) {
  pwmValue = msg.data;

  // Your motor control logic here
  // Example: move forward with received PWM value
  analogWrite(frontLeftPWM, pwmValue);
  digitalWrite(frontLeftDir, HIGH);
  // Implement the same logic for other motors

  // Determine ppm value based on pwm_to_arduino value
  if (pwmValue == 2) {
    moveForward();
  } else if (pwmValue == 1) {
    turnRight();
  } else if (pwmValue == 3) {
    turnLeft();
  } else {
    stop();
  }
}

// ROS Publisher and Subscriber
ros::Subscriber<std_msgs::UInt16> sub("pwm_to_arduino", &pwmCallback);

void setup() {
  // Initialize ROS
  nh.initNode();
 
  // Subscribe to the pwm_to_arduino topic
  nh.subscribe(sub);

  // Set PIN
  pinMode(frontLeftPWM, OUTPUT);
  pinMode(frontLeftDir, OUTPUT);
  pinMode(frontRightPWM, OUTPUT);
  pinMode(frontRightDir, OUTPUT);
  pinMode(rearLeftPWM, OUTPUT);
  pinMode(rearLeftDir, OUTPUT);
  pinMode(rearRightPWM, OUTPUT);
  pinMode(rearRightDir, OUTPUT);

  // Begin serial
  Serial.begin(57600);
}

void loop() {
  // Handle ROS communication
  nh.spinOnce();
    // Keyboard input

  // Your additional loop logic here
}

// Motor control functions
void moveForward() {
  digitalWrite(frontLeftDir, HIGH);
  analogWrite(frontLeftPWM, 70);
  digitalWrite(frontRightDir, HIGH);
  analogWrite(frontRightPWM, 70);
  digitalWrite(rearLeftDir, HIGH);
  analogWrite(rearLeftPWM, 70);
  digitalWrite(rearRightDir, HIGH);
  analogWrite(rearRightPWM, 70);
}

void moveBackward() {
  digitalWrite(frontLeftDir, LOW);
  analogWrite(frontLeftPWM, 30);
  digitalWrite(frontRightDir, LOW);
  analogWrite(frontRightPWM, 30);
  digitalWrite(rearLeftDir, LOW);
  analogWrite(rearLeftPWM, 30);
  digitalWrite(rearRightDir, LOW);
  analogWrite(rearRightPWM, 30);
}

void turnLeft() {
  digitalWrite(frontLeftDir, LOW);
  analogWrite(frontLeftPWM, 40);
  digitalWrite(frontRightDir, HIGH);
  analogWrite(frontRightPWM, 40);
  digitalWrite(rearLeftDir, LOW);
  analogWrite(rearLeftPWM, 40);
  digitalWrite(rearRightDir, HIGH);
  analogWrite(rearRightPWM, 40);
}

void turnRight() {
  digitalWrite(frontLeftDir, HIGH);
  analogWrite(frontLeftPWM, 40);
  digitalWrite(frontRightDir, LOW);
  analogWrite(frontRightPWM, 40);
  digitalWrite(rearLeftDir, HIGH);
  analogWrite(rearLeftPWM, 40);
  digitalWrite(rearRightDir, LOW);
  analogWrite(rearRightPWM, 40);
}

void stop() {
  analogWrite(frontLeftPWM, 0);
  analogWrite(frontRightPWM, 0);
  analogWrite(rearLeftPWM, 0);
  analogWrite(rearRightPWM, 0);
}
