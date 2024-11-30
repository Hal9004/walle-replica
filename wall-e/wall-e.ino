/**
 * WALL-E CONTROLLER CODE
 *
 * @file       wall-e.ino
 * @brief      Main Wall-E Controller Sketch
 * @author     Simon Bluett
 * @email      hello@chillibasket.com
 * @copyright  Copyright (C) 2021 - Distributed under MIT license
 * @version    2.9
 * @date       29th May 2021
 *
 * HOW TO USE:
 * 1. Install the Adafruit_PWMServoDriver library
 *    a. In the Arduino IDE, go to Sketch->Include Library->Manage Libraries
 *    b. Search for Adafruit PWM Library, and install the latest version
 * 2. Calibrate the servo motors, using the calibration sketch provided in the
 *    GitHub repository. Paste the calibrated values between lines 144 to 150.
 * 3. Upload the sketch to the micro-controller, and open the serial monitor 
 *    at a baud rate of 115200.
 * 4. Additional instructions and hints can be found at:
 *    https://wired.chillibasket.com/3d-printed-wall-e/
 */

#include <Wire.h>
#include "Queue.hpp"
#include <Adafruit_MotorShield.h>
#include "../Adafruit_Motor_Shield_V2_Library/utility/Adafruit_MS_PWMServoDriver.h"

/**
 * Battery level detection
 *
 *   .------R1-----.-------R2------.     | The diagram to the left shows the  |
 *   |             |               |     | potential divider circuit used by  |
 * V_Raw     Analogue pin A2      GND    | the battery level detection system |
 *
 * @note The scaling factor is calculated according to ratio of the two resistors:
 *       DIVIDER_SCALING_FACTOR = R2 / (R1 + R2)
 *       For example: 47000 / (100000 + 47000) = 0.3197
 */

#define BATTERY_LEVEL_PIN A2
#define BATTERY_MAX_VOLTAGE 12.6
#define BATTERY_MIN_VOLTAGE 10.2
#define DIVIDER_SCALING_FACTOR 0.2481

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

#define TFT_CS 10 // 
#define TFT_RST -1 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC 11
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);


/// Define other constants
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
#define NUMBER_OF_SERVOS 7        // Number of servo motors
#define SERVO_UPDATE_TIME 10      // Time in milliseconds of how often to update servo and motor positions
#define SERVO_OFF_TIME 6000       // Turn servo motors off after 6 seconds
#define STATUS_CHECK_TIME 10000   // Time in milliseconds of how often to check robot status (eg. battery level)
#define CONTROLLER_THRESHOLD 1    // The minimum error which the dynamics controller tries to achieve
#define MAX_SERIAL_LENGTH 5       // Maximum number of characters that can be received

/// Instantiate Objects
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
Adafruit_MS_PWMServoDriver pwm = Adafruit_MS_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Set up motor controller objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Adafruit_DCMotor *motorR = AFMS.getMotor(2);
bool reverseL;
bool reverseR;

// Queue for animations - buffer is defined outside of the queue class
// so that the compiler knows how much dynamic memory will be used
struct animation_t {
	uint16_t timer;
	int8_t servos[NUMBER_OF_SERVOS]; 
};

#define QUEUE_LENGTH 40
animation_t buffer[QUEUE_LENGTH];
Queue <animation_t> queue(QUEUE_LENGTH, buffer);


/// Motor Control Variables
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
int pwmspeed = 255;
int moveValue = 0;
int turnValue = 0;
int turnOffset = 0;
int motorDeadzone = 0;


/// Runtime Variables
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
unsigned long lastTime = 0;
unsigned long animeTimer = 0;
unsigned long motorTimer = 0;
unsigned long statusTimer = 0;
unsigned long updateTimer = 0;
bool autoMode = false;
bool booted = false;
bool warning = false;
#define DEBUG false


// Serial Parsing
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
char firstChar;
char serialBuffer[MAX_SERIAL_LENGTH];
uint8_t serialLength = 0;


// ****** SERVO MOTOR CALIBRATION *********************
// Servo Positions:  Low,High
int preset[][2] =  {{138,542},  // head rotation
                    {115,518},  // neck top
                    {98,448},  // neck bottom
                    {475,230},  // eye right
                    {270,440},  // eye left
                    {350,185},  // arm left
                    {188,360}}; // arm right
// *****************************************************


// Servo Control - Position, Velocity, Acceleration
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
// Servo Pins:	     0,   1,   2,   3,   4,   5,   6,   -,   -
// Joint Name:	  head,necT,necB,eyeR,eyeL,armL,armR,motL,motR
float curpos[] = { 248, 560, 140, 475, 270, 250, 290, 180, 180};  // Current position (units)
float setpos[] = { 248, 560, 140, 475, 270, 250, 290,   0,   0};  // Required position (units)
float curvel[] = {   0,   0,   0,   0,   0,   0,   0,   0,   0};  // Current velocity (units/sec)
float maxvel[] = { 500, 400, 500,2400,2400, 600, 600, 255, 255};  // Max Servo velocity (units/sec)
float accell[] = { 350, 300, 480,1800,1800, 500, 500, 800, 800};  // Servo acceleration (units/sec^2)



// -------------------------------------------------------------------
/// Initial setup
// -------------------------------------------------------------------

void setup() {

	// Initialize serial communication for debugging
	Serial.begin(115200);
	Serial.println(F("--- Wall-E Control Sketch ---"));

	randomSeed(analogRead(0));

	// Check if servo animation queue is working, and move servos to known starting positions
	if (queue.errors()) Serial.println(F("Error: Unable to allocate memory for servo animation queue"));
	
  // Communicate with servo shield (Analog servos run at ~60Hz)
	pwm.begin();
	pwm.setPWMFreq(50);
	// Soft start the servo motors
	Serial.println(F("Starting up the servo motors"));
	playAnimation(0);
	softStart(queue.pop(), 3500);
  // Turn off all servos
  for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
        pwm.setPWM(i, 0, 0);
    }

  // begin Motor Driver
  AFMS.begin();
  setSpeedL(motorL, 0);
  setSpeedR(motorR, 0);

	// Connect to display
  Serial.println(F("Starting up the display"));
  initDisplay();

	Serial.println(F("Startup complete; entering main loop"));

}

// -------------------------------------------------------------------
/// Read input from serial port
///
/// This function reads incoming characters in the serial port
/// and inserts them into a buffer to be processed later.
// -------------------------------------------------------------------

void readSerial() {

	// Read incoming byte
	char inchar = Serial.read();

	// If the string has ended, evaluate the serial buffer
	if (inchar == '\n' || inchar == '\r') {

		if (serialLength > 0) evaluateSerial();
		serialBuffer[0] = 0;
		serialLength = 0;

	// Otherwise add to the character to the buffer
	} else {
		if (serialLength == 0) firstChar = inchar;
		else {
			serialBuffer[serialLength-1] = inchar;
			serialBuffer[serialLength] = 0;
		}
		serialLength++;

		// To prevent overflows, evalute the buffer if it is full
		if (serialLength == MAX_SERIAL_LENGTH) {
			evaluateSerial();
			serialBuffer[0] = 0;
			serialLength = 0;
		}
	}
}



// -------------------------------------------------------------------
/// Evaluate input from serial port
///
/// Parse the received serial message which is stored in
/// the "serialBuffer" filled by the "readSerial()" function
// -------------------------------------------------------------------

void evaluateSerial() {

	// Evaluate integer number in the serial buffer
	int number = atoi(serialBuffer);

	Serial.print(firstChar); Serial.println(number);

  if (DEBUG) {
    String debugInfo = firstChar + String(number);
    showDebugInfo(debugInfo);
  }

	// Motor Inputs and Offsets
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	if (firstChar == 'X' && number >= -100 && number <= 100) {
    turnValue = int(number * 2.55);       // Left/right control
  } else if (firstChar == 'Y' && number >= -100 && number <= 100) {
    moveValue = int(number * 2.55);       // Forward/reverse control
  } else if (firstChar == 'S' && number >= -100 && number <= 100) {
    turnOffset = number;                  // Steering offset
  }	else if (firstChar == 'O' && number >=    0 && number <= 250) {
    motorDeadzone = int(number);          // Motor deadzone offset
  }

	// Animations
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	else if (firstChar == 'A') playAnimation(number);


	// Autonomous servo mode
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	else if (firstChar == 'M' && number == 0) autoMode = false;
	else if (firstChar == 'M' && number == 1) autoMode = true;


	// Manual servo control
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	else if (firstChar == 'L' && number >= 0 && number <= 100) {   // Move left arm
		autoMode = false;
		queue.clear();
		setpos[5] = int(number * 0.01 * (preset[5][1] - preset[5][0]) + preset[5][0]);
	} else if (firstChar == 'R' && number >= 0 && number <= 100) { // Move right arm
		autoMode = false;
		queue.clear();
		setpos[6] = int(number * 0.01 * (preset[6][1] - preset[6][0]) + preset[6][0]);
	} else if (firstChar == 'B' && number >= 0 && number <= 100) { // Move neck bottom
		autoMode = false;
		queue.clear();
		setpos[2] = int(number * 0.01 * (preset[2][1] - preset[2][0]) + preset[2][0]);
	} else if (firstChar == 'T' && number >= 0 && number <= 100) { // Move neck top
		autoMode = false;
		queue.clear();
		setpos[1] = int(number * 0.01 * (preset[1][1] - preset[1][0]) + preset[1][0]);
	} else if (firstChar == 'G' && number >= 0 && number <= 100) { // Move head rotation
		autoMode = false;
		queue.clear();
		setpos[0] = int(number * 0.01 * (preset[0][1] - preset[0][0]) + preset[0][0]);
	} else if (firstChar == 'E' && number >= 0 && number <= 100) { // Move eye left
		autoMode = false;
		queue.clear();
		setpos[4] = int(number * 0.01 * (preset[4][1] - preset[4][0]) + preset[4][0]);
	} else if (firstChar == 'U' && number >= 0 && number <= 100) { // Move eye right
		autoMode = false;
		queue.clear();
		setpos[3] = int(number * 0.01 * (preset[3][1] - preset[3][0]) + preset[3][0]);
	}
	

	// Manual Movements with WASD
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	else if (firstChar == 'w') {		// Forward movement
		moveValue = pwmspeed;
		turnValue = 0;
		setpos[0] = (preset[0][1] + preset[0][0]) / 2;
	}
	else if (firstChar == 'q') {		// Stop movement
		moveValue = 0;
		turnValue = 0;
		setpos[0] = (preset[0][1] + preset[0][0]) / 2;
	}
	else if (firstChar == 's') {		// Backward movement
		moveValue = -pwmspeed;
		turnValue = 0;
		setpos[0] = (preset[0][1] + preset[0][0]) / 2;
	}
	else if (firstChar == 'a') {		// Drive & look left
		moveValue = 0;
		turnValue = -pwmspeed;
		setpos[0] = preset[0][0];
	}
	else if (firstChar == 'd') {   		// Drive & look right
		moveValue = 0;
		turnValue = pwmspeed;
		setpos[0] = preset[0][1];
	}


	// Manual Eye Movements
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	else if (firstChar == 'j') {		// Left head tilt
		setpos[4] = preset[4][0];
		setpos[3] = preset[3][1];
	}
	else if (firstChar == 'l') {		// Right head tilt
		setpos[4] = preset[4][1];
		setpos[3] = preset[3][0];
	}
	else if (firstChar == 'i') {		// Sad head
		setpos[4] = preset[4][0];
		setpos[3] = preset[3][0];
	}
	else if (firstChar == 'k') {		// Neutral head
		setpos[4] = int(0.4 * (preset[4][1] - preset[4][0]) + preset[4][0]);
		setpos[3] = int(0.4 * (preset[3][1] - preset[3][0]) + preset[3][0]);
	}


	// Head movement
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	else if (firstChar == 'f') {		// Head up
		setpos[1] = preset[1][0];
		setpos[2] = (preset[2][1] + preset[2][0])/2;
	}
	else if (firstChar == 'g') {		// Head forward
		setpos[1] = preset[1][1];
		setpos[2] = preset[2][0];
	}
	else if (firstChar == 'h') {		// Head down
		setpos[1] = preset[1][0];
		setpos[2] = preset[2][0];
	}
	

	// Arm Movements
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	else if (firstChar == 'b') {		// Left arm low, right arm high
		setpos[5] = preset[5][0];
		setpos[6] = preset[6][1];
	}
	else if (firstChar == 'n') {		// Both arms neutral
		setpos[5] = (preset[5][0] + preset[5][1]) / 2;
		setpos[6] = (preset[6][0] + preset[6][1]) / 2;
	}
	else if (firstChar == 'm') {		// Left arm high, right arm low
		setpos[5] = preset[5][1];
		setpos[6] = preset[6][0];
	}

  // Reboot
  else if (firstChar == 'Z') {		// boot sequence
    tft.fillScreen(ST77XX_BLACK);
    booted = true;
    Serial.println(F("Booted"));
		bootScreen();
	}

}



// -------------------------------------------------------------------
/// Sequence and generate animations
// -------------------------------------------------------------------

void manageAnimations() {

	// If we are running an animation
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	if ((queue.size() > 0) && (animeTimer <= millis())) {
		// Set the next waypoint time
		animation_t newValues = queue.pop();
		animeTimer = millis() + newValues.timer;

		// Set all the joint positions
		for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
			// Scale the positions using the servo calibration values
			setpos[i] = int(newValues.servos[i] * 0.01 * (preset[i][1] - preset[i][0]) + preset[i][0]);
		}


	// If we are in autonomous mode and no movements are queued, generate random movements
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	} else if (autoMode && queue.empty() && (animeTimer <= millis())) {

		// For each of the servos
		for (int i = 0; i < NUMBER_OF_SERVOS; i++) {

			// Randomly determine whether or not to update the servo
			if (random(2) == 1) {

				// For most of the servo motors
				if (i == 0 || i == 1 || i == 5 || i == 6) {

					// Randomly determine the new position
					unsigned int min = preset[i][0];
					unsigned int max = preset[i][1];
					if (min > max) {
						min = max;
						max = preset[i][0];
					}
					
					setpos[i] = random(min, max+1);

				// Since the eyes should work together, only look at one of them
				} else if (i == 3) {

					int midPos1 = int((preset[i][1] - preset[i][0])*0.4 + preset[i][0]);
					int midPos2 = int((preset[i+1][1] - preset[i+1][0])*0.4 + preset[i+1][0]);

					// Determine which type of eye movement to do
					// Both eye move downwards
					if (random(2) == 1) {
						setpos[i] = random(midPos1, preset[i][0]);
						float multiplier = (setpos[i] - midPos1) / float(preset[i][0] - midPos1);
						setpos[i+1] = ((1 - multiplier) * (midPos2 - preset[i+1][0])) + preset[i+1][0];

					// Both eyes move in opposite directions
					} else {
						setpos[i] = random(midPos1, preset[i][0]);
						float multiplier = (setpos[i] - preset[i][1]) / float(preset[i][0] - preset[i][1]);
						setpos[i+1] = (multiplier * (preset[i+1][1] - preset[i+1][0])) + preset[i+1][0];
					}
				}

			}
		}

		// Finally, figure out the amount of time until the next movement should be done
		animeTimer = millis() + random(500, 3000);

	}
}



// -------------------------------------------------------------------
/// Manage the movement of the servo motors
///
/// @param  dt  Time in milliseconds since function was last called
///
/// This function uses the formulae:
///   (s = position, v = velocity, a = acceleration, t = time)
///   s = v^2 / (2*a)  <- to figure out whether to start slowing down
///   v = v + a*t      <- to calculate new servo velocity
///   s = s + v*t      <- to calculate new servo position
// -------------------------------------------------------------------

void manageServos(float dt) {

	bool moving = false;

	// For each of the servo motors
	for (int i = 0; i < NUMBER_OF_SERVOS; i++) {

		float posError = setpos[i] - curpos[i];

		// If position error is above the threshold
		if (abs(posError) > CONTROLLER_THRESHOLD && (setpos[i] != -1)) {

			moving = true;

			// Determine motion direction
			bool dir = true;
			if (posError < 0) dir = false;

			// Determine whether to accelerate or decelerate
			float acceleration = accell[i];
			if ((curvel[i] * curvel[i] / (2 * accell[i])) > abs(posError)) acceleration = -accell[i];

			// Update the current velocity
			if (dir) curvel[i] += acceleration * dt / 1000.0;
			else curvel[i] -= acceleration * dt / 1000.0;

			// Limit Velocity
			if (curvel[i] > maxvel[i]) curvel[i] = maxvel[i];
			if (curvel[i] < -maxvel[i]) curvel[i] = -maxvel[i];
			
			float dP = curvel[i] * dt / 1000.0;

			if (abs(dP) < abs(posError)) curpos[i] += dP;
			else curpos[i] = setpos[i];

			pwm.setPWM(i, 0, curpos[i]);

		} else {
			curvel[i] = 0;
		}
	}

	// Disable servos if robot is not moving
	// This helps prevents the motors from overheating
	if (moving) motorTimer = millis();
	else if (millis() - motorTimer >= SERVO_OFF_TIME) {
		for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
			pwm.setPWM(i, 0, 0);
		}
	}
}



// -------------------------------------------------------------------
/// Servo "Soft Start" function
/// 
/// This function tries to start the servos up servo gently,
/// reducing the sudden jerking motion which usually occurs
/// when the motors power up for the first time.
///
/// @param  targetPos  The target position of the servos after startup
/// @param  timeMs     Time in milliseconds in which soft start should complete
// -------------------------------------------------------------------

void softStart(animation_t targetPos, int timeMs) {

	for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
		if (targetPos.servos[i] >= 0) {
			curpos[i] = int(targetPos.servos[i] * 0.01 * (preset[i][1] - preset[i][0]) + preset[i][0]);

			unsigned long endTime = millis() + timeMs / NUMBER_OF_SERVOS;

			while (millis() < endTime) {
				pwm.setPWM(i, 0, curpos[i]);
				delay(10);
				pwm.setPWM(i, 0, 0);
				delay(50);
			}
			pwm.setPWM(i, 0, curpos[i]);
			setpos[i] = curpos[i];
		}
	}
}

// Set motor speed
void setSpeedL(Adafruit_DCMotor* motor, int pwmValue) {
    // Bound the PWM value to +-255
    if (pwmValue > 255) pwmValue = 255;
    else if (pwmValue < -255) pwmValue = -255;

    // Forward direction
    if (pwmValue > 0) {
        motor->run(FORWARD);

    // Reverse direction
    } else if (pwmValue < 0 ) {
        motor->run(BACKWARD);

    } else if (pwmValue == 0) {
      motor->run(RELEASE);
    }
    
    // Set the motor speed (absolute value of pwmValue)
    motor->setSpeed(abs(pwmValue));
}
void setSpeedR(Adafruit_DCMotor* motor, int pwmValue) {
    // Bound the PWM value to +-255
    if (pwmValue > 255) pwmValue = 255;
    else if (pwmValue < -255) pwmValue = -255;

    // Forward direction
    if (pwmValue > 0) {
        motor->run(FORWARD);

    // Reverse direction
    } else if (pwmValue < 0 ) {
        motor->run(BACKWARD);

    } else if (pwmValue == 0) {
      motor->run(RELEASE);
    }
    
    // Set the motor speed (absolute value of pwmValue)
    motor->setSpeed(abs(pwmValue));
}

// -------------------------------------------------------------------
/// Manage the movement of the main motors
///
/// @param  dt  Time in milliseconds since function was last called
// -------------------------------------------------------------------

void manageMotors(float dt) {

	// Update Main Motor Values
	setpos[NUMBER_OF_SERVOS] = moveValue - turnValue;
	setpos[NUMBER_OF_SERVOS + 1] = moveValue + turnValue;

	// // Apply turn offset (motor trim) only when motors are active
	// if (setpos[NUMBER_OF_SERVOS] != 0) setpos[NUMBER_OF_SERVOS] -= turnOffset;
	// if (setpos[NUMBER_OF_SERVOS + 1] != 0) setpos[NUMBER_OF_SERVOS + 1] += turnOffset;

	for (int i = NUMBER_OF_SERVOS; i < NUMBER_OF_SERVOS + 2; i++) {

		float velError = setpos[i] - curvel[i];

		// If velocity error is above the threshold
		if (abs(velError) > CONTROLLER_THRESHOLD && (setpos[i] != -1)) {

			// Determine whether to accelerate or decelerate
			float acceleration = accell[i];
			if (setpos[i] < curvel[i] && curvel[i] >= 0) acceleration = -accell[i];
			else if (setpos[i] < curvel[i] && curvel[i] < 0) acceleration = -accell[i]; 
			else if (setpos[i] > curvel[i] && curvel[i] < 0) acceleration = accell[i];

			// Update the current velocity
			float dV = acceleration * dt / 1000.0;
			if (abs(dV) < abs(velError)) curvel[i] += dV;
			else curvel[i] = setpos[i];
		} else {
			curvel[i] = setpos[i];
		}

		// Apply deadzone offset
		// if (curvel[i] > 0) curvel[i] += motorDeadzone;
		// else if (curvel[i] < 0) curvel[i] -= motorDeadzone; 

		// Limit Velocity
		if (curvel[i] > maxvel[i]) curvel[i] = maxvel[i];
		if (curvel[i] < -maxvel[i]) curvel[i] = -maxvel[i];
	}

	// Update motor speeds
	setSpeedL(motorL, curvel[NUMBER_OF_SERVOS]);
	setSpeedR(motorR, curvel[NUMBER_OF_SERVOS+1]);
}

void checkBatteryLevel() {

	// Read the analogue pin and calculate battery voltage
	float voltage = analogRead(BATTERY_LEVEL_PIN) * 3.3 / 1024.0;
	voltage = voltage / DIVIDER_SCALING_FACTOR;
	int percentage = int(100 * (voltage - BATTERY_MIN_VOLTAGE) / float(BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE));

  // Update the display
  displayBatteryLevel(percentage);

  if (DEBUG) {
    String debugInfo = "BATT_" + String(percentage);
    showDebugInfo(debugInfo);
  }

	// Send the percentage via serial
	Serial.print(F("Battery_")); Serial.println(percentage);
}



// -------------------------------------------------------------------
/// Main program loop
// -------------------------------------------------------------------

void loop() {

	// Read any new serial messages
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	if (Serial.available() > 0) {
		readSerial();
	}

  if (booted) { // only do stuff if connected and booted correctly
    // Load or generate new animations
    // -- -- -- -- -- -- -- -- -- -- -- -- -- --
    manageAnimations();

    // Move Servos and wheels at regular time intervals
    // -- -- -- -- -- -- -- -- -- -- -- -- -- --
    if (millis() - updateTimer >= SERVO_UPDATE_TIME) {
      updateTimer = millis();

      unsigned long newTime = micros();
      float dt = (newTime - lastTime) / 1000.0;
      lastTime = newTime;

      manageServos(dt);
      manageMotors(dt);
    }

  } else { // show warning until booted
      showWarning(warning);
      warning = !warning;
      delay(500);
    }

  // Update robot status
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  if (millis() - statusTimer >= STATUS_CHECK_TIME) {
    statusTimer = millis();

    checkBatteryLevel();
  }
}
