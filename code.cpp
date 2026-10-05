#include <Wire.h>
#include <VL53L0X.h>

// =====================================================
// MOTOR DRIVER PINS
// =====================================================
#define STBY 4

#define PWMA 5
#define PWMB 6

#define AIN1 7
#define AIN2 8

#define BIN1 9
#define BIN2 10

// =====================================================
// VL53L0X XSHUT PINS
// =====================================================
#define XSHUT_LEFT   2
#define XSHUT_FRONT  3
#define XSHUT_RIGHT  12

// =====================================================
// VL53L0X ADDRESSES
// =====================================================
#define ADDR_LEFT    0x30
#define ADDR_FRONT   0x31
#define ADDR_RIGHT   0x32

VL53L0X leftSensor;
VL53L0X frontSensor;
VL53L0X rightSensor;

// =====================================================
// ROBOT SETTINGS
// =====================================================

// Target distance from wall, in millimeters
#define WALL_DISTANCE 100

// How close the robot can get to a wall
#define FRONT_DISTANCE 130

// Distance at which a wall is considered present
#define WALL_DETECT 180

// -----------------------------------------------------
// IMPORTANT:
// Measure the distance between the LEFT and RIGHT
// wheel contact/center lines.
//
// Example:
// 120 mm -> TRACK_WIDTH = 120
//
// This is currently a PLACEHOLDER.
// -----------------------------------------------------
#define TRACK_WIDTH 120.0

// Motor speeds
#define BASE_SPEED 120
#define TURN_SPEED 150
#define MAX_SPEED 200

// Small correction applied for wall following
#define WALL_KP 1.2

// =====================================================
// SENSOR VARIABLES
// =====================================================
int leftDist;
int frontDist;
int rightDist;


// =====================================================
// MOTOR CONTROL
// =====================================================
void motorLeft(int speed)
{
  speed = constrain(speed, -255, 255);

  if (speed > 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -speed);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
}


void motorRight(int speed)
{
  speed = constrain(speed, -255, 255);

  if (speed > 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -speed);
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}


void stopMotors()
{
  motorLeft(0);
  motorRight(0);
}


void forward()
{
  motorLeft(BASE_SPEED);
  motorRight(BASE_SPEED);
}


// =====================================================
// SENSOR READING
// =====================================================
int readSensor(VL53L0X &sensor)
{
  int distance = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred())
  {
    return 800;
  }

  // Ignore obviously invalid readings
  if (distance <= 0 || distance > 800)
  {
    distance = 800;
  }

  return distance;
}


void readSensors()
{
  leftDist  = readSensor(leftSensor);
  frontDist = readSensor(frontSensor);
  rightDist = readSensor(rightSensor);

  Serial.print("L: ");
  Serial.print(leftDist);

  Serial.print("  F: ");
  Serial.print(frontDist);

  Serial.print("  R: ");
  Serial.println(rightDist);
}


// =====================================================
// TURNING
// =====================================================
//
// Without encoders, turns are TIME based.
//
// TRACK_WIDTH is provided as a placeholder for your
// robot geometry. You will need to experimentally tune
// TURN_90_TIME for your motors/battery.
//
// =====================================================

#define TURN_90_TIME 350

void turnLeft90()
{
  stopMotors();
  delay(50);

  motorLeft(-TURN_SPEED);
  motorRight(TURN_SPEED);

  delay(TURN_90_TIME);

  stopMotors();
  delay(80);
}


void turnRight90()
{
  stopMotors();
  delay(50);

  motorLeft(TURN_SPEED);
  motorRight(-TURN_SPEED);

  delay(TURN_90_TIME);

  stopMotors();
  delay(80);
}


void turnAround()
{
  stopMotors();
  delay(50);

  motorLeft(TURN_SPEED);
  motorRight(-TURN_SPEED);

  // Approximately 180 degrees
  delay(TURN_90_TIME * 2);

  stopMotors();
  delay(80);
}


// =====================================================
// WALL FOLLOWING
// =====================================================

void followRightWall()
{
  // Error:
  // Positive = robot is too far from right wall
  // Negative = robot is too close to right wall

  int error = rightDist - WALL_DISTANCE;

  int correction = error * WALL_KP;

  int leftSpeed  = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  leftSpeed  = constrain(leftSpeed, 60, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 60, MAX_SPEED);

  motorLeft(leftSpeed);
  motorRight(rightSpeed);
}


// =====================================================
// MAZE SOLVING LOGIC
// =====================================================
//
// RIGHT-HAND RULE:
//
// 1. If right side is open -> turn right
// 2. Else if front is open -> go forward
// 3. Else if left is open -> turn left
// 4. Else -> turn around
//
// =====================================================

void solveMaze()
{
  readSensors();

  bool leftWall  = leftDist < WALL_DETECT;
  bool frontWall = frontDist < FRONT_DISTANCE;
  bool rightWall = rightDist < WALL_DETECT;

  // ---------------------------------------------------
  // PRIORITY 1: RIGHT TURN
  // ---------------------------------------------------
  if (!rightWall)
  {
    turnRight90();

    // Move slightly forward after turn
    motorLeft(BASE_SPEED);
    motorRight(BASE_SPEED);
    delay(100);

    return;
  }

  // ---------------------------------------------------
  // PRIORITY 2: FORWARD
  // ---------------------------------------------------
  if (!frontWall)
  {
    followRightWall();
    return;
  }

  // ---------------------------------------------------
  // PRIORITY 3: LEFT TURN
  // ---------------------------------------------------
  if (!leftWall)
  {
    turnLeft90();

    motorLeft(BASE_SPEED);
    motorRight(BASE_SPEED);
    delay(100);

    return;
  }

  // ---------------------------------------------------
  // PRIORITY 4: DEAD END
  // ---------------------------------------------------
  turnAround();
}


// =====================================================
// VL53L0X INITIALIZATION
// =====================================================

void setupSensors()
{
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  // Turn all sensors OFF
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);

  delay(100);

  // ---------------------------------------------------
  // LEFT SENSOR
  // ---------------------------------------------------
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);

  if (!leftSensor.init())
  {
    Serial.println("LEFT SENSOR FAILED!");
    while (1);
  }

  leftSensor.setAddress(ADDR_LEFT);
  leftSensor.setTimeout(100);

  // ---------------------------------------------------
  // FRONT SENSOR
  // ---------------------------------------------------
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(50);

  if (!frontSensor.init())
  {
    Serial.println("FRONT SENSOR FAILED!");
    while (1);
  }

  frontSensor.setAddress(ADDR_FRONT);
  frontSensor.setTimeout(100);

  // ---------------------------------------------------
  // RIGHT SENSOR
  // ---------------------------------------------------
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);

  if (!rightSensor.init())
  {
    Serial.println("RIGHT SENSOR FAILED!");
    while (1);
  }

  rightSensor.setAddress(ADDR_RIGHT);
  rightSensor.setTimeout(100);

  // ---------------------------------------------------
  // Start continuous measurements
  // ---------------------------------------------------
  leftSensor.startContinuous();
  frontSensor.startContinuous();
  rightSensor.startContinuous();

  Serial.println("VL53L0X sensors ready.");
}


// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);

  Wire.begin();

  // Motor pins
  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Enable motor driver
  digitalWrite(STBY, HIGH);

  stopMotors();

  delay(500);

  setupSensors();

  delay(500);

  Serial.println("==============================");
  Serial.println(" MAZE SOLVER READY");
  Serial.println("==============================");

  delay(1000);
}


// =====================================================
// MAIN LOOP
// =====================================================
void loop()
{
  solveMaze();

  // Small loop delay
  delay(20);
}
