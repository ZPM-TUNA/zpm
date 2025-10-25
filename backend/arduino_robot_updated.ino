/*
 * Arduino Robot Controller with Serial Commands
 * Compatible with Python pathfinding system
 * 
 * Commands (send via Serial):
 * - "F,speed,duration" - Move forward (e.g., "F,200,1000")
 * - "B,speed,duration" - Move backward
 * - "L,speed,duration" - Turn left
 * - "R,speed,duration" - Turn right
 * - "S" - Stop
 * - "U" - Read ultrasonic
 * - "G" - Read gyro
 * - "A" - Read all sensors
 */

#include <Wire.h>
#include <MPU6050.h>

// ====== Pins ======
#define TRIG_PIN 13
#define ECHO_PIN 12

#define PWMA 5
#define AIN1 7
#define PWMB 6
#define BIN1 8
#define STBY 3

// ====== Globals ======
long duration;
float distance;

MPU6050 gyro;
float ax, ay, az;
float gx, gy, gz;

String inputCommand = "";
bool commandComplete = false;

// ====== Setup ======
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Motors
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Gyro
  gyro.initialize();
  if (gyro.testConnection()) {
    Serial.println("READY:MPU6050_CONNECTED");
  } else {
    Serial.println("ERROR:MPU6050_FAILED");
  }
  
  Serial.println("READY:ARDUINO_ROBOT");
}

// ====== Main loop ======
void loop() {
  // Check for incoming serial commands
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n') {
      commandComplete = true;
    } else {
      inputCommand += inChar;
    }
  }
  
  // Process command when complete
  if (commandComplete) {
    processCommand(inputCommand);
    inputCommand = "";
    commandComplete = false;
  }
}

// ====== Command processing ======
void processCommand(String cmd) {
  cmd.trim();
  
  if (cmd.startsWith("F,")) {
    // Forward: F,speed,duration
    int comma1 = cmd.indexOf(',');
    int comma2 = cmd.indexOf(',', comma1 + 1);
    
    int speed = cmd.substring(comma1 + 1, comma2).toInt();
    int duration = cmd.substring(comma2 + 1).toInt();
    
    moveForward(speed);
    delay(duration);
    stopMotors();
    Serial.println("OK:FORWARD_COMPLETE");
    
  } else if (cmd.startsWith("B,")) {
    // Backward: B,speed,duration
    int comma1 = cmd.indexOf(',');
    int comma2 = cmd.indexOf(',', comma1 + 1);
    
    int speed = cmd.substring(comma1 + 1, comma2).toInt();
    int duration = cmd.substring(comma2 + 1).toInt();
    
    moveBackward(speed);
    delay(duration);
    stopMotors();
    Serial.println("OK:BACKWARD_COMPLETE");
    
  } else if (cmd.startsWith("L,")) {
    // Left: L,speed,duration
    int comma1 = cmd.indexOf(',');
    int comma2 = cmd.indexOf(',', comma1 + 1);
    
    int speed = cmd.substring(comma1 + 1, comma2).toInt();
    int duration = cmd.substring(comma2 + 1).toInt();
    
    turnLeft(speed);
    delay(duration);
    stopMotors();
    Serial.println("OK:LEFT_COMPLETE");
    
  } else if (cmd.startsWith("R,")) {
    // Right: R,speed,duration
    int comma1 = cmd.indexOf(',');
    int comma2 = cmd.indexOf(',', comma1 + 1);
    
    int speed = cmd.substring(comma1 + 1, comma2).toInt();
    int duration = cmd.substring(comma2 + 1).toInt();
    
    turnRight(speed);
    delay(duration);
    stopMotors();
    Serial.println("OK:RIGHT_COMPLETE");
    
  } else if (cmd == "S") {
    // Stop
    stopMotors();
    Serial.println("OK:STOPPED");
    
  } else if (cmd == "U") {
    // Read ultrasonic
    readUltrasonic();
    Serial.print("ULTRASONIC:");
    Serial.println(distance);
    
  } else if (cmd == "G") {
    // Read gyro
    readGyro();
    Serial.print("GYRO:");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.println(gz);
    
  } else if (cmd == "A") {
    // Read all sensors
    readUltrasonic();
    readGyro();
    Serial.print("SENSORS:");
    Serial.print(distance); Serial.print(",");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.println(gz);
    
  } else {
    Serial.print("ERROR:UNKNOWN_COMMAND:");
    Serial.println(cmd);
  }
}

// ====== Sensor functions ======
void readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
}

void readGyro() {
  ax = gyro.getAccelerationX() / 16384.0;
  ay = gyro.getAccelerationY() / 16384.0;
  az = gyro.getAccelerationZ() / 16384.0;

  gx = gyro.getRotationX() / 131.0;
  gy = gyro.getRotationY() / 131.0;
  gz = gyro.getRotationZ() / 131.0;
}

// ====== Motor functions ======
void moveForward(int speed) {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, HIGH);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void moveBackward(int speed) {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void turnLeft(int speed) {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void turnRight(int speed) {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void enableMotors() { 
  digitalWrite(STBY, HIGH); 
}

void disableMotors() { 
  digitalWrite(STBY, LOW); 
}

