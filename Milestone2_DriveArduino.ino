/*
command_response_test

This Arduino sketch serves as an example to help students understand command data
parsing, response construction, and packetization. It will take any command received
over the serial connection, split out the command ID and data values, and parse the
data value as a floating point number. It will then respond to the command by echoing
back the data value. It can also toggle the built-in LED with a command "ld".
*/

/* Declarations and Constants */
String packet;
String responseString;
double DIFFERENCE = 0; // value to increment numerical data by before responding
bool DEBUG = false;    // If not debugging, set this to false to suppress debug messages

// Motor 1 (FR) Pin Definitions
int enA = 5;           // Motor speed control PWM pin
int in1 = 9;           // Motor direction pin 1
int in2 = 8;           // Motor direction pin 2

// Motor 2 (FL) Pin Definitions
int enB = 10;           // Motor speed control PWM pin
int in3 = 11;           // Motor direction pin 1
int in4 = 12;           // Motor direction pin 2

// Motor 1 Encoder Pin Definitions
#define ENCODER_A 3    // Encoder 1A pin
#define ENCODER_B 4  // Encoder 1B pin
#define ENCODER_C 2    // Encoder 2A pin
#define ENCODER_D 7   // Encoder 2B pin

// Constants
const float wheelDiameter = 60 / 25.4;  // Diameter of the wheel in inches
const float Turn_Dia = 185 / 25.4; // Distance between both wheels in inches
const float PPR = 700; // Encoder's Pulse Per Revolution

// Variables
volatile long motCount = 0;          // Encoder A count (Right Wheel)
volatile long motCount1 = 0;         // Encoder B count (Left Wheel)
bool moveForward = false;            // Flag to trigger forward movement
// bool moveSide = false;               // Flag to trigger side movement
bool rotate = false;                 // Flag to trigger rotation
float targetDistance = 0;            // Variable to store the distance to move in inches
float targetAngle = 0;               // Variable to store the rotation angle to move in degrees
float threshold = 1.0;               // Rotation threshold in degrees for when to stop
#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;


// Hardware Serial Definitions
char FRAMESTART = '[';
char FRAMEEND = ']';
int MAX_PACKET_LENGTH = 143; // equivalent to 16 8-byte commands of format "xx:#####", with 15 delimiting commas between them
int TIMEOUT = 250;           // Hardware Serial timeout in milliseconds
int BAUDRATE = 9600;

// /* Interrupt Service Routine for encoder count for right motor*/
// void encoderISR() {
//   int state = digitalRead(ENCODER_A);
//   if (digitalRead(ENCODER_B) == state) {
//     motCount--;
//   } else {
//     motCount++;
//   }
// }

// /* Interrupt Service Routine for encoder count for left motor*/
// void encoderISR1() {
//   int state = digitalRead(ENCODER_C);
//   if (digitalRead(ENCODER_D) == state) {
//     motCount1--;
//   } else {
//     motCount1++;
//   }
// }

/* Create a debug message */
void debugMessage(String msg) {
  if (DEBUG) {
    Serial.println(msg);
  }
}

/* Serial receive function */
String receiveSerial() {
  String frontmatter = "";
  String msg = "";
  char front_char = 0;
  char msg_char = 0;
  unsigned long start_time = 0;

  if (Serial.available()) {
    start_time = millis();
    while (millis() < start_time + TIMEOUT) {
      if (Serial.available()) {
        front_char = Serial.read();
        if (front_char == FRAMESTART) {
          msg += front_char;
          break;
        } else {
          frontmatter += front_char;
        }
      }
    }
    if (frontmatter.length() > 0) {
      debugMessage("Prior to FRAMESTART, received: " + frontmatter);
    }

    while (millis() < start_time + TIMEOUT) {
      if (Serial.available()) {
        msg_char = Serial.read();

        if (msg_char == FRAMESTART) {
          debugMessage("A new framestart character was received, dumping: " + msg);
          msg = "";
        }

        msg += msg_char;
        if (msg_char == FRAMEEND) {
          break;
        }
      }
    }

    if (msg.length() < 1) {
      debugMessage("Timed out without receiving any message.");
    } else if (msg_char != FRAMEEND) {
      debugMessage("Timed out while receiving a message.");
    } else {
      debugMessage("Depacketizing received message:" + msg);
      return depacketize(msg);
    }
  }

  return "";
}

/* Remove packet framing information */
String depacketize(String msg) {
  if (msg.length() > 1 && msg[0] == FRAMESTART) {
    if (msg[msg.length()-1] == FRAMEEND) {
      return msg.substring(1, msg.length()-1);
    }
  }
  debugMessage("Missing valid packet framing characters, instead detected these: " + String(msg[0]) + ',' + String(msg[msg.length()-1]));
  return "";
}

/* Add packet framing information */
String packetize(String msg) {
  return FRAMESTART + msg + FRAMEEND;
}

/* Handle the received packet */
String parsePacket(String pkt) {
  int cmdStartIndex = 0;
  String responseString = "";
  for (int ct = 0; ct < pkt.length(); ct++) {
    if (pkt[ct] == ',') {
      responseString += parseCmd(pkt.substring(cmdStartIndex, ct)) + ',';
      cmdStartIndex = ct + 1;
    }
  }
  responseString += parseCmd(pkt.substring(cmdStartIndex));
  debugMessage("Response String is: " + responseString);
  return responseString;
}

/* Handle the received commands (including driving forward by a variable distance) */
String parseCmd(String cmdString) {
  debugMessage("Parsing command: " + cmdString);

  String cmdID = cmdString.substring(0, min(2, cmdString.length()));
  double data = 0;
  if (cmdString.length() >= 4) {
    data = cmdString.substring(3).toDouble();
  }

  debugMessage("Command ID is: " + cmdID);
  debugMessage("The parsed data string is:" + String(data));

  // if (cmdID == "ld") {
  //   bool led_state = digitalRead(LED_BUILTIN);
  //   digitalWrite(LED_BUILTIN, !led_state);
  //   return cmdID + ':' + (!led_state ? "True" : "False");
  // }

  if (cmdID == "w0") {
    targetDistance = data;
    moveForward = true;
    return cmdID + ":Moving " + String(data) + " inches";
  }

  if (cmdID == "r0") {
    targetAngle = data;
    rotate = true;
    return cmdID + ":Rotating " + String(data) + " degrees";
  }

  return cmdID + ':' + String(data + DIFFERENCE);
}

void readGyroData() {
  // === Read accelerometer and gyroscope data === //
  // Code to read MPU6050 sensor and update roll, pitch, yaw

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 2.34;

  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;

  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;

  GyroX = GyroX + 0.30;
  gyroAngleX = gyroAngleX + GyroX * elapsedTime;
  roll = 0.95 * gyroAngleX + 0.05 * accAngleX;
}

/* Function to drive the motor forward by a specified distance in inches */
void driveForward(float inches) {
  // readGyroData(); // Get the current roll reading
  // float Before_Roll = roll;
  if (inches > 0) {  // Moving forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, 125);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, 97);
    delay(140);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 125);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, HIGH);
    analogWrite(enB, 125);
    }

  if (inches < 0) {  // Moving backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 125);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, 97);
    delay(140);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 125);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, HIGH);
    analogWrite(enB, 125);
  }}
  // if (targetCount > 0) {
  //   while (motCount < targetCount) {
  //     Serial.println(targetCount);
  //     Serial.print("Forward Movement - Encoder Count: ");
  //     Serial.println(motCount);  // Track and print the current encoder count
  //     delay(1000);  // Adjust this delay as needed to control print frequency
  //     //encoderISR();
  //   }
  // }
  // if (targetCount < 0) {
  //   while (motCount > targetCount) {
  //     Serial.println(targetCount);
  //     Serial.print("Forward Movement - Encoder Count: ");
  //     Serial.println(motCount);  // Track and print the current encoder count
  //     delay(1000);  // Adjust this delay as needed to control print frequency
  //     //encoderISR();
  // }}
  // analogWrite(enA, 0);
  // digitalWrite(in1, HIGH);
  // digitalWrite(in2, HIGH);
  // analogWrite(enB, 0);
  // digitalWrite(in3, HIGH);
  // digitalWrite(in4, HIGH);
  // motCount = 0;
  // readGyroData(); // Get the current roll reading
  // float After_Roll = roll;
  // float angle_drift = After_Roll - Before_Roll;
  // if (angle_drift > 1) {
  //   driveRotate(-angle_drift);
  //}

/* Function to rotate by a specified angle */
// void driveRotate(float degrees) {
//   //float targetCount = (-degrees * Turn_Dia * PPR) / (360 * wheelDiameter);
//   //float targetCount = 1.5 * ((-PPR * Turn_Dia / 2) * sin(degrees)) / (3.14 * wheelDiameter);
//   float targetCount = 750*(degrees/90);
//   if (targetCount < 0) {      // Turn CW
//     while(abs(motCount) < abs(targetCount)) {
//       digitalWrite(in1, LOW);
//       digitalWrite(in2, HIGH);
//       analogWrite(enA, 125);
//       digitalWrite(in3, HIGH);
//       digitalWrite(in4, LOW);
//       analogWrite(enB, 90);
// }
// }

void driveRotate(float targetAngle) {

  // Read the initial roll value and set it as the baseline           ((0.6237 * x) - 0.2772)
  readGyroData(); // Get the current roll reading         
  float initialRoll = roll;
  float targetRoll = initialRoll + targetAngle;
  float error = targetRoll - roll;
  // if (error > threshold)  {
  //   while ((targetRoll - roll) > threshold) {
  //       readGyroData(); // Continuously update roll from the MPU6050
  //       delay(10);
  //       digitalWrite(in1, LOW);
  //       digitalWrite(in2, HIGH);
  //       analogWrite(enA, 125);
  //       digitalWrite(in3, HIGH);
  //       digitalWrite(in4, LOW);
  //       analogWrite(enB, 90);
  //   }
  // }
  // if (error < threshold) {
  //   while ((targetRoll - roll) < threshold) {
  //     readGyroData(); // Continuously update roll from the MPU6050
  //     delay(10);
  //     digitalWrite(in1, HIGH);
  //     digitalWrite(in2, LOW);
  //     analogWrite(enA, 125);
  //     digitalWrite(in3, LOW);
  //     digitalWrite(in4, HIGH);
  //     analogWrite(enB, 90);
  // }
  // }

  while (abs(roll - targetRoll) > threshold) { // Loop until within threshold
    readGyroData(); // Continuously update roll from the MPU6050

    float error = targetRoll - roll;

    // Set motor speed based on the direction to rotate
    if (error > 0) { // Rotate CW
    // RIGHT MOTOR IN1 and IN2 
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, 125);
    // LEFT MOTOR IN3 IN4 
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enB, 90);
    } else { // Rotate CCW
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, 125);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enB, 90);
    }

    // Small delay to allow sensor readings to update
    delay(10);
  }

  // Stop the motors once target angle is within threshold
  analogWrite(enA, 125);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  analogWrite(enB, 125);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, HIGH);
}

//   if (targetCount > 0) {      // Turn CCW
//     while(motCount < targetCount)  {
//       digitalWrite(in1, HIGH);
//       digitalWrite(in2, LOW);
//       analogWrite(enA, 125);
//       digitalWrite(in3, LOW);
//       digitalWrite(in4, HIGH);
//       analogWrite(enB, 90);

// }
// }
// if (targetCount < 0) {
//     while (motCount > targetCount) {
//       Serial.println(targetCount);
//       Serial.print("Rotating CW - Encoder Count: ");
//       Serial.println(motCount);  // Track and print the current encoder count
//       delay(1000);  // Adjust this delay as needed to control print frequency
//       //encoderISR();
//     }
//   }
//   if (targetCount > 0) {
//     while (motCount < targetCount) {
//       Serial.println(targetCount);
//       Serial.print("Rotating CCW - Encoder Count: ");
//       Serial.println(motCount);  // Track and print the current encoder count
//       delay(1000);  // Adjust this delay as needed to control print frequency
//       //encoderISR();
//  }

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR1, CHANGE);

  Serial.begin(BAUDRATE);
  //pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
}

void loop() {
  if (Serial.available()) {
    packet = receiveSerial();
    if (packet.length() > 0) {
      responseString = parsePacket(packet);
      Serial.println(packetize(responseString));
    }
  }


  if (moveForward) {
    driveForward(targetDistance);
    moveForward = false;
  }

  if (rotate) {
    driveRotate(targetAngle);
    rotate = false;
  }
    // Test the encoder
  // Serial.print("Encoder A state: ");
  // Serial.println(digitalRead(ENCODER_A));
  // Serial.print("Encoder B state: ");
  // Serial.println(digitalRead(ENCODER_B));
  // delay(500);  // Slow down the loop for readability
}
