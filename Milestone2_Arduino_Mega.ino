/*
 * ian_bt_testing
 * 
 * This Arduino sketch serves as an example to help students understand command data
 * parsing, response construction, and packetization. It will take any command received
 * over the serial connection, split out the command ID and data values, and parse the
 * data value as a floating point number. It will then respond to the command by echoing
 * back the data value. It can also toggle the built-in LED with a command "ld".
 */

/* Include Libraries */
#include <NewPing.h>
#include <SoftwareSerial.h>
#include <RPLidar.h>

SoftwareSerial bluetoothSerial(10, 11); // RX, TX for Bluetooth
RPLidar lidar; // LiDAR connected to Serial2

/* Declarations and Constants */
String packet;
String responseString;
String sensorData[6]; // Declare the sensorData array to hold sensor data
const int numSensors = 6; // Total number of ultrasonic sensors
const int triggerPins[numSensors] = {48, 52, 44, 42, 50, 46}; // Trigger pins for each sensor
const int echoPins[numSensors] = {49, 53, 45, 43, 51, 47};     // Echo pins for each sensor
const int MAX_DISTANCE_INCHES = 200; // Desired max distance in inches
const int MAX_DISTANCE_CM = MAX_DISTANCE_INCHES * 2.54; // Convert to centimeters
double DIFFERENCE = 0; // value to increment numerical data by before responding
bool DEBUG = false; // If not debugging, set this to false to suppress debug messages

// LiDAR variables
int fullRotations = 0;  // Counter for completed 360-degree rotations
bool rotationStarted = false;
const int maxRotations = 30;
float distances[36];  // Array to store distance at each 10-degree increment
bool hasData[36];
const float tolerance = 2.5;  // Acceptable variance of ±1 degree

// Create an array of NewPing objects for each sensor
NewPing sonars[numSensors] = {
    NewPing(triggerPins[0], echoPins[0], MAX_DISTANCE_CM),
    NewPing(triggerPins[1], echoPins[1], MAX_DISTANCE_CM),
    NewPing(triggerPins[2], echoPins[2], MAX_DISTANCE_CM),
    NewPing(triggerPins[3], echoPins[3], MAX_DISTANCE_CM),
    NewPing(triggerPins[4], echoPins[4], MAX_DISTANCE_CM),
    NewPing(triggerPins[5], echoPins[5], MAX_DISTANCE_CM)
};

// Hardware Serial Definitions
char FRAMESTART = '[';
char FRAMEEND = ']';
int MAX_PACKET_LENGTH = 143; // equivalent to 16 8-byte commands of format "xx:#####", with 15 delimiting commas between them
int HWTIMEOUT = 250; // Hardware Serial timeout in milliseconds
int HWBAUDRATE = 9600;

/* Setup Function */
void setup() {
    Serial.begin(9600);
    Serial1.begin(HWBAUDRATE);
    bluetoothSerial.begin(HWBAUDRATE);
    lidar.begin(Serial2); // Start LiDAR
    Serial2.begin(115200);  // Serial2 for LiDAR

    // Verify LiDAR connection and start scan
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
        lidar.startScan();
        delay(1000);  // Stabilize motor
    } else {
        Serial.println("Failed to detect RPLIDAR device.");
    }
    resetLidarData();
}

/* Main Loop */
void loop() {
    if (bluetoothSerial.available()) {
        packet = receiveSerial();
        if (packet.length() > 0) {
            responseString = parsePacket(packet);
            //bluetoothSerial.println(packetize(responseString));
        }
    }
}

/* Bluetooth Functions */
String receiveSerial() {
    String msg = "";
    char msg_char = 0;
    unsigned long start_time = millis();

    if (bluetoothSerial.available()) {
        while (millis() < start_time + HWTIMEOUT) {
            if (bluetoothSerial.available()) {
                msg_char = bluetoothSerial.read();
                msg += msg_char;
                if (msg_char == FRAMEEND) {
                    break;
                }
            }
        }
    }

    if (msg.length() > 0) {
        return depacketize(msg);
    }
    return ""; // Return empty string if no valid message
}

String depacketize(String msg) {
    if (msg.length() > 1 && msg[0] == FRAMESTART) {
        if (msg[msg.length() - 1] == FRAMEEND) {
            return msg.substring(1, msg.length() - 1);
        }
    }
    return ""; // Return empty string if framing is incorrect
}

String packetize(String msg) {
    return FRAMESTART + msg + FRAMEEND;
}

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
    return responseString;
}

String parseCmd(String cmdString) {
    String cmdID = cmdString.substring(0, min(2, cmdString.length()));

    if (cmdID == "SD") {
        bool Collect_Lidar = true;
        while(Collect_Lidar) {
          if (fullRotations >= maxRotations) {
            gatherSensorData(); // Gather ultrasonic data
            String sensorOutput = "";
            // Add ultrasonic sensor data to sensorOutput
            for (int i = 0; i < numSensors; i++) {
                //sensorOutput += "u" + String(i) + ":" + sensorData[i];
                sensorOutput += sensorData[i];
                //int distanceValue = static_cast<int>(sensorData[i].toFloat() * 10); // Scale to avoid decimals    //For HEX DATA COMPRESSION
                //sensorOutput += String(distanceValue, HEX); // Convert to hex              //For HEX DATA COMPRESSION
                if (i < numSensors - 1) {
                    sensorOutput += ",";
                }
            }
            //Collect_Lidar = false;

            // Output the final distance values for each 10-degree increment
            for (int i = 0; i < 36; i++) {
            sensorOutput += ",";
            if (hasData[i]) {
                //sensorOutput += ",l" + String(i * 10) + ":" + String(distances[i]);
                sensorOutput += String(distances[i]);
                //int distanceValue = static_cast<int>(distances[i] * 10); // Scale and convert   //For HEX DATA COMPRESSION
                //String(distanceValue, HEX);         //For HEX DATA COMPRESSION
            } else {
                //sensorOutput += ",l" + String(i * 10) + ":NoData";
                sensorOutput += String(0);
            }
            }
            bluetoothSerial.println(packetize(sensorOutput));
            resetLidarData();  // Stop the LIDAR scan
            //Collect_Lidar = false;
            while (true);  // Halt further processing (infinite loop)
          }

  // Read LIDAR data if available
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle;
    byte quality = lidar.getCurrentPoint().quality;

    // Check if the angle is near zero to detect a new rotation
    if (!rotationStarted && angle < 10) {
      rotationStarted = true;   // Start of a new rotation
      fullRotations++;          // Increment full rotation count

    } else if (rotationStarted && angle > 350) {
      rotationStarted = false;  // Reset rotation flag at the end of a rotation
    }
    // Process readings that fall within ±1 degree of each 10-degree increment
    for (int i = 0; i < 36; i++) {
      float targetAngle = i * 10.0;
      if (quality > 10 && distance < 2500 && distance >= 100 && angle >= (targetAngle - tolerance) && angle <= (targetAngle + tolerance)) {
        // Store the first valid reading only
        if (!hasData[i]) {
          distances[i] = distance; // Store the distance for this 10-degree increment
          hasData[i] = true;       // Mark this increment as having valid data
          break;                   // Exit loop once a matching angle is found
        }
      }
    }
    delayMicroseconds(200);  // Small delay to improve timing consistency
    }
    }
        //bluetoothSerial.println(packetize(sensorOutput));
        return packetize("Sent:");  // Return combined sensor data
    }
     
    else {
        Serial1.println(packetize(cmdString)); // Forward command to Uno
        return packetize("Sent from Mega to Uno: " + cmdString);
    }
}

void gatherSensorData() {
    for (int i = 0; i < numSensors; i++) {
        float totalDistance = 0;
        const int numReadings = 3;
        int validReadings = 0;

        for (int j = 0; j < numReadings; j++) {
            float distance = sonars[i].ping_cm();
            float distance_in = distance / 2.54;

            if (distance_in > 0) {
                totalDistance += distance_in;
                validReadings++;
            }
            delay(20);
        }

        if (validReadings > 0) {
            float averageDistance = totalDistance / validReadings;
            sensorData[i] = String(averageDistance);
        } else {
            sensorData[i] = "200";
        }
    }
}

// Function to reset LIDAR data
void resetLidarData() {
  fullRotations = 0;
  for (int i = 0; i < 36; i++) {
    distances[i] = -1;
    hasData[i] = false;
  }
}
