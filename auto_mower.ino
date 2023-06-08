#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <NewPing.h>

#define GPS_RX_PIN 2
#define GPS_TX_PIN 3
#define LEFT_MOTOR_PIN 5
#define RIGHT_MOTOR_PIN 6
#define MOWING_MOTOR_PIN 7
#define TRIGGER_PIN 8
#define ECHO_PIN 9

Servo leftMotor;
Servo rightMotor;
Servo mowingMotor;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, 200); // Adjust the maximum distance (in cm) according to your setup

bool isMowingComplete = false;
bool isInsideMowingArea = false;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  leftMotor.attach(LEFT_MOTOR_PIN);
  rightMotor.attach(RIGHT_MOTOR_PIN);
  mowingMotor.attach(MOWING_MOTOR_PIN);
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();

        // Your navigation logic goes here
        // Determine if the lawnmower is within the mowing area

        // For example, let's assume a rectangular mowing area
        if (!isMowingComplete) {
          if (isInsideMowingArea) {
            if (checkObstacle()) {
              avoidObstacle();
            } else {
              startMowing();
            }
          } else {
            if (isWithinMowingArea(latitude, longitude)) {
              isInsideMowingArea = true;
            } else {
              // Navigate to the starting point of the mowing area
              float startLatitude = 42.12345; // Update with actual starting latitude
              float startLongitude = -71.98765; // Update with actual starting longitude
              navigateToLocation(startLatitude, startLongitude);
            }
          }
        }
      }
    }
  }
}

void startMowing() {
  // Start the lawnmower's cutting mechanism
  mowingMotor.write(180); // Adjust the angle as per your setup
  delay(2000); // Adjust the delay as per your setup

  // Start the wheels
  leftMotor.write(180); // Adjust the angle as per your setup
  rightMotor.write(180); // Adjust the angle as per your setup

  // Mow the lawn until the lawnmower crosses the boundary
  while (isInsideMowingArea) {
    // Implement your mowing logic here

    // Check if the lawnmower has crossed the boundary
    if (!isWithinMowingArea(gps.location.lat(), gps.location.lng())) {
      isInsideMowingArea = false;
    }
  }

  // Stop the lawnmower
  mowingMotor.write(90); // Adjust the angle to stop the mowing motor
  leftMotor.write(90); // Adjust the angle to stop the left motor
  rightMotor.write(90); // Adjust the angle to stop the right motor

  // Set the mowing complete flag
  isMowingComplete = true;
}

bool isWithinMowingArea(float latitude, float longitude) {
  // Implement your logic to check if the coordinates are within the mowing area
  // Return true if within the area, false otherwise
}

void navigateToLocation(float targetLatitude, float targetLongitude) {
  // Implement your navigation logic here
  // Move the lawnmower towards the target location in a linear path (similar to a Roomba)
  
  // Calculate the heading angle
  float currentLatitude = gps.location.lat();
  float currentLongitude = gps.location.lng();
  
  // Calculate the distance to the target location
  float distance = calculateDistance(currentLatitude, currentLongitude, targetLatitude, targetLongitude);
  
  // Move the lawnmower towards the target location while avoiding obstacles
  while (distance > 0) {
    if (checkObstacle()) {
      avoidObstacle();
    } else {
      moveForward();
      delay(1000); // Adjust the delay as per your setup
      stopMoving();
      delay(1000); // Adjust the delay as per your setup
      distance--;
    }
  }
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  // Implement your logic to calculate the distance between two GPS coordinates
  // Return the distance in meters
}

bool checkObstacle() {
  // Check if there is an obstacle in front using ultrasonic sensors
  int distance = sonar.ping_cm();
  
  // Adjust the threshold distance as per your setup
  if (distance > 0 && distance <= 30) {
    return true; // Obstacle detected
  } else {
    return false; // No obstacle detected
  }
}

void avoidObstacle() {
  // Implement your obstacle avoidance logic here
  // For simplicity, let's assume the lawnmower turns 90 degrees to the right and then continues moving
  
  // Turn right
  rightMotor.write(180); // Adjust the angle as per your setup
  delay(1000); // Adjust the delay as per your setup
  
  // Move forward
  moveForward();
  delay(1000); // Adjust the delay as per your setup
  
  // Stop moving
  stopMoving();
  delay(1000); // Adjust the delay as per your setup
}

void moveForward() {
  // Move the lawnmower forward
  leftMotor.write(180); // Adjust the angle as per your setup
  rightMotor.write(180); // Adjust the angle as per your setup
}

void stopMoving() {
  // Stop the lawnmower
  leftMotor.write(90); // Adjust the angle to stop the left motor
  rightMotor.write(90); // Adjust the angle to stop the right motor
}
