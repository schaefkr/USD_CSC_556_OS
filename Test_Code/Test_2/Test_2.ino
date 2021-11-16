// CSC 556 ARDUINO OBSTACLE AVOIDING CAR WTIH FREERTOS //
// Nicholas Rasmussen, Kristin Schaefer, Bruce Stofft //
// Original Source code from https://www.youtube.com/watch?v=1n_KjpMfVT0 //
// AFMotor Library https://learn.adafruit.com/adafruit-motor-shield/library-install //
// NewPing Library https://github.com/livetronic/Arduino-NewPing //
// Servo Library https://github.com/arduino-libraries/Servo.git //
// Arduino FreeRTOS Library https://github.com/feilipu/Arduino_FreeRTOS_Library //

// Test 2 //

#include <Arduino_FreeRTOS.h>
#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>
#include <semphr.h>

#define TRIG_PIN A0
#define ECHO_PIN A1
#define MAX_DISTANCE 200
#define MAX_SPEED 190 // set speed of DC motors
#define MAX_SPEED_OFFSET 20
#define MIN_DISTANCE 15

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);
Servo myservo;

// Globals
boolean goesForward = false;
int distance = 100;
int distanceR = 0;
int distanceL = 0;
int speedSet = 0;

static TaskHandle_t TaskDriveHandle = NULL;
static TaskHandle_t TaskSenseHandle = NULL;

void TaskDrive(void *pvParameters);
void TaskSense(void *pvParameters);


void setup() {

  // update this section
  myservo.attach(10);
  myservo.write(115);
  // update this section
  delay(2000);
  readPing();
  delay(100);
  readPing();
  delay(100);
  readPing();
  delay(100);

  // freeRTOS tasks
  xTaskCreate(
    TaskDrive                // function name
    ,  "Drive"               // task name
    ,  128                   // stack size
    ,  NULL                  // task parameters
    ,  1                     // task priority
    ,  &TaskDriveHandle );   // task handle

  xTaskCreate(
    TaskSense                // function name
    ,  "Sense"               // task name
    ,  128                   // stack size
    ,  NULL                  // task parameters
    ,  1                     // task priority
    ,  &TaskSenseHandle );   // task handle

}


//-----------------------------------------------------------------------Task Drive + Task Sense and Helper methods--------------------------------------------------------------------------------//

void lookRight() {
  myservo.write(50);
  delay(500);
  readPingRight();
  myservo.write(115);
  delay(100);
}

void lookLeft() {
  myservo.write(170);
  delay(500);
  readPingLeft();
  myservo.write(115);
  delay(100);
}

void readPingRight() {
  delay(70);
  int cm = sonar.ping_cm();
  distanceR = cm;
  delay(100); // release
}

void readPingLeft() {
  delay(70);
  int cm = sonar.ping_cm();
  distanceL = cm;
  delay(100); // release
}

void readPing() {
  delay(70);
  int cm = sonar.ping_cm();
  distance = cm;
  delay(100); // release
}

void moveStop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void moveForward() {
  if (!goesForward)
  {
    goesForward = true;
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) // slowly bring the speed up to avoid loading down the batteries too quickly
    {
      motor1.setSpeed(speedSet);
      motor2.setSpeed(speedSet);
      motor3.setSpeed(speedSet);
      motor4.setSpeed(speedSet);
      delay(5);
    }
  }
}

void moveBackward() {
  goesForward = false;
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet);
    motor3.setSpeed(speedSet);
    motor4.setSpeed(speedSet);
    delay(5);
  }
}

void turnRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(500);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void turnLeft() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(500);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}


// ***** WIP *****
void TaskDrive(void *pvParameters) {

  for (;;) {

    if (distance <= MIN_DISTANCE)
    {
      moveStop();
      vTaskDelay(pdMS_TO_TICKS(100));
      moveBackward();
      vTaskDelay(pdMS_TO_TICKS(100));
      moveStop();
      vTaskDelay(pdMS_TO_TICKS(100));
      lookRight();
      vTaskDelay(pdMS_TO_TICKS(200));
      lookLeft();
      vTaskDelay(pdMS_TO_TICKS(200));


      if (distanceR >= distanceL)
      {
        turnRight();
        moveStop();
      }
      else
      {
        turnLeft();
        moveStop();
      }

      distanceR = 0;
      distanceL = 0;
      distance = 250;
      vTaskDelay(pdMS_TO_TICKS(100));

    }

    else
    {
      moveForward();
    }

  }
}


// ***** WIP *****
void TaskSense(void *pvParameters) {
  for (;;) {
    while (distance > MIN_DISTANCE)
    {
       readPing();
    }
  }
 }


// keep this section empty
void loop() {

}
