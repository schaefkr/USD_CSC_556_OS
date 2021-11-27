// CSC 556 ARDUINO OBSTACLE AVOIDING CAR WTIH FREERTOS //
// Nicholas Rasmussen, Kristin Schaefer, Bruce Stofft //
// Original Source code from https://www.youtube.com/watch?v=1n_KjpMfVT0 //
// AFMotor Library https://learn.adafruit.com/adafruit-motor-shield/library-install //
// NewPing Library https://github.com/livetronic/Arduino-NewPing //
// Servo Library https://github.com/arduino-libraries/Servo.git //
// Arduino FreeRTOS Library https://github.com/feilipu/Arduino_FreeRTOS_Library //


#include <Arduino_FreeRTOS.h>
#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>
#include <semphr.h>

#define TRIG_PIN A0
#define ECHO_PIN A1
#define MAX_DISTANCE 200
#define MAX_SPEED 160
#define MAX_SPEED_OFFSET 20

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

  myservo.attach(10);
  myservo.write(90); 
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
    ,  256                   // stack size
    ,  NULL                  // task parameters
    ,  1                     // task priority
    ,  &TaskDriveHandle );   // task handle

  xTaskCreate(
    TaskSense                // function name
    ,  "Sense"               // task name
    ,  256                   // stack size
    ,  NULL                  // task parameters
    ,  1                     // task priority
    ,  &TaskSenseHandle );   // task handle

}


//-----------------------------------------------------------------------Task Drive + Task Sense and Helper methods--------------------------------------------------------------------------------//

void lookRight() {
  myservo.write(25);
  delay(500);
  readPingRight();
  delay(100);
  myservo.write(90); 
}

void lookLeft() {
  myservo.write(155);
  delay(500);
  readPingLeft();
  delay(100);
  myservo.write(90); 
}

void readPingRight() {
  delay(70);
  int cm = sonar.ping_cm();
  distanceR = cm;
}

void readPingLeft() {
  delay(70);
  int cm = sonar.ping_cm();
  distanceL = cm;
}

void readPing() {
  delay(70);
  int cm = sonar.ping_cm();
  if(cm==0)
  {
    cm = 250;
  }
  distance = cm;
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
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2)
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
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2)
  {
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet);
    motor3.setSpeed(speedSet);
    motor4.setSpeed(speedSet);
    delay(5);
  }
}

void turnRight() {
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

void turnLeft() {
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

void TaskDrive(void *pvParameters) {

  for (;;) {
    distanceR = 0;
    distanceL = 0;
    vTaskDelay(pdMS_TO_TICKS(40));

    if (distance <= 25)
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

    }

    else
    {
      moveForward();
    }

  }
}

void TaskSense(void *pvParameters) {
  for (;;) {
    readPing();
  }
 }

void loop() {

}
