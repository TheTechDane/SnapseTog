#include <Encoder.h>
#include <Preferences.h>
#define LOGLEVEL LOGLEVEL_DEBUGING
#include <logger.h>

#define motorEnablePin D0    //GPIO0; // PWM pin for speed control
#define motorIn1Pin D1       //GPIO1;
#define motorIn2Pin  D2       //GPIO2;
//Stop pins
#define forwardStopPin D7    //GPIO20;
#define reverseStopPin D8    //GPIO19;
//KY-040 Rotary Encoder Pins
#define encoderPinCLK D4     //GPIO22;
#define encoderPinDT D5      //GPIO23;
#define encoderButtonPin D6  //GPIO16;
//Light Pins
#define front_red_led D10

// Motor Control States
#define FORWARD 1
#define REVERSE 2
#define STOPPED 0


Preferences permData;

int currentDirection = STOPPED;
int previousDirection = REVERSE;
volatile int selectedSpeed = 150; // 0-255 for PWM control
bool isTrainMoving = false;

// Speed control parameters
#define speedStep 10 // Amount to change speed per encoder click
#define minSpeed 0
#define maxSpeed 255
#define encoderSensitivity 4 // Adjust for how many encoder steps per speed change
Encoder myEnc(encoderPinDT, encoderPinCLK);

void setup() {
  Serial.begin(115200);
  Serial.println("Current log level is :" + String(LOGLEVEL));

  //Signal start on the LED
  pinMode( front_red_led, OUTPUT);
  pinMode( LED_BUILTIN, OUTPUT);
  for (int i=0; i<3; i++) {
    digitalWrite(front_red_led, HIGH);  // turn the LED on (HIGH is the voltage level)
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(300);                      
    Serial.println("Blink..");  
    digitalWrite(front_red_led, LOW);   // turn the LED off by making the voltage LOW
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(300);   
  }
  digitalWrite(front_red_led, HIGH);  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)

  //Get speed from Memory
  permData.begin("Train");
  selectedSpeed = permData.getInt("speed",150);
  logI("CurrenSpeed Read from memory:");
  loglnI(selectedSpeed);

  pinMode(motorIn1Pin, OUTPUT);
  pinMode(motorIn2Pin, OUTPUT);
  pinMode(motorEnablePin, OUTPUT);
  pinMode(forwardStopPin, INPUT_PULLDOWN);
  pinMode(reverseStopPin, INPUT_PULLDOWN);

  // KY-040 Rotary Encoder Setup
  myEnc.write(0);
  pinMode(encoderButtonPin, INPUT_PULLUP); // Enable pull-up for the button

  // Initialize motor to stopped state
  stopMotor();
  loglnI("Train Ready!");
}

void loop() {
  bool forwardStopDetected = digitalRead(forwardStopPin);
  bool reverseStopDetected = digitalRead(reverseStopPin);
  bool startStopButtonPressed = !digitalRead(encoderButtonPin); // Assuming LOW when pressed

  int encoderValue = myEnc.read() / encoderSensitivity;
  if (encoderValue != 0) {
    if (encoderValue > 0) {
      loglnD("Rotated clockwise");
      selectedSpeed = min(maxSpeed, selectedSpeed + speedStep); 
    }
    if (encoderValue < 0) {
      loglnD("Rotated coungterClockwise");
      selectedSpeed = max(minSpeed, selectedSpeed - speedStep);
    }

    permData.putInt("speed",selectedSpeed);
    logI("Selected speed:");
    loglnI(selectedSpeed);    
    
    //Reset Encoder
    myEnc.write(0);
  }

  if (startStopButtonPressed) {
    isTrainMoving = !isTrainMoving; // Toggle the moving state
    delay(200); // Debounce the button

    if (isTrainMoving) {
      //First check for Stop oversthoot...
      if (!forwardStopDetected && !reverseStopDetected) {
        if (previousDirection == FORWARD)
          moveReverse();
         else
          moveForward(); 
      } else if (!forwardStopDetected) {
          moveForward();
      } else if (!reverseStopDetected) {
          moveReverse();
      } else {
        loglnW("Both stops active. Cannot start.");
        stopMotor();
      }
    } else {
      stopMotor();
    }
  }

  if (currentDirection == FORWARD && forwardStopDetected) {
    stopMotor();
    loglnD("Forward stop reached.");
  } else if (currentDirection == REVERSE && reverseStopDetected) {
    stopMotor();
    loglnD("Reverse stop reached.");
  }

  if (isTrainMoving) {
    setMotorSpeed(selectedSpeed);
  } else {
    setMotorSpeed(0); // Ensure motor is stopped when not moving
  }

  delay(50);
}

void moveForward() {
  digitalWrite(motorIn1Pin, HIGH);
  digitalWrite(motorIn2Pin, LOW);
  setMotorSpeed(selectedSpeed);
  currentDirection = FORWARD;
  isTrainMoving = true;
  logI("Moving forward at speed: ");
  loglnI(selectedSpeed);
}

void moveReverse() {
  digitalWrite(motorIn1Pin, LOW);
  digitalWrite(motorIn2Pin, HIGH);
  setMotorSpeed(selectedSpeed);
  currentDirection = REVERSE;
  isTrainMoving = true;
  logI("Moving reverse at speed: ");
  loglnI(selectedSpeed);
}

void stopMotor() {
  digitalWrite(motorIn1Pin, LOW);
  digitalWrite(motorIn2Pin, LOW);
  setMotorSpeed(0);
  previousDirection = currentDirection;
  currentDirection = STOPPED;
  isTrainMoving = false;
  loglnI("Motor stopped.");
}

void setMotorSpeed(int speed) {
  if (speed >= minSpeed && speed <= maxSpeed) {
    analogWrite(motorEnablePin, speed);
  } else {
    loglnE("Invalid speed value.");
  }
}
