#include <Encoder.h>
#include <Preferences.h>
#define LOGLEVEL LOGLEVEL_DEBUGING
#include <logger.h>

// ESP32 BLE Train Control Code
// This code sets up the ESP32 as a Bluetooth Low Energy (BLE) peripheral
// to receive commands for a toy train from a PWA.
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLECharacteristic.h>

// Define the UUIDs for our BLE Service and Characteristics.
// You can generate new UUIDs using an online UUID generator if needed.
// These must match the UUIDs used in your PWA.
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // Train Control Service
#define CHARACTERISTIC_UUID_SPEED     "beb5483e-36e1-4688-b7f5-ea07361b26a8" // Speed (0-255)
#define CHARACTERISTIC_UUID_DIRECTION "a8a2d1d4-1f19-4b6e-b302-3c2c1a89c9c1" // Direction (0=Stop, 1=Forward, 2=Backward)
#define CHARACTERISTIC_UUID_LIGHTS    "e7e3f1c1-4b1a-4d3f-8c3b-7f2a1b9d4c7d" // Lights (0=Off, 1=On)
#define CHARACTERISTIC_UUID_HORN      "f3d7c5b9-8e2b-4a5c-9d1a-6e3c2b1d4a8f" // Horn (0=Off, 1=Honk)

BLEServer* pServer = NULL;
BLECharacteristic* pSpeedCharacteristic = NULL;
BLECharacteristic* pDirectionCharacteristic = NULL;
BLECharacteristic* pLightsCharacteristic = NULL;
BLECharacteristic* pHornCharacteristic = NULL;

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
#define ledON 1
#define ledOFF 0

// Motor Control States
#define FORWARD 1
#define REVERSE 2
#define STOPPED 0

//The Permernent store
Preferences permData;

bool deviceConnected = false;
int currentDirection = STOPPED;
int previousDirection = REVERSE;
int currentLights = ledON;             //0=Off, 1=On
volatile int selectedSpeed = 150;   // 0-255 for PWM control
bool isTrainMoving = false;

// Speed control parameters
#define speedStep 10 // Amount to change speed per encoder click
#define minSpeed 0
#define maxSpeed 255
#define encoderSensitivity 4 // Adjust for how many encoder steps per speed change
Encoder myEnc(encoderPinDT, encoderPinCLK);

//Timer Variables, to avoid long Delays (blokking execution)
unsigned long prevBLEMillis = 0; // Stores the last time the event happened
const long bLEIinterval = 1000;      // Interval at which to run the code (in milliseconds, 1000ms = 1 second)

//Call backs for the BLE calls
// Callback class for handling BLE server events (connections/disconnections)
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      loglnI("Client connected!");
      // You can add logic here to indicate connection status, e.g., turn on an LED.
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      loglnI("Client disconnected. Starting advertising again...");
      BLEDevice::startAdvertising(); // Restart advertising to allow new connections
      // You can add logic here to indicate disconnection status, e.g., turn off an LED.
    }
};

// Callback class for handling characteristic write events
class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue();

      if (value.length() > 0) {
        Serial.print("Characteristic ");
        Serial.print(pCharacteristic->getUUID().toString().c_str());
        Serial.print(" received: ");
        uint8_t receivedValue = (uint8_t)value[0];
        loglnI(receivedValue);

        if (pCharacteristic->getUUID().equals(BLEUUID(CHARACTERISTIC_UUID_SPEED))) {
          selectedSpeed = receivedValue;
          Serial.print("Train Speed set to: ");
          loglnI(selectedSpeed);
          // TODO: Add your motor control logic here based on 'selectedSpeed'
          // Example: analogWrite(motorPin, selectedSpeed);
        } else if (pCharacteristic->getUUID().equals(BLEUUID(CHARACTERISTIC_UUID_DIRECTION))) {
          currentDirection = receivedValue;
          Serial.print("Train Direction set to: ");
          switch (currentDirection) {
            case 0: loglnI("STOP"); break;
            case 1: loglnI("FORWARD"); break;
            case 2: loglnI("BACKWARD"); break;
            default: loglnI("UNKNOWN"); break;
          }
          // TODO: Add your direction control logic here based on 'currentDirection'
          // Example:
          // if (currentDirection == 1) { digitalWrite(dirPin1, HIGH); digitalWrite(dirPin2, LOW); }
          // else if (currentDirection == 2) { digitalWrite(dirPin1, LOW); digitalWrite(dirPin2, HIGH); }
          // else { digitalWrite(dirPin1, LOW); digitalWrite(dirPin2, LOW); } // Stop
        } else if (pCharacteristic->getUUID().equals(BLEUUID(CHARACTERISTIC_UUID_LIGHTS))) {
          currentLights = receivedValue;
          Serial.print("Train Lights set to: ");
          if (currentLights == 1) {
            loglnI("ON");
          } else {
            loglnI("OFF");
          }
          // TODO: Add your lights control logic here based on 'currentLights'
          // Example: digitalWrite(lightPin, currentLights == 1 ? HIGH : LOW);
        } else if (pCharacteristic->getUUID().equals(BLEUUID(CHARACTERISTIC_UUID_HORN))) {
          // Horn is typically a momentary action, so we just trigger it and reset.
          if (receivedValue == 1) {
            loglnI("Horn activated!");
            // TODO: Add your horn activation logic here (e.g., play a sound, momentary high output to a buzzer)
            // It might be good to reset the horn characteristic value back to 0 after a short delay
            // or expect the PWA to send 0 after triggering the horn.
          }
        }
      }
    }
};

/************************************************************ SETUP *********************************************************/
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
    loglnI("Blink..");  
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

 // Initialize BLE device with a name that will appear to other devices
  BLEDevice::init("SnapseToget");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristics and add them to the service
  // Characteristics are initialized with read and write properties.
  // The PWA will write to these.
  pSpeedCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_SPEED,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pSpeedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  int tempSpeed = selectedSpeed;
  pSpeedCharacteristic->setValue(tempSpeed); // Set initial value

  pDirectionCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_DIRECTION,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pDirectionCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  int tempDirection = currentDirection;
  pDirectionCharacteristic->setValue(tempDirection); // Set initial value

  pLightsCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_LIGHTS,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pLightsCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  int tempLights = currentLights;
  pLightsCharacteristic->setValue(tempLights); // Set initial value

  pHornCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_HORN,
                      BLECharacteristic::PROPERTY_WRITE // Horn is write-only
                    );
  pHornCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  int tempHorn = 0;
  pHornCharacteristic->setValue(tempHorn); // Initial horn state is off

  // Start the service
  pService->start();

  // Start advertising so the device can be found by the PWA
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  loglnI("BLE advertising started. Waiting for a client connection...");
}

void loop() {
  bool forwardStopDetected = digitalRead(forwardStopPin);
  bool reverseStopDetected = digitalRead(reverseStopPin);
  bool startStopButtonPressed = !digitalRead(encoderButtonPin); // Assuming LOW when pressed
  unsigned long currentMillis = millis();    // for non blocking delays
    
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

  //Every secund update Train status to BLE connected device  
  if (deviceConnected && (currentMillis - prevBLEMillis  >= bLEIinterval) ) {
    int tempSpeed = selectedSpeed;
    pSpeedCharacteristic->setValue(tempSpeed); // Set initial value
    pSpeedCharacteristic->notify(); // Set initial value
    int tempDirection = currentDirection;
    pDirectionCharacteristic->setValue(tempDirection); // Set initial value
    pDirectionCharacteristic->notify();
    int tempLights = currentLights;
    pLightsCharacteristic->setValue(tempLights); // Set initial value
    pLightsCharacteristic->notify(); // Notify connected client if configured with NOTIFY property

    loglnI("BLE device updated");
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
