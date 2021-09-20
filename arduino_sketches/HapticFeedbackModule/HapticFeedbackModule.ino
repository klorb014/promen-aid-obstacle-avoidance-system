/*

    PROMENAID HAPTIC FEEDBACK MODULE SKETCH

    This program is responsible to communicating the distance and direction of
    obstacles in the environment to the user via vibrational motor pulses. This
    device relies on an ESP32 Microcontroller to recieve BLE messages from the
    Perception Module. The Haptic Feedback Module is also responsible for sending
    its battery state back to the Perception Module.

    The program is configured as a BLE server and is on the example created by
    Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleWrite.cpp
    Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>


#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// assign motor pins
const int motor1 = 5;
const int motor2 = 18;


// setting PWM properties
const int motor1PWMChannel = 0;
const int motor2PWMChannel = 1;
const int freq = 5000;
const int resolution = 8;

// variables that keep track of the current
// vibration motor duty cycles
int motor1DutyCycle = 0;
int motor2DutyCycle = 0;


/*
 * Class:  HapticCallback
 * --------------------
 * Class responsible for parsing BLE messages sent from the Perception
 * Module
 */
class HapticCallback: public BLECharacteristicCallbacks {
  /*
   * Function:  parseMotorSpeed
   * --------------------
   * parser function that reads the motor speed percentage (0-100%)
   *
   *  return: int from 0-100 corresponding to the desired motor speed.
   *          Returns -1 if the input is invalid.
   */
  int parseMotorSpeed(std::string& speedInput){
        int motorSpeed = 0;
        if (isdigit(speedInput[0])){
          motorSpeed += (speedInput[0]- '0') * 100;
          if (isdigit(speedInput[1])){
            motorSpeed += (speedInput[1]- '0') * 10;
            if (isdigit(speedInput[2])){
              motorSpeed += (speedInput[2]- '0');
              if (motorSpeed > 100){
                return -1;
              }else{
                return motorSpeed;
              }
            }
          }
        }
        return -1;
      }
  /*
   * Function:  parseMotorIndex
   * --------------------
   * parser function that reads the selected vibration motor
   *
   *  return: int from 0-2 representing the index of selected vibration motor.
   */
    int parseMotorIndex(std::string& indexInput){
      if (isdigit(indexInput[4])){
        int index = (indexInput[4]- '0');
        if (index >= 0 && index < 3){
          return index;
        }
      }
      return -1;
    }
    /*
     * Function:  onWrite
     * --------------------
     * callback function that accepts incoming BLE messages. The function is
     * expecting messages with the structure XXXMY, where XXX is a string
     * representing a value between zero and 100 (000-100) and Y is represents
     * the desired vibration motor index (0-2)
     *
     */
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() == 5) {
        value.c_str();
        if (value[3] == 'M'){

          Serial.println("*********");
          Serial.print("New value: ");
          for (int i = 0; i < value.length(); i++)
            Serial.print(value[i]);

          Serial.println();
          int motorSpeed = HapticCallback::parseMotorSpeed(value);
          int motorIndex = HapticCallback::parseMotorIndex(value);
          if (motorIndex == 1){
            motor1DutyCycle = motorSpeed;
          }
          else if (motorIndex == 2){
            motor2DutyCycle = motorSpeed;
          }
          Serial.println(motorSpeed);
          Serial.println(motorIndex);
          Serial.println("*********");
        }
      }
    }


};

void setup() {
  // Code for setting up the BLE server
  Serial.begin(115200);

  BLEDevice::init("PromenAid Haptic");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new HapticCallback());

  pCharacteristic->setValue("PromenAid Haptic Running");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();


  // Code for PWM channels for the vibration motors
  ledcSetup(motor1PWMChannel, freq, resolution);
  ledcAttachPin(motor1, motor1PWMChannel);

  ledcSetup(motor2PWMChannel, freq, resolution);
  ledcAttachPin(motor2, motor2PWMChannel);
}

void loop() {
  // Code to continously adjust the PWM signals according
  // to the received BLE messages:
  int motor1PWMValue = map(motor1DutyCycle,0,100,0,255);
  int motor2PWMValue = map(motor2DutyCycle,0,100,0,255);
  ledcWrite(motor1PWMChannel, motor1PWMValue);
  ledcWrite(motor2PWMChannel, motor2PWMValue);
  delay(1);


}
