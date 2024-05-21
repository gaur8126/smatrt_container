
#define SIM800C_AXP192_VERSION_20200609
#include "utilities.h"
#include <SimpleKalmanFilter.h>
#include "BluetoothSerial.h"


//#include "avr/sleep.h"
//#define USE_PIN // Uncomment this to use PIN during pairing. The pin is specified on the line below
const char *pin = "1234"; // Change this to more secure PIN.

String device_name = "ESP32-BT-Slave";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

const int FLEX_PIN = 34; // Pin connected to voltage divider output
const int but1 = 32; //check for ttgo
const int but2 = 33;
// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 3.3; // Measured voltage of Ardunio 5V line
const float R_DIV = 21500.0; // Measured resistance of 3.3k resistor

// Upload the code, then try to adjust these values to more
// accurately calculate bend degree.
float STRAIGHT_RESISTANCE = 0.0 ; // resistance when straight
float BEND_RESISTANCE = 0.0; // resistance at 90 deg

SimpleKalmanFilter simpleKalmanFilter1(2, 2, 0.01);
SimpleKalmanFilter simpleKalmanFilter2(4, 2, 0.01);
 SimpleKalmanFilter simpleKalmanFilter3(2, 1, 0.1);


#define SIM800C_AXP192_VERSION_20200609
// #define SIM800L_IP5306_VERSION_20200811

// Define the serial console for debug prints, if needed
#define DUMP_AT_COMMANDS
#define TINY_GSM_DEBUG          SerialMon


// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to the module)
#define SerialAT  Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800          // Modem is SIM800
#define TINY_GSM_RX_BUFFER      1024   // Set RX buffer to 1Kb

#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// Set phone numbers, if you want to test SMS and Calls
#define SMS_TARGET  "+918218108134"
#define CALL_TARGET "+918218108134"


#define SerialMon Serial
// Set serial for AT commands (to the module)
#define SerialAT  Serial1

void setup() 
{

  Serial.begin(115200);
  SerialBT.begin(device_name); //Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  //Serial.printf("The device with name \"%s\" and MAC address %s is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str(), SerialBT.getMacString()); // Use this after the MAC method is implemented
  #ifdef USE_PIN
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif

  // Start power management
  if (setupPMU() == false) {
      Serial.println("Setting power error");
  }

  // Some start operations
  setupModem();

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

  delay(6000);  

  bool flag = false;
  //Serial.begin(115200);
  pinMode(FLEX_PIN, INPUT);
  pinMode(but1, INPUT);
  pinMode(but2, INPUT);

  Serial.println("Entering While LOOP");
  while(!flag){
    int b1 = digitalRead(but1);
    int b2 = digitalRead(but2);
    int flexADC = analogRead(FLEX_PIN);
    Serial.print("B1: ");
    Serial.println(b1);
    Serial.print("B2: ");
    Serial.println(b2);
    delay(1000);
    if (SerialBT.available()){
      char b1 = SerialBT.read();
      if (b1 == 'a'){
        digitalWrite(but1,LOW);
        float flexV = flexADC * VCC / 4095.0;
        STRAIGHT_RESISTANCE = (R_DIV * flexV)/(VCC-flexV);
        Serial.println("0% Recorded:"+String(STRAIGHT_RESISTANCE));
      }
      
      char b2 = SerialBT.read();
      if (b2 =='b'){
        digitalWrite(but2,LOW);
        float flexV = flexADC * VCC / 4095.0;
        BEND_RESISTANCE = (R_DIV * flexV)/(VCC-flexV);
        Serial.println("100% recorded:"+String(BEND_RESISTANCE));
      }
    }  
    if (STRAIGHT_RESISTANCE!=0.0 && BEND_RESISTANCE!=0.0){
      flag = true;
    } 
  }
  Serial.println("Exiting While LOOP");
}


void sendSMSIfNeeded(float estimated_percent) {
  static bool flag = false; // Static variable to track whether SMS has been sent

  if (estimated_percent < 20 && !flag) {
    SerialMon.println("Initializing modem...");
    modem.restart();
    delay(1000);
    String imei = modem.getIMEI();
    DBG("IMEI:", imei);
    bool res = modem.sendSMS(SMS_TARGET, "Your volume is running down! Please check.");
    DBG("SMS:", res ? "OK" : "fail");
  

    flag = true; // Set flag to true indicating SMS has been sent
  } else if (estimated_percent > 25 && flag) {
    flag = false; // Reset flag when condition is no longer met
  }
}

void loop() 
{

  Serial.println(BEND_RESISTANCE);
  Serial.println(STRAIGHT_RESISTANCE);
  // Read the ADC, and calculate voltage and resistance from it
  int flexADC = analogRead(FLEX_PIN);
  int flexADC_measured  = flexADC + random(0,4095)/100.0;
  int estimated_flexADC = simpleKalmanFilter3.updateEstimate(flexADC_measured);
  
  Serial.println("RFlex Value  " + String(estimated_flexADC));

  float flexV = estimated_flexADC * VCC / 4095.0;
  float flexR = (R_DIV * flexV)/(VCC-flexV);
  float flexR_measured = flexR + random(0,40000)/100.0;
  float estimated_flexR = simpleKalmanFilter2.updateEstimate(flexR_measured);
  Serial.println("Resistance: " + String(estimated_flexR) + " ohms");
 
  float percent = map(estimated_flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
                   0, 100);
  float measured_percent = percent + random(0,100)/100.0;
  float estimated_percent = simpleKalmanFilter1.updateEstimate(measured_percent);
  estimated_percent = constrain(estimated_percent, 0 ,100);
 

  Serial.println("precent: " + String(estimated_percent) + " %");
  Serial.println();
  SerialBT.print(estimated_percent);
  
  static bool flag = false ;
  if (estimated_percent > 0 && !flag){
    flag = true;
  }else if (estimated_percent < 100 && flag){
    sendSMSIfNeeded(estimated_percent);
    flag = false;
  }
  

  delay(1000);

}


