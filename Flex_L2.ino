/******************************************************************************
Flex_Sensor_Example.ino
Example sketch for SparkFun's flex sensors
  (https://www.sparkfun.com/products/10264)
Jim Lindblom @ SparkFun Electronics
April 28, 2016

Create a voltage divider circuit combining a flex sensor with a 47k resistor.
- The resistor should connect from A0 to GND.
- The flex sensor should connect from A0 to 3.3V
As the resistance of the flex sensor increases (meaning it's being bent), the
voltage at A0 should decrease.

Development environment specifics:
Arduino 1.6.7
******************************************************************************/
#define SIM800C_AXP192_VERSION_20200609
#include "utilities.h"
const int FLEX_PIN_1 = 34; // Pin connected to voltage divider output
const int FLEX_PIN_2 = 25;
int but1 = 32; //check for ttgo
int but2 = 33;
// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 3.3; // Measured voltage of Ardunio 5V line
const float R_DIV_1 = 21300.0; // Measured resistance of 3.3k resistor
const float R_DIV_2 = 21300.0;
// Upload the code, then try to adjust these values to more
// accurately calculate bend degree.
float STRAIGHT_RESISTANCE_1 = 0.0 ; // resistance when straight
float STRAIGHT_RESISTANCE_2 = 0.0 ;
float STRAIGHT_RESISTANCE = 0.0 ;
float BEND_RESISTANCE_1 = 0.0; // resistance at 90 deg
float BEND_RESISTANCE_2 = 0.0;
float BEND_RESISTANCE = 0.0;

#define SerialMon Serial
// Set serial for AT commands (to the module)
#define SerialAT  Serial1

#include <Fuzzy.h>
#include <FuzzyComposition.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzyOutput.h>
#include <FuzzyRule.h>
#include <FuzzyRuleAntecedent.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzySet.h>

Fuzzy *fuzzy = new Fuzzy();


// Resistence
FuzzySet *small = new FuzzySet(0,5000,10000,12000);
FuzzySet *mid = new FuzzySet(8000,15000,22000,25000);
FuzzySet *big = new FuzzySet(20000,27000 ,32000,40000);
  
// Percent
FuzzySet *lowp = new FuzzySet(0,25,40,60);
FuzzySet *midp = new FuzzySet(40,65,70,100);
FuzzySet *highp = new FuzzySet(70,90,100,100);

// Declare FuzzyRuleAntecedent variables outside of setup function
FuzzyRuleAntecedent *ifFlexRsmall;
FuzzyRuleAntecedent *ifFlexRMid;
FuzzyRuleAntecedent *ifFlexRBig;

void setup() {
  bool flag = false;
  Serial.begin(115200);
  pinMode(FLEX_PIN_1, INPUT);
  pinMode(FLEX_PIN_2, INPUT);
  pinMode(but1, INPUT);
  pinMode(but2, INPUT);
   
  Serial.println("Entering While LOOP");
  while(!flag){
    int b1 = digitalRead(but1);
    int b2 = digitalRead(but2);
    int flexADC_1 = analogRead(FLEX_PIN_1);
    int flexADC_2 = analogRead(FLEX_PIN_2);
    Serial.print("B1: ");
    Serial.println(b1);
    Serial.print("B2: ");
    Serial.println(b2);
    delay(1000);
    if (b1 == 0){
      float flexV_1 = flexADC_1 * VCC / 4095.0;
      float flexV_2 = flexADC_2 * VCC / 4095.0;
      STRAIGHT_RESISTANCE_1 = (R_DIV_1 * flexV_1)/(VCC-flexV_1);
      STRAIGHT_RESISTANCE_2 = (R_DIV_2 * flexV_2)/(VCC-flexV_2);
      STRAIGHT_RESISTANCE = (STRAIGHT_RESISTANCE_1 + STRAIGHT_RESISTANCE_2)/2;
      Serial.println("0% Recorded:"+String(STRAIGHT_RESISTANCE));
      
    }
    if (b2 == 0){
      float flexV_1 = flexADC_1 * VCC / 4095.0;
      float flexV_2 = flexADC_2 * VCC / 4095.0;
      BEND_RESISTANCE_1 = (R_DIV_1 * flexV_1)/(VCC-flexV_1);
      BEND_RESISTANCE_2 = (R_DIV_2 * flexV_2)/(VCC-flexV_2);
      BEND_RESISTANCE = (BEND_RESISTANCE_1 + BEND_RESISTANCE_2)/2;
      Serial.println("100% recorded:"+String(BEND_RESISTANCE));
    }
    if (STRAIGHT_RESISTANCE_1!=0.0 && STRAIGHT_RESISTANCE_2!=0.0 && BEND_RESISTANCE_1!=0.0 && BEND_RESISTANCE_1!=0.0){
      flag = true;
    } 
  }
  Serial.println("Exiting While LOOP");


  // variable Resistance with universe 30-48k as input
  FuzzyInput *flexR = new FuzzyInput(1);
  flexR->addFuzzySet(small);
  flexR->addFuzzySet(mid);
  flexR->addFuzzySet(big);
  fuzzy->addFuzzyInput(flexR);

  // variable Percent with universe 0-100 as output
  FuzzyOutput *percent = new FuzzyOutput(1);
  percent->addFuzzySet(lowp);
  percent->addFuzzySet(midp);
  percent->addFuzzySet(highp);
  fuzzy->addFuzzyOutput(percent);

  // rules
  // if Resistance is small then percent is low
  ifFlexRsmall = new FuzzyRuleAntecedent();
  ifFlexRsmall->joinSingle(small);
  FuzzyRuleConsequent *thenPercentLowp = new FuzzyRuleConsequent();
  thenPercentLowp->addOutput(lowp);
  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, ifFlexRsmall, thenPercentLowp);
  fuzzy->addFuzzyRule(fuzzyRule1);

  // if Resistance is medium then percent is medium
  ifFlexRMid = new FuzzyRuleAntecedent();
  ifFlexRMid->joinSingle(mid);
  FuzzyRuleConsequent *thenPercentMidp = new FuzzyRuleConsequent();
  thenPercentMidp->addOutput(midp);
  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, ifFlexRMid, thenPercentMidp);
  fuzzy->addFuzzyRule(fuzzyRule2);

  // if Resistance is big then percent is high
  ifFlexRBig = new FuzzyRuleAntecedent();
  ifFlexRBig->joinSingle(big);
  FuzzyRuleConsequent *thenPercentHighp = new FuzzyRuleConsequent();
  thenPercentHighp->addOutput(highp);
  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, ifFlexRBig, thenPercentHighp);
  fuzzy->addFuzzyRule(fuzzyRule3);
}

void loop() {

  Serial.println(BEND_RESISTANCE);
  Serial.println(STRAIGHT_RESISTANCE);
  // Read the ADC, and calculate voltage and resistance from it
  int flexADC_1 = analogRead(FLEX_PIN_1);
  int flexADC_2 = analogRead(FLEX_PIN_2);
  int flexADC  = (flexADC_1 + flexADC_2)/2;
  Serial.println("RFlex Value  " + String(flexADC));

  float flexV_1 = flexADC_1 * VCC / 4095.0;
  float flexV_2 = flexADC_2 * VCC / 4095.0;
  float flexR_1 = (R_DIV_1 * flexV_1)/(VCC-flexV_1);
  float flexR_2 = (R_DIV_2 * flexV_2)/(VCC-flexV_2);
  float flexR = (flexR_1 + flexR_2)/2;

  fuzzy->setInput(1, flexR); // dist as fuzzy input 1 (Resistence))
  fuzzy->fuzzify();
  
  Serial.println("Resistance : " + String(flexR) + " ohms");
 

  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  float percent_1 = map(flexR_1, STRAIGHT_RESISTANCE_1, BEND_RESISTANCE_1, 0, 100);
  float percent_2 = map(flexR_2, STRAIGHT_RESISTANCE_2, BEND_RESISTANCE_2, 0, 100);
  float percent = (percent_1 + percent_2)/2;


  // defuzzyfication
  int output = fuzzy->defuzzify(1); // defuzzify fuzzy output 1 (percent)


  Serial.println("percent : " + String(percent) + " %");
  Serial.println();
  delay(500);

  
}
