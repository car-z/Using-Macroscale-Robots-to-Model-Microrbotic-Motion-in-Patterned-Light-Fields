#include <Wire.h>
#include <VL6180X.h>
#include <Romi32U4.h>

#define ADDRESS_R 0x30
#define ADDRESS_L 0x31


Romi32U4Motors motors;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
Romi32U4Encoders encoders;

VL6180X rightSensor; //5 PIN
VL6180X leftSensor; //0 PIN

uint16_t lowerBound;
uint16_t upperBound;
int count = 0;

void setup() {
  Serial.begin(9600);
  while(!Serial) {
    ;
  }
  Wire.begin();

  pinMode(0,OUTPUT);
  pinMode(5,OUTPUT);
  digitalWrite(0,LOW);
  digitalWrite(5,HIGH);

  leftSensor = VL6180X();
  leftSensor.init();
  leftSensor.configureDefault();

  leftSensor.setAddress(ADDRESS_L);

  digitalWrite(0,HIGH);

  rightSensor = VL6180X();
  rightSensor.init();
  rightSensor.configureDefault();

  rightSensor.setAddress(ADDRESS_R);

  rightSensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  rightSensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);

  rightSensor.setTimeout(500);

  rightSensor.stopContinuous();

  delay(300);

  rightSensor.startAmbientContinuous(100);

  leftSensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  leftSensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);

  leftSensor.setTimeout(500);

  leftSensor.stopContinuous();
  delay(300);

  readBounds();
  while (!checkValidity()) {
    readBounds();
  }

  leftSensor.startAmbientContinuous(100);
  
  //wait for button press to begin running continuous ALS measurements
  ledYellow(1);
  delay(250);
  ledYellow(0);
  buttonA.waitForButton();
}

void loop() {

  if (buttonB.isPressed()) {
    motors.setSpeeds(0,0);
    delay(1000);
    Serial.println("Stopped. Press B again to Start.");
    buttonB.waitForButton();
  }

  if (buttonC.isPressed()) {
    motors.setSpeeds(0,0);
    delay(1000);
    Serial.println("Reset. Press A to get lower bound");
    leftSensor.stopContinuous();
    rightSensor.stopContinuous();
    buttonA.waitForButton();
    delay(300);

    readBounds();
    while (!checkValidity()) {
      readBounds();
    }
  
    leftSensor.startAmbientContinuous(100);
    rightSensor.startAmbientContinuous(100);
  
    //wait for button press to begin running continuous ALS measurements
    ledYellow(1);
    delay(250);
    ledYellow(0);
    buttonA.waitForButton();

  }

  uint16_t alsValueRight = rightSensor.readAmbientContinuous();
  if (rightSensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  uint16_t alsValueLeft = leftSensor.readAmbientContinuous();
  if (leftSensor.timeoutOccurred()) { Serial.println(" TIMEOUT"); }

  uint16_t speedRight = speedMappingFunction(alsValueRight) + 1;
  uint16_t speedLeft = speedMappingFunction(alsValueLeft) -1;
  
  motors.setSpeeds(speedLeft,speedRight);

  Serial.print(encoders.getCountsAndResetLeft());
  Serial.print(" ");
  Serial.print(encoders.getCountsAndResetRight());
  Serial.print(" ");
  Serial.print(alsValueLeft);
  Serial.print(" ");
  Serial.println(alsValueRight);

  delay(10);  
}

uint16_t pixelMappingFunction(uint16_t s) {
  if (s <= lowerBound) {
    return 0;
  } else if (s >= upperBound) {
    return 255;
  } else {
    return ((s - lowerBound) * (255.0/(upperBound-lowerBound)));
  }
}

//where full speed is achieved at upperBound, and stopped at lowerBound
uint16_t speedMappingFunction(uint16_t s) {
  if (s <= lowerBound) {
    return 15;
  } else if (s >= upperBound) {
    return 175;
  } else {
    return (s - lowerBound) * (175.0/(upperBound-lowerBound)) + 15;
  }
}

void readBounds() {
  Serial.println("starting readings");
   // start continuous mode with period of 100 ms
  leftSensor.startAmbientContinuous(100);
  //get reading for lower Bound (ambient light)
  lowerBound = leftSensor.readAmbientSingle();
  Serial.print("Lower Bound reading: ");
  Serial.println(lowerBound);
  ledRed(1);
  delay(2000);
  ledRed(0);

  //set up bright light beam for upper bound
  //press button when ready
  buttonA.waitForButton();
  upperBound = leftSensor.readAmbientSingle();
  Serial.print("Upper Bound reading: ");
  Serial.println(upperBound);
  leftSensor.stopContinuous();
  ledRed(1);
  delay(2000);
  ledRed(0);
}

bool checkValidity() {
  return upperBound > lowerBound;
}

