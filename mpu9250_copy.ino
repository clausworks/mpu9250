/* MPU9250 Basic Example Code
by: Kris Winer
date: April 1, 2014
license: Beerware - Use this code however you'd like. If you
find it useful you can buy me a beer some time.

Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor,
getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to
allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and
Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

SDA and SCL should have external pull-up resistors (to 3.3V).
10k resistors are on the EMSENSR-9250 breakout board.

Hardware setup:
MPU9250 Breakout --------- Arduino
VDD ---------------------- 3.3V
VDDI --------------------- 3.3
SDA ----------------------- SDA
SCL ----------------------- SCL
GND ---------------------- GND

Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library.
Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
*/


/*
* Also using LCD_ON1602 display
* LCD_ON1602 --------- Arduino Uno
* VSS ------------- 5V
* VDD ------------- GND
* V0 -------------- 5V (w/ 2K-Ohm resistor)
* RS -------------- 3
* RW -------------- 5V
* E  -------------- 4
* D4 -------------- 5
* D5 -------------- 6
* D6 -------------- 7
* D7 -------------- 8
* A  -------------- 5V
* K  -------------- GND (w/ 220-Ohm resistor)
*/
#include <LiquidCrystal.h>
LiquidCrystal lcd(3,4,5,6,7,8);
// #include <SPI.h>
#include "mpu.h"

MPU myMPU = MPU();

// global vars
const int NUM_YAW_VALS = 10;
float yawVals[NUM_YAW_VALS];
float oldYawVals[NUM_YAW_VALS*5];
unsigned int updateCounter = 0;

// time
long now = millis();
long lastUpdate_50 = millis();
long lastUpdate_500 = millis();

void setup() {
    lcd.begin(16,2);
    Serial.begin(115200);
    lcd.print("Setup...");
    myMPU.setupMPU();
    lcd.clear();
    lcd.print("Done.");

    // initialize yawVals
    myMPU.updateMPU();
    for (size_t i = 0; i < NUM_YAW_VALS; i++) {
        yawVals[i] = myMPU.getYaw();
        oldYawVals[i] = myMPU.getYaw();
    }
}


void loop() {
    now = millis();
    myMPU.updateMPU();
    int curAvg, oldAvg;

    // update 10 times every second
    if (now - lastUpdate_50 > 50) {
        lastUpdate_50 = now;
        // Serial.print(myMPU.getYaw(),2);
        // Serial.print("\t");

        oldYawVals[updateCounter] = yawVals[updateCounter]; // copy oldest value
        yawVals[updateCounter] = myMPU.getYaw(); // update one value

        curAvg = round(calcAvg(yawVals, NUM_YAW_VALS));
        oldAvg = round(calcAvg(oldYawVals, NUM_YAW_VALS));

        lcd.clear();
        if ((abs(curAvg-oldAvg)%360) < 10) {
            lcd.print("Not turning");
        }
        else {
            lcd.print("Turning");
        }

        Serial.println(curAvg);
        Serial.print("\t");
        Serial.println(abs(curAvg-oldAvg)%360);

        ++updateCounter;
        if (updateCounter == NUM_YAW_VALS) updateCounter = 0;
    }

    // if (now - lastUpdate_500 > 500) {
    //     lastUpdate_500 = now;
    //     oldAvg = curAvg; // update only every 1/2 second
    // }
}



float calcAvg(float nums[], int numItems) {
    float sum = 0;
    for (size_t i = 0; i < numItems; i++) {
        sum += nums[i];
    }
    return sum / numItems;
}
