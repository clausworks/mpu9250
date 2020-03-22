/*
HARDWARE SETUP:

MPU9250 Breakout --------- Arduino
VDD ---------------------- 3.3V
SDA ---------------------- SDA
SCL ---------------------- SCL
GND ---------------------- GND
INT ---------------------- 12

LCD1602 display
LCD1602 ------------------ Arduino
VSS ---------------------- 5V
VDD ---------------------- GND
V0 ----------------------- 5V (w/ 2K-Ohm resistor)
RS ----------------------- 3
RW ----------------------- 5V
E  ----------------------- 4
D4 ----------------------- 5
D5 ----------------------- 6
D6 ----------------------- 7
D7 ----------------------- 8
A  ----------------------- 5V
K  ----------------------- GND (w/ 220-Ohm resistor)

Piezo Buzzer
Buzzer ------------------- Arduino
(+) ---------------------- 9
(-) ---------------------- GND

*/
#include <LiquidCrystal.h>
LiquidCrystal lcd(3,4,5,6,7,8);
// #include <SPI.h>
#include "mpu.h"

MPU myMPU = MPU();

// global vars
const int NUM_YAW_VALS = 10; // number of recent yaw values to keep track of
float yawVals[NUM_YAW_VALS]; // holds recent yaw values
unsigned int updateCounter_50 = 0; // counter for 50ms loop

// time
long now = millis();
long lastUpdate_50 = millis(); // last update time for 50ms loop
long lastUpdate_500 = millis(); // last update time for 500ms loop

void setup() {
    lcd.begin(16,2);

    lcd.clear();
    lcd.print("Think");
    lcd.setCursor(5,1);
    lcd.print("different.");
    delay(500);

    Serial.begin(115200);

    lcd.clear();
    lcd.print("Wave in figure-");
    lcd.setCursor(0,1);
    lcd.print("eight motion");
    myMPU.setupMPU();
    lcd.clear();
    lcd.print("Done.");

    // initialize yawVals
    myMPU.updateMPU();
    for (size_t i = 0; i < NUM_YAW_VALS; i++) {
        yawVals[i] = myMPU.getYaw();
    }
}

int desiredYaw = 90;
void loop() {
    now = millis();
    myMPU.updateMPU();

    int avg;
    if (now - lastUpdate_50 > 50) {
        lastUpdate_50 = now;

        yawVals[updateCounter_50] = myMPU.getYaw(); // update one value

        float sum = 0;
        for (size_t i = 0; i < NUM_YAW_VALS; i++) {
            sum += yawVals[i];
        }
        avg = round(sum / NUM_YAW_VALS);

        int deltaTheta = desiredYaw - avg;

        // buzzer feedback
        if (abs(deltaTheta) > 15) {
            tone(9, 500+deltaTheta); // mid tone varies with deltaTheta
        }
        else if (abs(deltaTheta) < 2) {
                tone(9,1000); // high tone for within 2 degrees
        }
        else {
            noTone(9); // no tone for within 15 degrees
        }

        lcd.clear();
        lcd.print("Position: ");
        lcd.print(avg);
        lcd.setCursor(0,1);
        lcd.print(deltaTheta);
        Serial.println(avg);
        Serial.print("\t");
        Serial.println(deltaTheta);

        ++updateCounter_50;
        if (updateCounter_50 == NUM_YAW_VALS) updateCounter_50 = 0;
    }
}
