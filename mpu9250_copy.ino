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

#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include "magRegisters.h"
#include "vars.h"
#include "functions.h"

#define AHRS true         // set to false for basic data read
#define SerialDebug true   // set to true to get Serial output for debugging
bool VERBOSE = false;
bool LCD_ON = true;

// lcd
LiquidCrystal lcd(3,4,5,6,7,8);

void setup()
{
    lcd.begin(16,2);
    Wire.begin();
    //  TWBR = 12;  // 400 kbit/sec I2C speed
    Serial.begin(115200);


    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(intPin, INPUT);
    digitalWrite(intPin, LOW);
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);

    if (LCD_ON) {
        // Print startup message
        lcd.clear();
        lcd.print("Think");
        lcd.setCursor(5,1);
        lcd.print("different.");
    }

    // Start device display with ID of sensor
    if (VERBOSE) {
        Serial.println("MPU9250");
        Serial.println("9-DOF 16-bit");
        Serial.println("motion sensor");
        Serial.println("60 ug LSB");
    }
    delay(800);


    // Read the WHO_AM_I register, this is a good test of communication
    byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250

    if (VERBOSE) {
        Serial.print("MPU9250 ");
        Serial.print("I AM ");
        Serial.print(c, HEX);
        Serial.print(" I should be ");
        Serial.println(0x71, HEX);
    }
    delay(800);



    if (c == 0x71) // WHO_AM_I should always be 0x68
    {
        if (VERBOSE) Serial.println("MPU9250 is online...");

        MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
        if (VERBOSE) {
            Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
            Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
            Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
            Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
            Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
            Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
        }

        calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

        if (VERBOSE) {
            Serial.println("MPU9250 bias");

            Serial.println(" x   y   z  ");


            Serial.print((int)(1000*accelBias[0]));
            Serial.print((int)(1000*accelBias[1]));
            Serial.print((int)(1000*accelBias[2]));
            Serial.print("mg");

            Serial.print(gyroBias[0], 1);
            Serial.print(gyroBias[1], 1);
            Serial.print(gyroBias[2], 1);
            Serial.println("o/s");

        }
        delay(1000);

        initMPU9250();

        if (VERBOSE) Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

        // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
        byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963

        if (VERBOSE) {
            Serial.print("AK8963 ");
            Serial.print("I AM ");
            Serial.print(d, HEX);
            Serial.print(" I should be ");
            Serial.println(0x48, HEX);
        }
        delay(1000);

        // Get magnetometer calibration from AK8963 ROM
        initAK8963(magCalibration);
        if (VERBOSE) Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
        getMres();

        if (LCD_ON) {
            lcd.clear();
            lcd.print("Wave in figure-");
            lcd.setCursor(0,1);
            lcd.print("eight motion");
        }
        magcalMPU9250(magBias,magScale);
        if (LCD_ON) {
            lcd.clear();
            lcd.print("Done.");
        }
        delay(500);


        if(VERBOSE) {
            //  Serial.println("Calibration values: ");
            Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
            Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
            Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);

            Serial.println("AK8963");
            Serial.println("ASAX ");
            Serial.println(magCalibration[0], 2);
            Serial.println("ASAY ");
            Serial.println(magCalibration[1], 2);
            Serial.println("ASAZ ");
            Serial.println(magCalibration[2], 2);
        }
        delay(1000);
    }
    else
    {
        Serial.print("Could not connect to MPU9250: 0x");
        Serial.println(c, HEX);
        lcd.clear();
        lcd.print("NO CONNECTION :(");
        while(1); // Loop forever if communication doesn't happen
    }
}

void loop()
{
    // If intPin goes high, all data registers have new data
    if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
        readAccelData(accelCount);  // Read the x/y/z adc values
        getAres();

        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
        ay = (float)accelCount[1]*aRes; // - accelBias[1];
        az = (float)accelCount[2]*aRes; // - accelBias[2];

        readGyroData(gyroCount);  // Read the x/y/z adc values
        getGres();

        // Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1]*gRes;
        gz = (float)gyroCount[2]*gRes;

        readMagData(magCount);  // Read the x/y/z adc values
        getMres();
        //    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
        //    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
        //    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
        my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];
        mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];
    }

    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum2wsaXZ for averaging filter update rate
    sumCount++;

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
    // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
    // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
    // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
    // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
    // This is ok by aircraft orientation standards!
    // Pass gyro rate as rad/s
    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
    //  MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);


    if (VERBOSE) {
        delt_t = millis() - count;
        if (delt_t > 500) {
            tempCount = readTempData();  // Read the adc values
            temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
            // Print temperature in degrees Centigrade
            Serial.print("Temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C

            count = millis();
            digitalWrite(myLed, !digitalRead(myLed));  // toggle led
        }
    }
    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 100) { // update LCD_ON independent of read rate
        if (VERBOSE) {
            Serial.print("ax = "); Serial.print((int)1000*ax);
            Serial.print(" ay = "); Serial.print((int)1000*ay);
            Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
            Serial.print("gx = "); Serial.print( gx, 2);
            Serial.print(" gy = "); Serial.print( gy, 2);
            Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
            Serial.print("mx = "); Serial.print( (int)mx );
            Serial.print(" my = "); Serial.print( (int)my );
            Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");

            Serial.print("q0 = "); Serial.print(q[0]);
            Serial.print(" qx = "); Serial.print(q[1]);
            Serial.print(" qy = "); Serial.print(q[2]);
            Serial.print(" qz = "); Serial.println(q[3]);
        }

        // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
        // In this coordinate system, the positive z-axis is down toward Earth.
        // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
        // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
        // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
        // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
        // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
        // applied in the correct order which for this configuration is yaw, pitch, and then roll.
        // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
        yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI;
        yaw   -= 12.0;          // Declination at Lancaster, CA
        roll  *= 180.0f / PI;

        if (VERBOSE) {
            Serial.print("Yaw, Pitch, Roll: ");
        }
        // just print these values (even if not in VERBOSE mode) for Serial Plotter purposes
        Serial.print(yaw+180, 2);
        Serial.print(" ");
        Serial.print(pitch, 2);
        Serial.print(" ");
        Serial.println(roll, 2);

        if (LCD_ON) {
            lcd.clear();
            lcd.print("Yaw  Pitch  Roll");
            lcd.setCursor(0,1);
            lcd.print((int)(yaw+180));
            lcd.setCursor(5,1);
            lcd.print((int)pitch);
            lcd.setCursor(12, 1);
            lcd.print((int)roll);
        }

        // display tone whose frequency maps to the yaw
        // tone(9, (int)yaw+500);

        //
        // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and
        // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
        // The filter update rate is determined mostly by the mathematical steps in the respective algorithms,
        // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
        // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
        // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively.
        // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
        // This filter update rate should be fast enough to maintain accurate platform orientation for
        // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
        // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
        // The 3.3 V 8 MHz Pro Mini is doing pretty well!
        //    display.setCursor(0, 40);
        if (VERBOSE) {
            Serial.println("rt: ");
            Serial.println((float) sumCount / sum, 2);
            Serial.println(" Hz");
        }
        //    display.display();

        count = millis();
        sumCount = 0;
        sum = 0;
    }
}
