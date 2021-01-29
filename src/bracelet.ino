// ================================================================
// ===                         BRACELET                         ===
// ================================================================

void(* reboot) (void) = 0; //declare reset function at address 0

#include <avr/sleep.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#include "FastLED.h"

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

#define MPU_INTERRUPT_PIN 3  // use pin #3 on Arduino Uno -- to be verified

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

#define WAKEUP_PIN 2 // pint to WakeUp from deepSleep

/// ===== MY DEFINE =====

#define NUM_CYLON   5
#define LED_STRIPE2_PIN 9
#define LED_STRIPE1_PIN 8
#define MOVEMENT_RUB 0
#define MOVEMENT_WASH 1
#define MOVEMENT_RANDOM 10
#define BUTTON_THRESHOLD 800
#define LOOP_DURATION 20000
#define MIN_MOVE 15
#define NUM_LEDS 11
CRGB leds[NUM_LEDS];

/// =======================

//bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion qq;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/// ===== MY VARIABLE =====
int16_t old_ax_value = 0;
int16_t new_ax_value = 0;
int16_t the_distance = 0;
int16_t old_counter = 0;
uint16_t identified_movement = MOVEMENT_RANDOM;
uint16_t counter_movement = 0;
uint16_t counter_no_movement = 0;
uint16_t millis_start = 0;
uint16_t counter = 0;
float threshold = 0;
// boolean button_pushed = false;
/// =======================

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)

  pinMode(WAKEUP_PIN, INPUT_PULLUP);

  // initialize device
  mpu.initialize();

  pinMode(MPU_INTERRUPT_PIN, INPUT);

  // verify connection

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-132);
  mpu.setYAccelOffset(-2104);
  mpu.setZAccelOffset(1056);
  mpu.setXGyroOffset(44);
  mpu.setYGyroOffset(-62);
  mpu.setZGyroOffset(-7);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    // configure LED for output
    FastLED.addLeds<WS2811, LED_STRIPE1_PIN, GRB>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.addLeds<WS2811, LED_STRIPE2_PIN, GRB>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );


  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    reboot();  // Reboot to retry initialization
  }

  millis_start = millis();
  counter_movement = 0;
  counter_no_movement = 0;

}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) {
    //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
    // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  }  else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {   // check for overflow (this should never happen unless our code is too inefficient)
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask

  } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) { // otherwise, check for DMP data ready interrupt (this should happen frequently)

    // read a packet from FIFO
    while (fifoCount >= packetSize) { // Lets catch up to NOW, someone is using the dreaded delay()!
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
    }

    counter++;
    mpu.dmpGetQuaternion(&qq, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &qq);
    mpu.dmpGetAccel(&aa, fifoBuffer);    //accel
    mpu.dmpGetYawPitchRoll(ypr, &qq, &gravity);    //gyro
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);  //accel


    old_ax_value = new_ax_value;
    new_ax_value = aaReal.x;

    uint16_t time_delta = millis() - millis_start;

    if (old_ax_value < threshold && new_ax_value > threshold) {

      the_distance = counter - old_counter;
      old_counter = counter;

      // basing on distance define movement
      if (the_distance > 10 && the_distance < 45) {
        // sfrega
        identified_movement = MOVEMENT_RUB;
        counter_movement++;
      } else if (the_distance > 45 && the_distance < 180) {
        // ruota
        identified_movement = MOVEMENT_WASH;
        counter_movement++;
      } else {
        // Fai_qualcosa
        identified_movement = MOVEMENT_RANDOM;
        counter_no_movement++;
      }
    } else {
    }

    //    int dot = (int)(time_delta / ((LOOP_DURATION + 100) / NUM_LEDS));
    int dot = (int)(NUM_LEDS *  (float) time_delta / (LOOP_DURATION + 100));

    leds[NUM_LEDS - dot - 1] = CRGB(53 / 3, 250 / 3, 60 / 3);
    FastLED.show();

    if (time_delta > LOOP_DURATION) {
      for (int c = 0; c < NUM_CYLON; c++) {
        CylonBounce(53, 250, 60, 2, 50, 50);
      }
      FastLED.clear();
      FastLED.show();

      float percentage_movement = ((float)counter_movement / ((float)counter_movement + (float)counter_no_movement));


      if (percentage_movement > 0.7 && counter_movement > MIN_MOVE) {
        FadeInOut(0, 0x77, 0);
        FastLED.clear();
        FastLED.show();
      } else {
      }

      counter_movement = 0;
      counter_no_movement = 0;

      //Go to sleep now
      goToSleep();

    } else {
    }

  }
}



// ================================================================
// ===                       FUNCTIONS                          ===
// ================================================================

void wakeUpNow() {

  sleep_disable();
  // Detach WAKEUP interrupt
  detachInterrupt(digitalPinToInterrupt(WAKEUP_PIN));

  reboot();

}

void goToSleep() {

  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  // Detach MPU interrupt && attach WAKEUP interrupt
  detachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN));
  attachInterrupt(digitalPinToInterrupt(WAKEUP_PIN), wakeUpNow, LOW);

  sleep_mode();

}


void FadeInOut(byte red, byte green, byte blue) {
  float r, g, b;

  for (int k = 0; k < 256; k = k + 1) {
    r = (k / 256.0) * red;
    g = (k / 256.0) * green;
    b = (k / 256.0) * blue;
    setAll(r, g, b);
    showStrip();
  }

  for (int k = 255; k >= 0; k = k - 2) {
    r = (k / 256.0) * red;
    g = (k / 256.0) * green;
    b = (k / 256.0) * blue;
    setAll(r, g, b);
    showStrip();
  }
}

void CylonBounce(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay) {

  for (int i = 0; i < NUM_LEDS - EyeSize - 2; i++) {
    setAll(52 / 5, 250 / 5, 60 / 5);
    setPixel(i, red / 10, green / 10, blue / 10);

    for (int j = 1; j <= EyeSize; j++) {

      setPixel(i + j, red, green, blue);
    }
    setPixel(i + EyeSize + 1, red / 10, green / 10, blue / 10);
    showStrip();
    delay(SpeedDelay);
  }

  delay(ReturnDelay);

  for (int i = NUM_LEDS - EyeSize - 2; i > 0; i--) {
    setAll(52 / 5, 250 / 5, 60 / 5);
    setPixel(i, red / 10, green / 10, blue / 10);

    for (int j = 1; j <= EyeSize; j++) {

      setPixel(i + j, red, green, blue);
    }
    setPixel(i + EyeSize + 1, red / 10, green / 10, blue / 10);
    showStrip();
    delay(SpeedDelay);
  }

  delay(ReturnDelay);
}

void showStrip() {
#ifdef ADAFRUIT_NEOPIXEL_H
  // NeoPixel
  strip.show();
#endif
#ifndef ADAFRUIT_NEOPIXEL_H
  // FastLED
  FastLED.show();
#endif
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
#ifdef ADAFRUIT_NEOPIXEL_H
  // NeoPixel
  strip.setPixelColor(Pixel, strip.Color(red, green, blue));
#endif
#ifndef ADAFRUIT_NEOPIXEL_H
  // FastLED
  leds[Pixel].r = red;
  leds[Pixel].g = green;
  leds[Pixel].b = blue;
#endif
}

void setAll(byte red, byte green, byte blue) {
  for (int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue);
  }
  showStrip();
}
