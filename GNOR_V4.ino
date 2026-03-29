#include "GNOR_V4.h"

/*---Servo library and objects (shared by USE_BOAT and USE_SERVO_TEST)---*/
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  #include <ESP32Servo.h>
#else
  #include <Servo.h>
#endif
Servo servo1;     // Rudder
Servo servo2;     // Left motor (dual motor config)
Servo servo3;     // Auxiliary servo
Servo servoEsc;   // Single motor OR right motor (dual motor config)

#ifdef USE_WS2812
#define WS2812_COUNT 3

/*---Platform-specific WS2812 library---*/
#if defined(__MSP430__)
  #include <WS2812_MSP430.h>
  WS2812_MSP430 _strip(WS2812_COUNT);

#elif defined(__MSP432P401R__) || defined(__MSP432__)
  #include <WS2812_MSP432.h>
  WS2812_MSP432 _strip(WS2812_COUNT);

#elif defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  #include <NeoPixelBus.h>
  NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> _strip(WS2812_COUNT, LED_PIN);

#else
  #error "USE_WS2812 requires MSP430, MSP432, or ESP32"
#endif

/*---WS2812 wrapper (uniform API across all platforms)---*/
inline void ws_begin() {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  _strip.Begin();
#else
  _strip.begin();
#endif
}
void ws_show() {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  _strip.Show();
#else
  _strip.show();
#endif
}
inline void ws_clear() {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  _strip.ClearTo(RgbColor(0));
#else
  _strip.clear();
#endif
}
void ws_setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  _strip.SetPixelColor(n, RgbColor(r, g, b));
#else
  _strip.setPixelColor(n, r, g, b);
#endif
}
inline void ws_fill(uint8_t r, uint8_t g, uint8_t b) {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  _strip.ClearTo(RgbColor(r, g, b));
#else
  _strip.fill(r, g, b);
#endif
}
#endif // USE_WS2812

#ifdef USE_WS2812_TEST
/*---WS2812 Color Cycle Test Variables---*/
struct RGB { uint8_t r, g, b; };
const RGB COLORS[] = {
  {50,   0,   0},  // Red
  {50, 25,   0},  // Orange
  {50, 50,   0},  // Yellow
  {  0, 50,   0},  // Green
  {  0,   0, 50},  // Blue
  {25,   0, 50},  // Purple
  {50, 50, 50},  // White
};
const int COLOR_COUNT = sizeof(COLORS) / sizeof(COLORS[0]);

int wsColorIndex = 0;
unsigned long wsLastChange = 0;
#define WS2812_INTERVAL_MS 500  // ms per color
#endif // USE_WS2812_TEST

#ifdef USE_SERVO_TEST
/*---Servo Test Variables---*/
int servoPos = 0;
bool sweepUp = true;
#endif // USE_SERVO_TEST

#ifdef USE_BOAT
void boatLoop(unsigned long timestamp, double heading);
#endif // USE_BOAT

#ifdef USE_MPU
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050_6Axis_MotionApps612.h" // Uncomment this library to work with DMP 6.12 and comment on the above library.

double yaw = 1.0;
/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.
int const INTERRUPT_PIN = INT_PIN;  // Interrupt pin

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}
#endif // USE_MPU

void setup() {
  Serial.begin(115200);
  while (!Serial);

#ifdef USE_MPU
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    #if defined(__MSP430__)
    delay(100); // extra settle time for MSP430 USCI_B I2C before DMP init
    //Wire.setClock(400000);
    #elif !defined(__MSP432P401R__) && !defined(__MSP432__)
    //Wire.setClock(400000); // 400kHz I2C clock
    #endif
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  delay(200);   // Allow MPU6050 I2C bus to fully settle after power-on or reset
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

  /* Initialize and configure the DMP — retry up to 3 times */
  Serial.println(F("Initializing DMP..."));
  int dmpTries = 0;
  do {
    if (dmpTries > 0) {
      Serial.print(F("DMP init failed (code "));
      Serial.print(devStatus);
      Serial.print(F("), retrying (attempt "));
      Serial.print(dmpTries + 1);
      Serial.println(F(")..."));
      delay(200);
    }
    devStatus = mpu.dmpInitialize();
  } while (devStatus != 0 && ++dmpTries < 3);

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  }
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
#endif // USE_MPU

#ifdef USE_WS2812
  ws_begin();
  ws_clear();
  Serial.println(F("WS2812 enabled."));
#endif // USE_WS2812

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
servo1.setPeriodHertz(50);
servo2.setPeriodHertz(50);
servo3.setPeriodHertz(50);
servoEsc.setPeriodHertz(50);
#endif
servo1.attach(SERVO1_PIN);
servo2.attach(SERVO2_PIN);
servo3.attach(SERVO3_PIN);
servoEsc.attach(ESC_PIN);
servo1.write(90);

#ifdef USE_SERVO_TEST
  Serial.println(F("Servo test enabled."));
#endif // USE_SERVO_TEST
}

//
// MAIN LOOP
//
void loop() {
#ifdef USE_MPU
  if (!DMPReady) return; // Stop the program if DMP initialization failed.

  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yaw = ypr[0] * 180/M_PI;
    unsigned long timestamp = millis();

    //Serial.print("yaw: ");
    //Serial.println(yaw);

#ifdef USE_BOAT
    boatLoop(timestamp, yaw);
#endif // USE_BOAT
  }
#endif // USE_MPU

#ifdef USE_WS2812_TEST
  /* Cycle all three LEDs through colors at WS2812_INTERVAL_MS per step */
  unsigned long now = millis();
  if (now - wsLastChange >= WS2812_INTERVAL_MS) {
    wsLastChange = now;
    RGB c = COLORS[wsColorIndex];
    wsColorIndex = (wsColorIndex + 1) % COLOR_COUNT;
    ws_setPixelColor(2, c.r, c.g, c.b);
    c = COLORS[(int)yaw % COLOR_COUNT];
    ws_setPixelColor(1, c.r, c.g, c.b);
    /* Test switches while we are here */
    bool motorPressed     = (digitalRead(MOTOR_SWITCH)     == LOW);
    bool calibratePressed = (digitalRead(CALIBRATE_SWITCH) == LOW);
    if (motorPressed) 
      ws_setPixelColor(0, 50, 0, 0);
    else if (calibratePressed) 
      ws_setPixelColor(0, 0, 50, 0);
    else if (motorPressed && calibratePressed)
      ws_setPixelColor(0, 50, 50, 0);
    else
      ws_setPixelColor(0, 50, 50, 50);
    ws_show();
    
  }
#endif // USE_WS2812_TEST

#ifdef USE_SERVO_TEST
  /* Sweep all servos from low (20°) to high (160°) and back */
  servo1.write(servoPos);
  servo2.write(servoPos);
  servo3.write(servoPos);
  servoEsc.write(servoPos);

  Serial.print(F("Servo pos: "));
  Serial.println(servoPos);

  if (sweepUp) {
    servoPos++;
    if (servoPos >= 160) sweepUp = false;
  } else {
    servoPos--;
    if (servoPos <= 20) sweepUp = true;
  }
  delay(15);
#endif // USE_SERVO_TEST
}
