/*
 * boat.hpp
 * Platform-specific definitions for boat control system
 */

#ifndef BOAT_HPP
#define BOAT_HPP

#include <Arduino.h>

#define USE_BOAT          // Include boat loop
//#define DUAL_MOTOR      // differential steering: left=servo2, right=servoEsc, no rudder

#define USE_MPU           // Comment out to disable MPU6050
#define USE_WS2812        // Comment out to disable WS2812 LEDs (MSP430, MSP432, ESP32)

//#define USE_SERVO_TEST  // Comment out to disable servo sweep test
//#define USE_WS2812_TEST // Comment out to disable WS2812 color cycle test (requires USE_WS2812)

// Platform-specific pin definitions
#if defined(__MSP430__)
    // MSP430 Platform
    #define SERVO1_PIN 40
    #define SERVO2_PIN 39
    #define SERVO3_PIN 38
    #define ESC_PIN 37
    #define MOTOR_SWITCH 33
    #define CALIBRATE_SWITCH 13
    #define RESET_SWITCH 16
    #define LED_PIN 11
    #define INT_PIN 8

#elif defined(__MSP432P401R__) || defined(__MSP432__)
    // MSP432 Platform
    #define SERVO1_PIN 40
    #define SERVO2_PIN 39
    #define SERVO3_PIN 38
    #define ESC_PIN 37
    #define MOTOR_SWITCH 33
    #define CALIBRATE_SWITCH 13
    #define RESET_SWITCH 16
    #define LED_PIN 11
    #define INT_PIN 8

#elif defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    // ESP32 Platform
    #define SERVO1_PIN 33
    #define SERVO2_PIN 25
    #define SERVO3_PIN 26
    #define ESC_PIN 17
    #define MOTOR_SWITCH 18
    #define CALIBRATE_SWITCH 5
    #define LED_PIN 32
    #define INT_PIN 19

#else
    #error "Unsupported platform. Please define MSP430, MSP432, or ESP32."
#endif

// Function declarations
bool motorSwitchPressed();
bool calibrateSwitchPressed();
bool escSwitchPressed();

// WS2812 LED functions (defined in GNOR_V4.ino)
void ws_setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
void ws_show();

#endif // BOAT_HPP
