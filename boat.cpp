/*
 * boat.cpp
 *
 * Servo assignments:
 *   servo1   — Rudder
 *   servo2   — Left motor  (dual motor config)
 *   servoEsc — Single motor (single motor config) OR right motor (dual motor config)
 *   servo3   — Auxiliary servo
 */

#include "GNOR_V4.h"
#include <Arduino.h>

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  #include <ESP32Servo.h>
#else
  #include <Servo.h>
#endif

extern Servo servo1;    // Rudder                        — declared in GNOR_V4.ino
extern Servo servo2;    // Left motor (dual motor)
extern Servo servo3;    // Auxiliary servo
extern Servo servoEsc;  // Single motor OR right motor (dual motor)

unsigned long last_time = 0;     // last time through the loop

#define LED2_DEAD_BAND      2.0f    // degrees — within this shows bright on-target green
#define LED2_MAX_ERROR      90.0f   // degrees — clamp for color gradient (positive=red, negative=blue)
#define LED2_MAX_BRIGHTNESS 128     // max channel brightness in the gradient zone (0–255)
#define LED2_ON_BRIGHTNESS  255     // green brightness when within LED2_DEAD_BAND

#define P 2.0                       // Proportional constant used by the rudder or for each dual motor
#define MOTOR_BASE_SPEED 0.5        // Default speed the single or dual motor(s) use (0.0-1.0)
#define THROTTLE_HIGH_DEGREES 145   // High throttle setting in servo degrees
#define THROTTLE_LOW_DEGREES 35     // Low throttle setting in servo degrees
#define MAX_RUDDER_DEGREES 90/2     // Max angle the rudder moves on each side of zreo (90). Normally 45 Degrees.

// Waypoint Array
//
struct Waypoint {
    unsigned long time_ms;  // elapsed mission time to activate this heading
    int heading360;         // compass heading in degrees (0–360)
};

static const Waypoint waypoints[] = {
    {     0,   0 },   // 0–10s:  straight ahead
    { 10000, 270 },   // 10–20s: turn to 270
    { 20000, 180 },   // 20s+:   turn to 180
};
static const int WAYPOINT_COUNT = sizeof(waypoints) / sizeof(waypoints[0]);

/*
 * setMotor1Speed
 * --------------
 * Set the ESC motor speed. speed is in the range 0.0 (off) to 1.0 (full).
 */
void setMotor1Speed(double speed) {
    servoEsc.write((int)(THROTTLE_LOW_DEGREES + (speed * (THROTTLE_HIGH_DEGREES - THROTTLE_LOW_DEGREES))));
}

/*
 * setMotor2Speed
 * --------------
 * Set the left motor speed (dual motor config). speed is in the range 0.0 (off) to 1.0 (full).
 */
void setMotor2Speed(double speed) {
    servoEsc.write((int)(THROTTLE_LOW_DEGREES + (speed * (THROTTLE_HIGH_DEGREES - THROTTLE_LOW_DEGREES))));
}

/*
 * Button/Switch functions
 * Buttons are active low (pressed = 0)
 */
bool motorSwitchPressed() {
    return digitalRead(MOTOR_SWITCH) == 0;
}

bool calibrateSwitchPressed() {
    return digitalRead(CALIBRATE_SWITCH) == 0;
}

/*
 * calculateDifferenceBetweenAngles
 * ---------------------------------
 * Return the signed difference between two angles in the -180 to +180 system.
 * Result is in the range (-180, +180].
 */
double calculateDifferenceBetweenAngles(double angle1, double angle2) {
    double delta = angle1 - angle2;
    if (delta >  180.0) delta -= 360.0;
    if (delta < -180.0) delta += 360.0;
    return delta;
}

/*
 * wrapTo180
 * ---------
 * Wrap any angle (degrees) into the range (-180, +180].
 */
double wrapTo180(double angle) {
    angle = fmod(angle, 360.0);
    if (angle >  180.0) angle -= 360.0;
    if (angle < -180.0) angle += 360.0;
    return angle;
}



/*
 * boatLoop
 * ----------------------------
 * This routine is called in the main loop at a rate of ~100 times/sec.
 * The current timestamp in milliseconds and the current heading (-180 to +180) is passed in.
 * Note: must use static variables if you need a persistence between calls.
 */
void boatLoop(unsigned long timestamp, double heading) {

	// Static variables.  Keep their values between calls to "boatLoop"
    static double last_heading=0.0;			// used to calculate the delta heading
    static unsigned long start_time;		// actual time the boat started.  Used as an offset to calculate elapsed time
    static int started=-1;					// has the boat started, -1=not ready, 0=ready, 1=started
    static double heading_zero_offset;		// heading offset.  Used to zero heading when button is pressed
    static boolean first_time=true;         // flag to run one time routines
    static boolean calibrate_time=true;     // Should we calibrate
    unsigned long running_time;				// elapsed time since the mission started
    static boolean motors_armed = false;         // is motor armed
    static boolean motor_switch_last = false;    // previous state of motor switch (for edge detection)
    static boolean motor_switch_init = false;    // has motor switch state been seeded
    static int waypoint_index = 0;               // current waypoint index
    
    int target360 = 0;                          // target heading in compass degrees (0-360)
    double target = 0.0;                        // target heading in -180 to +180
    double error = 0.0;                         // error between current heading and target heading
    int rudder = 0;                             // calculated rudder angle
    double diff = 0.0;                          // diff for dual motor drive
    
    double heading_rate;			            // Calculated delta between current and last reading of heading

    //--------------------------------------------------------------------------------
    // pre-start
    //--------------------------------------------------------------------------------

    // Calibrate ESC with max and min pulse widths
    if (calibrate_time) {
        if (calibrateSwitchPressed() == 1) {
            setMotor1Speed(1.0);
        }

        while (calibrateSwitchPressed() == 1)  {} // loop with pressed waiting for ESC beeps

        setMotor1Speed(0.0);

        calibrate_time = false;
    }

    // Run one time initialization routines.
    if (first_time) {
        Serial.println("************* BOAT LOOP STARTED *************");
        servo1.write(90);    // rudder straight
        setMotor1Speed(0.0);   // motor off
#ifdef DUAL_MOTOR
        setMotor2Speed(0.0);
#endif
        first_time = false;
        motors_armed = false;
    }

    // Detect rising edge on motor switch (not pressed -> pressed).
    // Seed last state on first call so a switch held at startup is ignored.
    boolean motor_switch_now = motorSwitchPressed();
    if (!motor_switch_init) {
        motor_switch_last = motor_switch_now;
        motor_switch_init = true;
    }

    // was switch just pressed (rising edge)
    if (!motor_switch_last && motor_switch_now) {
        servo1.write(90);    // rudder straight
        setMotor1Speed(0.0);   // motor off
#ifdef DUAL_MOTOR
        setMotor2Speed(0.0);
#endif
        heading_zero_offset = heading;
        started = 0;
        waypoint_index = 0;
        motors_armed = true;
    } else if (motor_switch_now) {
        motors_armed = true;
    } else {
        motors_armed = false;
    }
        
    motor_switch_last = motor_switch_now;

    // Apply heading_zero offset and wrap into -180 to +180
    heading = calculateDifferenceBetweenAngles(heading, heading_zero_offset);

    // calculate heading turn rate.  this can be used to give a measure of how fast the boat's heading is drifting when the
    // boat is still.  If good, stable rate should be less than .005.
    heading_rate = calculateDifferenceBetweenAngles(heading, last_heading);
    last_heading = heading;

    // print heading every .5 seconds
    if ((timestamp - last_time) > 500) {
        Serial.print("Heading: ");
        Serial.print(heading);
        Serial.print(", Rate: ");
        Serial.print(heading_rate,3);
        Serial.print(", Started: ");
        Serial.println(started);
        last_time = timestamp;
    }

    // if the heading rate is less than some constant then turn on the Green LED
#ifdef USE_WS2812
    if (fabs(heading_rate) < .005) {
        ws_setPixelColor(0, 0, 10, 0);
    } else {
        ws_setPixelColor(0, 10, 0, 0);
    }
#endif
    
    // check for boat start.  (currently rotate boat 90 degrees
    if (((calculateDifferenceBetweenAngles(heading, 90)) > 0.0) && (started==0)) {
        started = 1;
        start_time = timestamp;
        Serial.println("************* Started *************");
    }

    // handle orange "running LED"
    // blinking: pre-start (started==0), solid: running (started==1)
#ifdef USE_WS2812
    {
        static unsigned long last_blink_time = 0;
        static bool blink_state = false;

        if (started == 1) {
            ws_setPixelColor(1, 20, 8, 0);          // solid orange
        } else if (started == 0) {
            if ((timestamp - last_blink_time) >= 500) {
                blink_state = !blink_state;
                last_blink_time = timestamp;
            }
            ws_setPixelColor(1, blink_state ? 20 : 0, blink_state ? 8 : 0, 0);
        } else {
            ws_setPixelColor(1, 0, 0, 0);           // off when not ready
        }
    }
#endif

    //--------------------------------------------------------------------------------
    // Main routine that runs after boat has started
    //--------------------------------------------------------------------------------
    if (started==1) {
        running_time = timestamp - start_time;			// calculate elapsed time
        
        // Advance through waypoints as elapsed time passes
        while (waypoint_index + 1 < WAYPOINT_COUNT &&
               running_time >= waypoints[waypoint_index + 1].time_ms) {
            waypoint_index++;
        }
        target360 = waypoints[waypoint_index].heading360;

        // convert compass target to -180 to +180
        target = (target360 > 180) ? target360 - 360 : target360;
        
        // PID routine
        // bigger P causes boat to have more reaction to heading errors
        error = calculateDifferenceBetweenAngles(heading, target);


#ifdef DUAL_MOTOR
        //--------------------------------------------------
        // Dual motor differential steering
        // servo2: left motor, servoEsc: right motor, no rudder
        // NOTE: remember to detach the red power wire from the servo2 ESC
        //--------------------------------------------------
        diff = (P * error) / 180.0;      // scale error to motor-speed units
        if (diff >  MOTOR_BASE_SPEED) diff =  MOTOR_BASE_SPEED;
        if (diff < -MOTOR_BASE_SPEED) diff = -MOTOR_BASE_SPEED;
        if (motors_armed) {
            setMotor2Speed(MOTOR_BASE_SPEED - diff);   // left
            setMotor1Speed(MOTOR_BASE_SPEED + diff);   // right
        } else {
            setMotor2Speed(0.0);
            setMotor1Speed(0.0);
        }
#else
        //--------------------------------------------------
        // Single motor + rudder
        // servo1: Rudder, servoEsc: motor
        //--------------------------------------------------
        rudder = P * error;
        if (rudder >  MAX_RUDDER_DEGREES) rudder =  MAX_RUDDER_DEGREES;
        if (rudder < -MAX_RUDDER_DEGREES) rudder = -MAX_RUDDER_DEGREES;
        servo1.write(90 + rudder);
        if (motors_armed) {
            setMotor1Speed(MOTOR_BASE_SPEED);
        } else {
            setMotor1Speed(0.0);
        }
#endif
    }

    // Pixel 2: green when on target, red brightens for positive error, blue brightens for negative
#ifdef USE_WS2812
    {
        double led2_err = calculateDifferenceBetweenAngles(heading, target);
        uint8_t r = 0, g = 0, b = 0;
        if (fabs(led2_err) < LED2_DEAD_BAND) {
            g = LED2_ON_BRIGHTNESS;
        } else {
            double abs_err = fabs(led2_err);
            if (abs_err > LED2_MAX_ERROR) abs_err = LED2_MAX_ERROR;
            float t = (float)((abs_err - LED2_DEAD_BAND) / (LED2_MAX_ERROR - LED2_DEAD_BAND));
            if (led2_err > 0.0) {
                r = (uint8_t)(t * LED2_MAX_BRIGHTNESS);  // positive error: red brightens
            } else {
                b = (uint8_t)(t * LED2_MAX_BRIGHTNESS);  // negative error: blue brightens
            }
        }
        ws_setPixelColor(2, r, g, b);
    }

    // Update LEDs every 50ms
    static unsigned long last_led_time = 0;
    if ((timestamp - last_led_time) >= 50) {
        ws_show();
        last_led_time = timestamp;
    }
#endif

}



