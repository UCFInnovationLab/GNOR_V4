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

#define P 2.0
#define MOTOR_BASE_SPEED 90

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

bool escSwitchPressed() {
    return digitalRead(MOTOR_SWITCH) == 0;
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
            servoEsc.write(180);
        }

        while (calibrateSwitchPressed() == 1)  {} // loop with pressed waiting for ESC beeps

        servoEsc.write(0);

        calibrate_time = false;
    }

    // Run one time initialization routines.
    if (first_time) {
        servo1.write(90);    // rudder straight
        servoEsc.write(0);   // motor off
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

    if (motor_switch_now && !motor_switch_last) {
        servo1.write(90);    // rudder straight
        servoEsc.write(0);   // motor off
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
        Serial.print(", ");
        Serial.print(heading_rate,3);
        Serial.print(", ");
        Serial.println(started);
        last_time = timestamp;
    }

    // if the heading rate is less than some constant then turn on the Green LED
    if (fabs(heading_rate) < .005) {
        ws_setPixelColor(0, 0, 1, 0);
    } else {
        ws_setPixelColor(0, 1, 0, 0);
    }
    
    // check for boat start.  (currently rotate boat 90 degrees
    if (((calculateDifferenceBetweenAngles(heading, 90)) > 0.0) && (started==0)) {
        started = 1;
        start_time = timestamp;
        Serial.println("Started");

        //
        // Start the boat's motor(s)
        //

        //--------------------------------------------------
        // Single motor + rudder
        // servo1: Rudder, servoEsc: motor
        //--------------------------------------------------
        servoEsc.write(MOTOR_BASE_SPEED);          // start motor. range 0-180
        //--------------------------------------------------
        // Dual motor
        // servo2: left motor, servoEsc: right motor
        //--------------------------------------------------
            // servo2.write(MOTOR_BASE_SPEED);    // start left motor at base speed. range 0-180
            // servoEsc.write(MOTOR_BASE_SPEED);  // start right motor at base speed. range 0-180
        //--------------------------------------------------
        // Dual brush motors
        //--------------------------------------------------
            // set_pwm_duty_cycle_1(0.5);
            // set_pwm_duty_cycle_2(0.5);
    }

    // handle orange "running LED"
    // blinking: pre-start (started==0), solid: running (started==1)
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


        //--------------------------------------------------
        // Single motor + rudder
        // servo1: Rudder, servoEsc: motor
        //--------------------------------------------------
        rudder = P * error;
        // set limits on rudder movement
        if (rudder > 90) rudder  = 90;
        if (rudder < -90) rudder = -90;
        // set servo angles in response to PID
        servo1.write(90 + rudder);                      // if rudder is reversed, change + to -
        //--------------------------------------------------
        // Dual motor
        // servo2: left motor, servoEsc: right motor
        //--------------------------------------------------
            // diff = P * error;
            // servo2.write(MOTOR_BASE_SPEED + diff);
            // servoEsc.write(MOTOR_BASE_SPEED + diff);
        //--------------------------------------------------
        // Dual brush motors
        //--------------------------------------------------
            //set_pwm_duty_cycle_1(diff);
            //set_pwm_duty_cycle_2(diff);
    }

    // Turn on LED if heading +- 5 degrees of the target
    if (fabs(calculateDifferenceBetweenAngles(heading, target)) < 2.0) {
        ws_setPixelColor(2, 0, 10, 0);
    } else {
        ws_setPixelColor(2, 10, 0, 0);
    }

    // Update LEDs every 50ms
    static unsigned long last_led_time = 0;
    if ((timestamp - last_led_time) >= 50) {
        ws_show();
        last_led_time = timestamp;
    }

}



