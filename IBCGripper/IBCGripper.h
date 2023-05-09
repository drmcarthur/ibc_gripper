#ifndef BC_Gripper_h
#define BC_Gripper_h

#include "Arduino.h"

// Motor speed/direction control definitions
#define MOT_FOR          0  // Forward Direction
#define MOT_REV          1  // Reverse Direction
#define MIN_MOT_PWM     40  // Minimum mot command (out of 255) to move motor

// Motor controller defaults
//float K_P = 57;              // Proportional gain
//float K_I = 4560*cont_dt;    // Integral gain (22.8)
//float K_D = 0.1781/cont_dt;  // Derivative gain (35.6)
#define K_P             50     // Proportional gain
#define K_I              8     // Integral gain (22.8)
#define K_D             50     // Derivative gain (35.6)
#define ERR_THRESH     0.5     // Acceptable position error (degrees)

// Conversion definitions
//#define STEP_PER_DEG      5.0  // Steps per degree of output shaft rotation
#define STEP_PER_REV     1788  // 298*6
#define STEP_PER_DEG     4.97  // STEP_PER_REV/360
#define STEP_PER_MM      87.5  // Steps per millimeter of linear travel
#define STEP_PER_SPAN_MM   14  // Steps per millimeter of finger span opening
#define GRIP_OVERLAP     18.5  // millimeters between closed gripper arms

// Gripping Presets
#define MAX_OPEN        1700   // Steps from Max Closed to Max Open
#define HOME_RELIEF     -100   // Steps from Home to Max Closed
#define GRIP_TRIES        50   // # of times to reset grip pos

// Loop rate definitions
#define IBC_CONT_HZ    200  // Control rate

// Pololu TB6612FNG Motor Driver pin defaults
#define MOT_IN1 4           // AIN1
#define MOT_IN2 5           // AIN2
#define MOT_PWM 6           // PWMA

// Pololu Magnetic Encoder pin defaults
#define ENC_A 2             // OUTA
#define ENC_B 3             // OUTB

// Debugging
#define IBC_DEBUG   false   // Display debug statements
#define DEBUG_HZ       10   // Rate of debug display

enum GripperState:byte {
  IDLE,       // No power to motors
  HOME,       // Reverse until hard stop
  GRIP,       // Reverse until setpoint or object, then HOLD
  GRIP_SOFT,  // Reverse until setpiont or object, then IDLE
  HOLD,       // Hold current position
  OPEN,       // Forward until max open or obstruction then HOLD
  OPEN_SOFT  // Forward until max open or obstruction then IDLE
};

class IBCGripper
{
  public:
    // Constructor:
    //    mot_in1 & mot_in2 are motor direction control pins
    //    mot_pwm is pin number for setting motor throttle
    //    enc_a & enc_b are quadrature encoder pins
    IBCGripper(byte mot_in1=MOT_IN1, byte mot_in2=MOT_IN2, \
               byte mot_pwm=MOT_PWM);

    // Primary operation functions
    void initPinModes();    // Initialize microcontroller pins and interrupts
    void update();          // Update the gripper state & update motor control
    void home();            // Find and set fully closed position
    void grip(long sp);     // Close gripper until object or sp then HOLD
    void gripSoft(long sp); // Close gripper until object or sp then IDLE
    void hold();            // Hold gripper at current position
    void open();            // Open gripper to fully open position then HOLD
    void openSoft();        // Open gripper to fully open position then IDLE
    void idle();            // Cut power to motor

    // Getters
    GripperState getState();        // Gripper state
    long getSteps();        // Motor pos sp (steps from home)
    short getMM();          // Plunger pos sp (mm from home)
    short getSpanMM();      // mm sp between fingers)
    bool getDebug();        // Turn Serial debug statements on/off

    // Setters
    void setHome();         // Set home position 
    void setSteps(long step_setpoint);  // Set desired motor position 
    void setMM(short mm_setpoint);      // Set desired gripper position (mm)
    void setDebug(bool on, short hz=DEBUG_HZ); // Turns Serial debug statements on/off

    // Converters
    long mmToSteps(float mm_sp);          // Calc steps to achieve plunger pos in mm
    long spanMMToSteps(float span_mm_sp); // Calc steps to achieve gripper span in mm

    // Interrupt Service Routines for reading encoders
    void ISR_ENCODER_A();
    void ISR_ENCODER_B(); 
    
  private:
    GripperState _state;          // Primary state of gripper
    
    // Pins for motor control and encoder reading
    byte _mot_in1;                // MOT_IN1 pin
    byte _mot_in2;                // MOT_IN2 pin
    byte _mot_pwm;                // MOT_PWM pin
    //byte _enc_a;                  // ENC_A pin
    //byte _enc_b;                  // ENC_B pin

    // Motor state variables
    byte _dir;                    // Forward = MOT_FOR, Reverse = MOT_REV
    byte _pwm;                    // 0 = Off, 255 = Full On
    bool _home_set;               // Flag to indicate if home position has been determined

    // Motor control variables
    volatile long _pos;           // Encoder steps from home: + = opening, - = closing
    long _pos_sp;                 // position setpoint (# of encoder steps)
    long _pos_prev;               // Encoder steps from home: + = opening, - = closing
    short _stop_count;            // # of consecutive ctrl iter w/no motion
    short _grip_count;            // # of times to reset pos setpoint during grip
    unsigned long _control_t;     // Control loop timer
    float _kp;                    // Proportional Gain
    float _ki;                    // Integral Gain
    float _kd;                    // Derivative Gain
    float _i_term;                // Accumulated error
    float _err_thresh;            // Acceptable position error (degrees)
    float _err;                   // Proportional error (degrees)
    float _err_prev;              // Error from previous iteration
    float _err_d;                 // Derivative of error
    float _mot_cmd, _cmd_raw;     // For motor control calculation
    bool _STUCK;                  // Flag indicating motor shaft is stuck

    // Motor control functions
    void doControl();             // Compute motor throttle to reach pos setpoint
    void updateMotor();           // Set the motor direction and speed
    bool checkStuck();            // Check if output shaft is stuck
    void resetControl();          // Reset STUCK flags and control variables
    
    // Debugging tools
    bool _ibc_debug;              // Enable debug statements
    short _debug_hz;              // Debug display rate
    unsigned long _debug_t;       // Track last debug display time

    void printD(String msg);      // Debugging version of Serial.print() function
    void printlnD(String msg);    // Debugging version of Serial.println() function

    unsigned long _home_time;
    unsigned long _stuck_reset_t;      // Timer to allow reset of STUCK state...
};

#endif
