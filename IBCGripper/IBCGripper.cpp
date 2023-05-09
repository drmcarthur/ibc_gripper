/*
  IBC Gripper Class
  
  Class to operate the I-BoomCopter's compliant gripper mechanism.
  Provides functions for homing, opening and closing the gripper.
  Provides access to the absolute position of the gripper (rotary and linear).

  NOTE: Gripper Control pins (for reference):
    Pololu Magnetic Encoder pins:
      OUTA = 2             
      OUTB = 3            
    Pololu TB6612FNG Motor Driver pins:
      AIN1 = 4
      AIN2 = 5
      PWMA = 6
      
  Created 13 Oct 2017
  by Daniel McArthur

 */
#include "IBCGripper.h"

// Constructor:
    //    mot_in1 & mot_in2 are motor direction control pins
    //    mot_pwm is pin number for setting motor throttle
    //    enc_a & enc_b are quadrature encoder pins
IBCGripper::IBCGripper(byte mot_in1=MOT_IN1, byte mot_in2=MOT_IN2,
                       byte mot_pwm=MOT_PWM):
            _mot_in1(mot_in1), _mot_in2(mot_in2), _mot_pwm(mot_pwm), 
            _home_set(false),
            _kp(K_P), _ki(K_I), _kd(K_D), _err_thresh(ERR_THRESH),
            _state(IDLE), _ibc_debug(IBC_DEBUG), _debug_hz(DEBUG_HZ)
{ 
}

// Initialize microcontroller pins
void IBCGripper::initPinModes() {
  // Set up motor and encoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(_mot_pwm, OUTPUT);
  pinMode(_mot_in1, OUTPUT);
  pinMode(_mot_in2, OUTPUT);
}

void IBCGripper::update() {
  
  if(micros() - _control_t > 1000000/IBC_CONT_HZ) {
    if(_state != IDLE) {
      doControl();         // Calc desired motor setting (dir & throttle)
      updateMotor();       // Send command to motor
      if( checkStuck() ) { // Check if output shaft is stuck
        switch(_state) {
          case HOME:
            printlnD(String("Home found!"));
            setHome();          
            break;
          case GRIP:
            _grip_count += 1;
            setSteps(_pos);    // Setpoint to current position
            if(_grip_count > GRIP_TRIES) {
              _grip_count = 0;
              hold();
            }
            break;
          case GRIP_SOFT:
            idle();
            break;
          case HOLD:        // Keep powering motor to hold pos
            break;
          case OPEN:        
            hold();
            break;
          case OPEN_SOFT:        
            idle();
            break;
          default:
            break;   
        }
      }
    } 
    else {            
      _pwm = 0;            // Don't command motor in Idle state
      updateMotor();       // Send command to motor
      if(_state == HOME) {
        // Limit maximum home time to 5 seconds
        if(micros() - _home_time > 5000000) {
          printlnD("Homing failed (home not found after 10 seconds)!");
          idle();
          _home_time = 0;  // Stop homing timer
        }
      }
    }
  }
}

// Calculates motor command to reach target position
// Output: Sets _dir & _pwm for motor direction and speed
void IBCGripper::doControl() {
  // Calculate Errors
  float dt = (micros() - _control_t) / 1000.0;
  _err = 1.0*(_pos_sp - _pos)/STEP_PER_DEG;       // Error in degrees
  _err_d = 1.0*(_err - _err_prev);                // Change in Error
    
  // Calculate motor control command
  _i_term += _ki*_err;
  if(_i_term > 100) {
    _i_term = 100;     // Limit integral error to max
  } else if(_i_term < -100) {
    _i_term = -100;
  } else if(abs(_err) < _err_thresh) {
    _i_term = 0;
  }
  
  _cmd_raw = _kp*_err + _i_term + _kd*_err_d;  // 0 - 100 % speed
  //cmd_raw = cmd_raw*15;   // Make it so an error of 1 degree gives a 15 % motor speed command
  if(_cmd_raw >= 0) {
    _dir = MOT_FOR;
  } else {
    _dir = MOT_REV;
    _cmd_raw = -1*_cmd_raw;
  }
  if(_cmd_raw > 100) {
    _cmd_raw = 100;    // Cap command at 100 % speed
  } else if(abs(_err) < _err_thresh) { //_cmd_raw < 5) {
    _cmd_raw = 0;   
  }

  // Convert motor command range from 0-100% to 0-255 / 255 PWM duty cycle
  _mot_cmd = map(_cmd_raw, 0, 100, 0, 255);

  if(_mot_cmd < MIN_MOT_PWM) {
    _mot_cmd = 0;          // Zero out the motor command if it is sufficiently small
  }

  if(micros() - _debug_t > (1000000/_debug_hz)) {
    printD(String("p, err, err_d: ")); 
    printD(String(_pos)); printD(", ");
    printD(String(_err)); printD(", ");
    printlnD(String(_err_d));
    printD("cmd_raw, mot_cmd: "); printD(String(_cmd_raw)); printD(", ");
    printlnD(String(_mot_cmd)); printlnD("");
    _debug_t = micros();
  }

  _pwm = (byte) _mot_cmd;    // Set desired motor speed

  _err_prev = _err;  // Update previous position
  _control_t = micros();
}

void IBCGripper::updateMotor() {
  // Set motor direction
  if(_dir == MOT_FOR) {
    digitalWrite(_mot_in1, LOW);
    digitalWrite(_mot_in2, HIGH);
  } else {
    digitalWrite(_mot_in1, HIGH);
    digitalWrite(_mot_in2, LOW);
  }
  // Set motor speed
  analogWrite(_mot_pwm, _pwm);
}

bool IBCGripper::checkStuck() {
  if(_pwm > 0) {            // Motor should be in motion...
    if(_pos_prev != _pos) {
      _pos_prev = _pos;     // Reset _pos_prev to current pos
    } 
    else {                  // _pos hasn't changed... might be STUCK
      if(!_STUCK) {
        _stop_count += 1;
        if(_stop_count > 20 ) {  // _pos hasn't changed multiple times... STUCK
          //printlnD(String("Motor STUCK!"));
          //printlnD(String(_pwm));
          //printD(String("Err: ")); printlnD(String(_err));
          _STUCK = true;
          _stuck_reset_t = micros();
          _stop_count = 0;
        }
      }
    }
  }
  return _STUCK;
}

// Clear STUCK flag and counter and reset controller integral term
void IBCGripper::resetControl() {
  _STUCK = false;
  _pos_prev = _pos;
  _stop_count = 0;
  _i_term = 0;
}

// Debugging version of Serial.print() function
void IBCGripper::printD(String msg) {
  if(_ibc_debug) {
    Serial.print(msg);
  }
}

// Debugging version of Serial.println() function
void IBCGripper::printlnD(String msg) {
  if(_ibc_debug) {
    Serial.println(msg);
  }
}

static void IBCGripper::ISR_ENCODER_A () {
  if(digitalRead(ENC_B) == HIGH ) {
    _pos += 1;
  } else {
    _pos -= 1;
  }
}

static void IBCGripper::ISR_ENCODER_B () {
  if(digitalRead(ENC_A) == LOW ) {
    _pos += 1;
  } else {
    _pos -= 1;
  }
}

void IBCGripper::home() {
  _state = HOME;
  setSteps(-10000);     // Setpoint to -inf for continuous reverse
  _home_time = micros();
}

void IBCGripper::setHome() {
  _pos = HOME_RELIEF; // Set the home position
  gripSoft(0);        // Move to the home pos, then IDLE
  _home_time = 0;     // Note the time of home setting
}

void IBCGripper::grip(long sp) {
  _err_thresh = ERR_THRESH;
  _state = GRIP;
  _grip_count = 0;
  setSteps(sp);        // Setpoint to home for continuous reverse
}

void IBCGripper::gripSoft(long sp) {
  _err_thresh = ERR_THRESH;
  _state = GRIP_SOFT;
  setSteps(sp);        // Setpoint to home for continuous reverse
}

void IBCGripper::hold() {
  _state = HOLD;
  //_pwm = 0;
  //updateMotor();
  //delay(500);         // Wait for motor to settle at current position
  //_err_thresh = 10;   // Increase error threshold for holding
  setSteps(_pos);     // Setpoint to current position
}

void IBCGripper::open() {
  _err_thresh = ERR_THRESH;
  _state = OPEN;
  setSteps(MAX_OPEN); // Setpoint to full open position
}

void IBCGripper::openSoft() {
  _err_thresh = ERR_THRESH;
  _state = OPEN_SOFT;
  setSteps(MAX_OPEN); // Setpoint to full open position
}

void IBCGripper::idle() {
  _err_thresh = ERR_THRESH;
  _state = IDLE;
  setSteps(_pos);     // Setpoint to current position
}

GripperState IBCGripper::getState() { return _state; }  
long IBCGripper::getSteps(){ return _pos; }
bool IBCGripper::getDebug() { return _ibc_debug; }
short IBCGripper::getMM() { return _pos/STEP_PER_MM; }          
short IBCGripper::getSpanMM() { return _pos/STEP_PER_SPAN_MM; }

void IBCGripper::setSteps(long sp) {
  resetControl();
  _pos_sp = sp;
}

void IBCGripper::setMM(short sp_mm) {
  resetControl();
  _pos_sp = sp_mm*STEP_PER_MM;
}

long IBCGripper::mmToSteps(float pos_mm) {
  return (long) pos_mm*STEP_PER_MM;
}

long IBCGripper::spanMMToSteps(float span_mm) {
  return (long) (span_mm + GRIP_OVERLAP)*STEP_PER_SPAN_MM;
}

void IBCGripper::setDebug(bool on, short hz=DEBUG_HZ) {
  _ibc_debug = on;
  _debug_hz = hz;
}

