/*
  IBC Gripper Class
  
  Class to operate the I-BoomCopter's compliant gripper mechanism.
  Provides functions for homing, opening and closing the gripper.
  Provides access to the absolute position of the gripper (rotary and linear).
  
  Created 13 Oct 2017
  by Daniel McArthur

 */

#include "IBCGripper.h"

#define LOOP_HZ    500  // Main loop rate

// Serial input (from user)
String inString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

// Debug settings
int debug_hz = 10;            // Debug display rate

unsigned long loop_t = 0;
unsigned long disp_t = 0;

// Gripper control object
IBCGripper gripper;

// *****************************************************
void setup() {
  // initialize serial:
  Serial.begin(115200);
  // reserve 200 bytes for the inString:
  inString.reserve(200);

  // Intialize pin modes (input/output)
  gripper.initPinModes();
  
  // Set up interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderDispatchA, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderDispatchB, FALLING);
}
// _____________________________________________________

void encoderDispatchA() {
  gripper.ISR_ENCODER_A();
}
void encoderDispatchB() {
  gripper.ISR_ENCODER_B();
}

// *****************************************************
// Main loop
// *****************************************************
void loop() {
        
  // Handle User input
  if (stringComplete) {
    parseInput();
  }

  //unsigned long start = micros();

  // Run motor position controller
  unsigned long start = micros();
  gripper.update();

  //if(micros() - disp_t > 1000000/2) {
  //  Serial.println(String(micros() - start));
  //  disp_t = micros();
  //}
  
  // Enforce loop rate
  if(micros() - loop_t > 1000000/LOOP_HZ) {
    delayMicroseconds(500);
  }
  loop_t = micros();
}
// _____________________________________________________

// Parse and handle Serial input
void parseInput() {
  if(inString.length() == 0) {
    Serial.println("EMPTY COMMAND... Stopping motor!");
    gripper.idle();
  } 
  else if (inString.charAt(0) == 'd' || inString.charAt(1) == 'D') {
    short new_rate = 10;
    if(inString.length() > 1) {
      String val = inString.substring(1);  // Extract the desired brightness
      new_rate = val.toInt();              // Convert to integer
      if(new_rate < 0 || new_rate > 200) {
        Serial.println("Invalid debug display rate received: '" + val + "'");
      } else {
        Serial.print("Updated debug display rate to ");
        Serial.print(new_rate);
        Serial.println(" Hz");
      }
    }
    if(gripper.getDebug()) {
      Serial.println("Debug messages OFF");
      gripper.setDebug(false, new_rate);
    } else {
      Serial.println("Debug messages ON");
      gripper.setDebug(true, new_rate);
    }

  }
  else if (inString.charAt(0) == '0') {
    Serial.println("Resetting home position");
    gripper.setHome();
  }
  else if (inString.charAt(0) == '1') {
    Serial.println("Moving 1 complete revolution forward");
    gripper.grip(gripper.getSteps() + 360*STEP_PER_DEG);
  }
  else if (inString.charAt(0) == '2') {
    Serial.println("Moving 1 complete revolution in reverse");
    gripper.grip(gripper.getSteps() - 360*STEP_PER_DEG);
  }
  else if (inString.charAt(0) == '3') {
    Serial.print("Moving 1 mm forward: ");
    Serial.println(gripper.getSteps() + STEP_PER_MM);
    gripper.grip(gripper.getSteps() + STEP_PER_MM);
  }
  else if (inString.charAt(0) == '4') {
    Serial.print("Moving 1 mm in reverse: ");
    Serial.println(gripper.getSteps() - STEP_PER_MM);
    gripper.grip(gripper.getSteps() - STEP_PER_MM);
  }
  else if (inString.charAt(0) == 'h') {
    Serial.println("Homing...");
    gripper.home();
  }
  else if (inString.charAt(0) == 'o' || inString.charAt(1) == 'O') {
    if(inString.length() > 1) {
      String val = inString.substring(1);  // Extract the desired position
      if(val == "s" || val == "S") {
        Serial.println("Opening to full open (soft)");
        gripper.openSoft();
      }
    } else {
      Serial.println("Opening to full open");
      gripper.open();
    }
  }
  else if (inString.equalsIgnoreCase("gs")) {
    Serial.print("Gripper State: ");
    Serial.println(gripper.getState());
  }
  else if (inString.charAt(0) == 'g' || inString.charAt(1) == 'G') {
    // Set position setpoint
    if(inString.length() > 1) {
      String val = inString.substring(1);  // Extract the desired position
      long num_steps = val.toInt();         // Convert to integer
      if(num_steps < -5000 || num_steps > 5000) {
        Serial.println("Too many steps: '" + val + "'");
      } else {
        gripper.grip(num_steps);
        Serial.print("New position setpoint is: ");
        Serial.println(num_steps);
      }
    } else { 
      Serial.print("Position is: "); 
      Serial.println(gripper.getSteps()); Serial.println(" steps");
    }
  }
  else if (inString.charAt(0) == 's' || inString.charAt(1) == 'S') {
    // Set position setpoint
    if(inString.length() > 1) {
      String val = inString.substring(1);        // Extract the desired mm pos
      float new_mm = val.toFloat();              // Convert to number
      gripper.grip(gripper.spanMMToSteps(new_mm));
      Serial.print("New span setpoint: ");
      Serial.print(new_mm);
      Serial.println(" mm");
    } else { 
      Serial.print("Span setpoint is: "); 
      Serial.println(gripper.getSpanMM()); Serial.println(" mm");
    } 
  }
  else if (inString.substring(0,2) == "cm" || inString.substring(0,2) == "CM") {
    if(inString.length() > 2) {
      String val = inString.substring(2);         // Extract the desired cm pos
      float new_cm = val.toFloat();               // Convert to number
      gripper.grip(gripper.mmToSteps(new_cm*10)); // Convert pos setpoint to steps
      Serial.print("New position setpoint: ");
      Serial.print(new_cm);
      Serial.println(" cm");
    } else { 
      Serial.print("Position is: "); 
      Serial.print(gripper.getMM()/10.0); Serial.println(" cm");
    } 
  }
  else if (inString.substring(0,2) == "mm" || inString.substring(0,2) == "MM") {
    if(inString.length() > 2) {
      String val = inString.substring(2);        // Extract the desired mm pos
      float new_mm = val.toFloat();              // Convert to number
      gripper.grip(gripper.mmToSteps(new_mm));
      Serial.print("New position setpoint: ");
      Serial.print(new_mm);
      Serial.println(" mm");
    } else { 
      Serial.print("Position is: "); 
      Serial.print(gripper.getMM()); Serial.println(" mm");
    } 
  }
  else {
    Serial.print("Unknown command: "); Serial.println(inString);
  }
  
  // clear the string:
  inString = "";
  stringComplete = false;
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // Get the new byte:
    char inChar = (char)Serial.read();
    if (inChar == '\r') {
      stringComplete = true;  // Stop reading string at newline
    }
    else {
    // Add char to the inString:
    inString += inChar;
    }    
  }
}


