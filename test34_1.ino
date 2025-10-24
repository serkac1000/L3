// Pin definitions
const int enB = 5;    // PWM pin for Motor M4 speed control
const int irSensor = 11; // IR sensor pin
const int startSwitch = 2; // Start switch pin
const int potPin = A1; // Potentiometer pin for speed control

// Variables
int motorSpeed = 0; // Current motor speed (0-255)
bool isRunning = false; // Motor running state
String motorStatus = "Stopped"; // Status for Serial output

void setup() {
  // Initialize pins
  pinMode(enB, OUTPUT);
  pinMode(irSensor, INPUT_PULLUP); // IR sensor with internal pull-up
  pinMode(startSwitch, INPUT_PULLUP); // Start switch with internal pull-up
  pinMode(potPin, INPUT); // Potentiometer input
  
  // Start Serial communication
  Serial.begin(9600);
  
  // Initial homing on power-up
  if (digitalRead(startSwitch) == 1 && digitalRead(irSensor) == 1) {
    // Case: pin2=0 (HIGH with INPUT_PULLUP), pin11=1 -> Not at Home, start homing
    motorStatus = "Going to Home";
    motorSpeed = 100; // Fixed speed for homing
    analogWrite(enB, motorSpeed);
    Serial.println("Status: Going to Home, Speed: 100");
    
    // Run until IR sensor detects Home (IR = 0)
    while (digitalRead(irSensor) == 1) {
      delay(10); // Small delay to avoid blocking
    }
    
    // Stop motor when Home is reached
    analogWrite(enB, 0);
    motorSpeed = 0;
    motorStatus = "Stopped at Home";
    Serial.println("Status: Stopped at Home, Speed: 0");
  } else if (digitalRead(startSwitch) == 1 && digitalRead(irSensor) == 0) {
    // Case: pin2=0 (HIGH with INPUT_PULLUP), pin11=0 -> At Home, stop
    motorStatus = "Stopped at Home";
    motorSpeed = 0;
    analogWrite(enB, 0);
    Serial.println("Status: Stopped at Home, Speed: 0");
  } else {
    // Case: pin2=1 (LOW with INPUT_PULLUP, pressed at power-on, unlikely)
    motorStatus = "Waiting for switch release";
    motorSpeed = 0;
    analogWrite(enB, 0);
    Serial.println("Status: Waiting for switch release, Speed: 0");
  }
}

void loop() {
  // Check if start switch is pressed (pin2 = 0 due to INPUT_PULLUP, pressed = LOW)
  if (digitalRead(startSwitch) == 0 && !isRunning) {
    isRunning = true;
    motorStatus = "Running";
    
    // Read potentiometer and map to PWM (0-255)
    int potValue = analogRead(potPin);
    motorSpeed = map(potValue, 0, 1023, 0, 255);
    
    // Start motor with potentiometer speed
    analogWrite(enB, motorSpeed);
    Serial.print("Status: Running, Speed: ");
    Serial.println(motorSpeed);
  }
  
  // If switch is released (pin2 = 1) and motor is running, continue until Home
  if (isRunning && digitalRead(startSwitch) == 1 && digitalRead(irSensor) == 1) {
    // Continue running at last potentiometer speed until Home
    motorStatus = "Going to Home";
    int potValue = analogRead(potPin);
    motorSpeed = map(potValue, 0, 1023, 0, 255);
    analogWrite(enB, motorSpeed);
    Serial.println("Status: Going to Home, Speed: " + String(motorSpeed));
  }
  
  // Stop motor when Home is reached (pin11 = 0) and not running with switch
  if (isRunning && digitalRead(startSwitch) == 1 && digitalRead(irSensor) == 0) {
    analogWrite(enB, 0);
    motorSpeed = 0;
    motorStatus = "Stopped at Home";
    isRunning = false;
    Serial.println("Status: Stopped at Home, Speed: 0");
  }
  
  // If switch is not pressed and not at Home, home the motor
  if (digitalRead(startSwitch) == 1 && !isRunning && digitalRead(irSensor) == 1) {
    motorStatus = "Going to Home";
    motorSpeed = 100; // Fixed speed for homing
    analogWrite(enB, motorSpeed);
    Serial.println("Status: Going to Home, Speed: 100");
    
    // Run until IR sensor detects Home (IR = 0)
    while (digitalRead(irSensor) == 1) {
      delay(10); // Small delay to avoid blocking
    }
    
    // Stop motor when Home is reached
    analogWrite(enB, 0);
    motorSpeed = 0;
    motorStatus = "Stopped at Home";
    Serial.println("Status: Stopped at Home, Speed: 0");
  }
  
  // Print status periodically (every 1 second)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    Serial.print("Status: ");
    Serial.print(motorStatus);
    Serial.print(", Speed: ");
    Serial.println(motorSpeed);
    Serial.print("IR Sensor: ");
    Serial.println(digitalRead(irSensor));
    Serial.print("Switch: ");
    Serial.println(digitalRead(startSwitch));
    lastPrint = millis();
  }
}
