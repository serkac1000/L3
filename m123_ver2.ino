#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3);  // RX=2, TX=3 (TX unused)
const int ledPin = 13;  // Built-in LED pin
const int modePin = 7;  // Mode switch: HIGH = manual, LOW = program
const int buttonPin = 8;  // Start/Stop button: HIGH = not pressed, LOW = pressed
const int motorPin1 = 3;  // PWM pin for Motor M1 (ENA)
const int motorPin2 = 5;  // PWM pin for Motor M2 (ENB)
const int motorPin3 = 9;  // PWM pin for Motor M3 (ENA)
const int potPin1 = A0;   // POT1 for M1 manual PWM
const int potPin2 = A1;   // POT2 for M2 manual PWM
const int potPin3 = A2;   // POT3 for M3 manual PWM
bool ledState = false;  // Track pin 13 state (false = LOW, true = HIGH)
bool lastButtonState = HIGH;  // Track last button state for debouncing
bool motorRunning = false;  // Track motor running state
unsigned long lastDebounceTime = 0;  // Last debounce time
const unsigned long debounceDelay = 50;  // Debounce delay (ms)
unsigned long lastPrintTime = 0;  // Last print time for periodic status
const unsigned long printInterval = 1000;  // Print every 1 second

// Variables for sequential motor start in manual mode
bool inManualSequence = false;  // Track if in sequential start
unsigned long sequenceStartTime = 0;  // Time when sequence started
int sequencePwm1 = 0;  // PWM for M1
int sequencePwm2 = 0;  // PWM for M2
int sequencePwm3 = 0;  // PWM for M3
enum SequenceState { IDLE, M1_STARTED, M2_STARTED, M3_STARTED };
SequenceState sequenceState = IDLE;  // Current state of motor sequence

void setup() {
  Serial.begin(9600);  // Serial Monitor at 9600 baud
  mySerial.begin(9600);  // Match ESP32 baud rate
  pinMode(ledPin, OUTPUT);  // Set pin 13 as output
  pinMode(modePin, INPUT_PULLUP);  // Mode switch with internal pull-up
  pinMode(buttonPin, INPUT_PULLUP);  // Start/Stop button with internal pull-up
  pinMode(motorPin1, OUTPUT);  // PWM pin for Motor M1
  pinMode(motorPin2, OUTPUT);  // PWM pin for Motor M2
  pinMode(motorPin3, OUTPUT);  // PWM pin for Motor M3
  pinMode(potPin1, INPUT);  // POT1 input
  pinMode(potPin2, INPUT);  // POT2 input
  pinMode(potPin3, INPUT);  // POT3 input
  analogWrite(motorPin1, 0);  // Initialize motors off
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  // Initialize LED based on mode
  bool isManualMode = digitalRead(modePin) == HIGH;
  ledState = !isManualMode;  // HIGH for program mode, LOW for manual
  digitalWrite(ledPin, ledState ? HIGH : LOW);
  Serial.println("Setup complete");
  Serial.print("Initial Mode: ");
  Serial.println(isManualMode ? "Manual" : "Program");
}

void loop() {
  // Update LED based on mode
  bool isManualMode = digitalRead(modePin) == HIGH;  // HIGH = manual, LOW = program
  ledState = !isManualMode;  // LED on (HIGH) for program mode, off (LOW) for manual
  digitalWrite(ledPin, ledState ? HIGH : LOW);

  // Handle button on pin 8 with debouncing
  int buttonReading = digitalRead(buttonPin);
  unsigned long currentTime = millis();

  // Update motorRunning based on button state
  if (buttonReading != lastButtonState && (currentTime - lastDebounceTime) > debounceDelay) {
    lastButtonState = buttonReading;
    if (isManualMode) {
      if (buttonReading == LOW && !motorRunning) {  // Button pressed (pin8 = 0): start motors
        motorRunning = true;
        inManualSequence = true;
        sequenceState = M1_STARTED;
        sequenceStartTime = currentTime;
        sequencePwm1 = map(analogRead(potPin1), 0, 1023, 0, 255);
        sequencePwm2 = map(analogRead(potPin2), 0, 1023, 0, 255);
        sequencePwm3 = map(analogRead(potPin3), 0, 1023, 0, 255);
        analogWrite(motorPin1, sequencePwm1);  // Start M1
        Serial.print("Manual: Starting M1, PWM: ");
        Serial.println(sequencePwm1);
      } else if (buttonReading == HIGH && motorRunning) {  // Button released (pin8 = 1): stop motors
        motorRunning = false;
        inManualSequence = false;
        sequenceState = IDLE;
        analogWrite(motorPin1, 0);  // M1 off
        analogWrite(motorPin2, 0);  // M2 off
        analogWrite(motorPin3, 0);  // M3 off
        Serial.println("Button released (pin8 = 1): Stopping motors");
      }
    }
    lastDebounceTime = currentTime;
  }

  // Manual mode: Handle sequential motor start
  if (isManualMode && motorRunning && inManualSequence) {
    // 1-second delay for M2 start
    if (sequenceState == M1_STARTED && currentTime - sequenceStartTime >= 1000) {
      analogWrite(motorPin2, sequencePwm2);  // Start M2
      sequenceState = M2_STARTED;
      Serial.print("Manual: Starting M2, PWM: ");
      Serial.println(sequencePwm2);
    }
    // 1-second delay for M3 start
    else if (sequenceState == M2_STARTED && currentTime - sequenceStartTime >= 2000) {
      analogWrite(motorPin3, sequencePwm3);  // Start M3
      sequenceState = M3_STARTED;
      inManualSequence = false;  // Sequence complete
      Serial.print("Manual: Starting M3, PWM: ");
      Serial.println(sequencePwm3);
    }
  }

  // Manual mode: Update motor speeds if sequence is complete
  if (isManualMode && motorRunning && !inManualSequence) {
    int potValue1 = analogRead(potPin1);
    int potValue2 = analogRead(potPin2);
    int potValue3 = analogRead(potPin3);
    int pwm1 = map(potValue1, 0, 1023, 0, 255);
    int pwm2 = map(potValue2, 0, 1023, 0, 255);
    int pwm3 = map(potValue3, 0, 1023, 0, 255);
    analogWrite(motorPin1, pwm1);
    analogWrite(motorPin2, pwm2);
    analogWrite(motorPin3, pwm3);
  } else if (isManualMode && !motorRunning) {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, 0);
  }

  // Program mode: Handle serial input
  if (!isManualMode) {
    // Debug: Confirm program mode is active
    static unsigned long lastProgramCheck = 0;
    if (currentTime - lastProgramCheck >= printInterval) {
      Serial.println("Program Mode Active: Waiting for mySerial data on pin 2");
      lastProgramCheck = currentTime;
    }

    if (mySerial.available() > 0) {
      String data = mySerial.readStringUntil('\n');
      data.trim();
      Serial.print("Received mySerial data on pin 2: '");
      Serial.print(data);
      Serial.println("'");

      int pwm1 = 0, pwm2 = 0, pwm3 = 0;  // Default PWM values
      bool validCommand = false;  // Track if command is valid

      if (data == "0") {
        motorRunning = false;
        pwm1 = 0;
        pwm2 = 0;
        pwm3 = 0;
        analogWrite(motorPin1, 0);
        analogWrite(motorPin2, 0);
        analogWrite(motorPin3, 0);
        validCommand = true;
      } else if (data == "1") {
        motorRunning = true;
        pwm1 = 255;
        pwm2 = 255;
        pwm3 = 255;
        analogWrite(motorPin1, 255);
        analogWrite(motorPin2, 255);
        analogWrite(motorPin3, 255);
        validCommand = true;
      } else if (data == "2") {
        motorRunning = true;
        pwm1 = 100;
        pwm2 = 100;
        pwm3 = 100;
        analogWrite(motorPin1, 100);
        analogWrite(motorPin2, 100);
        analogWrite(motorPin3, 100);
        validCommand = true;
      } else if (data == "3") {
        motorRunning = true;
        pwm1 = 103;
        pwm2 = 103;
        pwm3 = 103;
        analogWrite(motorPin1, 103);
        analogWrite(motorPin2, 103);
        analogWrite(motorPin3, 103);
        validCommand = true;
      } else if (data == "4") {
        motorRunning = true;
        pwm1 = 104;
        pwm2 = 104;
        pwm3 = 104;
        analogWrite(motorPin1, 104);
        analogWrite(motorPin2, 104);
        analogWrite(motorPin3, 104);
        validCommand = true;
      } else if (data == "5") {
        motorRunning = true;
        pwm1 = 105;
        pwm2 = 105;
        pwm3 = 105;
        analogWrite(motorPin1, 105);
        analogWrite(motorPin2, 105);
        analogWrite(motorPin3, 105);
        validCommand = true;
      } else if (data == "6") {
        motorRunning = true;
        pwm1 = 106;
        pwm2 = 106;
        pwm3 = 106;
        analogWrite(motorPin1, 106);
        analogWrite(motorPin2, 106);
        analogWrite(motorPin3, 106);
        validCommand = true;
      } else if (data == "7") {
        motorRunning = true;
        pwm1 = 107;
        pwm2 = 107;
        pwm3 = 107;
        analogWrite(motorPin1, 107);
        analogWrite(motorPin2, 107);
        analogWrite(motorPin3, 107);
        validCommand = true;
      } else if (data == "8") {
        motorRunning = true;
        pwm1 = 108;
        pwm2 = 108;
        pwm3 = 108;
        analogWrite(motorPin1, 108);
        analogWrite(motorPin2, 108);
        analogWrite(motorPin3, 108);
        validCommand = true;
      } else if (data == "9") {
        motorRunning = true;
        pwm1 = 109;
        pwm2 = 109;
        pwm3 = 109;
        analogWrite(motorPin1, 109);
        analogWrite(motorPin2, 109);
        analogWrite(motorPin3, 109);
        validCommand = true;
      } else if (data == "10") {
        motorRunning = true;
        pwm1 = 110;
        pwm2 = 110;
        pwm3 = 110;
        analogWrite(motorPin1, 110);
        analogWrite(motorPin2, 110);
        analogWrite(motorPin3, 110);
        validCommand = true;
      }

      // Print PWM values for valid commands
      if (validCommand) {
        Serial.print("Mode: Program, Command: ");
        Serial.print(data);
        Serial.print(", Motors: M1 PWM: ");
        Serial.print(pwm1);
        Serial.print(", M2 PWM: ");
        Serial.print(pwm2);
        Serial.print(", M3 PWM: ");
        Serial.println(pwm3);
      } else {
        Serial.print("Mode: Program, Invalid Command: ");
        Serial.print(data);
        Serial.println(", Motors: Unchanged");
      }

      while (mySerial.available() > 0) {
        mySerial.read();  // Clear buffer
      }
    }
  }

  // Periodic status print
  if (currentTime - lastPrintTime >= printInterval) {
    Serial.print("Mode: ");
    Serial.print(isManualMode ? "Manual" : "Program");
    Serial.print(", LED: ");
    Serial.print(ledState ? "On" : "Off");
    Serial.print(", Button: ");
    Serial.print(digitalRead(buttonPin) == LOW ? "Pressed (pin8 = 0)" : "Released (pin8 = 1)");
    Serial.print(", Motors: ");
    if (isManualMode && motorRunning) {
      int potValue1 = analogRead(potPin1);
      int potValue2 = analogRead(potPin2);
      int potValue3 = analogRead(potPin3);
      int pwm1 = map(potValue1, 0, 1023, 0, 255);
      int pwm2 = map(potValue2, 0, 1023, 0, 255);
      int pwm3 = map(potValue3, 0, 1023, 0, 255);
      Serial.print("M1 PWM: ");
      Serial.print(pwm1);
      Serial.print(", M2 PWM: ");
      Serial.print(pwm2);
      Serial.print(", M3 PWM: ");
      Serial.println(pwm3);
      Serial.print("Raw POT1: ");
      Serial.print(potValue1);
      Serial.print(", POT2: ");
      Serial.print(potValue2);
      Serial.print(", POT3: ");
      Serial.println(potValue3);
    } else {
      Serial.println("Stopped");
    }
    lastPrintTime = currentTime;
  }
}
