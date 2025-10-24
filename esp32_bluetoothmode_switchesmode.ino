#include <BluetoothSerial.h>
#include <HardwareSerial.h>

#define TX_PIN 17  // ESP32 TX pin for Serial1
#define MODE_PIN 32 // Mode switch pin: HIGH = Bluetooth Mode (1), LOW = Switches Mode (0)
#define LED_PIN 2   // LED pin for Bluetooth mode indication (HIGH = Bluetooth, LOW = Switches)

// Array of GPIO pins for 10 switches
const int switchPins[] = {4, 5, 13, 14, 15, 18, 19, 21, 22, 23};
const int numSwitches = 10;

HardwareSerial MySerial(1);  // Use Serial1
BluetoothSerial SerialBT;    // Bluetooth serial instance

bool lastModeState = HIGH;  // Track last mode switch state for debouncing
unsigned long lastModeDebounceTime = 0;  // Last debounce time for mode switch
const unsigned long debounceDelay = 100; // Increased debounce delay to 100ms
bool isBluetoothMode = false;  // Current mode state

void setup() {
  Serial.begin(115200);  // Serial Monitor for debugging
  MySerial.begin(9600, SERIAL_8N1, -1, TX_PIN);  // Serial1, TX only (RX disabled)
  SerialBT.begin("ESP32_BT");  // Bluetooth device name
  delay(1000);  // Wait for serial ports to initialize

  // Initialize LED pin as output
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize mode switch pin as input with internal pull-up resistor
  pinMode(MODE_PIN, INPUT_PULLUP);
  delay(100);  // Allow pin state to stabilize
  int initialMode = digitalRead(MODE_PIN);  // Read initial state after delay
  isBluetoothMode = (initialMode == HIGH);  // HIGH = Bluetooth Mode (1), LOW = Switches Mode (0)
  
  // Set LED based on initial mode
  digitalWrite(LED_PIN, isBluetoothMode ? HIGH : LOW);
  
  Serial.print("ESP32 Started - Initial Mode: ");
  Serial.println(isBluetoothMode ? "Bluetooth" : "Switches");
  Serial.print("Pin 32 State at Startup: ");
  Serial.println(initialMode);  // Debug initial pin state

  // Initialize switch pins with internal pull-up resistors
  for (int i = 0; i < numSwitches; i++) {
    pinMode(switchPins[i], INPUT_PULLUP);
  }
}

void loop() {
  // Handle mode switch debouncing
  int modeReading = digitalRead(MODE_PIN);
  unsigned long currentTime = millis();

  if (modeReading != lastModeState) {
    lastModeDebounceTime = currentTime;  // Record time of state change
    Serial.print("Pin 32 State Changed to: ");
    Serial.println(modeReading);  // Debug immediate state change
  }

  if ((currentTime - lastModeDebounceTime) > debounceDelay) {
    if (modeReading != lastModeState) {
      lastModeState = modeReading;
      isBluetoothMode = (modeReading == HIGH);  // HIGH = Bluetooth Mode (1), LOW = Switches Mode (0)
      digitalWrite(LED_PIN, isBluetoothMode ? HIGH : LOW);  // Update LED
      Serial.print("Mode switched to: ");
      Serial.println(isBluetoothMode ? "Bluetooth" : "Switches");
      Serial.print("Pin 32 State after Debounce: ");
      Serial.println(modeReading);  // Debug debounced state
    }
  }

  // Force sync mode with current state to prevent sticking
  if (isBluetoothMode != (modeReading == HIGH)) {
    isBluetoothMode = (modeReading == HIGH);
    digitalWrite(LED_PIN, isBluetoothMode ? HIGH : LOW);  // Update LED
    Serial.print("Mode synced to: ");
    Serial.println(isBluetoothMode ? "Bluetooth" : "Switches");
    Serial.print("Pin 32 State synced: ");
    Serial.println(modeReading);  // Debug synced state
  }

  if (isBluetoothMode) {
    // Bluetooth Mode: Receive data from Bluetooth and forward to Uno
    if (SerialBT.available() > 0) {
      String data = SerialBT.readStringUntil('\n');  // Read until newline
      data.trim();  // Remove whitespace or control characters
      int value = data.toInt();  // Convert to integer
      if (value >= 1 && value <= 10) {
        MySerial.println(value);  // Send to Uno
        Serial.print("Bluetooth Received: ");
        Serial.print(value);
        Serial.println(" sent to Uno");
      } else if (data.length() > 0) {
        MySerial.println("0");  // Invalid input, send 0
        Serial.println("Invalid Bluetooth data, sent 0 to Uno");
      }
    }
  } else {
    // Switches Mode: Check switches and send corresponding value
    bool switchPressed = false;
    for (int i = 0; i < numSwitches; i++) {
      if (digitalRead(switchPins[i]) == LOW) {  // Active low: pressed = LOW
        int valueToSend = i + 1;  // Send 1 for switch 1, 2 for switch 2, etc.
        MySerial.println(valueToSend);  // Send to Arduino Uno
        Serial.print("Sent: ");
        Serial.println(valueToSend);  // Debug to Serial Monitor
        switchPressed = true;
        delay(50);  // Brief delay to ensure transmission
        break;  // Send only one value per loop (first pressed switch)
      }
    }
    // Send 0 only if no switch was pressed
    if (!switchPressed) {
      MySerial.println("0");
      Serial.println("Sent: 0");
    }
  }

  delay(50);  // Further reduced delay for better responsiveness
}
 
