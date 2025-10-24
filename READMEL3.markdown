# ESP32 Motor Control System (M1, M2, M3, M4)

## Overview
This project implements a motor control system using an ESP32 as the central controller, communicating via Bluetooth to an Arduino Uno. The system manages four DC motors (M1, M2, M3, and M4) with PWM (Pulse Width Modulation) control. The ESP32 receives commands from a Bluetooth terminal app and forwards them to the Uno, which executes the motor control logic. The system operates in two modes: **Program Mode** (controlled by ESP32 commands) and **Manual Mode** (controlled by potentiometers).

## Hardware Requirements
- **ESP32**: Acts as the Bluetooth interface and command transmitter.
  - TX Pin 17: Connected to Uno RX (pin 2) for serial communication.
  - GND: Shared with Uno for common ground.
- **Arduino Uno**: Executes motor control based on ESP32 commands or manual inputs.
  - RX Pin 2: Receives data from ESP32 TX.
  - PWM Pins: 3 (M1), 5 (M2), 9 (M3), 10 (M4).
  - Mode Pin 7: HIGH for Manual Mode, LOW for Program Mode (with pull-up).
  - Button Pin 8: Triggers sequential motor start (LOW when pressed, pull-up).
  - Potentiometer Pins: A0 (M1), A1 (M2), A2 (M3).
  - LED Pin 13: Indicates mode (HIGH in Program Mode).
- **DC Motors**: M1, M2, M3, M4 connected to PWM pins with appropriate motor drivers (e.g., L298N).
- **Power Supply**: 5V for Uno, 3.3V for ESP32, and motor power as per driver specs.
- **Level Shifter**: Recommended between ESP32 TX (3.3V) and Uno RX (5V) for reliable communication.

## Software Requirements
- Arduino IDE with ESP32 and Arduino board support.
- Bluetooth terminal app (e.g., Serial Bluetooth Terminal) on a smartphone or PC.

## Setup Instructions
1. **Wiring**:
   - Connect ESP32 TX (pin 17) to Uno RX (pin 2) via a level shifter (3.3V to 5V).
   - Share GND between ESP32 and Uno.
   - Connect motor driver outputs to Uno PWM pins: 3 (M1), 5 (M2), 9 (M3), 10 (M4).
   - Attach potentiometers to A0, A1, A2 with 5V and GND.
   - Wire Mode Pin 7 to GND (Program Mode) or 5V (Manual Mode) via a switch.
   - Connect Button Pin 8 to GND via a switch with a pull-up resistor or internal pull-up.
   - Power the system with a suitable supply.
2. **Code Upload**:
   - Upload `esp32_bluetooth.ino` to the ESP32.
   - Upload `motor_control_m123_m4.ino` to the Uno (see code section below).
3. **Testing**:
   - Open ESP32 Serial Monitor (115200 baud) to verify Bluetooth reception.
   - Open Uno Serial Monitor (9600 baud) to monitor motor status.
   - Pair the Bluetooth app with "ESP32_BT" and send commands (e.g., "1" with Enter).

## Functionality
### ESP32 Role
- **Bluetooth Interface**: Receives commands from a Bluetooth terminal app.
- **Command Transmission**: Forwards commands to the Uno via Serial1 (TX pin 17) at 9600 baud, appending a newline (`\n`).
- **Supported Commands**: "0" to "10", where:
  - "0": Sets all motors to PWM 0 (off).
  - "1": Sets all motors to PWM 255 (full speed).
  - "2" to "10": Sets all motors to PWM 100, 103, 104, 105, 106, 107, 108, 109, 110 respectively.

### Arduino Uno Role
- **Mode Control**:
  - **Program Mode (Mode Pin 7 = LOW)**: LED (pin 13) is HIGH, prints "PROGRAM" every second. Motors follow ESP32 commands.
  - **Manual Mode (Mode Pin 7 = HIGH)**: LED is LOW, motors follow potentiometer inputs (A0, A1, A2).
- **Motor Control**:
  - **M1, M2, M3**: Controlled via PWM pins 3, 5, 9.
  - **M4**: Controlled via PWM pin 10 (new addition).
  - In Program Mode, PWM values are set by the last valid ESP32 command.
  - In Manual Mode, PWM values are mapped from potentiometer readings (0-1023 to 0-255).
- **Sequential Start**:
  - When Button Pin 8 is pressed (LOW), motors start sequentially: M1 → 1s → M2 → 1s → M3 → 1s → M4.
  - Uses the last valid PWM values (from ESP32 in Program Mode, from potentiometers in Manual Mode).
  - Releasing the button stops all motors.
- **Debug Output**:
  - Prints raw `mySerial` bytes and processed commands on the Serial Monitor (9600 baud).
  - Periodic status updates every second.

### Example Workflow
1. Set Mode Pin 7 to LOW (Program Mode).
2. Send "1" via Bluetooth (with Enter).
   - ESP32: "Bluetooth Received: 1", "Sent to Uno: '1\n'".
   - Uno: "Raw mySerial bytes received on pin 2: 31 0A", "Received mySerial data on pin 2: '1'", "Mode: Program, Command: 1, Motors: M1 PWM: 255, M2 PWM: 255, M3 PWM: 255, M4 PWM: 255".
3. Press Button Pin 8:
   - Uno: "Program: Starting M1, PWM: 255" → [1s] → "Program: Starting M2, PWM: 255" → [1s] → "Program: Starting M3, PWM: 255" → [1s] → "Program: Starting M4, PWM: 255".
4. Release Button Pin 8:
   - Uno: "Button released (pin8 = 1): Stopping motors".

## RAW Problem
### Issue Description
In Program Mode, the Uno is expected to print raw `mySerial` bytes received on pin 2 (e.g., "Raw mySerial bytes received on pin 2: 31 0A" for "1\n") when the ESP32 sends a command. However, this output is not appearing, even though the ESP32 confirms sending the data (e.g., "Sent to Uno: '1\n'"). This prevents the Uno from processing commands, resulting in motors M1, M2, M3, and M4 remaining at PWM 0, 0, 0, 0 instead of updating to the commanded values (e.g., 255, 255, 255, 255 for "1").

### Symptoms
- Uno Serial Monitor shows:
  ```
  mySerial initialized at 9600 baud
  Setup complete
  Initial Mode: PROGRAM
  PROGRAM
  Mode: PROGRAM, LED: On, Button: Released (pin8 = 1), Motors: Stopped
  ```
  without "Raw mySerial bytes received" or "Received mySerial data" lines.
- ESP32 Serial Monitor shows:
  ```
  Bluetooth Received: 1
  Sent to Uno: '1\n'
  ```
  indicating successful transmission.

### Possible Causes
- **Wiring Issue**: Faulty connection between ESP32 TX (pin 17) and Uno RX (pin 2) or inadequate GND.
- **Voltage Mismatch**: ESP32’s 3.3V TX signal is too weak for Uno’s 5V RX.
- **Baud Rate Mismatch**: Uno’s `mySerial.begin(9600)` may not align with ESP32’s `Serial1.begin(9600)`.
- **SoftwareSerial Conflict**: PWM on pin 3 (M1) may interfere with `mySerial` RX on pin 2.
- **ESP32 Transmission**: Data may not reach the Uno due to timing or buffer issues.

### Troubleshooting Steps
1. **Check Wiring**:
   - Verify ESP32 TX (pin 17) to Uno RX (pin 2) with a multimeter for continuity.
   - Test an alternative RX pin (e.g., pin 4) by updating:
     ```cpp
     SoftwareSerial mySerial(4, 3);  // RX=4, TX=3
     ```
     and adjusting ESP32 wiring.
2. **Add Level Shifter**:
   - Use a 3.3V to 5V level shifter between ESP32 TX and Uno RX.
3. **Verify Baud Rate**:
   - Ensure both devices use 9600 baud. Test 115200:
     ```cpp
     // Uno
     mySerial.begin(115200);
     // ESP32
     Serial1.begin(115200, SERIAL_8N1, -1, TX_PIN);
     ```
4. **Mitigate SoftwareSerial Conflict**:
   - Temporarily disable PWM on pin 3 during reads:
     ```cpp
     if (mySerial.available() > 0) {
       analogWrite(motorPin1, 0);
       // Read logic
       analogWrite(motorPin1, sequencePwm1);
     }
     ```
5. **Test ESP32 Transmission**:
   - Modify ESP32 to send explicit bytes:
     ```cpp
     Serial1.write("1\n");  // Instead of Serial1.println(data)
     ```
   - Confirm with a Bluetooth app sending "1" with Enter.
6. **Monitor Signal**:
   - Use an oscilloscope or logic analyzer on Uno pin 2 to check for a 9600 baud signal.
   - If unavailable, connect ESP32 TX to another device (e.g., another Uno with HardwareSerial).

## Code
- **ESP32**: `esp32_bluetooth.ino` handles Bluetooth and serial transmission.
- **Uno**: `motor_control_m123_m4.ino` includes M4 on pin 10 (see below).