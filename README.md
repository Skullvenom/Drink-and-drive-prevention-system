Drink & Drive Prevention System using ESP32 | GSM | GPS | Bluetooth
A smart safety system designed to prevent drunk driving by automatically detecting alcohol, stopping the vehicle, disabling manual control, and sending an SOS alert with live GPS coordinates via GSM.
This project uses ESP32, MQ-3 alcohol sensor, GSM SIM800/SIM900 module, GPS neo-6m, and Bluetooth App control.

Features
1. Alcohol Detection
Uses MQ-3 sensor to measure alcohol levels.
If threshold is crossed → triggers alert, locks controls, sends SOS.
2. Automatic Vehicle Lock
Stops all motors instantly.
Disables manual driving control until alcohol levels drop.
3. GPS Tracking
Fetches real-time latitude & longitude using TinyGPS++.
Sends coordinates via SMS.
4. GSM SMS Alerts
Automatically sends SMS with:
“Alcohol Detected!”
Live GPS location.
Emergency contact number configurable.
Bluetooth Manual Control
Bluetooth commands:
F → Forward
B → Backward
L → Left
R → Right
S → Stop
The car only moves when no alcohol is detected.

Hardware Used
Component	
ESP32:-	Main microcontroller
MQ-3:- Alcohol Sensor	Detects alcohol level
SIM800L / SIM900:- GSM Module	Sends SMS
Neo-6M GPS Module:-	Gets live location
L298N Motor Driver (or similar)	:-Controls car motors
Bluetooth (Built-in ESP32)	:-Controls the car manually
DC Motors (4):-	Vehicle movement

Pin Connections (Summary)
Alcohol Sensor
MQ-3 → ESP32 GPIO34 (ADC)
Motor Driver Pins
Front Left:  IN1 = 16, IN2 = 17
Front Right: IN1 = 18, IN2 = 19
Back Left:   IN1 = 21, IN2 = 22
Back Right:  IN1 = 23, IN2 = 25
GSM (UART1)
ESP32 TX → GSM RX (GPIO26)
ESP32 RX → GSM TX (GPIO27)
GPS (UART2)
ESP32 TX → GPS RX (GPIO13)
ESP32 RX → GPS TX (GPIO14)

Libraries Used
Install these libraries from Arduino IDE Library Manager:
TinyGPS++
BluetoothSerial (built-in)
HardwareSerial (built-in)

How It Works
MQ-3 constantly senses alcohol.
If alcohol level > threshold:
Stops car instantly
Sends SMS with location
Disables Bluetooth control
When levels return to normal:
Manual control is automatically restored
User can control the car with a mobile app via Bluetoo
