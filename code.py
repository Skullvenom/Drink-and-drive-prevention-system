#include <TinyGPS++.h>
#include <BluetoothSerial.h>

// Pin Definitions
const int alcoholSensorPin = 34;  // MQ-3 Alcohol Sensor (ADC pin on ESP32)
const int threshold = 4000;

// Motor Driver Pins
const int FRONT_LEFT_IN1 = 16;
const int FRONT_LEFT_IN2 = 17;
const int FRONT_RIGHT_IN1 = 18;
const int FRONT_RIGHT_IN2 = 19;
const int BACK_LEFT_IN1 = 21;
const int BACK_LEFT_IN2 = 22;
const int BACK_RIGHT_IN1 = 23;
const int BACK_RIGHT_IN2 = 25;

// GSM and GPS Modules
HardwareSerial gsm(1);           // UART1 for GSM
HardwareSerial gpsSerial(2);     // UART2 for GPS
TinyGPSPlus gps;

// Bluetooth
BluetoothSerial bluetooth;
String phone_number = "+911234567890";  // Change this to the desired phone number

// Control Flags
bool manualControlEnabled = true;

void setup() {
  // Motor Driver Pins
  pinMode(FRONT_LEFT_IN1, OUTPUT);
  pinMode(FRONT_LEFT_IN2, OUTPUT);
  pinMode(FRONT_RIGHT_IN1, OUTPUT);
  pinMode(FRONT_RIGHT_IN2, OUTPUT);
  pinMode(BACK_LEFT_IN1, OUTPUT);
  pinMode(BACK_LEFT_IN2, OUTPUT);
  pinMode(BACK_RIGHT_IN1, OUTPUT);
  pinMode(BACK_RIGHT_IN2, OUTPUT);

  // Serial Communication
  Serial.begin(115200);
  gsm.begin(9600, SERIAL_8N1, 26, 27);       // GSM TX: GPIO26, RX: GPIO27
  gpsSerial.begin(9600, SERIAL_8N1, 13, 14); // GPS TX: GPIO13, RX: GPIO14
  bluetooth.begin("ESP32-Car");              // Initialize Bluetooth with name

  // Initial Messages
  Serial.println("System Initialized. Monitoring Started.");
  bluetooth.println("====== SYSTEM STATUS ======");
  bluetooth.println("System Initialized. Monitoring Started.");
  bluetooth.println("===========================");
}

void loop() {
  int alcoholValue = analogRead(alcoholSensorPin);

  // Send Alcohol Level for Monitoring
  bluetooth.println("------ ALCOHOL SENSOR ------");
  bluetooth.printf("Alcohol Level: %d (Threshold: %d)\n", alcoholValue, threshold);
  bluetooth.println("----------------------------");

  if (alcoholValue > threshold) {
    triggerAlert();
    stopCar();
    disableManualControl();
    sendLocation();
  } else {
    if (!manualControlEnabled) {
      enableManualControl();  // Re-enable manual control
    }
    if (manualControlEnabled) {
      handleBluetoothControl();
    }
  }

  delay(100); // Delay to prevent overloading
}

// Handle Bluetooth-Based Commands
void handleBluetoothControl() {
  if (bluetooth.available()) {
    String btCommand = bluetooth.readString();
    btCommand.trim();
    executeCommand(btCommand);
  }
}

// Execute Movement Commands
void executeCommand(String command) {
  if (command == "F") {
    moveForward();
  } else if (command == "B") {
    moveBackward();
  } else if (command == "L") {
    turnLeft();
  } else if (command == "R") {
    turnRight();
  } else if (command == "S") {
    stopCar();
  }
}

// Alert on Alcohol Detection
void triggerAlert() {
  Serial.println("ALERT: Alcohol detected! Car stopped.");
  bluetooth.println("====== ALERT ======");
  bluetooth.println("ALERT: Alcohol Detected!");
  bluetooth.println("Car Status: Stopped");
  bluetooth.println("===================");
}

// Send SOS Location
void sendLocation() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    String locationData = "ALERT! Alcohol detected!\nLat: " +
                          String(gps.location.lat(), 6) +
                          " Lon: " + String(gps.location.lng(), 6);
    sendSMS(locationData);
    bluetooth.println("====== LOCATION UPDATE ======");
    bluetooth.println(locationData);
    bluetooth.println("=============================");
    Serial.println("SOS Alert Sent with Location!");
  } else {
    String errorMsg = "ALERT! Alcohol detected, but GPS location not available.";
    sendSMS(errorMsg);
    bluetooth.println(errorMsg);
  }

  delay(10000); // Prevent SMS spamming
}

// Send SMS Function
void sendSMS(String message) {
  gsm.println("AT+CMGF=1");
  delay(100);
  gsm.printf("AT+CMGS=\"%s\"\r", phone_number.c_str());
  delay(100);
  gsm.print(message);
  delay(100);
  gsm.write(26); // ASCII CTRL+Z
  delay(2000);
}

// Enable and Disable Manual Control
void disableManualControl() {
  manualControlEnabled = false;
  Serial.println("Manual control disabled due to alcohol detection.");
  bluetooth.println("Manual control disabled due to alcohol detection.");
}

void enableManualControl() {
  manualControlEnabled = true;
  Serial.println("Manual control re-enabled.");
  bluetooth.println("Manual control re-enabled.");
}

// Movement Functions
void moveForward() {
  digitalWrite(FRONT_LEFT_IN1, LOW);
  digitalWrite(FRONT_LEFT_IN2, HIGH);
  digitalWrite(FRONT_RIGHT_IN1, LOW);
  digitalWrite(FRONT_RIGHT_IN2, HIGH);
  digitalWrite(BACK_LEFT_IN1, LOW);
  digitalWrite(BACK_LEFT_IN2, HIGH);
  digitalWrite(BACK_RIGHT_IN1, LOW);
  digitalWrite(BACK_RIGHT_IN2, HIGH);
}

void moveBackward() {
  digitalWrite(FRONT_LEFT_IN1, HIGH);
  digitalWrite(FRONT_LEFT_IN2, LOW);
  digitalWrite(FRONT_RIGHT_IN1, HIGH);
  digitalWrite(FRONT_RIGHT_IN2, LOW);
  digitalWrite(BACK_LEFT_IN1, HIGH);
  digitalWrite(BACK_LEFT_IN2, LOW);
  digitalWrite(BACK_RIGHT_IN1, HIGH);
  digitalWrite(BACK_RIGHT_IN2, LOW);
}
void turnLeft() {
  digitalWrite(FRONT_LEFT_IN1, HIGH);
  digitalWrite(FRONT_LEFT_IN2, LOW);
  digitalWrite(FRONT_RIGHT_IN1, LOW);
  digitalWrite(FRONT_RIGHT_IN2, HIGH);
  digitalWrite(BACK_LEFT_IN1, HIGH);
  digitalWrite(BACK_LEFT_IN2, LOW);
  digitalWrite(BACK_RIGHT_IN1, LOW);
  digitalWrite(BACK_RIGHT_IN2, HIGH);
}

void turnRight() {
  digitalWrite(FRONT_LEFT_IN1, LOW);
  digitalWrite(FRONT_LEFT_IN2, HIGH);
  digitalWrite(FRONT_RIGHT_IN1, HIGH);
  digitalWrite(FRONT_RIGHT_IN2, LOW);
  digitalWrite(BACK_LEFT_IN1, LOW);
  digitalWrite(BACK_LEFT_IN2, HIGH);
  digitalWrite(BACK_RIGHT_IN1, HIGH);
  digitalWrite(BACK_RIGHT_IN2, LOW);
}

void stopCar() {
  digitalWrite(FRONT_LEFT_IN1, LOW);
  digitalWrite(FRONT_LEFT_IN2, LOW);
  digitalWrite(FRONT_RIGHT_IN1, LOW);
  digitalWrite(FRONT_RIGHT_IN2, LOW);
  digitalWrite(BACK_LEFT_IN1, LOW);
  digitalWrite(BACK_LEFT_IN2, LOW);
  digitalWrite(BACK_RIGHT_IN1, LOW);
  digitalWrite(BACK_RIGHT_IN2, LOW);
}
