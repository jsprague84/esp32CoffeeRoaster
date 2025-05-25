//This is the code that was updated 4/14/25. added mqtt function.  It does need work on autotune.
#include <SPI.h>
#include <Arduino.h>
#include <WiFi.h>
#include <ModbusTCP.h>
#include <ArduinoOTA.h>
#include <PID_v1.h>
#include "MAX6675Handler.h"
#include <EEPROM.h>
#include <WebServer.h>
#include "driver/gpio.h"

// Debugging Macro
#define DEBUG true
#if DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(fmt, ...)
#endif

// Wi-Fi Credentials
const char* ssid = "jsprague";
const char* password = "d2422ab770";

// Web Server
WebServer server(80);
// MAX6675 Bean Temperature (BT) Pins
#define BT_CS   4
#define ET_CS   5

// Solid State Relay (SSR) for heating
#define SSR_PIN  33

// L298N Module Fan Control Pin
#define FAN_PIN  25

// Modes for control
#define MODE_MANUAL 0
#define MODE_AUTO 1
#define MODE_PID_AUTOTUNE 2

// Modbus TCP server
ModbusTCP modbusTCP;

// PID Variables
double beanSetpoint = 0.0;
double beanTemperature = 0.0;
double heaterOutput = 0.0;

// PID Tuning Parameters
double Kp = 15.0, Ki = 1.0, Kd = 25.0;

// Calibration offsets
float beanTempOffset = 0.0;
float envTempOffset = 0.0;

// Create PID instance
PID beanPID(&beanTemperature, &heaterOutput, &beanSetpoint, Kp, Ki, Kd, DIRECT);

// Variables for control
float envTemperature = 0.0;
int fanPWM = 0;

// Modbus Registers
#define REG_BEAN_TEMP 0
#define REG_ENV_TEMP 1
#define REG_HEATER_PWM 2
#define REG_FAN_PWM 3
#define REG_BEAN_SP 4
#define REG_CONTROL_MODE 5
#define REG_OVERRIDE_HEATER 6
#define REG_KP 7
#define REG_KI 8
#define REG_KD 9

// Timers for non-blocking tasks
unsigned long lastTempRead = 0;
const unsigned long tempReadInterval = 1000;

unsigned long lastModbusTask = 0;
const unsigned long modbusTaskInterval = 10;

unsigned long lastPidCompute = 0;
const unsigned long pidComputeInterval = 100;

unsigned long lastSerialOutput = 0;
const unsigned long serialOutputInterval = 1000;

// Initialize MAX6675 thermocouples
MAX6675Handler beanThermocouple(BT_CS);
MAX6675Handler envThermocouple(ET_CS);

// Function Prototypes
void initializePins();
void connectToWiFi(const char* ssid, const char* password);
void autoTunePID();
void applyCalibration();
void updatePIDParameters();
void savePIDParameters();
void loadPIDParameters();
void handleWebServer();
void handleDataRequest(); // Prototype for the /data endpoint

void setup() {
    Serial.begin(115200);
    delay(1000);
    DEBUG_PRINTLN("Starting Coffee Roaster Control...");

    // Initialize EEPROM
    EEPROM.begin(512);

    // Initialize GPIO pins
    initializePins();

    // Initialize LEDC for PWM
    ledcSetup(0, 5000, 8);        // Channel 0, 5kHz frequency, 8-bit resolution
    ledcAttachPin(SSR_PIN, 0);
    ledcSetup(1, 5000, 8);
    ledcAttachPin(FAN_PIN, 1);

    // Set the drive strength of pin 25 to the highest level
    gpio_set_drive_capability((gpio_num_t)FAN_PIN, GPIO_DRIVE_CAP_3);

    // Load PID Parameters
    loadPIDParameters();

    // Connect to WiFi
    connectToWiFi(ssid, password);

    // Initialize Modbus server
    modbusTCP.server();
    for (int i = 0; i < 10; i++) {
        modbusTCP.addHreg(i, 0);
    }

    // Initialize thermocouples
    beanThermocouple.begin();
    envThermocouple.begin();
    DEBUG_PRINTLN("MAX6675 Initialized");

    // Initialize OTA
    ArduinoOTA.begin();
    DEBUG_PRINTLN("OTA Ready");

    // Initialize PID
    beanPID.SetMode(AUTOMATIC);
    beanPID.SetOutputLimits(0, 255);

    // Start Web Server
    server.on("/", handleWebServer);
    server.on("/data", handleDataRequest);
    server.begin();
    DEBUG_PRINTLN("Web Server Started");
}

void loop() {
    ArduinoOTA.handle();
    modbusTCP.task();
    server.handleClient();

    updatePIDParameters();

    int controlMode = modbusTCP.Hreg(REG_CONTROL_MODE); // 0: Manual, 1: Auto
    int heaterOverride = modbusTCP.Hreg(REG_OVERRIDE_HEATER); // 0: Off, 1: On
    fanPWM = modbusTCP.Hreg(REG_FAN_PWM);

    // Update Fan PWM
    ledcWrite(1, fanPWM);

    // Read Temperatures Periodically
    if (millis() - lastTempRead >= tempReadInterval) {
        lastTempRead = millis();
        beanTemperature = beanThermocouple.readTemperature();
        envTemperature = envThermocouple.readTemperature();
        applyCalibration();
    }

    // Update Modbus Registers Periodically
    if (millis() - lastModbusTask >= modbusTaskInterval) {
        lastModbusTask = millis();
        modbusTCP.Hreg(REG_BEAN_TEMP, static_cast<uint16_t>(beanTemperature * 10)); // BT
        modbusTCP.Hreg(REG_ENV_TEMP, static_cast<uint16_t>(envTemperature * 10));  // ET
        modbusTCP.Hreg(REG_HEATER_PWM, static_cast<uint16_t>(heaterOutput));      // Heater PWM
    }

    // Heater Control Logic
    if (heaterOverride == 0) {
        // Heater Override OFF - Always Disable Heater
        heaterOutput = 0;
        ledcWrite(0, 0);
        //DEBUG_PRINTLN("Heater Override Active: Heater Disabled");
    } else {
        // Heater Override ON - Proceed with Normal Control Logic
        if (controlMode == MODE_MANUAL) {
            heaterOutput = modbusTCP.Hreg(REG_HEATER_PWM) / 100.0 * 255;
            if (fanPWM > 100) {
                ledcWrite(0, static_cast<int>(heaterOutput));
            } else {
                ledcWrite(0, 0);
                DEBUG_PRINTLN("Failsafe: Fan too low, heater off.");
            }
        } else if (controlMode == MODE_AUTO) {
            beanSetpoint = modbusTCP.Hreg(REG_BEAN_SP) / 10.0;
            if (!isnan(beanTemperature)) {
                if (millis() - lastPidCompute >= pidComputeInterval) {
                    lastPidCompute = millis();
                    beanPID.Compute();
                }
                if (fanPWM > 100) {
                    ledcWrite(0, static_cast<int>(heaterOutput));
                } else {
                    ledcWrite(0, 0);
                    DEBUG_PRINTLN("Failsafe: Fan too low, heater off.");
                }
            } else {
                DEBUG_PRINTLN("Invalid bean temperature! Heater disabled.");
                ledcWrite(0, 0);
            }
        }
    }

    // Periodic Serial Output
    if (millis() - lastSerialOutput >= serialOutputInterval) {
    lastSerialOutput = millis();
    DEBUG_PRINTLN("Modbus Register Values:");
    DEBUG_PRINTF("0: Bean Temp (BT)        = %d\n", modbusTCP.Hreg(REG_BEAN_TEMP));
    DEBUG_PRINTF("1: Env Temp (ET)         = %d\n", modbusTCP.Hreg(REG_ENV_TEMP));
    DEBUG_PRINTF("2: Heater PWM            = %d\n", modbusTCP.Hreg(REG_HEATER_PWM));
    DEBUG_PRINTF("3: Fan PWM               = %d\n", modbusTCP.Hreg(REG_FAN_PWM));
    DEBUG_PRINTF("4: Bean Setpoint (SP)    = %d\n", modbusTCP.Hreg(REG_BEAN_SP));
    DEBUG_PRINTF("5: Control Mode          = %d\n", modbusTCP.Hreg(REG_CONTROL_MODE));
    DEBUG_PRINTF("6: Heater Override       = %d\n", modbusTCP.Hreg(REG_OVERRIDE_HEATER));
    DEBUG_PRINTF("7: Kp                    = %d\n", modbusTCP.Hreg(REG_KP));
    DEBUG_PRINTF("8: Ki                    = %d\n", modbusTCP.Hreg(REG_KI));
    DEBUG_PRINTF("9: Kd                    = %d\n", modbusTCP.Hreg(REG_KD));
    DEBUG_PRINTF("Mode: %s, BT: %.2f, SP: %.2f, Output: %d, Heater Override: %d\n",
                     (controlMode == MODE_MANUAL) ? "Manual" : "Auto",
                     beanTemperature, beanSetpoint, static_cast<int>(heaterOutput), heaterOverride);
    }
}

void initializePins() {
    pinMode(SSR_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(SSR_PIN, LOW); // SSR off
    digitalWrite(FAN_PIN, LOW); // Fan off
}

void applyCalibration() {
    beanTemperature += beanTempOffset;
    envTemperature += envTempOffset;
}

void connectToWiFi(const char* ssid, const char* password) {
    DEBUG_PRINTLN("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
        delay(500);
        DEBUG_PRINT(".");
    }
    if (WiFi.status() != WL_CONNECTED) {
        DEBUG_PRINTLN("\nWiFi connection failed! Running in offline mode.");
    } else {
        DEBUG_PRINTLN("\nWiFi connected!");
        DEBUG_PRINTF("IP Address: %s\n", WiFi.localIP().toString().c_str());
    }
}

void handleWebServer() {
    String html = "<html><head>";
    html += "<title>ESP32 Coffee Roaster</title>";
    html += "<style>";
    html += "body { font-family: Arial, sans-serif; background-color: #121212; color: #e0e0e0; margin: 0; padding: 0; display: flex; flex-direction: column; align-items: center; }";
    html += "h1 { color: #2196F3; text-align: center; margin-top: 20px; }";
    html += "h2 { color: #64B5F6; text-align: center; margin-bottom: 20px; }";
    html += "table { border-collapse: collapse; width: 60%; margin: 20px auto; background-color: #1e1e1e; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2); }";
    html += "th, td { border: 1px solid #333; padding: 10px; text-align: center; }";
    html += "th { background-color: #2196F3; color: white; font-weight: bold; }";
    html += "td { color: #BBDEFB; }";
    html += "tr:nth-child(even) { background-color: #2c2c2c; }";
    html += "tr:hover { background-color: #37474F; }";
    html += "table, th, td { border-radius: 8px; }";
    html += "footer { margin-top: 20px; color: #757575; font-size: 14px; text-align: center; }";
    html += "</style>";
    html += "</head><body>";
    html += "<h1>ESP32 Coffee Roaster Dashboard</h1>";
    html += "<h2>System Parameters</h2>";
    html += "<table>";
    html += "<tr><th>Register</th><th>Name</th><th>Value</th></tr>";
    html += "<tr><td>0</td><td>Bean Temp (BT)</td><td id='beanTemp'>Loading...</td></tr>";
    html += "<tr><td>1</td><td>Env Temp (ET)</td><td id='envTemp'>Loading...</td></tr>";
    html += "<tr><td>2</td><td>Heater PWM</td><td id='heaterPwm'>Loading...</td></tr>";
    html += "<tr><td>3</td><td>Fan PWM</td><td id='fanPwm'>Loading...</td></tr>";
    html += "<tr><td>4</td><td>Bean Setpoint (SP)</td><td id='setpoint'>Loading...</td></tr>";
    html += "<tr><td>5</td><td>Control Mode</td><td id='controlMode'>Loading...</td></tr>";
    html += "<tr><td>6</td><td>Heater Override</td><td id='heaterOverride'>Loading...</td></tr>";
    html += "<tr><td>7</td><td>Kp</td><td id='Kp'>Loading...</td></tr>";
    html += "<tr><td>8</td><td>Ki</td><td id='Ki'>Loading...</td></tr>";
    html += "<tr><td>9</td><td>Kd</td><td id='Kd'>Loading...</td></tr>";
    html += "</table>";
    html += "<footer>ESP32 Coffee Roaster &copy; 2025</footer>";
    html += "<script>";
    html += "function fetchData() {";
    html += "  fetch('/data')"; // AJAX request to /data endpoint
    html += "    .then(response => response.json())";
    html += "    .then(data => {";
    html += "      document.getElementById('beanTemp').innerHTML = data.beanTemp + ' °C';";
    html += "      document.getElementById('envTemp').innerHTML = data.envTemp + ' °C';";
    html += "      document.getElementById('heaterPwm').innerHTML = data.heaterPwm;";
    html += "      document.getElementById('fanPwm').innerHTML = data.fanPwm;";
    html += "      document.getElementById('setpoint').innerHTML = data.setpoint + ' °C';";
    html += "      document.getElementById('controlMode').innerHTML = data.controlMode;";
    html += "      document.getElementById('heaterOverride').innerHTML = data.heaterOverride;";
    html += "      document.getElementById('Kp').innerHTML = data.Kp;";
    html += "      document.getElementById('Ki').innerHTML = data.Ki;";
    html += "      document.getElementById('Kd').innerHTML = data.Kd;";
    html += "    });";
    html += "}";
    html += "setInterval(fetchData, 1000);"; // Refresh every 1 second
    html += "</script>";
    html += "</body></html>";

    server.send(200, "text/html", html);
}

void handleDataRequest() {
    String json = "{";
    json += "\"beanTemp\":" + String(beanTemperature, 2) + ",";
    json += "\"envTemp\":" + String(envTemperature, 2) + ",";
    json += "\"setpoint\":" + String(beanSetpoint, 2) + ",";
    json += "\"fanPwm\":" + String(fanPWM) + ",";
    json += "\"heaterPwm\":" + String(static_cast<int>(heaterOutput)) + ",";
    json += "\"controlMode\":" + String(modbusTCP.Hreg(REG_CONTROL_MODE)) + ",";
    json += "\"heaterOverride\":" + String(modbusTCP.Hreg(REG_OVERRIDE_HEATER)) + ",";
    json += "\"Kp\":" + String(Kp, 2) + ",";
    json += "\"Ki\":" + String(Ki, 2) + ",";
    json += "\"Kd\":" + String(Kd, 2);
    json += "}";
    server.send(200, "application/json", json);
}

void autoTunePID() {
    DEBUG_PRINTLN("Starting PID Auto-Tune...");
    // Placeholder: Implement PID auto-tuning here
    // This could involve oscillating the heater and capturing responses
    DEBUG_PRINTLN("Auto-Tune Complete!");
}

void updatePIDParameters() {
    double newKp = modbusTCP.Hreg(REG_KP) / 10.0;
    double newKi = modbusTCP.Hreg(REG_KI) / 10.0;
    double newKd = modbusTCP.Hreg(REG_KD) / 10.0;

    if (newKp != Kp || newKi != Ki || newKd != Kd) {
        Kp = newKp;
        Ki = newKi;
        Kd = newKd;
        beanPID.SetTunings(Kp, Ki, Kd);
        DEBUG_PRINTF("PID Parameters Updated: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp, Ki, Kd);
        savePIDParameters();
    }
}

void savePIDParameters() {
    EEPROM.put(0, Kp);
    EEPROM.put(8, Ki);
    EEPROM.put(16, Kd);
    EEPROM.commit();
    DEBUG_PRINTLN("PID Parameters Saved to EEPROM");
}

void loadPIDParameters() {
    EEPROM.get(0, Kp);
    EEPROM.get(8, Ki);
    EEPROM.get(16, Kd);
    beanPID.SetTunings(Kp, Ki, Kd);
    DEBUG_PRINTLN("PID Parameters Loaded from EEPROM");
}
