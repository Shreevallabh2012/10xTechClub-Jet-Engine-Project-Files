/*
  Project: Jet Engine Simulation using ESP32
  Features:
    - Air Quality Monitoring (MQ135)
    - Temperature & Humidity Monitoring (DHT11)
    - Flame Detection
    - Real-time Web Interface via WebSocket
    - DC Motor Control using L298N Motor Driver

  [Future Support]:
    - Can be extended to support BLDC motor via ESC (Electronic Speed Controller)
    - Only one PWM output needed for BLDC control
    - Adjust WebSocket logic and PWM signal range for ESC control
*/

// ========== INCLUDE REQUIRED LIBRARIES ==========
#include <Arduino.h>            // Core Arduino functions
#include <DHT.h>                // DHT sensor library for temperature & humidity
#include <WiFi.h>               // For setting up Wi-Fi access point
#include <WebServer.h>          // To serve files and handle HTTP requests
#include <WebSocketsServer.h>   // For real-time communication using WebSockets
#include <LittleFS.h>           // Filesystem to store and serve HTML, CSS, JS
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneButton.h>


#define DELAYVAL 500 

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const uint8_t bitmap6[] = {
  0x03, 0xfc, 0x00, 0x07, 0xfe, 0x00, 0x0f, 0xfe, 0x00,
  0x07, 0xff, 0x00, 0x03, 0xff, 0x06, 0x00, 0xff, 0x0f,
  0x00, 0xff, 0x1f, 0x00, 0x7e, 0x3f, 0x00, 0x3e, 0x7f,
  0x1f, 0x7f, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xe3, 0xff,
  0xff, 0xe3, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xff, 0x7c,
  0xff, 0x3e, 0x00, 0xfe, 0x3f, 0x00, 0x7c, 0x3f, 0x00,
  0x78, 0x7f, 0x80, 0x30, 0x7f, 0xe0, 0x00, 0x7f, 0xf0,
  0x00, 0x3f, 0xf8, 0x00, 0x3f, 0xf0, 0x00, 0x3f, 0xe0
};

// ========== PIN DEFINITIONS FOR SENSORS & MOTOR ==========
// DHT11 Sensor (Temperature and Humidity)
#define DHTPIN 27   
#define DHTTYPE DHT11

// MQ135 Gas Sensor (Analog pin)
#define MQ135_PIN 34   

// Flame Sensor (Analog pin)
#define FLAME_SENSOR_PIN 33

// L298N Motor Driver Pins
#define MOTOR_EN_PIN 14       // PWM Enable Pin for motor speed
#define MOTOR_IN1_PIN 25      // Motor direction pin 1
#define MOTOR_IN2_PIN 26      // Motor direction pin 2

/*If using a BLDC motor with ESC (Electronic Speed Controller),only a single PWM control pin (e.g., MOTOR_BLDC_PIN) will be required,and comment the L298 Motor driver part
#define MOTOR_BLDC_PIN 14
*/
#define FLOW_PIXELS_PIN 12  // First NeoPixel LED strip (Motor status with animation)
#define STATIC_PIXELS_PIN 32  // Second NeoPixel LED strip (Always ON)

#define NUM_FLOW_PIXELS 7
#define NUM_STATIC_PIXELS 6

Adafruit_NeoPixel flowPixels(NUM_FLOW_PIXELS, FLOW_PIXELS_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel staticPixels(NUM_STATIC_PIXELS, STATIC_PIXELS_PIN, NEO_GRB + NEO_KHZ800);

unsigned long lastFlowUpdate = 0;
int currentFlowPixel = NUM_FLOW_PIXELS - 1;  // Start from pixel 6

// Load resistance in kilo-ohms used for MQ135 calculation
#define RL_VALUE 5   
int currentWebSpeed = 0;  // Store last web speed for reference
const int BUTTON1_PIN = 4 ;  // Green - Start Engine / Mode Switch
const int BUTTON2_PIN = 5 ;  // Yellow - Stop in Manual Mode
const int BUTTON3_PIN = 17 ;  // Red - Emergency Stop

#define POT_PIN 35
#define RED_LED_PIN 18
#define GREEN_LED_PIN 19
#define BUZZER_PIN 15

bool engineRunning = false;
bool isManualMode = true;
String statusMessage = "";
OneButton button1(BUTTON1_PIN, true);  // Start Engine / Mode Switch
OneButton button2(BUTTON2_PIN, true);  // Stop in Manual Mode
OneButton button3(BUTTON3_PIN, true);  // Emergency Stop

// ========== OBJECT CREATION ==========
DHT dht(DHTPIN, DHTTYPE);       // Initialize DHT sensor
WebServer server(80);           // HTTP server on port 80
WebSocketsServer webSocket(81); // WebSocket server on port 81
float R0 = 10.00;               // MQ135 baseline resistance (calibrated)

// ========== WI-FI ACCESS POINT CREDENTIALS ==========
const char* ssid = "Jet Engine";    // SSID name of ESP32 AP    
const char* password = "12345678";  // Password of ESP32 AP

// ========== AQI BREAKPOINT TABLES FOR DIFFERENT GASES ==========
// These tables are used to convert gas concentrations into AQI values
float breakpoints_CO2[][2]      = {{0, 0}, {1000, 50}, {2000, 100}, {3000, 150}, {4000, 200}, {5000, 300}, {6000, 400}, {7000, 500}};
float breakpoints_NH3[][2]      = {{0, 0}, {100, 50}, {200, 100}, {300, 150}, {400, 200}, {500, 300}, {600, 400}, {700, 500}};
float breakpoints_NOx[][2]      = {{0, 0}, {50, 50}, {100, 100}, {150, 150}, {200, 200}, {300, 300}, {400, 400}, {500, 500}};
float breakpoints_Alcohol[][2]  = {{0, 0}, {50, 50}, {100, 100}, {150, 150}, {200, 200}, {300, 300}, {400, 400}, {500, 500}};
float breakpoints_Benzene[][2]  = {{0, 0}, {1.6, 50}, {3.2, 100}, {4.8, 150}, {6.4, 200}, {8.0, 300}, {9.6, 400}, {11.2, 500}};
float breakpoints_Smoke[][2]    = {{0, 0}, {50, 50}, {100, 100}, {150, 150}, {200, 200}, {300, 300}, {400, 400}, {500, 500}};

//====================UTILITY FUNCTION===============
// Calculates sensor resistance from ADC value using MQ135's formula
float calculateResistance(int adcVal) {
  return ((4095.0 / adcVal) - 1.0) * RL_VALUE;
}
// Calculates gas concentration using log-log characteristics
float calculateConcentration(float ratio, float slope, float intercept) {
  return pow(10, ((log10(ratio) - intercept) / slope));
}
// Maps gas concentration to AQI using standard breakpoint ranges
float calculateAQI(float conc, float breakpoints[][2], int len) {
  for (int i = 0; i < len - 1; i++) {
    if (conc >= breakpoints[i][0] && conc < breakpoints[i + 1][0]) {
      float AQI_low = breakpoints[i][1], AQI_high = breakpoints[i + 1][1];
      float conc_low = breakpoints[i][0], conc_high = breakpoints[i + 1][0];
      return ((AQI_high - AQI_low) / (conc_high - conc_low)) * (conc - conc_low) + AQI_low;
    }
  }
  return -1;      // Return -1 if out of range
}

//===============FILE HANDLING FOR WEB UI==============
// Handles reading and serving files stored in LittleFS
// Converts file extension to content-type for browser rendering
bool handleFileRead(String path) {
  if (path.endsWith("/")) path += "index.html";   // Default route
  String contentType = "text/plain";
  if (path.endsWith(".html")) contentType = "text/html";
  else if (path.endsWith(".css")) contentType = "text/css";
  else if (path.endsWith(".js")) contentType = "application/javascript";
  File file = LittleFS.open(path, "r");
  if (file) {
    server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;   // File not found
}

void updateFlowingLED() {
  static unsigned long previousMillis = 0;
  const long interval = 100;

  if (millis() - previousMillis >= interval) {
    previousMillis = millis();

    flowPixels.clear();

    flowPixels.setPixelColor(currentFlowPixel, flowPixels.Color(255, 100, 0));

    int trailLength = 3;
    for (int i = 1; i <= trailLength; i++) {
      int trailIndex = currentFlowPixel + i;
      if (trailIndex < NUM_FLOW_PIXELS) {
        uint8_t fade = 255 / (i + 1);
        flowPixels.setPixelColor(trailIndex, flowPixels.Color(fade, fade / 2, 0));
      }
    }

    flowPixels.show();

    currentFlowPixel--;
    if (currentFlowPixel < 0) {
      currentFlowPixel = NUM_FLOW_PIXELS - 1;
    }
  }
}
//==================WEBSOCKET LOGIC============================
// Handles WebSocket events like new connection or messages from client
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    IPAddress ip = webSocket.remoteIP(num);
    Serial.printf("[%u] Connected: %s\n", num, ip.toString().c_str());
    webSocket.sendTXT(num, "Connected to ESP32 WebSocket");
  }
  else if (type == WStype_TEXT) {
    String message = String((char*)payload);
    Serial.println("WebSocket Message Received: " + message);

    if (message.startsWith("MOTOR_SPEED:")) {
      if (!isManualMode) {
        int speed = message.substring(12).toInt();
        int pwmVal = map(speed, 0, 100, 80, 255);
        
        // Motor control logic for Web mode
        digitalWrite(MOTOR_IN1_PIN, HIGH);
        digitalWrite(MOTOR_IN2_PIN, LOW);
        ledcWrite(0, pwmVal);
        statusMessage = "Web Mode: Speed set to " + String(speed);
        Serial.println(statusMessage);
      }
      else {
        webSocket.sendTXT(num, "Manual mode active. Web control is disabled.");
      }
    }
  }
}
void playLoadingMusic() {
  tone(BUZZER_PIN, 440, 200); delay(220);   // A4
  tone(BUZZER_PIN, 494, 200); delay(220);   // B4
  tone(BUZZER_PIN, 523, 200); delay(220);   // C5
  tone(BUZZER_PIN, 659, 200); delay(220);   // E5
  tone(BUZZER_PIN, 784, 300); delay(320);   // G5
  noTone(BUZZER_PIN);
}



void updateDisplay(int barWidth) {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setFont(NULL);
  display.setCursor(32, 32);
  display.println("JET ENGINE ");
  display.drawBitmap(49, 3, bitmap6, 24, 24, 1);
  display.setCursor(2, 42);
  display.println("Manual Control System");
  
  display.drawRect(2, 51, 124, 10, 1);
  display.setTextColor(BLACK, WHITE);
  display.fillRoundRect(2, 51, barWidth, 10, 3, 1);

  display.display();
}



void toggleMode() {
  isManualMode = !isManualMode;
  if (isManualMode) {
    Serial.println("Switched to Manual Mode");
    analogWrite(MOTOR_EN_PIN, 0);  // Stop motor in Manual Mode
  } else {
    Serial.println("Switched to Web Mode");
  }
}
void startEngine() {
  if (isManualMode) {
    engineRunning = true;
    Serial.println("Engine started in Manual Mode");
    statusMessage = "Engine ON (Manual)";
    updateFlowingLED();
  }
}
void stopEngine() {
  if (isManualMode) {
    engineRunning = false;
    analogWrite(MOTOR_EN_PIN, 0);
    Serial.println("Engine stopped manually");
    statusMessage = "Engine OFF (Manual)";
  }
}

void emergencyStop() {
  engineRunning = false;
  analogWrite(MOTOR_EN_PIN, 0);
  Serial.println("Emergency Stop Activated");
  statusMessage = "EMERGENCY STOP";
}

// ========== RPM CODE INTEGRATION ==========
// IR Sensor for RPM measurement using a falling edge detection
const int irPin = 16;      // IR sensor output connected to digital pin 2
int pulseCount = 0;
bool pulseDetected = false;
unsigned long lastTime = 0;

//========SENSOR DATA COLLECTION AND WEBSOCKET BROADCAST===============
void updateSensorData() {
  // ----- MQ135 Gas Sensor -----
  int sensorValue = analogRead(MQ135_PIN);
  float Rs = calculateResistance(sensorValue);
  float ratio = Rs / R0;

  // Estimate PPM for each gas
  float ppm_CO2      = calculateConcentration(ratio, -0.42, 0.37);
  float ppm_NH3      = calculateConcentration(ratio, -0.45, 0.35);
  float ppm_NOx      = calculateConcentration(ratio, -0.40, 0.30);
  float ppm_Alcohol  = calculateConcentration(ratio, -0.35, 0.40);
  float ppm_Benzene  = calculateConcentration(ratio, -0.42, 0.29);
  float ppm_Smoke    = calculateConcentration(ratio, -0.50, 0.31);

  // Convert to AQI using breakpoint tables
  float AQI_CO2      = calculateAQI(ppm_CO2, breakpoints_CO2, 8);
  float AQI_NH3      = calculateAQI(ppm_NH3, breakpoints_NH3, 8);
  float AQI_NOx      = calculateAQI(ppm_NOx, breakpoints_NOx, 8);
  float AQI_Alcohol  = calculateAQI(ppm_Alcohol, breakpoints_Alcohol, 8);
  float AQI_Benzene  = calculateAQI(ppm_Benzene, breakpoints_Benzene, 8);
  float AQI_Smoke    = calculateAQI(ppm_Smoke, breakpoints_Smoke, 8);

  float finalAQI = max({AQI_CO2, AQI_NH3, AQI_NOx, AQI_Alcohol, AQI_Benzene, AQI_Smoke});

  // ----- DHT11 -----
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
  if (isnan(temp) || isnan(humidity)) return;
  Serial.print("Temperature");
  Serial.println(temp);
  Serial.print("Humidity");
  Serial.println(humidity);

  // ----- Flame Sensor -----
  int flameAnalog = analogRead(FLAME_SENSOR_PIN);
  Serial.print("flameAnalog:");
  Serial.println(flameAnalog);
  String flameStatus = (flameAnalog < 10) ? "1" : "0";
  
  // ----- Send Sensor Data via WebSocket -----
  String data = "Temp:" + String(temp) +
                ",Humidity:" + String(humidity) +
                ",AQI:" + String(finalAQI) +
                ",NH3:" + String(ppm_NH3) +
                ",NOx:" + String(ppm_NOx) +
                ",Alcohol:" + String(ppm_Alcohol) +
                ",Benzene:" + String(ppm_Benzene) +
                ",Smoke:" + String(ppm_Smoke) +
                ",CO2:" + String(ppm_CO2) +
                ",Flame:" + flameStatus;
                //",RPM:" + String(rpm);
  webSocket.broadcastTXT(data);   // Real-time update to connected clients
 
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(29, 3);
  display.println("SENSOR DATA");
  display.setCursor(2, 12);
  display.print("Temp: "); display.print(temp); display.println(" C");
  display.print("Humidity: "); display.print(humidity); display.println(" %");
  display.print("AQI: "); display.println(finalAQI);
  display.print("Mode: ");
  display.println(isManualMode ? "Manual" : "Web");

  // Show status message at bottom
  if (statusMessage != "") {
    display.setCursor(25, 54);
    display.println(statusMessage);
  }

  display.display();


}

// ======SETUP FUNCTION========
void setup() {
  Serial.begin(115200);           // Start Serial Monitor
  dht.begin();                    // Start Serial Monitor
  playLoadingMusic();

  
  // Set sensor pin modes
  pinMode(MQ135_PIN, INPUT);
  pinMode(FLAME_SENSOR_PIN, INPUT);

  //-------L298N DC Motor Driver setup--------
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
 
  button2.attachLongPressStart(toggleMode);  // Long press to switch mode
  button1.attachClick(startEngine);          // Single click = start engine in Manual mode
  button2.attachClick(stopEngine);           // Stop in Manual mode
  button3.attachClick(emergencyStop);        // Emergency Stop

  // Manual Control
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(RED_LED_PIN, HIGH);  // Red LED on startup
  digitalWrite(GREEN_LED_PIN, LOW);  // Green LED off
  digitalWrite(BUZZER_PIN, LOW);

  // OLED INIT
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed");
    while (true);
  }
  for (int i = 0; i <= 124; i += 2) {
    updateDisplay(i);
    delay(30);
  }

  delay(1000); // Optional pause after loading
/*If using BLDC with ESC:
//ESCs typically require PWM signals between 1000us (stop) to 2000us (full speed).You can use the Servo library or custom PWM to control it.
ledcWrite(0, map(speed, 0, 100, 52, 102)); // 52 ≈ 1000us, 102 ≈ 2000us for 8-bit
*/

  // PWM setup for EN pin (Channel 0, 5kHz, 8-bit resolution)
  ledcSetup(0, 5000, 8);          // Channel 0, 5 KHz PWM, 8-bit resolution
  ledcAttachPin(MOTOR_EN_PIN, 0); // Attach motor enable pin to channel 0

   
  flowPixels.begin();
  flowPixels.clear();
  flowPixels.show();
  
  staticPixels.begin();
  staticPixels.show();
  
  // Set static LED pattern: 3 blue and 3 red
  for (int i = 0; i < 3; i++) {
    staticPixels.setPixelColor(i, staticPixels.Color(0, 0, 255));  // Blue
  }
  for (int i = 3; i < 6; i++) {
    staticPixels.setPixelColor(i, staticPixels.Color(255, 0, 0));  // Red
  }
  staticPixels.show();

  // ----- Initialize File System -----
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed!");
    return;
  }

  // ----- Start Wi-Fi Access Point -----
  IPAddress local_IP(192, 168, 1, 125);
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);
  Serial.println("WiFi AP Started: " + WiFi.softAPIP().toString());

  // ----- WebSocket Initialization -----
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // ----- HTTP Server Setup -----
  server.on("/", HTTP_GET, []() {
    if (!handleFileRead("/index.html")) {
      server.send(404, "text/plain", "Not Found");
    }
  });
  server.onNotFound([]() {
    if (!handleFileRead(server.uri())) {
      server.send(404, "text/plain", "Not Found");
    }
    
  });
  server.begin(); // Start HTTP server

 // Attach double click handler
 button1.attachLongPressStart([]() {
  isManualMode = !isManualMode;  // Toggle mode
  String modeMsg = isManualMode ? "Switched to MANUAL Mode" : "Switched to WEB Mode";
  Serial.println(modeMsg);
});
}


void showMessage(String message) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setFont(NULL);
  display.setCursor(21, 45);
  display.println(message);
  display.display();

}


// ====== LOOP FUNCTION ======
void loop() {
  // WebSocket and server handlers
  webSocket.loop();
  server.handleClient();
  updateSensorData();
  updateFlowingLED();

  // Fixed delay (can be replaced with millis-based logic)
  delay(600);

  // ---- Manual Jet Engine Control ----
  static bool manualEngineOn = false;

  // Start Engine Button
  if (digitalRead(BUTTON1_PIN) == LOW) {
    manualEngineOn = true;
    statusMessage = "ENGINE STARTED";
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("Manual Engine Start Triggered");
  }
  // Stop Engine Button
  if (digitalRead(BUTTON2_PIN) == LOW) {
    manualEngineOn = false;
    statusMessage = "ENGINE STOP";
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("Manual Engine Stop Triggered");
  
  }

  // Emergency Stop Button
  if (digitalRead(BUTTON3_PIN) == HIGH) {
    manualEngineOn = false;
    statusMessage = "EMERGENCY STOP";
    digitalWrite(RED_LED_PIN, HIGH);  // Blink Red LED
    digitalWrite(GREEN_LED_PIN, LOW); // Turn off Green LED
    digitalWrite(BUZZER_PIN, HIGH);   // Sound Buzzer
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }

  // If engine is manually started
  if (manualEngineOn) {
    int potValue = analogRead(POT_PIN);  // Read potentiometer (0–4095)
    int pwmValue = map(potValue, 0, 2350, 150, 255); // Limit min PWM for startup
    Serial.printf("POT VALUE: %d\n", potValue);
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    ledcWrite(0, pwmValue);
    Serial.printf("Manual PWM: %d\n", pwmValue);
  } else {
    ledcWrite(0, 0);  // Stop motor if not in manual mode
  }
  


  // If in manual mode and engine is running
  if (isManualMode && engineRunning) {
    int potValue = analogRead(POT_PIN);
    int pwmValue = map(potValue, 0, 4095, 0, 255);
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    analogWrite(MOTOR_EN_PIN, pwmValue);
  }

  // Process button events (for OneButton library)
  button1.tick();
  button2.tick();
  button3.tick();
}

