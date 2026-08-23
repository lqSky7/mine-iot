#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_NeoPixel.h>
#include <DHT.h>
#include <vector>

// ==========================================
// NODE IDENTITY SELECTOR
// ==========================================
// Set NODE_IDENTITY to:
//   1 -> Configures node as esp32_sensor_node_1 (ESP-NODE-01) [FITTED WITH WS2812 MATRIX & BUZZER]
//   2 -> Configures node as esp32_sensor_node_2 (ESP-NODE-02) [SENSOR-ONLY NODE]
#define NODE_IDENTITY 1

#if NODE_IDENTITY == 1
  const char* mqtt_client_id = "esp32_sensor_node_1";
  const char* wifi_hostname  = "esp32-sensor-node-1";
  #define HAS_ACTUATORS 1 // Fitted with Pin 15 (WS2812) & Pin 19 (Buzzer)
#elif NODE_IDENTITY == 2
  const char* mqtt_client_id = "esp32_sensor_node_2";
  const char* wifi_hostname  = "esp32-sensor-node-2";
  #define HAS_ACTUATORS 0 // Pure sensor node
#else
  #error "Invalid NODE_IDENTITY. Choose 1 or 2."
#endif

// ==========================================
// WIFI & MQTT CONFIGURATION
// ==========================================

// Wi-Fi
const char* ssid     = "Pi4B-Hotspot";
const char* password = "abcdefgh";

// MQTT Broker & Topics
const char* mqtt_server         = "10.42.0.1";
const int   mqtt_port           = 1883;
const char* mqtt_topic_telemetry = "esp32/sensor_data";
const char* mqtt_topic_commands  = "esp32/commands";
const char* mqtt_topic_alarm     = "esp32/alarm";

// ==========================================
// PIN CONFIGURATION
// ==========================================

// SW-420 vibration sensor
const int SW420_PIN = 13;

// WS2812 64-bit (8x8) RGB LED Matrix
const int WS2812_PIN = 15;
const int NUM_PIXELS = 64; // 8x8 Matrix

// High-Decibel Piezo Buzzer
const int BUZZER_PIN = 19;

// I2C Pins (ADXL345 & MPU6050)
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// HC-SR04 Ultrasonic Sensor #1
const int HCSR04_1_TRIG_PIN = 25;
const int HCSR04_1_ECHO_PIN = 26;

// HC-SR04 Ultrasonic Sensor #2
const int HCSR04_2_TRIG_PIN = 32;
const int HCSR04_2_ECHO_PIN = 35;

// MQ-2 analog output
const int MQ2_PIN = 34;

// DHT11
const int DHT_PIN = 33;
#define DHTTYPE DHT11

// ==========================================
// I2C ADDRESSES
// ==========================================
const uint8_t ADXL345_ADDRESS = 0x53;
const uint8_t MPU6050_ADDRESS = 0x69;

// ==========================================
// PWM BUZZER CHANNELS (ESP32 LEDC - Core v2 & v3 Compatible)
// ==========================================
#include <esp_arduino_version.h>

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define USE_ESP32_CORE_V3 1
#else
  #define USE_ESP32_CORE_V3 0
#endif

const int BUZZER_PWM_CHANNEL = 0;
const int BUZZER_PWM_FREQ    = 2800; // 2.8 kHz audible resonance
const int BUZZER_PWM_RES     = 8;    // 8-bit resolution (0-255)

// ==========================================
// SENSOR SETTINGS & SAFETY THRESHOLDS
// ==========================================
const unsigned long SENSOR_INTERVAL    = 5000;
const unsigned long RECONNECT_INTERVAL = 10000;

// Local safety fallback thresholds (auto-trigger when offline)
const int   LOCAL_GAS_CRITICAL_ADC    = 2800; // ~700 ppm
const float LOCAL_DIST_CRITICAL_CM    = 1.5;  // 1.5 cm wall collapse
const float LOCAL_TILT_CRITICAL_DEG   = 18.0; // 18 deg tilt

// ==========================================
// 8x8 MATRIX PATTERNS (Bitmaps: 8 rows of uint8_t)
// ==========================================
enum LedPattern {
  PAT_IDLE = 0,
  PAT_NORMAL_CHECK,
  PAT_WARNING_PULSE,
  PAT_DANGER_FLASH,
  PAT_EVACUATE_ARROW
};

const uint8_t BITMAP_IDLE[8] = {
  0b00000000,
  0b00000000,
  0b00111100,
  0b00100100,
  0b00100100,
  0b00111100,
  0b00000000,
  0b00000000
};

const uint8_t BITMAP_CHECK[8] = {
  0b00000000,
  0b00000001,
  0b00000010,
  0b00000100,
  0b10001000,
  0b01010000,
  0b00100000,
  0b00000000
};

const uint8_t BITMAP_WARN[8] = {
  0b00011000,
  0b00111100,
  0b00111100,
  0b01100110,
  0b01100110,
  0b11111111,
  0b00011000,
  0b00011000
};

const uint8_t BITMAP_DANGER[8] = {
  0b11000011,
  0b11100111,
  0b01111110,
  0b00111100,
  0b00111100,
  0b01111110,
  0b11100111,
  0b11000011
};

const uint8_t BITMAP_EVAC[8] = {
  0b00001000,
  0b00001100,
  0b00001110,
  0b11111111,
  0b11111111,
  0b00001110,
  0b00001100,
  0b00001000
};

// ==========================================
// DATA STRUCTURE
// ==========================================
struct DataPoint {
  uint32_t seq;
  uint32_t timestamp_ms;
  int vibration;
  float adxl_ax, adxl_ay, adxl_az;
  float mpu_ax, mpu_ay, mpu_az;
  float mpu_gx, mpu_gy, mpu_gz;
  float distance_cm_1;
  float distance_cm_2;
  int mq2_raw;
  float temperature;
  float humidity;
  int buzzer_state;
  int matrix_pattern;
};

// ==========================================
// GLOBAL OBJECTS & STATE
// ==========================================
const size_t MAX_BUFFER_SIZE = 1000;
std::vector<DataPoint> dataBuffer;
uint32_t sequenceNumber = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

Adafruit_MPU6050 mpu;
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);
DHT dht(DHT_PIN, DHTTYPE);
Adafruit_NeoPixel matrix(NUM_PIXELS, WS2812_PIN, NEO_GRB + NEO_KHZ800);

bool mpu_initialized  = false;
bool adxl_initialized = false;
volatile bool vibrationTriggered = false;

// Actuator Runtime State
LedPattern currentPattern     = PAT_NORMAL_CHECK;
bool buzzerActive             = false;
unsigned long buzzerAutoOffAt = 0;
unsigned long lastAnimUpdate  = 0;
bool animFlashState           = false;
uint8_t pulseBrightness       = 40;
int pulseDirection            = 2;

unsigned long lastSensorReadTime  = 0;
unsigned long lastReconnectAttempt = 0;

// ==========================================
// FUNCTION DECLARATIONS
// ==========================================
void setupWiFi();
void connectToMQTT();
void onMqttMessage(char* topic, byte* payload, unsigned int length);
float readDistance(int trigPin, int echoPin);
void readSensors(
  int &vib,
  float &adxl_ax, float &adxl_ay, float &adxl_az,
  float &mpu_ax, float &mpu_ay, float &mpu_az,
  float &mpu_gx, float &mpu_gy, float &mpu_gz,
  float &dist1, float &dist2,
  int &mq2_raw,
  float &temp, float &hum
);
bool publishDataPoint(const DataPoint &dp);
void processQueue();

void setMatrixPattern(LedPattern pat);
void setBuzzer(bool active, unsigned long durationMs = 0);
void updateActuatorAnimations();
void renderBitmap(const uint8_t bitmap[8], uint32_t color, uint8_t brightness = 50);
uint16_t getPixelIndex(int row, int col);

// ==========================================
// SW-420 VIBRATION INTERRUPT
// ==========================================
void IRAM_ATTR handleVibrationInterrupt() {
  vibrationTriggered = true;
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("==================================================");
  Serial.printf(" ESP32 SENSOR NODE (IDENTITY: %s)\n", mqtt_client_id);
  Serial.println(" ACTUATORS: WS2812 64-bit Matrix (GPIO 15) | Buzzer (GPIO 19)");
  Serial.println(" SENSORS: ADXL345 + MPU6050 + SW-420 + 2x HC-SR04 + MQ-2 + DHT11");
  Serial.println("==================================================");

  // 1. Initialize Actuators (NODE_IDENTITY 1 only)
#if HAS_ACTUATORS
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  #if USE_ESP32_CORE_V3
    ledcAttach(BUZZER_PIN, BUZZER_PWM_FREQ, BUZZER_PWM_RES);
    ledcWrite(BUZZER_PIN, 0); // Buzzer silent initially
  #else
    ledcSetup(BUZZER_PWM_CHANNEL, BUZZER_PWM_FREQ, BUZZER_PWM_RES);
    ledcAttachPin(BUZZER_PIN, BUZZER_PWM_CHANNEL);
    ledcWrite(BUZZER_PWM_CHANNEL, 0); // Buzzer silent initially
  #endif

  matrix.begin();
  matrix.setBrightness(40);
  matrix.show(); // Clear all pixels
  setMatrixPattern(PAT_NORMAL_CHECK);
#else
  Serial.println("[NODE] Pure sensor mode (no actuators on this node).");
#endif

  // 2. Initialize SW-420 Vibration Sensor
  pinMode(SW420_PIN, INPUT);
  attachInterrupt(
    digitalPinToInterrupt(SW420_PIN),
    handleVibrationInterrupt,
    RISING
  );
  Serial.printf("[SW420] Interrupt on GPIO %d\n", SW420_PIN);

  // 3. Initialize Dual HC-SR04 Ultrasonic Sensors
  pinMode(HCSR04_1_TRIG_PIN, OUTPUT);
  pinMode(HCSR04_1_ECHO_PIN, INPUT);
  digitalWrite(HCSR04_1_TRIG_PIN, LOW);

  pinMode(HCSR04_2_TRIG_PIN, OUTPUT);
  pinMode(HCSR04_2_ECHO_PIN, INPUT);
  digitalWrite(HCSR04_2_TRIG_PIN, LOW);
  Serial.printf("[HCSR04] #1 (TRIG=%d, ECHO=%d) | #2 (TRIG=%d, ECHO=%d)\n",
    HCSR04_1_TRIG_PIN, HCSR04_1_ECHO_PIN, HCSR04_2_TRIG_PIN, HCSR04_2_ECHO_PIN);

  // 4. Initialize MQ-2 Gas Sensor
  pinMode(MQ2_PIN, INPUT);
  analogSetPinAttenuation(MQ2_PIN, ADC_11db);
  Serial.printf("[MQ2] Analog ADC on GPIO %d\n", MQ2_PIN);

  // 5. Initialize DHT11
  dht.begin();
  Serial.printf("[DHT11] Temperature/Humidity on GPIO %d\n", DHT_PIN);

  // 6. Initialize I2C Bus & Inertial Sensors
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Serial.printf("[I2C] SDA = GPIO %d | SCL = GPIO %d\n", SDA_PIN, SCL_PIN);

  if (adxl.begin(ADXL345_ADDRESS)) {
    Serial.println("[SUCCESS] ADXL345 initialized at 0x53");
    adxl_initialized = true;
    adxl.setRange(ADXL345_RANGE_16_G);
    adxl.setDataRate(ADXL345_DATARATE_100_HZ);
  } else {
    Serial.println("[ERROR] ADXL345 not detected!");
  }

  if (mpu.begin(MPU6050_ADDRESS)) {
    Serial.println("[SUCCESS] MPU6050 initialized at 0x69");
    mpu_initialized = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  } else {
    Serial.println("[ERROR] MPU6050 at 0x69 not detected!");
  }

  // 7. Initialize Wi-Fi & MQTT
  setupWiFi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(onMqttMessage);
  mqttClient.setBufferSize(1024);

#if HAS_ACTUATORS
  // Startup Beep Confirmation
  setBuzzer(true, 150);
#endif

  Serial.println("System fully armed and operational.");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  unsigned long now = millis();

#if HAS_ACTUATORS
  // 1. Actuator Animation & Failsafe Timer Update (Non-blocking)
  updateActuatorAnimations();
#endif

  // 2. Sensor Sampling Routine
  if (now - lastSensorReadTime >= SENSOR_INTERVAL || lastSensorReadTime == 0) {
    lastSensorReadTime = now;

    int vib = 0;
    float adxl_ax = 0.0, adxl_ay = 0.0, adxl_az = 0.0;
    float mpu_ax = 0.0, mpu_ay = 0.0, mpu_az = 0.0;
    float mpu_gx = 0.0, mpu_gy = 0.0, mpu_gz = 0.0;
    float dist1 = -1.0;
    float dist2 = -1.0;
    int mq2_raw = 0;
    float temp = NAN;
    float hum = NAN;

    readSensors(
      vib,
      adxl_ax, adxl_ay, adxl_az,
      mpu_ax, mpu_ay, mpu_az,
      mpu_gx, mpu_gy, mpu_gz,
      dist1, dist2,
      mq2_raw,
      temp, hum
    );

    // Compute Local Hazard Check (Autonomous safety when disconnected)
    float activeDist = (dist1 > 0 && dist2 > 0) ? min(dist1, dist2) : (dist1 > 0 ? dist1 : dist2);
    bool criticalHazard = (mq2_raw >= LOCAL_GAS_CRITICAL_ADC) ||
                          (activeDist > 0 && activeDist <= LOCAL_DIST_CRITICAL_CM);

    if (criticalHazard) {
      setMatrixPattern(PAT_DANGER_FLASH);
      setBuzzer(true, 3000); // 3-second alarm burst
    }

    sequenceNumber++;
    DataPoint dp = {
      sequenceNumber,
      now,
      vib,
      adxl_ax, adxl_ay, adxl_az,
      mpu_ax, mpu_ay, mpu_az,
      mpu_gx, mpu_gy, mpu_gz,
      dist1, dist2,
      mq2_raw,
      temp, hum,
      buzzerActive ? 1 : 0,
      (int)currentPattern
    };

    if (dataBuffer.size() >= MAX_BUFFER_SIZE) {
      dataBuffer.erase(dataBuffer.begin());
    }
    dataBuffer.push_back(dp);

    Serial.printf(
      "[TELEMETRY] #%lu | Vib=%d | Gas=%d | Dist=[%.1f, %.1f] cm | Temp=%.1f C | Buzzer=%d | Matrix=%d\n",
      (unsigned long)dp.seq, dp.vibration, dp.mq2_raw, dp.distance_cm_1, dp.distance_cm_2,
      dp.temperature, dp.buzzer_state, dp.matrix_pattern
    );
  }

  // 3. Network Connection Management
  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastReconnectAttempt >= RECONNECT_INTERVAL) {
      lastReconnectAttempt = now;
      setupWiFi();
    }
  } else if (!mqttClient.connected()) {
    if (now - lastReconnectAttempt >= RECONNECT_INTERVAL) {
      lastReconnectAttempt = now;
      connectToMQTT();
    }
  }

  // 4. MQTT Processing & Command Dispatch
  if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    mqttClient.loop();
    processQueue();
  }
}

// ==========================================
// ACTUATOR CONTROL & WS2812 MATRIX RENDERING
// ==========================================

// Maps (row, col) to 1D WS2812 index. Supports progressive and serpentine layouts.
uint16_t getPixelIndex(int row, int col) {
  // Standard progressive row-major mapping (8x8)
  return (row * 8) + col;
  // If your physical matrix is serpentine zigzag, use:
  // return (row % 2 == 0) ? (row * 8 + col) : (row * 8 + (7 - col));
}

void renderBitmap(const uint8_t bitmap[8], uint32_t color, uint8_t brightness) {
  matrix.setBrightness(brightness);
  for (int r = 0; r < 8; r++) {
    uint8_t rowBits = bitmap[r];
    for (int c = 0; c < 8; c++) {
      int index = getPixelIndex(r, c);
      if ((rowBits >> (7 - c)) & 0x01) {
        matrix.setPixelColor(index, color);
      } else {
        matrix.setPixelColor(index, 0); // Off
      }
    }
  }
  matrix.show();
}

void setMatrixPattern(LedPattern pat) {
  currentPattern = pat;
  switch (pat) {
    case PAT_NORMAL_CHECK:
      renderBitmap(BITMAP_CHECK, matrix.Color(0, 255, 0), 45); // Vivid Green Checkmark
      break;
    case PAT_WARNING_PULSE:
      renderBitmap(BITMAP_WARN, matrix.Color(255, 140, 0), 50); // Amber Warning Beacon
      break;
    case PAT_DANGER_FLASH:
      renderBitmap(BITMAP_DANGER, matrix.Color(255, 0, 0), 75); // High-Intensity Red Hazard X
      break;
    case PAT_EVACUATE_ARROW:
      renderBitmap(BITMAP_EVAC, matrix.Color(255, 60, 0), 65);  // Emergency Evac Arrow
      break;
    case PAT_IDLE:
    default:
      renderBitmap(BITMAP_IDLE, matrix.Color(0, 40, 100), 20); // Dim Blue Standby
      break;
  }
}

void setBuzzer(bool active, unsigned long durationMs) {
#if HAS_ACTUATORS
  buzzerActive = active;
  if (active) {
    #if USE_ESP32_CORE_V3
      ledcWrite(BUZZER_PIN, 128); // 50% duty cycle 2.8 kHz tone (ESP32 Core v3)
    #else
      ledcWrite(BUZZER_PWM_CHANNEL, 128); // 50% duty cycle 2.8 kHz tone (ESP32 Core v2)
    #endif
    if (durationMs > 0) {
      buzzerAutoOffAt = millis() + durationMs;
    } else {
      buzzerAutoOffAt = 0; // Continuous until explicit clear
    }
  } else {
    #if USE_ESP32_CORE_V3
      ledcWrite(BUZZER_PIN, 0);   // Silence buzzer (ESP32 Core v3)
    #else
      ledcWrite(BUZZER_PWM_CHANNEL, 0);   // Silence buzzer (ESP32 Core v2)
    #endif
    buzzerAutoOffAt = 0;
  }
#else
  (void)active;
  (void)durationMs;
#endif
}

void updateActuatorAnimations() {
  unsigned long now = millis();

  // Safety auto-silence timer for buzzer
  if (buzzerActive && buzzerAutoOffAt > 0 && now >= buzzerAutoOffAt) {
    setBuzzer(false);
    Serial.println("[ACTUATOR] Buzzer safety auto-silenced.");
  }

  // Visual Animation Handler
  if (currentPattern == PAT_DANGER_FLASH) {
    // 300 ms urgent flash rate
    if (now - lastAnimUpdate >= 300) {
      lastAnimUpdate = now;
      animFlashState = !animFlashState;
      if (animFlashState) {
        renderBitmap(BITMAP_DANGER, matrix.Color(255, 0, 0), 80);
      } else {
        matrix.clear();
        matrix.show();
      }
    }
  } else if (currentPattern == PAT_WARNING_PULSE) {
    // Smooth breathing amber pulse (50 ms step)
    if (now - lastAnimUpdate >= 50) {
      lastAnimUpdate = now;
      pulseBrightness += pulseDirection;
      if (pulseBrightness >= 75 || pulseBrightness <= 15) {
        pulseDirection = -pulseDirection;
      }
      renderBitmap(BITMAP_WARN, matrix.Color(255, 140, 0), pulseBrightness);
    }
  } else if (currentPattern == PAT_EVACUATE_ARROW) {
    // Fast pulsing evacuation directional beacon (200 ms)
    if (now - lastAnimUpdate >= 200) {
      lastAnimUpdate = now;
      animFlashState = !animFlashState;
      if (animFlashState) {
        renderBitmap(BITMAP_EVAC, matrix.Color(255, 50, 0), 75);
      } else {
        renderBitmap(BITMAP_EVAC, matrix.Color(180, 20, 0), 20);
      }
    }
  }
}

// ==========================================
// MQTT COMMAND HANDLER (Remote POST/WS -> ESP32)
// ==========================================
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  char message[512];
  if (length >= sizeof(message)) length = sizeof(message) - 1;
  memcpy(message, payload, length);
  message[length] = '\0';

  Serial.printf("[MQTT RX] Topic: %s | Msg: %s\n", topic, message);

  // Process Alarm Directives
  if (strstr(topic, "alarm") != NULL) {
    if (strstr(message, "RAISE") != NULL || strstr(message, "CRITICAL") != NULL) {
      Serial.println("[COMMAND] Raising emergency alarm!");
      setMatrixPattern(PAT_DANGER_FLASH);
      setBuzzer(true, 10000); // 10s auto-silence
    } else if (strstr(message, "CLEAR") != NULL || strstr(message, "RESOLVE") != NULL) {
      Serial.println("[COMMAND] Clearing alarm.");
      setBuzzer(false);
      setMatrixPattern(PAT_NORMAL_CHECK);
    }
    return;
  }

  // Process Remote Actuator Commands
  if (strstr(message, "BUZZER_TEST") != NULL || strstr(message, "buzzer") != NULL) {
    bool active = (strstr(message, "\"active\":false") == NULL);
    Serial.printf("[COMMAND] Buzzer toggle: %s\n", active ? "ON" : "OFF");
    setBuzzer(active, active ? 5000 : 0); // 5s test duration
  }

  if (strstr(message, "LED_TEST") != NULL || strstr(message, "pattern") != NULL) {
    if (strstr(message, "DANGER_FLASH") != NULL) {
      setMatrixPattern(PAT_DANGER_FLASH);
    } else if (strstr(message, "WARNING_PULSE") != NULL) {
      setMatrixPattern(PAT_WARNING_PULSE);
    } else if (strstr(message, "EVACUATE_ARROW") != NULL) {
      setMatrixPattern(PAT_EVACUATE_ARROW);
    } else if (strstr(message, "IDLE") != NULL) {
      setMatrixPattern(PAT_IDLE);
    } else {
      setMatrixPattern(PAT_NORMAL_CHECK);
    }
    Serial.printf("[COMMAND] Matrix pattern set to: %d\n", (int)currentPattern);
  }
}

// ==========================================
// WIFI & MQTT NETWORK HELPERS
// ==========================================
void setupWiFi() {
  Serial.printf("[WIFI] Connecting to %s ...\n", ssid);
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(wifi_hostname);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(400);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("[WIFI] Connected! IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WIFI] Connection failed. Will retry in background.");
  }
}

void connectToMQTT() {
  Serial.printf("[MQTT] Connecting to broker as '%s'... ", mqtt_client_id);
  if (mqttClient.connect(mqtt_client_id)) {
    Serial.println("Connected!");
    mqttClient.subscribe(mqtt_topic_commands);
    mqttClient.subscribe(mqtt_topic_alarm);
    mqttClient.subscribe("mine/actuators");
    Serial.println("[MQTT] Subscribed to actuator command topics.");
  } else {
    Serial.printf("Failed, rc=%d\n", mqttClient.state());
  }
}

// ==========================================
// SENSOR READING IMPLEMENTATION
// ==========================================
void readSensors(
  int &vib,
  float &adxl_ax, float &adxl_ay, float &adxl_az,
  float &mpu_ax, float &mpu_ay, float &mpu_az,
  float &mpu_gx, float &mpu_gy, float &mpu_gz,
  float &dist1, float &dist2,
  int &mq2_raw,
  float &temp, float &hum
) {
  // SW-420
  if (vibrationTriggered) {
    vib = 1;
    vibrationTriggered = false;
  } else {
    vib = 0;
  }

  // ADXL345
  if (adxl_initialized) {
    sensors_event_t event;
    adxl.getEvent(&event);
    adxl_ax = event.acceleration.x;
    adxl_ay = event.acceleration.y;
    adxl_az = event.acceleration.z;
  }

  // MPU6050
  if (mpu_initialized) {
    sensors_event_t a, g, t;
    if (mpu.getEvent(&a, &g, &t)) {
      mpu_ax = a.acceleration.x;
      mpu_ay = a.acceleration.y;
      mpu_az = a.acceleration.z;
      mpu_gx = g.gyro.x;
      mpu_gy = g.gyro.y;
      mpu_gz = g.gyro.z;
    }
  }

  // Dual HC-SR04
  dist1 = readDistance(HCSR04_1_TRIG_PIN, HCSR04_1_ECHO_PIN);
  delay(15);
  dist2 = readDistance(HCSR04_2_TRIG_PIN, HCSR04_2_ECHO_PIN);

  // MQ-2 Gas
  mq2_raw = analogRead(MQ2_PIN);

  // DHT11
  float newHum = dht.readHumidity();
  float newTemp = dht.readTemperature();
  if (!isnan(newHum) && !isnan(newTemp)) {
    hum = newHum;
    temp = newTemp;
  }
}

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseInLong(echoPin, HIGH, 30000);
  if (duration == 0) return -1.0;
  return (duration * 0.0343) / 2.0;
}

// ==========================================
// MQTT PUBLISHING & BUFFER QUEUE
// ==========================================
bool publishDataPoint(const DataPoint &dp) {
  char payload[900];
  bool validDHT = !isnan(dp.temperature) && !isnan(dp.humidity);

  float unified_dist = -1.0;
  if (dp.distance_cm_1 > 0 && dp.distance_cm_2 > 0) {
    unified_dist = min(dp.distance_cm_1, dp.distance_cm_2);
  } else if (dp.distance_cm_1 > 0) {
    unified_dist = dp.distance_cm_1;
  } else if (dp.distance_cm_2 > 0) {
    unified_dist = dp.distance_cm_2;
  } else {
    unified_dist = 40.0;
  }

  if (validDHT) {
    snprintf(
      payload,
      sizeof(payload),
      "{\"dev\":\"%s\",\"seq\":%lu,\"ms\":%lu,\"vib\":%d,"
      "\"adxl345\":{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f},"
      "\"mpu6050\":{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f},"
      "\"distance_cm\":%.2f,\"buzzer\":%d,\"mq2_raw\":%d,\"temperature\":%.2f,\"humidity\":%.2f}",
      mqtt_client_id, (unsigned long)dp.seq, (unsigned long)dp.timestamp_ms, dp.vibration,
      dp.adxl_ax, dp.adxl_ay, dp.adxl_az,
      dp.mpu_ax, dp.mpu_ay, dp.mpu_az, dp.mpu_gx, dp.mpu_gy, dp.mpu_gz,
      unified_dist, dp.buzzer_state, dp.mq2_raw, dp.temperature, dp.humidity
    );
  } else {
    snprintf(
      payload,
      sizeof(payload),
      "{\"dev\":\"%s\",\"seq\":%lu,\"ms\":%lu,\"vib\":%d,"
      "\"adxl345\":{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f},"
      "\"mpu6050\":{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f},"
      "\"distance_cm\":%.2f,\"buzzer\":%d,\"mq2_raw\":%d,\"temperature\":null,\"humidity\":null}",
      mqtt_client_id, (unsigned long)dp.seq, (unsigned long)dp.timestamp_ms, dp.vibration,
      dp.adxl_ax, dp.adxl_ay, dp.adxl_az,
      dp.mpu_ax, dp.mpu_ay, dp.mpu_az, dp.mpu_gx, dp.mpu_gy, dp.mpu_gz,
      unified_dist, dp.buzzer_state, dp.mq2_raw
    );
  }

  return mqttClient.publish(mqtt_topic_telemetry, payload);
}

void processQueue() {
  while (!dataBuffer.empty()) {
    const DataPoint &dp = dataBuffer.front();
    if (publishDataPoint(dp)) {
      dataBuffer.erase(dataBuffer.begin());
    } else {
      break;
    }
    delay(5);
  }
}

