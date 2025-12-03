//---------------------------------------------
// Includes
//---------------------------------------------
#include <WiFi.h>
#include <Preferences.h>
#include <DebugLog.h>
#include <TMP1075.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <NTPClient.h>
#include <MQTT.h>
#include <RV-3028-C7.h>
#include <esp_system.h>
#include <math.h>
#include <FRAM.h>
#include <ArduinoJson.h>
#include <esp_bt.h>
#include <driver/adc.h>
#include <esp_sleep.h>
#include <driver/usb_serial_jtag.h>
#include <driver/rtc_io.h>
//---------------------------------------------



//---------------------------------------------
// Defines GIOS
//---------------------------------------------
#define HEARTBEAT_LED 23
#define VBAT_ADC 3
#define nWKUP 2
#define USB_DETECT 19
//---------------------------------------------



//---------------------------------------------
// Preamble WLAN
//---------------------------------------------
WiFiClient net;
void printWiFiStatus();
//---------------------------------------------



//---------------------------------------------
// Preamble I2C
//---------------------------------------------
#define I2C_SDA 6
#define I2C_SCL 7
#define I2C_ADC_ADDRESS 0x49
#define I2C_FRAM_ADDRESS 0x51
#define I2C_TEMP_ADDRESS 0x48
//---------------------------------------------



//---------------------------------------------
// Preamble MQTT
//---------------------------------------------
MQTTClient mqttClient;

struct mqtt_struct {
  uint8_t index;
  bool status;
  unsigned long timestamp;
  uint16_t voltage_battery;
  float internal_temperature;
  float internal_offset;
  float ext_ch[4];
  float ext_offset[4];
} mqtt_data;

enum mqtt_struct_status {
  READY_TO_WRITE,
  READY_TO_SEND,
  READY_TO_DELETE
};

void initMQTT();
//---------------------------------------------



//---------------------------------------------
// Preamble RTC
//---------------------------------------------
RV3028 rtc;

void updateRTCFromNTP();
unsigned long getRTCEpoch();
String getRTC_ISO8601();
bool setInterrupt(uint32_t newTimeSeconds, bool repeat = true);
//---------------------------------------------



//---------------------------------------------
// Preamble NTP
//---------------------------------------------
WiFiUDP ntpUDP;

NTPClient timeClient(ntpUDP);
//---------------------------------------------



//---------------------------------------------
// Preamble TMP1075
//---------------------------------------------
TMP1075::TMP1075 tmp1075 = TMP1075::TMP1075(Wire, I2C_TEMP_ADDRESS);

float tmp_read_temperature(TMP1075::ConversionTime time);
//---------------------------------------------



//---------------------------------------------
// Preamble ADS1x15
//---------------------------------------------
#define ADC_GAIN GAIN_TWO  //GAIN_EIGHT

Adafruit_ADS1015 ads;

float tmp_read_ext_temperature[4];
float calcNTCTemperature(int channel);
float lastValidTemp[4] = { NAN, NAN, NAN, NAN};
float filteredNTC(int channel);
//---------------------------------------------



//---------------------------------------------
// Preamble Bootmenu
//---------------------------------------------
enum InputMode {
  SINGLE_CHAR,
  LINE_INPUT
};
bool bootmenuActive = false;
unsigned long bootTimer;

void runBootmenu();
void bootmenu_Main();
void bootmenu_WLAN_Submenu();
void bootmenu_MQTT_Submenu();
void bootmenu_Temp_Internal_Submenu();
void bootmenu_Temp_External_Submenu();
String readSerialLine();
String maskPassword(const String &pw);
void clearScreen();
//---------------------------------------------



//---------------------------------------------
// Preamble NVS
//--------------------------------------------
Preferences prefs;

struct struct_system_config {
  String wifi_ssid;
  String wifi_pass;
  String mqtt_server_ip;
  int mqtt_server_port;
  String mqtt_id;
  String mqtt_user;
  String mqtt_user_pass;
  String mqtt_topic;
  int interval;
  float internal_offset;
  float ext_offset_ch[4];
  float ext_rs_ch[4];
  float ext_rn_ch[4];
  float ext_tn_ch[4];
  float ext_b_ch[4];
  int sps;
  bool debug_led = false;
  bool deep_sleep = true;
  float ref_volt;
} system_config;

void saveConfig(struct_system_config &config);
void loadConfig(struct_system_config &config);
//---------------------------------------------



//---------------------------------------------
// Preamble ESP
//---------------------------------------------
volatile bool uploadRunning = false;
volatile bool samplingActive = false;

uint16_t uploadStartIndex = 0;
uint16_t uploadEndIndex = 0;
float lastValidBattery = NAN;

float adc_read_voltage();
void scanI2CBus();
void manageData();

void uploadTask(void *param);
void samplingTask(void *param);
void runBootmenu();
float filteredBattery();
//---------------------------------------------



//---------------------------------------------
// Preamble Error Code
//---------------------------------------------
enum LedMode {
  LED_OFF,
  LED_HEARTBEAT,
  LED_UPLOAD,
  LED_ERROR,
  LED_SAMPLING
};

enum ErrorCode {
  ERROR_NONE,
  ERROR_MQTT,
  ERROR_WIFI,
  ERROR_GENERAL,
  ERROR_ADC,
  ERROR_I2C,
  ERROR_FRAM,
  ERROR_RTC,
  ERROR_SEND
};

volatile LedMode currentLedMode = LED_OFF;
volatile ErrorCode currentError = ERROR_NONE;

const int SHORT_DELAY = 300;
const int LONG_DELAY = 700;
const int CYCLE_DELAY = 2000;
void crashHandler();
void ledTask(void *param);
//---------------------------------------------



//---------------------------------------------
// Preamble FRAM
//---------------------------------------------
FRAM fram;
const uint16_t FRAM_ADDR_START = 0;
const size_t STRUCT_SIZE = sizeof(mqtt_struct);
const uint16_t MAX_RECORDS = 2048;

struct FramHeader {
  uint32_t magic;
  uint32_t writeIndex;
  uint32_t readIndex;
};

const uint32_t FRAM_MAGIC = 0xA3F71C2B;
const uint16_t FRAM_HEADER_ADDR = 0;

FramHeader header;


void writeMQTTToFRAM(uint16_t addr, const mqtt_struct &data);
void readMQTTFromFRAM(uint16_t addr, mqtt_struct &data);
void loadFramHeader();
void saveFramHeader();
//---------------------------------------------



//================================================================================
// Setup
//================================================================================
void setup() {

  unsigned long startWait = millis();

  // Try to save power
  setCpuFrequencyMhz(80);
  btStop();

  // Initilize GPIOs
  pinMode(nWKUP, INPUT);
  pinMode(USB_DETECT, INPUT);

  // Initilize LED
  statusLedInit();

  // Switch Serial depending on an attached USB device
  if (usb_serial_jtag_is_connected()) {

    Serial.begin(115200);
    while (!Serial && millis() - startWait < 1000 * 5) {
      // Just wait
    }

    LOG_INFO("USB interface active\r");
  } else {
    LOG_SET_LEVEL(DebugLogLevel::LVL_NONE);
  }

  // Load NVS
  loadConfig(system_config);

  // Initilize I2C
  LOG_INFO("Initilize I2C\r");
  Wire.setClock(400000);
  if (!Wire.begin(I2C_SDA, I2C_SCL)) {
    setErrorCode(ERROR_I2C);
    crashHandler();
  }

  // Initilize FRAM
  LOG_INFO("Initilize FRAM\r");
  if (fram.begin(I2C_FRAM_ADDRESS) != 0) {
    LOG_ERROR("Initilize FRAM failed!\r");
    setErrorCode(ERROR_FRAM);
    crashHandler();
  } else {
    loadFramHeader();
  }

  // Initilize TMP1075
  LOG_INFO("Initilize TMP1075\r");
  tmp1075.begin();

  // Initilize ADS1x15
  LOG_INFO("Initilize ADS1x15\r");
  if (!ads.begin(I2C_ADC_ADDRESS)) {
    LOG_ERROR("Initilize ADS1x15 failed!\r");
    setErrorCode(ERROR_ADC);
    crashHandler();
  } else {
    ads.setGain(ADC_GAIN);
  }

  // Initilize RTC
  if (!rtc.begin()) {
    LOG_ERROR("Initilize RV-3028-C7 failed!");
    setErrorCode(ERROR_RTC);
    crashHandler();
  } else {
    setInterrupt(system_config.sps, true);
    rtc.set24Hour();
  }

  esp_reset_reason_t resetReason = esp_reset_reason();

  esp_sleep_wakeup_cause_t wakeBefore = esp_sleep_get_wakeup_cause();
  bool wokeFromDeepSleep = (wakeBefore != ESP_SLEEP_WAKEUP_UNDEFINED);

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  esp_sleep_wakeup_cause_t wakeAfter = esp_sleep_get_wakeup_cause();

  LOG_INFO("Reset reason: ", resetReason, "\r");
  LOG_INFO("Wake BEFORE clear: ", wakeBefore, "\r");
  LOG_INFO("Wake AFTER clear: ", wakeAfter, "\r");
  LOG_INFO("DeepSleep wake flag: ", wokeFromDeepSleep, "\r");

  if (!wokeFromDeepSleep && (resetReason == ESP_RST_POWERON || resetReason == ESP_RST_EXT)) {
      runBootmenu();
  } else if (wokeFromDeepSleep) {
      LOG_INFO("DeepSleep wakeup — Bootmenu skipped.\r");
  } else if (resetReason == ESP_RST_SW) {
      LOG_INFO("Software reset — Bootmenu skipped.\r");
  }

  pinMode(nWKUP, INPUT);
  rtc_gpio_pullup_dis((gpio_num_t)nWKUP);
  rtc_gpio_pulldown_dis((gpio_num_t)nWKUP);
  esp_deep_sleep_enable_gpio_wakeup(1ULL << nWKUP, ESP_GPIO_WAKEUP_GPIO_LOW);

  LOG_INFO("Setup finished\r");
  xTaskCreatePinnedToCore(samplingTask, "Sampler", 4096, &system_config.sps, 1, NULL, 0);
  xTaskCreatePinnedToCore(uploadTask, "Uploader", 8192, &system_config.interval, 1, NULL, 0);
  xTaskCreatePinnedToCore(ledTask, "LED_Task", 2048, NULL, 2, NULL, 0);
}
//================================================================================



//================================================================================
// Main loop
//================================================================================
void loop() {
}
//================================================================================



//---------------------------------------------
// ESP
//---------------------------------------------
void samplingTask(void *param) {
  unsigned long sps_s = *(unsigned long *)param;

  while (bootmenuActive) {
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  setLedMode(LED_SAMPLING);

  while (1) {
    mqtt_struct data;
    data.index = header.writeIndex;
    data.status = true;
    data.timestamp = getRTCEpoch();
    data.voltage_battery = filteredBattery();
    data.internal_temperature = tmp_read_temperature(TMP1075::ConversionTime220ms);
    data.internal_offset = system_config.internal_offset;
    for (int i = 0; i < 4; i++) {
      data.ext_ch[i] = filteredNTC(i);
      data.ext_offset[i] = system_config.ext_offset_ch[i];
    }

    uint16_t addr = FRAM_ADDR_START + header.writeIndex * STRUCT_SIZE;
    writeMQTTToFRAM(addr, data);

    header.writeIndex = (header.writeIndex + 1) % MAX_RECORDS;
    saveFramHeader();

    String logMsg = "Saved dataset #" + String(data.index) + " | Timestamp: " + String(data.timestamp) + " | Bat: " + String(data.voltage_battery / 1000.0, 2) + " V" + " | IntTemp: " + String(data.internal_temperature, 2) + " °C" + " | Offset: " + String(data.internal_offset, 2) + " °C";
    for (int i = 0; i < 4; i++) {
      logMsg += " | CH" + String(i + 1) + ": " + String(data.ext_ch[i], 2) + " °C (Offset: " + String(data.ext_offset[i], 2) + ")";
    }
    LOG_INFO(logMsg.c_str(), "\r");

    uint16_t count = (header.writeIndex >= header.readIndex) ? (header.writeIndex - header.readIndex) : (MAX_RECORDS - header.readIndex + header.writeIndex);

    if (count >= system_config.interval && !uploadRunning) {
      uploadRunning = true;
    }

    if (!uploadRunning && !bootmenuActive && system_config.deep_sleep) {
      LOG_INFO("Entering Deep Sleep until RTC pulse...\r");

      pinMode(nWKUP, INPUT);
      rtc_gpio_pullup_dis((gpio_num_t)nWKUP);
      rtc_gpio_pulldown_dis((gpio_num_t)nWKUP);

      esp_deep_sleep_enable_gpio_wakeup(1ULL << nWKUP, ESP_GPIO_WAKEUP_GPIO_LOW);

      delay(10);

      esp_deep_sleep_start();
    }

    vTaskDelay(pdMS_TO_TICKS(sps_s * 1000));
  }
}

void uploadTask(void *param) {

  while (1) {
    if (!uploadRunning || bootmenuActive) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    uint16_t endIndex = header.writeIndex;

    LOG_INFO("Start upload cycle\r");
    setLedMode(LED_UPLOAD);

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin(system_config.wifi_ssid, system_config.wifi_pass);
    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 8000) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (WiFi.status() != WL_CONNECTED) {
      LOG_ERROR("WiFi connection failed\r");
      setErrorCode(ERROR_WIFI);
      uploadRunning = false;
      continue;
    }

    LOG_INFO("Initilize NTP\r");
    timeClient.begin();
    timeClient.update();
    updateRTCFromNTP();

    initMQTT();
    unsigned long mqttStart = millis();
    while (!mqttClient.connected() && millis() - mqttStart < 5000) {
      mqttClient.loop();
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (!mqttClient.connected()) {
      LOG_ERROR("MQTT connection failed\r");
      setErrorCode(ERROR_MQTT);
      uploadRunning = false;
      continue;
    }

    while (header.readIndex != endIndex) {
      mqtt_struct temp;
      readMQTTFromFRAM(FRAM_ADDR_START + header.readIndex * STRUCT_SIZE, temp);
      publishMqttData(system_config.mqtt_topic.c_str(), temp);

      header.readIndex = (header.readIndex + 1) % MAX_RECORDS;
      saveFramHeader();

      mqttClient.loop();
      vTaskDelay(pdMS_TO_TICKS(30));
    }

    LOG_INFO("Transmission complete\r");
    uploadRunning = false;

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
}

void runBootmenu() {

  if (!digitalRead(USB_DETECT)) {
    return;
  }

  Serial.println("\nGreetings Professor Falken.\n");

  unsigned long lastPrint = 0;
  uint8_t secondsLeft = 10;

  while (secondsLeft > 0) {

    if (millis() - lastPrint >= 1000) {
      lastPrint = millis();
      Serial.printf("Starting in %u seconds... (press SPACE to enter Bootmenu)\r", secondsLeft);
      secondsLeft--;
    }

    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == ' ') {
        Serial.println("\nBootmenu activated!\n");

        LOG_SET_LEVEL(DebugLogLevel::LVL_NONE);
        bootmenu_Main();
        LOG_SET_LEVEL(DebugLogLevel::LVL_INFO);
        return; 
      }
    }
  }

  Serial.println("\nNo key pressed → starting main program...\n");
}

void bootmenuTask(void *param) {
  if (digitalRead(USB_DETECT)) {
    Serial.println("\nGreetings Professor Falken.\n");
    Serial.println("Press SPACE for Bootmenu...\n");
    while (1) {

      bootTimer = millis();
      while (millis() - bootTimer < 10000) {
        if (Serial.available() > 0) {
          char c = Serial.read();

          if (c == ' ') {
            bootmenuActive = true;
            break;
          }
        }
      }

      if (bootmenuActive) {
        LOG_SET_LEVEL(DebugLogLevel::LVL_NONE);
        bootmenu_Main();
        LOG_SET_LEVEL(DebugLogLevel::LVL_INFO);
      }
      bootmenuActive = false;
    }
  }
}

void crashHandler() {
  while (1) {};
}

void scanI2CBus() {
  LOG_INFO("Scanning I2C bus...\r");

  byte error;
  int nDevices = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      LOG_INFO("I2C device found at address 0x", address, "\r");
      nDevices++;
    } else if (error == 4) {
      LOG_ERROR("Unknown error at address 0x", address, "\r");
    }
  }
  if (nDevices == 0) {
    LOG_ERROR("No I2C devices found\r");
  } else {
    LOG_INFO("Scan done\r");
  }
}
//---------------------------------------------



//---------------------------------------------
// MQTT
//---------------------------------------------
void initMQTT() {
  LOG_INFO("Initilize MQTT\r");

  LOG_INFO("MQTT ID: " + system_config.mqtt_id, "\r");
  LOG_INFO("MQTT User: " + system_config.mqtt_user, "\r");
  LOG_INFO("MQTT Pass: " + maskPassword(system_config.mqtt_user_pass), "\r");
  LOG_INFO("MQTT IP/Host: " + system_config.mqtt_server_ip, "\r");
  LOG_INFO("MQTT Port: ", system_config.mqtt_server_port, "\r");
  LOG_INFO("MQTT Topic: ", system_config.mqtt_topic, "\r");

  mqttClient.begin(system_config.mqtt_server_ip.c_str(), system_config.mqtt_server_port, net);

  if (!mqttClient.connect(
        system_config.mqtt_id.c_str(),
        system_config.mqtt_user.c_str(),
        system_config.mqtt_user_pass.c_str())) {
    LOG_ERROR("Initilize MQTT failed!\r");
  } else {
    LOG_INFO("MQTT connected successfully!\r");
  }
}

bool publishMqttData(const char *topic, const mqtt_struct &data) {
  if (!mqttClient.connected()) {
    LOG_WARN("MQTT not connected, skipping publish\r");
    return false;
  }

  StaticJsonDocument<256> doc;

  doc["index"] = data.index;
  doc["status"] = data.status;
  doc["timestamp"] = data.timestamp;
  doc["voltage_battery"] = data.voltage_battery;
  doc["internal_temperature"] = data.internal_temperature;
  doc["internal_offset"] = data.internal_offset;

  for (int i = 0; i < 4; i++) {
    doc["EXT_CH" + String(i + 1)] = data.ext_ch[i];
    doc["EXT_CH" + String(i + 1) + "_Offset"] = data.ext_offset[i];
  }

  String logMsg = "Send dataset #" + String(data.index) + " | Timestamp: " + String(data.timestamp) + " | Bat: " + String(data.voltage_battery / 1000.0, 2) + " V" + " | IntTemp: " + String(data.internal_temperature, 2) + " °C" + " | Offset: " + String(data.internal_offset, 2) + " °C";
  for (int i = 0; i < 4; i++) {
    logMsg += " | CH" + String(i + 1) + ": " + String(data.ext_ch[i], 2) + " °C (Offset: " + String(data.ext_offset[i], 2) + ")";
  }
  LOG_INFO(logMsg.c_str(), "\r");

  String payloadStr;
  serializeJson(doc, payloadStr);

  for (int attempt = 1; attempt <= 3; attempt++) {
    if (mqttClient.publish(topic, payloadStr.c_str(), 1, false)) {
      LOG_INFO("MQTT publish success on attempt ", attempt, "\r");
      return true;
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
    mqttClient.loop();
  }

  LOG_ERROR("MQTT publish failed after retries\r");
  return false;
}
//---------------------------------------------



//---------------------------------------------
// Internal ADC
//---------------------------------------------
float adc_read_voltage() {
    const int SAMPLES = 5;          // Anzahl Messungen für Median
    float readings[SAMPLES];

    // mehrere Messungen
    for (int i = 0; i < SAMPLES; i++) {
        readings[i] = analogRead(VBAT_ADC) * 1.4311f;
        vTaskDelay(pdMS_TO_TICKS(2));  // kurze Pause zwischen Samples
    }

    // Median bestimmen
    std::sort(readings, readings + SAMPLES);
    float voltage = readings[SAMPLES / 2];  // Median

    LOG_INFO("BAT ADC median=", String(voltage, 2), " mV\r");

    return voltage;
}

float filteredBattery() {
    const int MAX_RETRIES = 5;
    const float MAX_DELTA = 0.15f;  // 150 mV Sprung unrealistisch
    const float MIN_VOLT = 0.0f;
    const float MAX_VOLT = 5000.0f;

    float best = NAN;

    for (int i = 0; i < MAX_RETRIES; i++) {
        float v = adc_read_voltage();

        LOG_INFO("BAT sample ", String(i), ": ", String(v, 3), " mV\r");

        // plausibler Bereich?
        if (v > MIN_VOLT && v < MAX_VOLT) {
            if (!isnan(lastValidBattery)) {
                float delta = fabs(v - lastValidBattery);

                if (delta > MAX_DELTA) {
                    LOG_WARN("BAT outlier detected: Δ=", String(delta, 3),
                             " mV → retry\r");
                    vTaskDelay(pdMS_TO_TICKS(5));
                    continue;  // retry
                }
            }

            best = v;
            break;
        }

        LOG_WARN("BAT invalid sample → retry\r");
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // kein gültiger Wert → alten Wert verwenden
    if (isnan(best)) {
        LOG_ERROR("BAT no valid sample; using lastValidBattery\r");
        if (!isnan(lastValidBattery)) return lastValidBattery;
        return NAN;
    }

    // Glättung (EMA)
    if (!isnan(lastValidBattery)) {
        float alpha = 0.3f;
        float smoothed = lastValidBattery + alpha * (best - lastValidBattery);

        LOG_INFO("BAT smooth: raw=", String(best, 3),
                 " → smoothed=", String(smoothed, 3), "\r");

        best = smoothed;
    }

    lastValidBattery = best;

    LOG_INFO("BAT final=", String(best, 3), " mV\r");

    return best;
}
/*
float adc_read_voltage() {
  float tmp;
  tmp = analogRead(VBAT_ADC) * 1.4311;

  LOG_INFO("Battery Voltage:", String(tmp, 2), " mV\r");

  return tmp;
}


float filteredBattery() {
    const int MAX_RETRIES = 5;
    const float MAX_DELTA = 0.15;   // 150 mV Sprung ist unrealistisch
    const float MIN_VOLT = 0.0;
    const float MAX_VOLT = 5000.0;  // 5V (ESP32 ADC Fehler)

    float best = NAN;

    for (int i = 0; i < MAX_RETRIES; i++) {
        float v = adc_read_voltage(); // deine bestehende Funktion

        if (v > MIN_VOLT && v < MAX_VOLT) {

            if (!isnan(lastValidBattery)) {
                if (fabs(v - lastValidBattery) > MAX_DELTA) {
                    // Ausreißer → nochmal messen
                    delay(5);
                    continue;
                }
            }

            best = v;
            break;
        }
    }

    if (isnan(best)) {
        if (!isnan(lastValidBattery)) return lastValidBattery;
        return NAN;
    }

    // Glättung
    float alpha = 0.3;
    if (!isnan(lastValidBattery)) {
        best = lastValidBattery + alpha * (best - lastValidBattery);
    }

    lastValidBattery = best;
    return best;
}
*/
//---------------------------------------------



//---------------------------------------------
// TMP1075
//---------------------------------------------
float tmp_read_temperature(TMP1075::ConversionTime time) {
  float tmp;

  tmp1075.setConversionTime(time);
  tmp = tmp1075.getTemperatureCelsius();
  LOG_INFO("TMP1075: ", String(tmp, 2), " °C\r");
  return tmp;
}
//---------------------------------------------



//---------------------------------------------
// ADS1x15
//---------------------------------------------
float filteredNTC(int channel) {

    const int MAX_RETRIES = 5;
    const float MAX_DELTA = 4.0;
    const float MIN_TEMP = -40.0;
    const float MAX_TEMP = 150.0;

    float last = lastValidTemp[channel];
    float finalVal = NAN;

    for (int retry = 0; retry < MAX_RETRIES; retry++) {

        float t = calcNTCTemperature(channel);

        // ungültig?
        if (isnan(t) || t < MIN_TEMP || t > MAX_TEMP) {
            vTaskDelay(pdMS_TO_TICKS(8));
            continue;
        }

        // erster gültiger Wert?
        if (isnan(last)) {
            finalVal = t;
            break;
        }

        // Ausreißerprüfung
        if (fabs(t - last) > MAX_DELTA) {
            vTaskDelay(pdMS_TO_TICKS(8));
            continue;
        }

        // gültiger Wert
        finalVal = t;
        break;
    }

    // kein gültiger Wert nach allen Retries
    if (isnan(finalVal)) {
        return last;   // fallback
    }

    // leichte Glättung
    float alpha = 0.2f;
    if (!isnan(last)) {
        finalVal = last + alpha * (finalVal - last);
    }

    lastValidTemp[channel] = finalVal;
    return finalVal;
}

float calcNTCTemperature(int channel) {

    // Spannung am NTC lesen
    float Vout = ads.computeVolts(ads.readADC_SingleEnded(channel));

    // Pull-Up Widerstand aus Config
    float Rserie = system_config.ext_rs_ch[channel];

    // NTC-Parameter aus Config
    float R0 = system_config.ext_rn_ch[channel];
    float T0 = system_config.ext_tn_ch[channel];
    float B  = system_config.ext_b_ch[channel];

    // Referenzspannung
    float Vin = system_config.ref_volt;

    // T0 in Kelvin
    float T0_K = T0;
    if (T0_K < 200.0f) T0_K += 273.15f;

    // Spannungsteiler-Check
    if (Vout <= 0.0f || Vout >= Vin) {
        LOG_ERROR("CH", String(channel),
                  " invalid Vout=", String(Vout, 6),
                  " (Vin=", String(Vin, 6), ") -> NAN\r");
        return NAN;
    }

    // NTC-Widerstand berechnen
    float Rntc = Rserie * (Vout / (Vin - Vout));

    if (Rntc <= 0.0f) {
        LOG_ERROR("CH", String(channel),
                  " invalid Rntc=", String(Rntc, 4), " -> NAN\r");
        return NAN;
    }

    // B-Formel zur Temperaturberechnung
    float tempK = 1.0f / (1.0f / T0_K + (1.0f / B) * log(Rntc / R0));
    float tempC = tempK - 273.15f;

    // Logging
    LOG_INFO("CH", String(channel),
             " R0=", String(R0, 2),
             " T0=", String(T0_K, 2), "K",
             " B=", String(B, 2),
             " Vin=", String(Vin, 4),
             " Vout=", String(Vout, 5),
             " Rs=", String(Rserie, 2),
             " Rntc=", String(Rntc, 2),
             " Temp=", String(tempC, 2), "°C\r");

    return tempC;
}
/*
float filteredNTC(int channel) {
    const int MAX_RETRIES = 4;
    const int RETRY_DELAY_MS = 10;
    const float MIN_TEMP = -40.0;
    const float MAX_TEMP = 150.0;
    const float MAX_DELTA = 5.0;  // max erlaubte Änderung pro Sample

    float best = NAN;

    for (int i = 0; i < MAX_RETRIES; i++) {
        float t = calcNTCTemperature(channel);

        if (!isnan(t) && t > MIN_TEMP && t < MAX_TEMP) {

            // haben wir einen gültigen alten Wert?
            if (!isnan(lastValidTemp[channel])) {
                if (fabs(t - lastValidTemp[channel]) > MAX_DELTA) {
                    // Ausreißer → nochmal messen
                    vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                    continue;
                }
            }

            best = t;
            break;
        }

        // ungültig → retry
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
    }

    // Wenn kein gültiger Wert → alten Wert behalten
    if (isnan(best)) {
        if (!isnan(lastValidTemp[channel])) {
            return lastValidTemp[channel]; // fallback
        }
        return NAN; // noch nie ein valider Wert
    }

    // optional leichte Glättung (EMA)
    float alpha = 0.2;
    if (!isnan(lastValidTemp[channel])) {
        best = lastValidTemp[channel] + alpha * (best - lastValidTemp[channel]);
    }

    lastValidTemp[channel] = best;
    return best;
}

float calcNTCTemperature(int channel) {
    // Spannung am NTC
    float Vout = ads.computeVolts(ads.readADC_SingleEnded(channel));
    vTaskDelay(pdMS_TO_TICKS(10));
    Vout = ads.computeVolts(ads.readADC_SingleEnded(channel));

    // Pull-Up Widerstand aus Config
    float Rserie = system_config.ext_rs_ch[channel];

    // NTC-Parameter aus Config
    float R0 = system_config.ext_rn_ch[channel];
    float T0 = system_config.ext_tn_ch[channel];
    float B  = system_config.ext_b_ch[channel];

    if (T0 < 200.0f) T0 += 273.15f;  // T0 in Kelvin

    // Spannungsteiler-Check
    float Vin = system_config.ref_volt;
    if (Vout <= 0.0f || Vout >= Vin) {
        LOG_ERROR("Invalid voltage on CH", String(channel), "\r");
        return NAN;
    }

    // NTC-Widerstand berechnen
    float Rntc = Rserie * (Vout / (Vin - Vout));

    if (Rntc <= 0.0f) {
        LOG_ERROR("Invalid NTC resistance on CH", String(channel), "\r");
        return NAN;
    }

    // B-Formel zur Temperaturberechnung
    float tempK = 1.0f / (1.0f / T0 + (1.0f / B) * log(Rntc / R0));
    float tempC = tempK - 273.15f;

    // Logging
    LOG_INFO("CH", String(channel),
             " R0=", String(R0),
             " T0=", String(T0),
             " B=", String(B),
             " Vout=", String(Vout),
             " Vin=", String(Vin,3),
             " Rs=", String(Rserie),
             " Rntc=", String(Rntc),
             " Temp=", String(tempC, 2), "°C\r");

    return tempC;
}
*/
//---------------------------------------------



//---------------------------------------------
// RV-3028-C7
//---------------------------------------------

bool setInterrupt(uint32_t timeSeconds, bool repeat) {
  rtc.disableTimerInterrupt();
  rtc.disableTimer();

  rtc.setTimer(repeat, 1, (uint16_t)timeSeconds, true, true, false);

  return true;
}

unsigned long getRTCEpoch() {
  struct tm t = { 0 };
  rtc.updateTime();
  t.tm_sec = rtc.getSeconds();
  t.tm_min = rtc.getMinutes();
  t.tm_hour = rtc.getHours();
  t.tm_mday = rtc.getDate();
  t.tm_mon = rtc.getMonth() - 1;
  t.tm_year = rtc.getYear() - 1900;
  t.tm_isdst = -1;

  time_t rtcTime = mktime(&t);

  LOG_INFO(("Epoch: " + String((unsigned long)rtcTime)).c_str(), "\r");
  return (unsigned long)rtcTime;
}

String getRTC_ISO8601() {
  unsigned long epoch = getRTCEpoch();
  time_t t = (time_t)epoch;
  struct tm *ptm = gmtime(&t);

  char buffer[25];
  snprintf(buffer, sizeof(buffer),
           "%04d-%02d-%02dT%02d:%02d:%02dZ",
           ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
           ptm->tm_hour, ptm->tm_min, ptm->tm_sec);

  String isoTime(buffer);

  LOG_INFO(("ISO Time: " + isoTime).c_str(), "\r");

  return isoTime;
}

void updateRTCFromNTP() {
  if (WiFi.status() != WL_CONNECTED) {
    LOG_ERROR("No WiFi → cannot sync RTC!");
    return;
  }

  timeClient.update();
  unsigned long epoch = timeClient.getEpochTime();
  time_t t = (time_t)epoch;

  struct tm *ptm = gmtime(&t);

  rtc.setSeconds(ptm->tm_sec);
  rtc.setMinutes(ptm->tm_min);
  rtc.setHours(ptm->tm_hour);
  rtc.setDate(ptm->tm_mday);
  rtc.setMonth(ptm->tm_mon + 1);
  rtc.setYear(ptm->tm_year + 1900);

  LOG_INFO(("RTC Updated: " + String(getRTC_ISO8601())).c_str(), "\r");
}
//---------------------------------------------



//---------------------------------------------
// Bootmenu
//---------------------------------------------
void bootmenu_Main() {
  bool inMenu = true;
  String buffer = "";

  while (inMenu) {
    // Hauptmenü
    clearScreen();
    Serial.println("TechBird - SmartSense");
    Serial.println("====================================");
    Serial.println("[1] WLAN Config");
    Serial.println("[2] MQTT Config");
    Serial.println("[3] Internal Temperature Config");
    Serial.println("[4] External Temperature Config");
    Serial.println("[5] Sample every Seconds: " + String(system_config.sps) + " s/Sa");
    Serial.println("[6] Send saved MQTT after n messages: " + String(system_config.interval));
    Serial.println("[7] Debug LED ON/OFF: " + String(system_config.debug_led));
    Serial.println("[8] DeepSleep ON/OFF: " + String(system_config.deep_sleep));
    Serial.println("[s] Save Config");
    Serial.println("[l] Load Config");
    Serial.println("[x] Exit Bootmenu");
    Serial.println("====================================");

    buffer = readSerialLine(SINGLE_CHAR);

    if (buffer == "1") {
      bootmenu_WLAN_Submenu();
    } else if (buffer == "2") {
      bootmenu_MQTT_Submenu();
    } else if (buffer == "3") {
      bootmenu_Temp_Internal_Submenu();
    } else if (buffer == "4") {
      bootmenu_Temp_External_Submenu();
    } else if (buffer == "5") {
      Serial.println("Enter Seconds per Samples:");
      system_config.sps = readSerialLine(LINE_INPUT).toInt();
    } else if (buffer == "6") {
      Serial.println("Enter Size of Ringbuffer:");
      system_config.interval = readSerialLine(LINE_INPUT).toInt();
    } else if (buffer == "7") {
      Serial.println("LED 1 = ON, 0 = OFF:");
      system_config.debug_led = readSerialLine(LINE_INPUT).toInt();
    } else if (buffer == "8") {
      Serial.println("DeepSleep 1 = ON, 0 = OFF:");
      system_config.deep_sleep = readSerialLine(LINE_INPUT).toInt();
    } else if (buffer == "s") {
      saveConfig(system_config);
      Serial.println("Configuration saved to EEPROM/NVS.");
      vTaskDelay(1000);
    } else if (buffer == "l") {
      loadConfig(system_config);
      Serial.println("Configuration loaded from EEPROM/NVS.");
      vTaskDelay(1000);
    } else if (buffer == "x") {
      inMenu = false;
      esp_restart();
    }
  }
}

void bootmenu_WLAN_Submenu() {
  bool inSub = true;
  String buffer;
  while (inSub) {
    clearScreen();
    Serial.println("WLAN Config");
    Serial.println("====================================");
    Serial.println("[0] SSID: " + system_config.wifi_ssid);
    Serial.println("[1] Password: " + maskPassword(system_config.wifi_pass));
    Serial.println("[x] Return");
    Serial.println("===========================");
    buffer = readSerialLine(SINGLE_CHAR);
    if (buffer == "0") {
      Serial.println("Enter new SSID:");
      system_config.wifi_ssid = readSerialLine(LINE_INPUT);
    } else if (buffer == "1") {
      Serial.println("Enter new Password:");
      system_config.wifi_pass = readSerialLine(LINE_INPUT);
    } else if (buffer == "x") inSub = false;
  }
}

void bootmenu_MQTT_Submenu() {
  bool inSub = true;
  String buffer;
  while (inSub) {
    clearScreen();
    Serial.println("MQTT Config");
    Serial.println("====================================");
    Serial.println("[0] Server IP: " + system_config.mqtt_server_ip);
    Serial.println("[1] Server Port: " + String(system_config.mqtt_server_port));
    Serial.println("[2] MQTT ID: " + system_config.mqtt_id);
    Serial.println("[3] User: " + system_config.mqtt_user);
    Serial.println("[4] Password: " + maskPassword(system_config.mqtt_user_pass));
    Serial.println("[5] Topic: " + system_config.mqtt_topic);
    Serial.println("[x] Return");
    Serial.println("===========================");
    buffer = readSerialLine(SINGLE_CHAR);
    if (buffer == "0") {
      Serial.println("Enter new Server IP:");
      system_config.mqtt_server_ip = readSerialLine(LINE_INPUT);
    } else if (buffer == "1") {
      Serial.println("Enter new Server Port:");
      system_config.mqtt_server_port = readSerialLine(LINE_INPUT).toInt();
    } else if (buffer == "2") {
      Serial.println("Enter new MQTT ID:");
      system_config.mqtt_id = readSerialLine(LINE_INPUT);
    } else if (buffer == "3") {
      Serial.println("Enter new User:");
      system_config.mqtt_user = readSerialLine(LINE_INPUT);
    } else if (buffer == "4") {
      Serial.println("Enter new Password:");
      system_config.mqtt_user_pass = readSerialLine(LINE_INPUT);
    } else if (buffer == "5") {
      Serial.println("Enter new Topic:");
      system_config.mqtt_topic = readSerialLine(LINE_INPUT);
    } else if (buffer == "x") inSub = false;
  }
}

void bootmenu_Temp_Internal_Submenu() {
  bool inSub = true;
  String buffer;
  while (inSub) {
    clearScreen();
    Serial.println("Internal Temp Config");
    Serial.println("====================================");
    Serial.println("[0] Current Value: " + String(tmp_read_temperature(TMP1075::ConversionTime220ms)) + "°C");
    Serial.println("[1] Offset: " + String(system_config.internal_offset) + "°C");
    Serial.println("[u] Update Current Temp");
    Serial.println("[x] Return");
    Serial.println("====================================");
    buffer = readSerialLine(SINGLE_CHAR);
    if (buffer == "1") {
      Serial.println("Enter new Offset:");
      system_config.internal_offset = readSerialLine(LINE_INPUT).toFloat();
    } else if (buffer == "u") {
      Serial.println("Temperature updated: " + String(tmp_read_temperature(TMP1075::ConversionTime220ms) * 100) + "°C");
    } else if (buffer == "x")
      inSub = false;
  }
}

void bootmenu_Temp_External_Submenu() {
  bool inSub = true;
  String buffer;
  while (inSub) {
    clearScreen();
    Serial.println("======= External Temp Config =======");
    for (int ch = 1; ch <= 4; ch++)
      Serial.println("[" + String(ch) + "] Config CH" + String(ch));
    Serial.println("[5] Reference Voltage: " + String(system_config.ref_volt, 3));
    Serial.println("[x] Return");
    Serial.println("====================================");
    buffer = readSerialLine(SINGLE_CHAR);
    if (buffer == "1" || buffer == "2" || buffer == "3" || buffer == "4") {
      int ch = buffer.toInt();
      int idx = ch - 1;
      bool inCH = true;
      while (inCH) {
        clearScreen();
        Serial.println("CH" + String(ch) + " Config");
        Serial.println("====================================");
        Serial.println("[0] Current Value: " + String(tmp_read_ext_temperature[idx]) + "°C");
        Serial.println("[1] Offset: " + String(system_config.ext_offset_ch[idx]) + "°C");
        Serial.println("[2] NTC Rs: " + String(system_config.ext_rs_ch[idx]) + " Ω");
        Serial.println("[3] NTC Rn: " + String(system_config.ext_rn_ch[idx]) + " Ω");
        Serial.println("[4] NTC Tn: " + String(system_config.ext_tn_ch[idx]) + " K");
        Serial.println("[5] NTC B: " + String(system_config.ext_b_ch[idx]));
        Serial.println("[u] Update Current Temp");
        Serial.println("[x] Return");
        Serial.println("-----------------------------");
        String chBuf = readSerialLine(SINGLE_CHAR);
        if (chBuf == "1") {
          Serial.println("Enter Offset:");
          system_config.ext_offset_ch[idx] = readSerialLine(LINE_INPUT).toFloat();
        } else if (chBuf == "2") {
          Serial.println("Enter Rs:");
          system_config.ext_rs_ch[idx] = readSerialLine(LINE_INPUT).toFloat();
        } else if (chBuf == "3") {
          Serial.println("Enter Rn:");
          system_config.ext_rn_ch[idx] = readSerialLine(LINE_INPUT).toFloat();
        } else if (chBuf == "4") {
          Serial.println("Enter Tn:");
          system_config.ext_tn_ch[idx] = readSerialLine(LINE_INPUT).toFloat();
        } else if (chBuf == "5") {
          Serial.println("Enter B:");
          system_config.ext_b_ch[idx] = readSerialLine(LINE_INPUT).toFloat();
        } else if (chBuf == "u") {
          tmp_read_ext_temperature[idx] = filteredNTC(idx);
          Serial.println("Temperature updated: " + String(tmp_read_ext_temperature[idx]));
        } else if (chBuf == "x") {
          inCH = false;
        }
      }
    } else if (buffer == "5") {
      Serial.println("Enter reference voltage:");
      system_config.ref_volt = readSerialLine(LINE_INPUT).toFloat();
    } else if (buffer == "x") {
      inSub = false;
    }
  }
}

String readSerialLine(InputMode mode) {
  String input = "";

  while (!Serial.available()) {}

  while (true) {
    if (Serial.available()) {
      char c = Serial.read();

      if (mode == SINGLE_CHAR) {
        input = c;
        break;
      }

      if (mode == LINE_INPUT) {
        if (c == '\n' || c == '\r') {
          Serial.println();
          break;
        }
        input += c;
        Serial.print(c);
      }
    }
  }
  return input;
}

String maskPassword(const String &pw) {
  if (pw.length() <= 2) return "**";
  String masked = String(pw.charAt(0));
  for (int i = 1; i < pw.length() - 1; i++) masked += '*';
  masked += pw.charAt(pw.length() - 1);
  return masked;
}

void clearScreen() {
  Serial.write(27);
  Serial.print("[2J");
  Serial.write(27);
  Serial.print("[H");
}
//---------------------------------------------



//---------------------------------------------
// NVS
//---------------------------------------------
void saveConfig(struct_system_config &config) {
  prefs.begin("syscfg", false);

  prefs.putString("ssid", config.wifi_ssid);
  prefs.putString("pass", config.wifi_pass);
  prefs.putString("msrv", config.mqtt_server_ip);
  prefs.putString("mid", config.mqtt_id);
  prefs.putString("musr", config.mqtt_user);
  prefs.putString("mpass", config.mqtt_user_pass);
  prefs.putString("topic", config.mqtt_topic);
  prefs.putInt("port", config.mqtt_server_port);
  prefs.putInt("interval", config.interval);
  prefs.putFloat("internal_offset", config.internal_offset);
  prefs.putInt("sps", config.sps);
  prefs.putInt("debug_led", config.debug_led);
  prefs.putInt("deep_sleep", config.deep_sleep);
  prefs.putFloat("ref_volt", config.ref_volt);

  for (int ch = 0; ch < 4; ch++) {
    prefs.putFloat(("ext_offset" + String(ch)).c_str(), config.ext_offset_ch[ch]);
    prefs.putFloat(("ext_rs_" + String(ch)).c_str(), config.ext_rs_ch[ch]);
    prefs.putFloat(("ext_rn_" + String(ch)).c_str(), config.ext_rn_ch[ch]);
    prefs.putFloat(("ext_tn_" + String(ch)).c_str(), config.ext_tn_ch[ch]);
    prefs.putFloat(("ext_b_" + String(ch)).c_str(), config.ext_b_ch[ch]);
  }

  prefs.end();
  LOG_INFO("Saved config\r");
}

void loadConfig(struct_system_config &config) {
  prefs.begin("syscfg", true);

  config.wifi_ssid = prefs.getString("ssid", "");
  config.wifi_pass = prefs.getString("pass", "");
  config.mqtt_server_ip = prefs.getString("msrv", "");
  config.mqtt_id = prefs.getString("mid", "");
  config.mqtt_user = prefs.getString("musr", "");
  config.mqtt_user_pass = prefs.getString("mpass", "");
  config.mqtt_topic = prefs.getString("topic", "");
  config.mqtt_server_port = prefs.getInt("port", 1883);
  config.interval = prefs.getInt("interval", 60);
  config.internal_offset = prefs.getFloat("internal_offset", -4.31);
  config.sps = prefs.getInt("sps", 10);
  config.debug_led = prefs.getInt("debug_led", 1);
  config.deep_sleep = prefs.getInt("deep_sleep", 0);
  config.ref_volt = prefs.getFloat("ref_volt", 3.300);

  for (int ch = 0; ch < 4; ch++) {
    config.ext_offset_ch[ch] = prefs.getFloat(("ext_offset" + String(ch)).c_str(), 0);
    config.ext_rs_ch[ch] = prefs.getFloat(("ext_rs_" + String(ch)).c_str(), 1500);
    config.ext_rn_ch[ch] = prefs.getFloat(("ext_rn_" + String(ch)).c_str(), 2000);
    config.ext_tn_ch[ch] = prefs.getFloat(("ext_tn_" + String(ch)).c_str(), 298.15);
    config.ext_b_ch[ch] = prefs.getFloat(("ext_b_" + String(ch)).c_str(), 3535);
  }

  prefs.end();
  LOG_INFO("Got config\r");
}
//---------------------------------------------



//---------------------------------------------
// WLAN
//---------------------------------------------
void printWiFiStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    LOG_INFO("SSID: ", WiFi.SSID(), "\r");
    LOG_INFO("IP-Adresse: ", WiFi.localIP(), "\r");
    LOG_INFO("Gateway: ", WiFi.gatewayIP(), "\r");
    LOG_INFO("Subnetzmaske: ", WiFi.subnetMask(), "\r");
    LOG_INFO("MAC-Adresse: ", WiFi.macAddress(), "\r");
  } else {
    LOG_ERROR("Could not connect to W-LAN.", "\r");
  }
}
//---------------------------------------------



//---------------------------------------------
// Status LED
//---------------------------------------------
void statusLedInit() {
  pinMode(HEARTBEAT_LED, OUTPUT);
  digitalWrite(HEARTBEAT_LED, LOW);
}

void setLedMode(LedMode mode) {
  currentLedMode = mode;
}

void setErrorCode(ErrorCode code) {
  currentError = code;
  currentLedMode = LED_ERROR;
}

void ledTask(void *param) {

  while (true) {

    if (!system_config.debug_led) {
      digitalWrite(HEARTBEAT_LED, LOW);
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    switch (currentLedMode) {
      case LED_OFF:
        digitalWrite(HEARTBEAT_LED, LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
        break;

      case LED_UPLOAD:
        digitalWrite(HEARTBEAT_LED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(200));
        digitalWrite(HEARTBEAT_LED, LOW);
        vTaskDelay(pdMS_TO_TICKS(200));
        break;

      case LED_ERROR:
        // Fehlercode blinkt die LED
        for (int i = 0; i < (currentError + 1); i++) {
          digitalWrite(HEARTBEAT_LED, HIGH);
          vTaskDelay(pdMS_TO_TICKS(300));
          digitalWrite(HEARTBEAT_LED, LOW);
          vTaskDelay(pdMS_TO_TICKS(700));
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
        break;

      case LED_SAMPLING:
        // kurzes Blinken jede Sekunde für Sampling
        digitalWrite(HEARTBEAT_LED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(HEARTBEAT_LED, LOW);
        vTaskDelay(pdMS_TO_TICKS(900));
        break;

      case LED_HEARTBEAT:
        // optional: normaler Herzschlag
        digitalWrite(HEARTBEAT_LED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
        digitalWrite(HEARTBEAT_LED, LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
        break;
    }
  }
}
//---------------------------------------------



//---------------------------------------------
// FRAM
//---------------------------------------------
void writeMQTTToFRAM(uint16_t addr, const mqtt_struct &data) {
  const uint8_t *ptr = (const uint8_t *)&data;
  for (uint16_t i = 0; i < sizeof(mqtt_struct); i++) {
    fram.write8(addr + i, ptr[i]);
  }
}

void readMQTTFromFRAM(uint16_t addr, mqtt_struct &data) {
  uint8_t *ptr = (uint8_t *)&data;
  for (uint16_t i = 0; i < sizeof(mqtt_struct); i++) {
    ptr[i] = fram.read8(addr + i);
  }
}

void loadFramHeader() {
  fram.read(FRAM_HEADER_ADDR, (uint8_t *)&header, sizeof(header));

  if (header.magic != FRAM_MAGIC) {
    LOG_INFO("FRAM header invalid → initializing…");
    header.magic = FRAM_MAGIC;
    header.writeIndex = 0;
    header.readIndex = 0;
    saveFramHeader();
  }
}

void saveFramHeader() {
  fram.write(FRAM_HEADER_ADDR, (uint8_t *)&header, sizeof(header));
}
//---------------------------------------------
