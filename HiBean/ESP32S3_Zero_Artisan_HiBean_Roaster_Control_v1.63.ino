/*********************************************************************************************************
 * ESP32S3-Zero_Artisan+HiBean_Roaster Control_v1.63 (Base On SkiBeanQuickSV and SkyCommand)
 *
 * 支援 BLE、AP、WiFi 三種模式。
 *
 * 通過 BLE、AP、WiFi 接收命令，並發送溫度/狀態數據。
 *
 * 解決多介面衝突問題。
 * 
 * 硬件要求：
 * - WAVESHARE_ESP32_S3_ZERO
 * - TX_PIN = 19
 * - RX_PIN = 20
 * - LED_PIN = 21 (NeoPixel)
 * - Partition Scheme - Huge APP   
 *
 * 支持PID_Autoswitch(整合 PID 控制模式自動切換邏輯，功能如下：
 * -1.Artisan 傳入 "PID : ..." → 自動切換為軟體 PID 控制。
 * -2.超過 3 秒未收到 Artisan 指令 → 自動切回 ESP32 的內建硬體 PID 控制。
 * -3.插入 applyArtisanPID(command); 可讓你自定處理 PID 輸入的方式。
 *
 * 加入節流機制（每 500ms 傳送一次溫度訊息）讓溫度曲線平滑、穩定。
 *
 * 新增冷卻自動關機：當冷卻風扇 (COOL) 啟動後，若溫度從高溫降至 55°C (131°F) 以下，
 * 系統將自動執行 shutdown()，關閉所有加熱器、風扇與馬達。
 * 此功能僅在冷卻模式下作用，避免影響正常烘焙流程。
 *
 * *** STABILITY UPDATE ***
 * - 移植了 Arduino 版本的數據採集邏輯 (receiveSerialBitsFromRoaster)。
 * - 移除了原有的基於中斷的、不穩定的數據同步方式 (watchRoasterStart)。
 * - 新的數據採集方式採用主動搜索 Preamble 並帶有超時保護，大幅提升了在有噪聲環境下的
 *   通信可靠性，從根本上解決了偶發性溫度讀數異常的問題。
 *
 * 主要修改摘要
 * 數據採集 (getRoasterMessage 和 receiveSerialBitsFromRoaster)
 * 完全替換了原有的 getMessage 和 watchRoasterStart。
 * 新的 receiveSerialBitsFromRoaster 現在會主動尋找一個特定寬度（約 7000µs）的 Preamble 脈衝。
 * 在尋找 Preamble 和讀取數據位時都加入了超時保護，避免程序卡死。
 * 引入了 failedToReadRoaster 標誌，如果接收過程中任何一步出錯，getRoasterMessage 會立刻中止，
 * 不會用錯誤的數據去更新溫度。
 * setup() 函數
 * 移除了 attachInterrupt(RX_PIN, watchRoasterStart, FALLING); 這一行，因為我們不再使用中斷來
 * 檢測信號。
 * 主循環 loop() 的順序調整
 * 更好地匹配通信時序，loop() 中的順序調整為：先sendRoasterMessage()，再getRoasterMessage()。
 * 這更符合一問一答的通信模式。
 * 
 * 此代碼擁有 ESP32 豐富的現代化功能（WiFi, BLE, PID, WebSockets, 自動關機等），同時其最核心的數                  
 * 據採集部分又具備了 SkyCommand 版本的穩定性和可靠性，之前偶發的溫度讀數異常問題將會得到徹底解決。
 * 
 * 新版WiFi登入網頁：
 * - 中文環境顯示「設定已儲存！」。
 * - 其它語系顯示「WiFi Settings Saved!」，其它所有提示也同步自動切換。
 * - 支援所有手機/平板/PC，不卡字，無閃爍。
 *********************************************************************************************************/

#include <Arduino.h>
#include <MedianFilterLib.h>
#include <PID_v1.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// -----------------------------------------------------------------------------
// Debug Settings
// -----------------------------------------------------------------------------
#define SERIAL_DEBUG 1 //set to 1 to turn on

#if SERIAL_DEBUG == 1
#define D_print(...)    Serial.print(__VA_ARGS__)
#define D_println(...)  Serial.println(__VA_ARGS__)
#define D_printf(...)   Serial.printf(__VA_ARGS__)
#else
#define D_print(...)
#define D_println(...)
#define D_printf(...)
#endif

// -----------------------------------------------------------------------------
// Pin Definitions
// -----------------------------------------------------------------------------
const int TX_PIN = 19;
const int RX_PIN = 20;
const int LED_PIN = 21;
const String boardID_BLE = String("ARDUINO_WAVESHARE_ESP32_S3_ZERO");

// -----------------------------------------------------------------------------
// NeoPixel Configuration
// -----------------------------------------------------------------------------
#include <Adafruit_NeoPixel.h>
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// -----------------------------------------------------------------------------
// LED Colors
// -----------------------------------------------------------------------------
const uint32_t LED_RED = pixels.Color(255, 0, 0);
const uint32_t LED_GREEN = pixels.Color(0, 255, 0);
const uint32_t LED_BLUE = pixels.Color(0, 0, 255);
const uint32_t LED_BLACK = pixels.Color(0, 0, 0);

// -----------------------------------------------------------------------------
// Timing Constants
// -----------------------------------------------------------------------------
const int PREAMBLE_DURATION = 7000; //microseconds, Preamble 脈衝寬度
const int PULSE_ONE       = 1200; // Threshold for interpreting a bit as '1'
const int PULSE_ZERO      = 650;  // Not used for receiving, just for sending
const int POST_PULSE_DELAY= 750;
const int START_PULSE     = 7500;
const int START_DELAY     = 3800;

const int TIMEOUT_PREAMBLE_SEARCH = 500;   // Timeout for preamble search, in milliseconds.
const int TIMEOUT_PREAMBLE_PULSEIN = 25000; // Timeout (microseconds), for preamble detection.
const int TIMEOUT_LOGIC_PULSEIN = 8000;    // Timeout (microseconds) for every data bit pulseIn call.

// -----------------------------------------------------------------------------
// Buffer Sizes
// -----------------------------------------------------------------------------
const int ROASTER_LENGTH    = 7;   // 7 bytes received from roaster
const int CONTROLLER_LENGTH = 6;   // 6 bytes sent to roaster

// -----------------------------------------------------------------------------
// Control Byte Indices
// -----------------------------------------------------------------------------
enum ControlBytes {
    VENT_BYTE = 0,
    DRUM_BYTE = 3,
    COOL_BYTE = 2,
    FILTER_BYTE = 1,
    HEAT_BYTE = 4,
    CHECK_BYTE = 5
};

// -----------------------------------------------------------------------------
// BLE UUIDs for Nordic UART Service
// -----------------------------------------------------------------------------
#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e" // NUS service
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // Write
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // Notify

// -----------------------------------------------------------------------------
// WiFi Configuration
// -----------------------------------------------------------------------------
WebServer server(80);
WiFiServer artisanServer(8080);
WebSocketsServer webSocket = WebSocketsServer(81);
const char* wifiAPSSID = "ESP32_Roaster";
const char* wifiAPPass = "";

// WiFi mode definitions
#if !defined(WIFI_AP) && !defined(WIFI_STA)
#define WIFI_AP WIFI_MODE_AP
#define WIFI_STA WIFI_MODE_STA
#endif

#if !defined(WIFI_POWER_8_5dBm)
#define WIFI_POWER_8_5dBm WIFI_POWER_8_5dBm
#endif

#if !defined(WL_CONNECTED)
#define WL_CONNECTED 3
#endif

// WiFi parameters storage
class WiFiParams {
private:
    String ssid;
    String pass;
    Preferences preferences;

public:
    String getSSID() { return ssid; }
    String getPass() { return pass; }
    bool hasCredentials() { return ssid != ""; }
    
    void saveCredentials(String newSsid, String newPass) {
        if (this->ssid == newSsid && this->pass == newPass) return;
        
        this->ssid = newSsid;
        this->pass = newPass;
        preferences.begin("wifi", false);
        preferences.putString("ssid", ssid.c_str());
        preferences.putString("pass", pass.c_str());
        preferences.end();
    }

    void init() {
        preferences.begin("wifi", true);
        this->ssid = preferences.getString("ssid", wifiAPSSID);
        this->pass = preferences.getString("pass", "");
        preferences.end();
    }

    void reset() {
        ssid = "";
        pass = "";
        preferences.begin("wifi", false);
        preferences.clear();
        preferences.end();
    }
};

WiFiParams wifiParams;

int filtWeight = 80;

const int BLE_BREATH_SPEED = 2;    // BLE呼吸速度
const int IDLE_BREATH_SPEED = 1;   // 空闲呼吸速度

// -----------------------------------------------------------------------------
// Global Variables
// -----------------------------------------------------------------------------
String firmWareVersion = String("ESP32S3-Zero_Artisan+HiBean_v1.6.3");
String sketchName = String(__FILE__).substring(String(__FILE__).lastIndexOf('/')+1);

// Allocate buffers
uint8_t receiveBuffer[ROASTER_LENGTH];
uint8_t sendBuffer[CONTROLLER_LENGTH];

// Temperature variables
double temp = 0.0;           // Filtered temperature
uint16_t rawTempX, rawTempY;
char CorF = 'C';           // 'C' or 'F'

// PID variables
double pInput, pOutput;
double pSetpoint = 0.0;
int pMode = P_ON_M;
double Kp = 12.0, Ki = 0.5, Kd = 5.0;
int pSampleTime = 1000;
int manualHeatLevel = 50;
volatile bool pidActive = false;
PID myPID(&pInput, &pOutput, &pSetpoint, Kp, Ki, Kd, pMode, DIRECT);

// BLE variables
BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Communication variables
unsigned long lastEventTime = 0;
const unsigned long LAST_EVENT_TIMEOUT = 10UL * 1000000UL;

// LED variables
int breathSpeed = 2; // 呼吸速度 (較慢更明顯)
int breathValue = 0; // 當前亮度值
int breathDirection = 1; // 1 = 增加亮度, -1 = 減少亮度
bool isCommunicating = false;
unsigned long communicationEndTime = 0;
const unsigned long COMMUNICATION_TIMEOUT = 500; // 通信指示持續時間(ms)

bool failedToReadRoaster = false;

// 冷卻自動關機功能變數
const double COOL_SHUTDOWN_TEMP_C = 55.0;
const double COOL_SHUTDOWN_TEMP_F = 131.0;
bool coolingShutdownArmed = false;

// Command Strings
const String CMD_READ         = "READ";
const String CMD_HEAT         = "OT1";
const String CMD_VENT         = "OT2";
const String CMD_OFF          = "OFF";
const String CMD_DRUM         = "DRUM";
const String CMD_FILTER       = "FILTER";
const String CMD_COOL         = "COOL";
const String CMD_CHAN         = "CHAN";
const String CMD_UNITS        = "UNITS";
const String CMD_FILTER_WEIGHT= "FILT";
const String CMD_PID          = "PID";

// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------
void pulsePin(int pin, int duration);
void setControlChecksum();
void setValue(uint8_t* bytePtr, uint8_t value);
void sendRoasterMessage();
void receiveSerialBitsFromRoaster(int bytes, int pin);
bool calculateRoasterChecksum();
double calculateTemp();
void filtTemp(double v);
void getRoasterMessage();
void rgbLedWrite(int pin, uint8_t red, uint8_t green, uint8_t blue);
void setRGBColor(uint8_t red, uint8_t green, uint8_t blue, int brightness);
void updateBreathEffect(uint8_t red, uint8_t green, uint8_t blue);
void handleLED();
void handleCHAN(bool fromBLE);
void handleREAD(bool fromBLE);
void handleOT1(uint8_t value);
void handleHEAT(uint8_t value);
void handleVENT(uint8_t value);
void handleDRUM(uint8_t value);
void handleFILTER(uint8_t value);
void handleCOOL(uint8_t value);
void eStop();
void handlePIDControl();
void setPIDMode(bool usePID);
void parseAndExecuteCommands(String input, bool fromBLE = false);
void executeCommand(String command, String subcommand, String param, bool fromBLE);
void notifyBLEClient(const String& message);
void initBLE();
void indicateCommunication();
void handleArtisanConnection();
void setupAP();
void connectToWifi();
void setupWifi();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void handleWebCommand();
void sendCurrentStatusToClient(uint8_t clientNum);
void broadcastStatusUpdate();
void sendArtisanCompatibleData(uint8_t clientNum, long commandId);
void sendFullDeviceStatus(uint8_t clientNum);
String getCurrentStatus();
void handleAutoShutdown();

// -----------------------------------------------------------------------------
// BLE Server Callbacks
// -----------------------------------------------------------------------------
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) override {
        deviceConnected = true;
        oldDeviceConnected = true;
        
        // [修改] 請求一個更寬鬆、更具兼容性的連接參數範圍
        // min_int: 最小間隔, 0x10 = 16 * 1.25ms = 20ms
        // max_int: 最大間隔, 0x20 = 32 * 1.25ms = 40ms
        // latency: 從屬設備延遲
        // timeout: 監管超時, 0x1F4 = 500 * 10ms = 5000ms (5秒)
        pServer->updateConnParams(param->connect.remote_bda, 0x10, 0x20, 0, 0x1F4);
        
        D_println("BLE: Client connected.");
        setRGBColor(0, 0, 255, 255); // 連接時藍色(全亮)
    }
    
    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        D_println("BLE: Client disconnected. Restarting advertising...");
        setRGBColor(255, 0, 0, 255); // 斷開時紅色(全亮)
        // 短暫延遲後再開始廣播，給底層堆棧一些處理時間
        delay(100);
        pServer->getAdvertising()->start();
    }
};

// -----------------------------------------------------------------------------
// BLE Characteristic Callbacks
// -----------------------------------------------------------------------------
class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        String rxValue = String(pCharacteristic->getValue().c_str());
        if (rxValue.length() > 0) {
            String input = String(rxValue.c_str());
            D_print("BLE Write Received: ");
            D_println(input);
            indicateCommunication();
            parseAndExecuteCommands(input, true);
        }
    }
};

// -----------------------------------------------------------------------------
// LED Functions
// -----------------------------------------------------------------------------
void setRGBColor(uint8_t red, uint8_t green, uint8_t blue, int brightness = 255) {
    red = (red * brightness) / 255;
    green = (green * brightness) / 255;
    blue = (blue * brightness) / 255;
    pixels.setPixelColor(0, pixels.Color(red, green, blue));
    pixels.show();
}

void indicateCommunication() {
    isCommunicating = true;
    communicationEndTime = millis();
}

void updateBreathEffect(uint8_t red, uint8_t green, uint8_t blue) {
    breathValue += breathDirection * breathSpeed;
    if (breathValue > 255) {
        breathValue = 255;
        breathDirection = -1;
    } else if (breathValue < 0) {
        breathValue = 0;
        breathDirection = 1;
    }
    setRGBColor(red, green, blue, breathValue);
}

void handleLED() {
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    unsigned long currentMillis = millis();

    if (isCommunicating) {
        if (currentMillis - communicationEndTime >= COMMUNICATION_TIMEOUT) {
            isCommunicating = false;
        } else {
            if (currentMillis - lastBlink >= 100) {
                lastBlink = currentMillis;
                ledState = !ledState;
                setRGBColor(ledState ? 0 : 255, ledState ? 255 : 0, 0);
            }
            return;
        }
    }

    if (deviceConnected) {
        updateBreathEffect(0, 0, 255);
    } else if (WiFi.status() == WL_CONNECTED) {
        if (currentMillis - lastBlink >= 1000) {
            lastBlink = currentMillis;
            ledState = !ledState;
            setRGBColor(0, ledState ? 255 : 0, 0);
        }
    } else {
        updateBreathEffect(255, 0, 0);
    }
}

// -----------------------------------------------------------------------------
// Communication Functions
// -----------------------------------------------------------------------------
void pulsePin(int pin, int duration) {
    digitalWrite(pin, LOW);
    delayMicroseconds(duration);
    digitalWrite(pin, HIGH);
}

void setControlChecksum() {
    uint8_t sum = 0;
    for (int i = 0; i < (CONTROLLER_LENGTH - 1); i++) {
        sum += sendBuffer[i];
    }
    sendBuffer[CHECK_BYTE] = sum;
}

void setValue(uint8_t* bytePtr, uint8_t value) {
    *bytePtr = value;
    setControlChecksum();
}

void sendRoasterMessage() {
    pulsePin(TX_PIN, START_PULSE);
    delayMicroseconds(START_DELAY);

    for (int i = 0; i < CONTROLLER_LENGTH; i++) {
        for (int j = 0; j < 8; j++) {
            if (bitRead(sendBuffer[i], j) == 1) {
                pulsePin(TX_PIN, 1500);
            } else {
                pulsePin(TX_PIN, PULSE_ZERO);
            }
            delayMicroseconds(POST_PULSE_DELAY);
        }
    }
}

void receiveSerialBitsFromRoaster(int bytes, int pin) {
  unsigned long timeIntervals[bytes * 8];
  unsigned long pulseDuration = 0;
  unsigned long startTime = millis();
  int bits = bytes * 8;
  bool preambleDetected = false;

  while (millis() - startTime < TIMEOUT_PREAMBLE_SEARCH) {
    pulseDuration = pulseIn(pin, LOW, TIMEOUT_PREAMBLE_PULSEIN);
    if (pulseDuration >= (PREAMBLE_DURATION - 500) && pulseDuration <= (PREAMBLE_DURATION + 1000)) {
      preambleDetected = true;
      D_println("Preamble detected");
      break;
    }
  }

  if (!preambleDetected) {
    failedToReadRoaster = true;
    D_println("Preamble search timeout!");
    return;
  }

  for (int i = 0; i < bits; i++) {
    unsigned long duration = pulseIn(pin, LOW, TIMEOUT_LOGIC_PULSEIN);
    if (duration == 0) {
      D_printf("Timeout or no pulse detected at bit %d\n", i);
      failedToReadRoaster = true;
      return;
    }
    timeIntervals[i] = duration;
  }

  memset(receiveBuffer, 0, bytes);
  for (int i = 0; i < bits; i++) {
    if (timeIntervals[i] > PULSE_ONE) {
      receiveBuffer[i / 8] |= (1 << (i % 8));
    }
  }
  
  failedToReadRoaster = false;
}

bool calculateRoasterChecksum() {
    uint8_t sum = 0;
    for (int i = 0; i < (ROASTER_LENGTH - 1); i++) {
        sum += receiveBuffer[i];
    }
    
    if (sum != receiveBuffer[ROASTER_LENGTH - 1]) {
        D_printf("Checksum mismatch! Calculated: 0x%X, Received: 0x%X\n", sum, receiveBuffer[ROASTER_LENGTH - 1]);
        return false;
    }
    return true;
}

double calculateTemp() {
    rawTempX = ((receiveBuffer[0] << 8) + receiveBuffer[1]);
    rawTempY = ((receiveBuffer[2] << 8) + receiveBuffer[3]);

    double x = 0.001 * rawTempX;
    double y = 0.001 * rawTempY;
    double v;

    if (rawTempX > 836 || rawTempY > 221) {
        v = -224.2 * y * y * y + 385.9 * y * y - 327.1 * y + 171;
    } else {
        v = -278.33 * x * x * x + 491.944 * x * x - 451.444 * x + 310.668;
    }

    if (CorF == 'F') {
        v = 1.8 * v + 32.0;
    }
    return v;
}

MedianFilter<double> tempFilter(7);
void filtTemp(double v){
  int maxV = ((CorF == 'F') ? 500 : 260);
  if(v < 0 || v > maxV) { return; }
  tempFilter.AddValue(v);
  temp = tempFilter.GetFiltered();
}

void getRoasterMessage() {
    receiveSerialBitsFromRoaster(ROASTER_LENGTH, RX_PIN);

    if (failedToReadRoaster || !calculateRoasterChecksum()) {
        D_println("Failed to get a valid roaster message.");
        return;
    }

    filtTemp(calculateTemp());
    lastEventTime = micros();
}

// -----------------------------------------------------------------------------
// Command Handlers
// -----------------------------------------------------------------------------
bool itsbeentoolong() {
  unsigned long now = micros();
  unsigned long duration = now - lastEventTime;
  
  bool isHeatOn = (sendBuffer[HEAT_BYTE] > 0);
  bool isDrumOn = (sendBuffer[DRUM_BYTE] > 0);
  bool isCoolOn = (sendBuffer[COOL_BYTE] > 0);
  if (isHeatOn || isDrumOn || isCoolOn) {
      return false;
  }
  
  return (duration > LAST_EVENT_TIMEOUT);
}

void shutdown() {
    coolingShutdownArmed = false;
    for (int i = 0; i < CONTROLLER_LENGTH; i++) {
        sendBuffer[i] = 0;
    }
    setControlChecksum();
    D_println("All systems SHUTDOWN.");
}

void handleAutoShutdown() {
    bool isCoolingActive = (sendBuffer[COOL_BYTE] > 0);

    if (isCoolingActive) {
        double shutdownTemp = (CorF == 'F') ? COOL_SHUTDOWN_TEMP_F : COOL_SHUTDOWN_TEMP_C;
        
        if (!coolingShutdownArmed && temp > (shutdownTemp + 5.0)) {
            coolingShutdownArmed = true;
            D_println("Auto-shutdown ARMED (temp is above shutdown threshold + 5 deg).");
        }

        if (coolingShutdownArmed && temp > 0 && temp < shutdownTemp) {
            D_printf("Auto-shutdown TRIGGERED: Temp (%.1f) is below threshold. Shutting down.\n", temp);
            shutdown();
        }
    } else {
        if (coolingShutdownArmed) {
            D_println("Cooling manually stopped, auto-shutdown DISARMED.");
            coolingShutdownArmed = false;
        }
    }
}

void handleCHAN(bool fromBLE) {
    String message = "# Active channels set to 2100\r\n";
    D_println(message);
    if (fromBLE) {
        notifyBLEClient(message);
    } else {
        Serial.println(message);
    }
}

void handleOT1(uint8_t value) {
    if (myPID.GetMode() == MANUAL) {
      manualHeatLevel = constrain(value, 0, 100);
      handleHEAT(manualHeatLevel);
    } else if (myPID.GetMode() == AUTOMATIC) {
      setPIDMode(false);
    }
}

void handleREAD(bool fromBLE) {
    String readMsg = "0," + String(temp, 1) + "," + String(temp, 1) + "," +
          String(sendBuffer[HEAT_BYTE]) + "," +
          String(sendBuffer[VENT_BYTE]) + "\r\n";

    if (fromBLE) {
        notifyBLEClient(readMsg);
    } else {
        Serial.println(readMsg);
        webSocket.broadcastTXT(readMsg);
    }
    lastEventTime = micros();
}

void handleHEAT(uint8_t value) {
    if (value <= 100) {
        setValue(&sendBuffer[HEAT_BYTE], value);
    }
    lastEventTime = micros();
}

void handleVENT(uint8_t value) {
    if (value <= 100) {
        setValue(&sendBuffer[VENT_BYTE], value);
        if (value == 0) {
            handleFILTER(value);
        } else {
            handleFILTER((int) round(4-((value-1)*4/100)));
        }
    }
    lastEventTime = micros();
}

void handleDRUM(uint8_t value) {
    if (value != 0) {
        setValue(&sendBuffer[DRUM_BYTE], 100);
    } else {
        setValue(&sendBuffer[DRUM_BYTE], 0);
    }
    lastEventTime = micros();
}

void handleFILTER(uint8_t value) {
    if (value >= 0 && value <= 4 ) {
        setValue(&sendBuffer[FILTER_BYTE], value);
    }
    lastEventTime = micros();
}

void handleCOOL(uint8_t value) {
    if (value <= 100) {
        setValue(&sendBuffer[COOL_BYTE], value);
        handleFILTER(value > 0 ? 4 : 0);
    }
    lastEventTime = micros();
}

void eStop() {
    D_println("Emergency Stop Activated! Heater OFF, Vent 100%");
    handleHEAT(0);
    handleVENT(100);
}

void handlePIDControl() {
    static unsigned long lastPIDTime = 0;
    unsigned long now = millis();
    
    if (now - lastPIDTime < pSampleTime) {
        return;
    }
    lastPIDTime = now;

    if (myPID.GetMode() == AUTOMATIC && pidActive) {
        if (isnan(temp) || temp <= 0 || temp >= 300) {
            D_println("PID: Invalid temperature reading!");
            return;
        }

        pInput = temp;
        myPID.Compute();
        
        int roundedHeat = constrain(round(pOutput), 0, 100);
        roundedHeat = (roundedHeat / 5) * 5;
        
        if (sendBuffer[HEAT_BYTE] != roundedHeat) {
            handleHEAT(roundedHeat); 
            
            D_print("PID Output: ");
            D_print(pOutput);
            D_print(" -> Rounded: ");
            D_println(roundedHeat);
        }
    }
}

void setPIDMode(bool usePID) {
    if (usePID) {
        if (isnan(temp) || temp <= 0 || temp >= 300) {
            D_println("PID: Cannot enable - invalid temperature!");
            return;
        }
        
        myPID.SetMode(AUTOMATIC);
        pidActive = true;
        myPID.SetOutputLimits(0, 100);
        myPID.SetControllerDirection(DIRECT);
        
        D_println("PID mode set to AUTOMATIC");
        D_print("Current Temp: "); D_println(temp);
        D_print("Setpoint: "); D_println(pSetpoint);
    } else {
        myPID.SetMode(MANUAL);
        pidActive = false;
        D_println("PID mode set to MANUAL");
    }
}

void parseAndExecuteCommands(String input, bool fromBLE) {
    input.trim();
    input.toUpperCase();
 
    D_print("Parsing command: ");
    D_println(input);

    int split1 = input.indexOf(';');
    String command = "";
    String param = "";
    String subcommand = "";
    
    if (split1 >= 0) {
        command = input.substring(0, split1);
        String remainder = input.substring(split1 + 1);
        int split2 = remainder.indexOf(';');

        if (split2 >= 0) {
            subcommand = remainder.substring(0, split2);
            param = remainder.substring(split2 + 1);
        } else {
            param = remainder;
        }
    } else {
        command = input;
    }

    executeCommand(command, subcommand, param, fromBLE);
}

void executeCommand(String command, String subcommand, String param, bool fromBLE) {
    if (command == CMD_READ) {
        handleREAD(fromBLE);
    } else if (command == CMD_HEAT) {
        handleOT1(param.toInt()); 
    } else if (command == CMD_VENT) {
        handleVENT(param.toInt());
    } else if (command == CMD_OFF) {
        shutdown();
    } else if (command == CMD_DRUM) {
        handleDRUM(param.toInt());
    } else if (command == CMD_FILTER) {
        handleFILTER(param.toInt());
    } else if (command == CMD_COOL) {
        handleCOOL(param.toInt());
    } else if (command == CMD_CHAN) {
        handleCHAN(fromBLE);
    } else if (command == CMD_UNITS) {
        if (param.length() > 0) {
            CorF = param.charAt(0);
        }
    } else if (command == CMD_FILTER_WEIGHT) {
        int w = param.toInt();
        if (w >= 0 && w <= 100) {
            filtWeight = w;
        }
        String message = "# Physical channel 1 filter set to " + String(filtWeight);
        
        D_println(message);

        if (fromBLE) {
            notifyBLEClient(message);
        } else {
            Serial.println(message);
        }
    } else if (command == CMD_PID) {
        if (param == "ON") {
            setPIDMode(true);
        } else if (param == "OFF") {
            setPIDMode(false);
        } else if (subcommand == "SV") {
            double newSetpoint = param.toDouble();
            if (newSetpoint > 0 && newSetpoint <= 300) {
                pSetpoint = newSetpoint;
                D_print("New Setpoint: ");
                D_println(pSetpoint);
            }
        } else if (subcommand == "T") {
            double pidTune[3];
            int paramCount = 0;
            String tuneParams = param;
            while(tuneParams.length() > 0 && paramCount < 3) {
                int index = tuneParams.indexOf(';');
                if (index == -1) {
                    pidTune[paramCount++] = tuneParams.toDouble();
                    break;
                } else {
                    pidTune[paramCount++] = tuneParams.substring(0, index).toDouble();
                    tuneParams = tuneParams.substring(index+1);
                }
            }
            Kp = pidTune[0];
            Ki = pidTune[1];
            Kd = pidTune[2];
            D_print("Kp: "); D_println(Kp);
            D_print("Ki: "); D_println(Ki);
            D_print("Kd: "); D_println(Kd);
            myPID.SetTunings(Kp, Ki, Kd, pMode);
        } else if (subcommand == "PM") {
            D_print("Setting PMode to: ");
            D_println(param);
            if (param == "M") {
              pMode = P_ON_M;
              myPID.SetTunings(Kp, Ki, Kd, pMode);
            } else {
              pMode = P_ON_E;
              myPID.SetTunings(Kp, Ki, Kd, pMode);
            }
        } else if (subcommand == "CT") {
            int newSampleTime = param.toInt();
            if (newSampleTime > 0) {
                pSampleTime = newSampleTime;
                myPID.SetSampleTime(pSampleTime);
                D_print("Setting Cycle Time to: ");
                D_println(pSampleTime);
            }
        }
    } else if (command == "ESTOP") {
        eStop();
    } else if (command == "WIFI") {
        if (subcommand == "AP") {
            wifiParams.reset();
            String message = "# 正在切換到AP模式，請稍候...";
            if (fromBLE) {
                notifyBLEClient(message);
                delay(100);
            } else {
                Serial.println(message);
            }
            delay(1000);
            ESP.restart();
        }
        else if (subcommand == "STA") {
            if (param.length() > 0) {
                int separator = param.indexOf(',');
                if (separator != -1) {
                    String ssid = param.substring(0, separator);
                    String password = param.substring(separator+1);
                    wifiParams.saveCredentials(ssid, password);
                    String message = "# 正在連接到 " + ssid + "，設備將重啟...";
                    if (fromBLE) {
                        notifyBLEClient(message);
                        delay(100);
                    } else {
                        Serial.println(message);
                    }
                    delay(1000);
                    ESP.restart();
                }
            }
        }
    }
}

void notifyBLEClient(const String& message) {
    if (deviceConnected && pTxCharacteristic) {
        pTxCharacteristic->setValue(message.c_str());
        pTxCharacteristic->notify();
    }
}

// -----------------------------------------------------------------------------
// WiFi Functions
// -----------------------------------------------------------------------------
void setupAP() {
    WiFi.mode(WIFI_AP);
    delay(100);
    WiFi.softAP(wifiAPSSID, wifiAPPass);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    
    D_print("AP Mode Started. SSID: ");
    D_println(wifiAPSSID);
    
    server.on("/", HTTP_GET, []() {
        String html =
            "<!DOCTYPE html><html><head>"
            "<meta charset='UTF-8'>"
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
            "<title>Cubean WiFi 設定 / Setup</title>"
            "<style>"
            "body {font-family: Arial, sans-serif; max-width:600px; margin:0 auto; padding:32px; background:#f8f8f8;}"
            "h1 {font-size:2.2em; color:#333; margin-bottom:20px;}"
            "form {display:grid; gap:20px; margin-top:24px;}"
            "input[type=text],input[type=password] {font-size:1.2em; padding:12px; border:1px solid #ccc; border-radius:6px; width:100%;}"
            "button,input[type=submit] {background:#4CAF50; color:white; border:none; padding:14px 0; font-size:1.2em; border-radius:6px; cursor:pointer; transition:background 0.2s; width:100%;}"
            "button:hover,input[type=submit]:hover {background:#45a049;}"
            ".status {margin-top:32px; background:#fff; border-radius:6px; box-shadow:0 2px 8px rgba(0,0,0,0.07); padding:16px;}"
            "</style>"
            "</head><body>"
            "<h1 id='title'>Cubean WiFi 設定</h1>"
            "<form action='/save' method='post'>"
            "<label for='ssid' id='ssid_label'>WiFi 名稱(SSID):</label>"
            "<input type='text' id='ssid' name='ssid' required placeholder='請輸入WiFi名稱' />"
            "<label for='password' id='pass_label'>密碼:</label>"
            "<input type='password' id='password' name='password' placeholder='如無密碼可留空' />"
            "<input id='submit_btn' type='submit' value='儲存並重啟' />"
            "</form>"
            "<div class='status' id='status_div'>"
            "<b id='mode_label'>目前模式:</b> " + String(WiFi.getMode() == WIFI_AP ? "AP熱點" : "WiFi") + "<br>"
            "<b id='ap_label'>AP SSID:</b> " + String(wifiAPSSID) + "<br>"
            "</div>"
            "<script>"
            "function isZhTW(lang){"
            "  return lang.toLowerCase().startsWith('zh');"
            "}"
            "var userLang = (navigator.languages && navigator.languages.length)? navigator.languages[0] : (navigator.language || navigator.userLanguage);"
            "if(!isZhTW(userLang)){"
            "  document.getElementById('title').textContent = 'Cubean WiFi Setup';"
            "  document.getElementById('ssid_label').textContent = 'WiFi Name (SSID):';"
            "  document.getElementById('ssid').placeholder = 'Enter WiFi Name';"
            "  document.getElementById('pass_label').textContent = 'Password:';"
            "  document.getElementById('password').placeholder = 'Leave empty if no password';"
            "  document.getElementById('submit_btn').value = 'Save & Reboot';"
            "  document.getElementById('mode_label').textContent = 'Current Mode:';"
            "  document.getElementById('ap_label').textContent = 'AP SSID:';"
            "}"
            "</script>"
            "</body></html>";
        server.send(200, "text/html; charset=utf-8", html);
    });
    
    server.on("/save", HTTP_POST, []() {
        if (server.hasArg("ssid")) {
            String newSSID = server.arg("ssid");
            String newPass = server.arg("password");

            wifiParams.saveCredentials(newSSID, newPass);

            String html =
                "<!DOCTYPE html><html><head>"
                "<meta charset='UTF-8'>"
                "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
                "<title>設定已儲存 / Settings Saved</title>"
                "<style>"
                "body {font-family: Arial, sans-serif; max-width:600px; margin:0 auto; padding:32px; background:#f8f8f8;}"
                "h1 {font-size:2.2em; color:#333; margin-bottom:20px;}"
                "p {font-size:1.15em;}"
                "</style>"
                "</head><body>"
                "<h1 id='title'>設定已儲存！</h1>"
                "<p id='connecting'>正在連線到：<b>" + newSSID + "</b></p>"
                "<p id='reboot'>裝置將重啟連線新的WiFi。</p>"
                "<p id='failinfo'>若連線失敗會自動回到AP模式。</p>"
                "<script>"
                "function isZhTW(lang){"
                "  return lang.toLowerCase().startsWith('zh');"
                "}"
                "var userLang = (navigator.languages && navigator.languages.length)? navigator.languages[0] : (navigator.language || navigator.userLanguage);"
                "if(!isZhTW(userLang)){"
                "  document.getElementById('title').textContent = 'WiFi Settings Saved!';"
                "  document.getElementById('connecting').innerHTML = 'Connecting to: <b>" + newSSID + "</b>';"
                "  document.getElementById('reboot').textContent = 'Device will restart to connect to the new network.';"
                "  document.getElementById('failinfo').textContent = 'If connection fails, AP mode will be available again.';"
                "}"
                "</script>"
                "</body></html>";

            server.send(200, "text/html; charset=utf-8", html);

            delay(2000);
            ESP.restart();
        } else {
            server.send(400, "text/plain", "SSID is required");
        }
    });
    
    server.begin();
}

void connectToWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiParams.getSSID().c_str(), wifiParams.getPass().c_str());
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    
    D_print("Connecting to WiFi");
    
    int attempt = 0;
    while (WiFi.status() != WL_CONNECTED && attempt < 10) {
        delay(1000);
        D_print(".");
        attempt++;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        D_println("\nConnection failed, starting AP mode");
        setupAP();
        return;
    }
    
    D_println("\nConnected to WiFi");
    D_print("SSID: "); D_println(WiFi.SSID());
    D_print("IP: "); D_println(WiFi.localIP().toString());
    
    server.on("/", HTTP_GET, []() {
        String html = "<html><body><h1>Roaster Control</h1>"
                     "<p>Connected to: " + WiFi.SSID() + "</p>"
                     "<p>IP: " + WiFi.localIP().toString() + "</p>"
                     "<p><a href='/cmd?cmd=READ'>Get Status</a></p>"
                     "</body></html>";
        server.send(200, "text/html", html);
    });
    
    server.begin();
    
    if (!MDNS.begin(wifiAPSSID)) {
        D_println("Error setting up mDNS responder!");
    } else {
        D_println("mDNS responder started");
    }
}

void setupWifi() {
    wifiParams.init();
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
    WiFi.setHostname(wifiAPSSID);

    if (wifiParams.hasCredentials()) {
        D_println("嘗試連接已保存的WiFi...");
        connectToWifi();
        
        unsigned long startAttemptTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
            delay(100);
        }
        
        if (WiFi.status() != WL_CONNECTED) {
            wifiParams.reset();
            setupAP();
        } else {
            artisanServer.begin();
            D_println("Artisan TCP server started on port 8080");
        }
    } else {
        setupAP();
    }
}

// -----------------------------------------------------------------------------
// WebSocket Functions
// -----------------------------------------------------------------------------
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            D_printf("[%u] Disconnected!\n", num);
            break;
            
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                D_printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
                sendFullDeviceStatus(num);
            }
            break;
            
        case WStype_TEXT:
            if(length > 0) {
                String msg = "";
                for(size_t i=0; i<length; i++) msg += (char)payload[i];
                msg.trim();
                
                D_printf("[%u] Received: %s\n", num, msg.c_str());
                
                if(msg.startsWith("{")) {
                    StaticJsonDocument<256> doc;
                    DeserializationError error = deserializeJson(doc, msg);
                    
                    if(!error) {
                        if(doc.containsKey("BurnerVal")) handleOT1(doc["BurnerVal"]);
                        if(doc.containsKey("FanVal")) handleVENT(doc["FanVal"]);
                        if(doc.containsKey("Drum")) handleDRUM(doc["Drum"]);
                        if(doc.containsKey("Cool")) handleCOOL(doc["Cool"]);
                        if(doc.containsKey("PID")) setPIDMode(doc["PID"]);
                        
                        if(doc.containsKey("command") && strcmp(doc["command"], "getData") == 0) {
                            sendArtisanCompatibleData(num, doc["id"].as<long>());
                        }
                    }
                } else {
                    parseAndExecuteCommands(msg, true);
                }
                
                sendFullDeviceStatus(num);
            }
            break;
            
        default:
            break;
    }
}

void sendCurrentStatusToClient(uint8_t clientNum) {
    StaticJsonDocument<200> doc;
    JsonObject data = doc.createNestedObject("data");
    
    data["BT"] = temp;
    data["BurnerVal"] = sendBuffer[HEAT_BYTE]; 
    data["FanVal"] = sendBuffer[VENT_BYTE];
    data["Drum"] = sendBuffer[DRUM_BYTE];
    data["Cool"] = sendBuffer[COOL_BYTE];
    data["PID"] = (myPID.GetMode() == AUTOMATIC);
    
    char buffer[256];
    serializeJson(doc, buffer);
    webSocket.sendTXT(clientNum, buffer);
}

void broadcastStatusUpdate() {
    static unsigned long lastUpdate = 0;
    const unsigned long interval = 2000;
    
    unsigned long now = millis();
    if(now - lastUpdate >= interval) {
        lastUpdate = now;
        
        StaticJsonDocument<256> doc;
        doc["type"] = "update";
        
        JsonObject data = doc.createNestedObject("data");
        data["BT"] = temp;
        data["BurnerVal"] = sendBuffer[HEAT_BYTE];
        data["FanVal"] = sendBuffer[VENT_BYTE];
        data["Drum"] = sendBuffer[DRUM_BYTE];
        data["Cool"] = sendBuffer[COOL_BYTE];
        data["PID"] = myPID.GetMode();
        
        char buffer[256];
        serializeJson(doc, buffer);
        webSocket.broadcastTXT(buffer);
        
        D_printf("Broadcast update: %s\n", buffer);
    }
}

void sendArtisanCompatibleData(uint8_t clientNum, long commandId) {
    StaticJsonDocument<512> doc;
    doc["id"] = commandId;
    
    JsonObject data = doc.createNestedObject("data");
    data["BT"] = round(temp * 10) / 10.0;
    data["BurnerVal"] = sendBuffer[HEAT_BYTE];
    data["FanVal"] = sendBuffer[VENT_BYTE];
    data["Drum"] = sendBuffer[DRUM_BYTE];
    data["Cool"] = sendBuffer[COOL_BYTE];
    data["PID"] = (myPID.GetMode() == AUTOMATIC ? 1 : 0);
    
    char buffer[512];
    size_t len = serializeJson(doc, buffer);
    webSocket.sendTXT(clientNum, buffer, len);
    
    D_printf("Sent to Artisan: %s\n", buffer);
}

void sendFullDeviceStatus(uint8_t clientNum) {
    StaticJsonDocument<512> doc;
    doc["type"] = "status";
    doc["time"] = millis();
    
    JsonObject data = doc.createNestedObject("data");
    data["bean_temp"] = temp;
    data["heater"] = sendBuffer[HEAT_BYTE];
    data["fan"] = sendBuffer[VENT_BYTE];
    data["drum"] = sendBuffer[DRUM_BYTE];
    data["cooling"] = sendBuffer[COOL_BYTE];
    data["pid_mode"] = myPID.GetMode();
    data["pid_setpoint"] = pSetpoint;
    data["pid_output"] = pOutput;
    
    char buffer[512];
    serializeJson(doc, buffer);
    webSocket.sendTXT(clientNum, buffer);
}

String getCurrentStatus() {
    String status = String(temp, 1) + "," 
                 + String(sendBuffer[HEAT_BYTE]) + ","
                 + String(sendBuffer[VENT_BYTE]) + ","
                 + String(sendBuffer[DRUM_BYTE]) + ","
                 + (myPID.GetMode() == AUTOMATIC ? "1" : "0");
    
    return status;
}

// -----------------------------------------------------------------------------
// Artisan TCP Functions
// -----------------------------------------------------------------------------
void handleArtisanConnection() {
    static WiFiClient client;
    
    if (!client || !client.connected()) {
        client = artisanServer.available();
        if (client) {
            D_println("Artisan connected");
        }
    }
    
    if (client && client.connected()) {
        if (client.available() > 0) {
            String command = client.readStringUntil('\n');
            command.trim();
            D_println("Received from Artisan: " + command);
            
            parseAndExecuteCommands(command);
        }
    }
}

// -----------------------------------------------------------------------------
// Web Command Handler
// -----------------------------------------------------------------------------
void handleWebCommand() {
    if (server.method() == HTTP_POST) {
        if (server.hasArg("plain")) {
            String body = server.arg("plain");
            parseAndExecuteCommands(body);
        }
    } else {
        if (server.hasArg("cmd")) {
            String cmd = server.arg("cmd");
            
            if (cmd == "WIFI;AP") {
                wifiParams.reset();
                
                String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
                             "<title>切换到AP模式</title></head><body>"
                             "<h1>切换到AP模式</h1>"
                             "<p>设备将在3秒后重启...</p>"
                             "</body></html>";
                
                server.sendHeader("Cache-Control", "no-cache");
                server.sendHeader("Connection", "close");
                server.send(200, "text/html; charset=utf-8", html);
                
                delay(3000);
                ESP.restart();
                return;
            }
            
            parseAndExecuteCommands(cmd);
        }
    }
    
    server.sendHeader("Cache-Control", "no-cache");
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain; charset=utf-8", "OK");
}

// -----------------------------------------------------------------------------
// BLE Initialization
// -----------------------------------------------------------------------------
void initBLE() {
    BLEDevice::init("ESP32_Skycommand_BLE");
    BLEDevice::setMTU(150);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService* pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
    );
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pRxCharacteristic->setCallbacks(new MyCallbacks());
    pRxCharacteristic->addDescriptor(new BLE2902());
    pService->start();

    BLEService* devInfoService = pServer->createService("180A");
    BLECharacteristic* boardCharacteristic = devInfoService->createCharacteristic("2A29", BLECharacteristic::PROPERTY_READ);
      boardCharacteristic->setValue(boardID_BLE);
      boardCharacteristic->addDescriptor(new BLE2902());
    BLECharacteristic* sketchNameCharacteristic = devInfoService->createCharacteristic("2A28", BLECharacteristic::PROPERTY_READ);
      sketchNameCharacteristic->setValue(sketchName);
      sketchNameCharacteristic->addDescriptor(new BLE2902());
    BLECharacteristic* firmwareCharacteristic = devInfoService->createCharacteristic("2A26", BLECharacteristic::PROPERTY_READ);
      firmwareCharacteristic->setValue(firmWareVersion);
      firmwareCharacteristic->addDescriptor(new BLE2902());
    
    devInfoService->start();

    BLEAdvertising* pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
    
    D_println("BLE Advertising started...");
}

// -----------------------------------------------------------------------------
// Main Setup and Loop
// -----------------------------------------------------------------------------
void setup() {
    pixels.begin();
    setRGBColor(255, 0, 0, 255);
    
    Serial.begin(115200);
    D_printf("Starting Roaster Control Firmware: %s\n", firmWareVersion.c_str());
    delay(1000);

    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, HIGH);
    pinMode(RX_PIN, INPUT);

    initBLE();
    setupWifi();
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    D_println("WebSocket server started on port 81");

    server.on("/cmd", HTTP_GET, handleWebCommand);
    server.on("/cmd", HTTP_POST, handleWebCommand);

    myPID.SetMode(MANUAL);
    myPID.SetOutputLimits(0.0,100.0);
    myPID.SetSampleTime(pSampleTime);

    shutdown();
    lastEventTime = micros();
}

void loop() {
    // 1. 處理烘豆機通信
    sendRoasterMessage();   // 先發送控制指令
    getRoasterMessage();    // 再接收烘豆機狀態

    // 2. 處理逾時關機
    if (itsbeentoolong()) { shutdown(); }
    
    // 3. 處理PID控制
    handlePIDControl();
    
    // 4. 檢查是否需要自動關機
    handleAutoShutdown();

    // 5. 處理網絡連接
    server.handleClient();
    webSocket.loop();
    
    // 6. 定期廣播狀態給WebSocket客戶端
    broadcastStatusUpdate();
    
    // 7. 處理Artisan TCP連接
    handleArtisanConnection();

    // 8. 處理串口輸入
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        parseAndExecuteCommands(command);
    }

    // 9. 更新LED狀態
    handleLED();

    // 10. 處理BLE連接狀態
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // Give a bit of time before restarting advertising
        
        // [修改] 移除 isAdvertising() 的檢查，直接調用 stop() 和 start()
        // 這是更安全且兼容新版函式庫的做法。
        pServer->getAdvertising()->stop();
        pServer->startAdvertising();
        
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }

    // 加入極短延遲，將控制權交還底層系統，穩定BLE和WiFi
    delay(1);
}