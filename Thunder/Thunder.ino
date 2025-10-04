//================================================================
// ライブラリ
//================================================================
#include <WiFi.h> // Raspberry Pi Pico W用のWiFiライブラリ
#include <WiFiServer.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <time.h>
#include <stdlib.h>
#include <DFRobot_DHT20.h>
#include <SparkFun_AS3935.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <LittleFS.h>
#include <math.h> // for round()
#include <ArduinoOTA.h> // 標準の無線書き込み(OTA)用ライブラリ
#include "config.h"      // ★★★ 設定ファイルを読み込む

//================================================================
// ★★★ 動作設定 (このエリアは編集可能です) ★★★
//================================================================
// --- 雷センサー感度設定 ---
// 各設定値を変更することで、雷の検知感度を調整できます。
// AUTO_NOISE_LEVEL_CONTROL:
//   trueにすると、ノイズを検知した際に自動で感度を下げます（誤検知は減りますが、遠雷を逃す可能性も上がります）。
//   falseにすると、常に設定した感度(INITIAL_NOISE_LEVEL)を維持します（より多くの雷を検知できますが、誤検知も増える可能性があります）。
const bool AUTO_NOISE_LEVEL_CONTROL = false;
// LIGHTNING_WATCHDOG_THRESHOLD: 雷と判断するための信号強度の閾値です。(調整範囲: 1～10, 小さいほど高感度)
const uint8_t LIGHTNING_WATCHDOG_THRESHOLD = 2;
// LIGHTNING_SPIKE_REJECTION: 電気的スパイクノイズの除去レベルです。(調整範囲: 0～11, 小さいほど高感度)
const uint8_t LIGHTNING_SPIKE_REJECTION = 2;
// INITIAL_NOISE_LEVEL: 周囲の環境ノイズレベルの初期値です。(調整範囲: 1～7, 小さいほど高感度)
const uint8_t INITIAL_NOISE_LEVEL = 2;

// --- 動作設定 ---
const unsigned long LONG_PRESS_DURATION_MS = 1000;
const unsigned long INACTIVITY_TIMEOUT_MS = 5000;
const unsigned long BACKLIGHT_DURATION_MS = 3000;
const unsigned long IP_DISPLAY_DURATION_MS = 5000;
const unsigned long ACTION_RESULT_DISPLAY_MS = 2000; // アクション結果の表示時間
const int HISTORY_SIZE = 3;
const int LCD_COLS = 20;
const int LCD_ROWS = 4;


//================================================================
// ピン定義
//================================================================
namespace Pins {
const int LCD_RS = 2, LCD_E = 3, LCD_D4 = 4, LCD_D5 = 5, LCD_D6 = 6, LCD_D7 = 7;
const int LCD_BACKLIGHT = 14;
const int LED_R = 28, LED_G = 27, LED_B = 26;
const int BUTTON = 15;
const int I2C_SDA = 0, I2C_SCL = 1;
const int LIGHTNING_IRQ = 16;
const int AS3935_ADDR = 0x03;
}

//================================================================
// グローバル状態管理
//================================================================
namespace State {
// --- 動作モード ---
enum Mode {
    MAIN_DISPLAY, MENU, HISTORY, SWITCHBOT_APPLIANCE_SELECT,
    DEVICE_CONTROL, WAKE_ON_LAN, ULTRASONIC_MONITOR, SENSOR_DIAGNOSTICS
};

// --- メニューの状態 ---
struct MenuState {
    Mode currentMode = MAIN_DISPLAY;
    int menuSelection = 0;
    int deviceSelection = 0;
    int commandSelection = 0;
};

// --- システムの状態 ---
struct SystemState {
    bool illuminationOn = false;
    bool backlightAlwaysOn = false;
    bool ntpInitialized = false;
    bool isAutoResync = false;
    bool needsRedraw = true;
    bool forceMainScreenRedraw = true;
    uint8_t currentNoiseLevel = 2;
    bool initialLogSent = false;
    bool otaInitialized = false; // OTA初期化フラグ
};

// --- センサーデータ ---
struct SensorData {
    float temperature = -999.0;
    float humidity = -999.0;
    float ultrasonicDistance = -1.0;
    char lastEventTime[20] = "N/A";
    String lastEventType = "None"; // "Lightning" or "Noise"
    int lastLightningDistance = -1;
};

// --- 本体メモリに記録する雷履歴 ---
struct EventRecord {
    char timestamp[20];
    String type;
    int distance;
};

struct HistoryState {
    EventRecord records[HISTORY_SIZE];
    int index = 0;
    int count = 0;
};


// --- タイマー ---
struct TimerState {
    unsigned long lastActivity = 0;
    unsigned long backlightOn = 0;
};

// --- 暗号化コンテキスト ---
struct SHA256_CTX { uint8_t data[64]; uint32_t datalen; uint64_t bitlen; uint32_t state[8]; };


// --- グローバルオブジェクトと状態変数のインスタンス ---
MenuState menu;
SystemState system;
SensorData sensors;
HistoryState history;
TimerState timers;

LiquidCrystal lcd(Pins::LCD_RS, Pins::LCD_E, Pins::LCD_D4, Pins::LCD_D5, Pins::LCD_D6, Pins::LCD_D7);
DFRobot_DHT20 dht20;
SparkFun_AS3935 lightning(Pins::AS3935_ADDR);
WiFiServer server(80);
String childIpAddress = "";
volatile bool lightningInterruptFlag = false;
// ★★★ マルチコア間でWi-Fi接続状態を共有するためのフラグ ★★★
// volatileキーワードは、コンパイラの最適化によって変数が無視されるのを防ぎます
volatile bool g_wifiConnected = false;
volatile bool g_wolTriggered = false;
volatile byte g_wolTarget = 0;
volatile bool g_trigger_dht_read = false;
volatile bool g_trigger_wol_poll = false;
volatile bool g_pending_lightning_log_core1 = false;
volatile bool g_update_sensor_display = false;

}

//================================================================
// 暗号化ユーティリティ (編集不要)
//================================================================
namespace Crypto {
const uint32_t K[64] = {0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
uint32_t rotr(uint32_t x, uint32_t n) { return (x >> n) | (x << (32 - n)); }
void sha256_transform(State::SHA256_CTX *ctx, const uint8_t data[]) {
    uint32_t a, b, c, d, e, f, g, h, i, j, t1, t2, m[64];
    for (i = 0, j = 0; i < 16; ++i, j += 4) m[i] = (data[j] << 24) | (data[j + 1] << 16) | (data[j + 2] << 8) | (data[j + 3]);
    for (; i < 64; ++i) m[i] = (((m[i - 2] >> 17) | (m[i - 2] << 15)) ^ ((m[i - 2] >> 19) | (m[i - 2] << 13)) ^ (m[i - 2] >> 10)) + m[i - 7] + (((m[i - 15] >> 7) | (m[i - 15] << 25)) ^ ((m[i - 15] >> 18) | (m[i - 15] << 14)) ^ (m[i - 15] >> 3)) + m[i - 16];
    a = ctx->state[0]; b = ctx->state[1]; c = ctx->state[2]; d = ctx->state[3]; e = ctx->state[4]; f = ctx->state[5]; g = ctx->state[6]; h = ctx->state[7];
    for (i = 0; i < 64; ++i) {
        t1 = h + (((e >> 6) | (e << 26)) ^ ((e >> 11) | (e << 21)) ^ ((e >> 25) | (e << 7))) + ((e & f) ^ (~e & g)) + K[i] + m[i];
        t2 = (((a >> 2) | (a << 30)) ^ ((a >> 13) | (a << 19)) ^ ((a >> 22) | (a << 10))) + ((a & b) ^ (a & c) ^ (b & c));
        h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    ctx->state[0] += a; ctx->state[1] += b; ctx->state[2] += c; ctx->state[3] += d; ctx->state[4] += e; ctx->state[5] += f; ctx->state[6] += g; ctx->state[7] += h;
}
void sha256_init(State::SHA256_CTX *ctx) {
    ctx->datalen = 0; ctx->bitlen = 0;
    ctx->state[0] = 0x6a09e667; ctx->state[1] = 0xbb67ae85; ctx->state[2] = 0x3c6ef372; ctx->state[3] = 0xa54ff53a;
    ctx->state[4] = 0x510e527f; ctx->state[5] = 0x9b05688c; ctx->state[6] = 0x1f83d9ab; ctx->state[7] = 0x5be0cd19;
}
void sha256_update(State::SHA256_CTX *ctx, const uint8_t data[], size_t len) {
    for (uint32_t i = 0; i < len; ++i) {
        ctx->data[ctx->datalen] = data[i]; ctx->datalen++;
        if (ctx->datalen == 64) { sha256_transform(ctx, ctx->data); ctx->bitlen += 512; ctx->datalen = 0; }
    }
}
void sha256_final(State::SHA256_CTX *ctx, uint8_t hash[]) {
    uint32_t i = ctx->datalen;
    if (ctx->datalen < 56) { ctx->data[i++] = 0x80; while (i < 56) ctx->data[i++] = 0x00;
    } else { ctx->data[i++] = 0x80; while (i < 64) ctx->data[i++] = 0x00; sha256_transform(ctx, ctx->data); memset(ctx->data, 0, 56); }
    ctx->bitlen += ctx->datalen * 8;
    ctx->data[63] = ctx->bitlen; ctx->data[62] = ctx->bitlen >> 8; ctx->data[61] = ctx->bitlen >> 16; ctx->data[60] = ctx->bitlen >> 24;
    ctx->data[59] = ctx->bitlen >> 32; ctx->data[58] = ctx->bitlen >> 40; ctx->data[57] = ctx->bitlen >> 48; ctx->data[56] = ctx->bitlen >> 56;
    sha256_transform(ctx, ctx->data);
    for (i = 0; i < 4; ++i) {
        hash[i]      = (ctx->state[0] >> (24 - i * 8)) & 0xff; hash[i + 4]  = (ctx->state[1] >> (24 - i * 8)) & 0xff;
        hash[i + 8]  = (ctx->state[2] >> (24 - i * 8)) & 0xff; hash[i + 12] = (ctx->state[3] >> (24 - i * 8)) & 0xff;
        hash[i + 16] = (ctx->state[4] >> (24 - i * 8)) & 0xff; hash[i + 20] = (ctx->state[5] >> (24 - i * 8)) & 0xff;
        hash[i + 24] = (ctx->state[6] >> (24 - i * 8)) & 0xff; hash[i + 28] = (ctx->state[7] >> (24 - i * 8)) & 0xff;
    }
}
void hmac_sha256(const uint8_t *key, size_t keylen, const uint8_t *data, size_t datalen, uint8_t *out) {
    State::SHA256_CTX ctx; uint8_t k_ipad[65], k_opad[65], tk[32];
    memset(k_ipad, 0, sizeof(k_ipad)); memset(k_opad, 0, sizeof(k_opad));
    if (keylen > 64) { sha256_init(&ctx); sha256_update(&ctx, key, keylen); sha256_final(&ctx, tk); key = tk; keylen = 32; }
    memcpy(k_ipad, key, keylen); memcpy(k_opad, key, keylen);
    for (int i = 0; i < 64; i++) { k_ipad[i] ^= 0x36; k_opad[i] ^= 0x5c; }
    sha256_init(&ctx); sha256_update(&ctx, k_ipad, 64); sha256_update(&ctx, data, datalen); sha256_final(&ctx, out);
    sha256_init(&ctx); sha256_update(&ctx, k_opad, 64); sha256_update(&ctx, out, 32); sha256_final(&ctx, out);
}
String base64_encode(const uint8_t *data, size_t len) {
    static const char* b64_table = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    String ret; ret.reserve((len + 2) / 3 * 4);
    for (size_t i = 0; i < len; i += 3) {
        uint32_t octet_a = i < len ? data[i] : 0;
        uint32_t octet_b = i + 1 < len ? data[i + 1] : 0;
        uint32_t octet_c = i + 2 < len ? data[i + 2] : 0;
        uint32_t triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;
        ret += b64_table[(triple >> 3 * 6) & 0x3F]; ret += b64_table[(triple >> 2 * 6) & 0x3F];
        ret += b64_table[(triple >> 1 * 6) & 0x3F]; ret += b64_table[(triple >> 0 * 6) & 0x3F];
    }
    if (len % 3 == 1) { ret[ret.length() - 1] = '='; ret[ret.length() - 2] = '='; }
    if (len % 3 == 2) { ret[ret.length() - 1] = '='; }
    return ret;
}
}

//================================================================
// プロトタイプ宣言
//================================================================
namespace Menu { void changeMode(State::Mode newMode); }
namespace Display { 
    void triggerWolMessage(byte target);
    void printLcdLine(int line, const char* text); 
    void updateMainDisplay(); 
}
namespace Network {
void resyncNtpTimeFromMenu();
void sendSwitchBotCommand(const char* deviceId, const char* command, const char* parameter, const char* commandType = "command");
void resyncNtpTime();
void getSwitchBotDeviceList();
void sendLineTestMessage();
void sendWakeOnLan(const byte mac[6]);
bool sendWakeOnLanPacket(const byte mac[6]);
void requestDistance();
void showIpAddressAndHold();
void logDataToGoogleSheet(const char* params);
void manualLogToSheet();
void pollGasForWol();
void initOTA();
void handleNtpSync();
void urlEncode(const char* msg, char* encodedMsg, size_t bufferSize);
void createTempHumParams(char* buffer, size_t bufferSize);
}
namespace Utils { void toggleBacklightMode(); void toggleIlluminationMode(); void rebootDevice(); }
namespace Sensors {
    void calibrateSensor();
    void updateDht();
}
void handlePeriodicTasks_Core1();
void setup1();
void loop1();

//================================================================
// メニュー定義
//================================================================
namespace Menu {
struct MenuItem { const char* text; void (*action)(); };
void performMenuAction(void (*action)(), bool returnToMain);

void enterHistory() { changeMode(State::HISTORY); }
void enterSwitchbotMenu() { changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }
void enterWakeOnLan() { changeMode(State::WAKE_ON_LAN); }
void enterUltrasonic() { changeMode(State::ULTRASONIC_MONITOR); }
void enterDiagnostics() { changeMode(State::SENSOR_DIAGNOSTICS); }
void enterDeviceControl() {
    State::menu.deviceSelection = State::menu.menuSelection;
    State::menu.commandSelection = 0;
    changeMode(State::DEVICE_CONTROL);
}

const MenuItem mainMenu[] = {
    {"1. Lightning Log", enterHistory},
    {"2. Device Control", enterSwitchbotMenu},
    {"3. Wake on LAN", enterWakeOnLan},
    {"4. Backlight Mode", [](){ performMenuAction(Utils::toggleBacklightMode, true); }},
    {"5. RGB Illumination", [](){ performMenuAction(Utils::toggleIlluminationMode, true); }},
    {"6. Measure Dist", enterUltrasonic},
    {"7. Resync Time", [](){ performMenuAction(Network::resyncNtpTimeFromMenu, true); }},
    {"8. Show IP Address", [](){ performMenuAction(Network::showIpAddressAndHold, true); }},
    {"9. Get SwitchBot IDs", [](){ performMenuAction(Network::getSwitchBotDeviceList, true); }},
    {"10. Send LINE Test", [](){ performMenuAction(Network::sendLineTestMessage, true); }},
    {"11. Log Data Manually", [](){ performMenuAction(Network::manualLogToSheet, true); }},
    {"12. Reboot", Utils::rebootDevice},
    {"13. Sensor Diag", enterDiagnostics},
    {"14. Calibrate Sensor", [](){ performMenuAction(Sensors::calibrateSensor, true); }}
};
const int MAIN_MENU_COUNT = sizeof(mainMenu) / sizeof(mainMenu[0]);

const MenuItem applianceMenu[] = {
    {"1. Light", enterDeviceControl}, {"2. TV", enterDeviceControl},
    {"3. Air Conditioner", enterDeviceControl}, {"4. Fan", enterDeviceControl},
    {"5. Speaker", enterDeviceControl}, {"6. Others", enterDeviceControl},
    {"7. Back", [](){ changeMode(State::MENU); }}
};
const int APPLIANCE_MENU_COUNT = sizeof(applianceMenu) / sizeof(applianceMenu[0]);

const MenuItem wolMenu[] = {
    {"1. Desktop PC", [](){ performMenuAction([](){ Network::sendWakeOnLan(MAC_DESKTOP); }, true); }},
    {"2. Server PC", [](){ performMenuAction([](){ Network::sendWakeOnLan(MAC_SERVER); }, true); }}
};
const int WOL_MENU_COUNT = sizeof(wolMenu)/sizeof(wolMenu[0]);

const MenuItem lightControlMenu[] = {
    {"On", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_LIGHT, "turnOn", "default"); }, false); }},
    {"Off", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_LIGHT, "turnOff", "default"); }, false); }},
    {"Bright+", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_LIGHT, "brightnessUp", "default"); }, false); }},
    {"Bright-", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_LIGHT, "brightnessDown", "default"); }, false); }},
    {"Warm", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_LIGHT, "setColorTemperature", "2700"); }, false); }},
    {"White", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_LIGHT, "setColorTemperature", "6500"); }, false); }},
    {"Back", [](){ changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}
};
const MenuItem tvControlMenu[] = {
    {"Power", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_TV, "turnOn", "default", "command"); }, false); }},
    {"CH +", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_TV, "channelAdd", "default"); }, false); }},
    {"CH -", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_TV, "channelSub", "default"); }, false); }},
    {"Vol +", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_TV, "volumeAdd", "default"); }, false); }},
    {"Vol -", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_TV, "volumeSub", "default"); }, false); }},
    {"Back", [](){ changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}
};
const MenuItem acControlMenu[] = {
    {"Run", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_AC, "turnOn", "default"); }, false); }},
    {"Stop", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_AC, "turnOff", "default"); }, false); }},
    {"Temp +", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_AC, "setTemperature", "26,auto,1,on"); }, false); }},
    {"Temp -", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_AC, "setTemperature", "24,auto,1,on"); }, false); }},
    {"Cooling", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_AC, "setAll", "25,2,1,on"); }, false); }},
    {"Heating", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_AC, "setAll", "22,5,1,on"); }, false); }},
    {"Dehumidify",[](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_AC, "setAll", "25,3,1,on"); }, false); }},
    {"Back", [](){ changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}
};
const MenuItem fanControlMenu[] = {
    {"On", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_FAN, "press", "運転"); }, false); }},
    {"Off", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_FAN, "press", "切/入"); }, false); }},
    {"Speed +", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_FAN, "press", "風量+"); }, false); }},
    {"Speed -", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_FAN, "press", "風量-"); }, false); }},
    {"Swing", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_FAN, "press", "スウィング"); }, false); }},
    {"Back", [](){ changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}
};
const MenuItem speakerControlMenu[] = {
    {"Power", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_SPEAKER, "press", "Power"); }, false); }},
    {"Vol +", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_SPEAKER, "volumeAdd", "default"); }, false); }},
    {"Vol -", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_SPEAKER, "volumeSub", "default"); }, false); }},
    {"Prev", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_SPEAKER, "previousTrack", "default"); }, false); }},
    {"Next", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_SPEAKER, "nextTrack", "default"); }, false); }},
    {"Back", [](){ changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}
};
const MenuItem othersControlMenu[] = {
    {"Command 1", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_OTHERS, "turnOn", "default"); }, false); }},
    {"Command 2", [](){ performMenuAction([](){ Network::sendSwitchBotCommand(DEVICE_ID_OTHERS, "turnOff", "default"); }, false); }},
    {"Back", [](){ changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}
};

const MenuItem* const applianceControlMenus[] = { lightControlMenu, tvControlMenu, acControlMenu, fanControlMenu, speakerControlMenu, othersControlMenu };
const int applianceControlMenuCounts[] = { 7, 6, 8, 6, 6, 3 };

const MenuItem* getCurrentMenu(int& count, int& selection) {
    switch(State::menu.currentMode) {
        case State::MENU:
            count = MAIN_MENU_COUNT;
            selection = State::menu.menuSelection;
            return mainMenu;
        case State::SWITCHBOT_APPLIANCE_SELECT:
            count = APPLIANCE_MENU_COUNT;
            selection = State::menu.menuSelection;
            return applianceMenu;
        case State::WAKE_ON_LAN:
            count = WOL_MENU_COUNT;
            selection = State::menu.menuSelection;
            return wolMenu;
        case State::DEVICE_CONTROL:
            if(State::menu.deviceSelection < sizeof(applianceControlMenus)/sizeof(MenuItem**)) {
                count = applianceControlMenuCounts[State::menu.deviceSelection];
                selection = State::menu.commandSelection;
                return applianceControlMenus[State::menu.deviceSelection];
            }
        default:
            count = 0;
            selection = 0;
            return nullptr;
    }
}
}

//================================================================
// ユーティリティ関数
//================================================================
namespace Utils {
void setRGB(int r, int g, int b) {
    analogWrite(Pins::LED_R, r);
    analogWrite(Pins::LED_G, g);
    analogWrite(Pins::LED_B, b);
}

void blinkLED(String color, int times, int duration) {
    for (int i = 0; i < times; i++) {
        if (color == "green") setRGB(0, 255, 0);
        else if (color == "yellow") setRGB(255, 255, 0);
        else if (color == "blue") setRGB(0, 0, 255);
        else if (color == "white") setRGB(255, 255, 255);
        delay(duration);
        setRGB(0, 0, 0);
        delay(duration);
    }
}

void handleSmoothIllumination() {
    unsigned long hue = millis() / 10;
    hue %= 360;
    float s = 1.0, v = 1.0, r, g, b;
    int i = floor(hue / 60.0);
    float f = hue / 60.0 - i, p = v * (1.0 - s), q = v * (1.0 - (s * f)), t = v * (1.0 - (s * (1.0 - f)));
    switch (i % 6) {
        case 0: r=v,g=t,b=p; break;
        case 1: r=q,g=v,b=p; break;
        case 2: r=p,g=v,b=t; break;
        case 3: r=p,g=q,b=v; break;
        case 4: r=t,g=p,b=v; break;
        case 5: r=v,g=p,b=q; break;
    }
    setRGB(r * 255, g * 255, b * 255);
}

void toggleBacklightMode() {
    State::lcd.clear();
    State::system.backlightAlwaysOn = !State::system.backlightAlwaysOn;
    digitalWrite(Pins::LCD_BACKLIGHT, State::system.backlightAlwaysOn ? HIGH : LOW);
    Display::printLcdLine(0, "Backlight Mode");
    Display::printLcdLine(1, State::system.backlightAlwaysOn ? "Always ON" : "Auto OFF");
    delay(ACTION_RESULT_DISPLAY_MS);
}

void toggleIlluminationMode() {
    State::lcd.clear();
    State::system.illuminationOn = !State::system.illuminationOn;
    if (!State::system.illuminationOn) {
        setRGB(0, 0, 0);
    }
    Display::printLcdLine(0, "Illumination Mode");
    Display::printLcdLine(1, State::system.illuminationOn ? "ON" : "OFF");
    delay(ACTION_RESULT_DISPLAY_MS);
}

void rebootDevice() {
    State::lcd.clear();
    Display::printLcdLine(0, "Rebooting...");
    delay(1000);
    rp2040.reboot();
}
}

//================================================================
// ディスプレイ管理 (コア0)
//================================================================
namespace Display {
void printLcdLine(int line, const char* text) {
    char buf[LCD_COLS + 1];
    snprintf(buf, sizeof(buf), "%-*s", LCD_COLS, text);
    State::lcd.setCursor(0, line);
    State::lcd.print(buf);
}

namespace { 
    unsigned long wolMessageEndTime = 0;
    char wolMessage[LCD_COLS + 1] = "";

    void drawTime() {
        char buf[LCD_COLS + 1];
        if (State::g_wifiConnected && State::system.ntpInitialized && time(nullptr) > 100000) {
            time_t now = time(nullptr);
            strftime(buf, sizeof(buf), "%m/%d(%a) %H:%M:%S", localtime(&now));
        } else if (State::g_wifiConnected) {
            snprintf(buf, sizeof(buf), "Syncing Time (NTP)...");
        }
        else {
             snprintf(buf, sizeof(buf), "Connecting Network..");
        }
        printLcdLine(0, buf);
    }

    void drawSensors() {
        char buf[LCD_COLS + 1];
        if (State::sensors.temperature > -100) {
            snprintf(buf, sizeof(buf), "T:%.1fC  H:%.1f%%", State::sensors.temperature, State::sensors.humidity);
        } else {
            snprintf(buf, sizeof(buf), "Sensor Reading...");
        }
        printLcdLine(1, buf);
    }

    void drawLightningInfo() {
        char buf[LCD_COLS + 1];
        if (State::sensors.lastEventType == "Lightning") {
            snprintf(buf, sizeof(buf), "Last Evt:%s", State::sensors.lastEventTime);
            printLcdLine(2, buf);
            snprintf(buf, sizeof(buf), "Distance: %d km", State::sensors.lastLightningDistance);
            printLcdLine(3, buf);
        } else if (State::sensors.lastEventType == "Noise") {
            snprintf(buf, sizeof(buf), "Last Evt:%s", State::sensors.lastEventTime);
            printLcdLine(2, buf);
            printLcdLine(3, "Distance: Noise");
        } else {
            printLcdLine(2, "Last Event: None");
            printLcdLine(3, "Distance: ---");
        }
    }
}

void triggerWolMessage(byte target) {
    if (target == 1) {
        snprintf(wolMessage, sizeof(wolMessage), "WoL: Desktop PC");
    } else if (target == 2) {
        snprintf(wolMessage, sizeof(wolMessage), "WoL: Server PC");
    }
    wolMessageEndTime = millis() + 2000;
    State::system.forceMainScreenRedraw = true;
}

void updateMainDisplay() {
    static unsigned long lastTimeUpdate = 0;

    bool wolMessageActive = (wolMessageEndTime > 0 && millis() < wolMessageEndTime);

    if (wolMessageEndTime > 0 && !wolMessageActive) {
        wolMessageEndTime = 0;
        State::system.forceMainScreenRedraw = true;
    }

    if (State::system.forceMainScreenRedraw) {
        State::lcd.clear();
        drawTime();
        if (wolMessageActive) {
            printLcdLine(1, wolMessage);
        } else {
            drawSensors();
        }
        drawLightningInfo();
        lastTimeUpdate = millis();
    } else {
        if (millis() - lastTimeUpdate >= 1000) {
            drawTime();
            lastTimeUpdate = millis();
        }
        if (!wolMessageActive && State::g_update_sensor_display) {
            State::g_update_sensor_display = false;
            drawSensors();
        }
    }
}

void drawMenu() {
    int itemCount = 0, currentSelection = 0;
    const Menu::MenuItem* menuItems = Menu::getCurrentMenu(itemCount, currentSelection);
    if (!menuItems) return;
    
    int page = currentSelection / LCD_ROWS;
    for(int i=0; i < LCD_ROWS; i++) {
        int index = page * LCD_ROWS + i;
        char buf[LCD_COLS + 1];
        if (index < itemCount) {
            sprintf(buf, "%s%s", (index == currentSelection ? ">" : " "), menuItems[index].text);
        } else {
            strcpy(buf, "");
        }
        printLcdLine(i, buf);
    }
}

void drawHistoryScreen() {
    printLcdLine(0, "--- Lightning Log ---");
    for (int i = 0; i < HISTORY_SIZE; i++) {
        char buf[LCD_COLS + 1];
        if (i < State::history.count) {
            int idx = (State::history.index - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
            if (State::history.records[idx].type == "Noise") {
                snprintf(buf, sizeof(buf), "%d: %s Noise", i + 1, State::history.records[idx].timestamp);
            } else {
                snprintf(buf, sizeof(buf), "%d: %s %2dkm", i + 1, State::history.records[idx].timestamp, State::history.records[idx].distance);
            }
        } else {
            snprintf(buf, sizeof(buf), "%d: ---", i + 1);
        }
        printLcdLine(i + 1, buf);
    }
}

void drawUltrasonicMonitorScreen() {
    static unsigned long lastRequestTime = 0;
    if (millis() - lastRequestTime > 500) {
        lastRequestTime = millis();
        Network::requestDistance();
    }

    printLcdLine(0, "Ultrasonic Sensor");
    char buf[LCD_COLS + 1];
    if (State::childIpAddress == "") snprintf(buf, sizeof(buf), "Child not found");
    else if (State::sensors.ultrasonicDistance < 0) snprintf(buf, sizeof(buf), "Requesting data...");
    else snprintf(buf, sizeof(buf), "Dist: %.1f cm", State::sensors.ultrasonicDistance);
    printLcdLine(1, buf);
    printLcdLine(2, "");
    printLcdLine(3, "(Long press to exit)");
}

void drawDiagnosticsScreen() {
    static unsigned long lastReadTime = 0;
    if (millis() - lastReadTime > 1000) { 
        lastReadTime = millis();

        printLcdLine(0, "--- Sensor Diag ---");
        char buf[LCD_COLS + 1];

        uint8_t noise = State::lightning.readNoiseLevel();
        uint8_t watchdog = State::lightning.readWatchdogThreshold();
        snprintf(buf, sizeof(buf), "Noise:%d Watchdog:%d", noise, watchdog);
        printLcdLine(1, buf);

        uint8_t spike = State::lightning.readSpikeRejection();
        uint8_t intReg = State::lightning.readInterruptReg();
        snprintf(buf, sizeof(buf), "Spike:%d IntReg:0x%02X", spike, intReg);
        printLcdLine(2, buf);
        
        int irqPinState = digitalRead(Pins::LIGHTNING_IRQ);
        snprintf(buf, sizeof(buf), "IRQ Pin State: %d", irqPinState);
        printLcdLine(3, buf);
    }
}

void update() {
    switch(State::menu.currentMode) {
        case State::MAIN_DISPLAY:
            updateMainDisplay();
            break;
        case State::ULTRASONIC_MONITOR:
            drawUltrasonicMonitorScreen();
            break;
        case State::HISTORY:
            if (State::system.needsRedraw) drawHistoryScreen();
            break;
        case State::SENSOR_DIAGNOSTICS:
            drawDiagnosticsScreen();
            break;
        default: // MENU, DEVICE_CONTROLなど
            if(State::system.needsRedraw) drawMenu();
            break;
    }
    
    if (State::system.needsRedraw) {
        State::system.needsRedraw = false;
    }
    if (State::system.forceMainScreenRedraw) {
        State::system.forceMainScreenRedraw = false;
    }
}

void init() {
    State::lcd.begin(LCD_COLS, LCD_ROWS);
    Display::printLcdLine(0, "System Starting...");
}
}

//================================================================
// ネットワーク管理 (コア1の処理が中心)
//================================================================
namespace Network {

void initOTA() {
    if (State::system.otaInitialized || !State::g_wifiConnected) {
        return;
    }

    ArduinoOTA.setHostname("pico-lightning-sensor");

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else // U_SPIFFS
            type = "filesystem";
        if (DEBUG) Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
        if (DEBUG) Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        if (DEBUG) Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        if (DEBUG) {
          Serial.printf("Error[%u]: ", error);
          if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
          else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
          else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
          else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
          else if (error == OTA_END_ERROR) Serial.println("End Failed");
        }
    });

    ArduinoOTA.begin();
    State::system.otaInitialized = true;
    if (DEBUG) Serial.println("OTA Ready. Hostname: pico-lightning-sensor");
}

void urlEncode(const char* msg, char* encodedMsg, size_t bufferSize) {
    const char *hex = "0123456789abcdef";
    size_t index = 0;
    while (*msg != '\0' && index < bufferSize - 1) {
        if (('a' <= *msg && *msg <= 'z') || ('A' <= *msg && *msg <= 'Z') || ('0' <= *msg && *msg <= '9') ||
            *msg == '-' || *msg == '_' || *msg == '.' || *msg == '~') {
            encodedMsg[index++] = *msg;
        } else {
            if (index + 3 < bufferSize) {
                encodedMsg[index++] = '%';
                encodedMsg[index++] = hex[*msg >> 4];
                encodedMsg[index++] = hex[*msg & 15];
            } else {
                break;
            }
        }
        msg++;
    }
    encodedMsg[index] = '\0';
}

void createTempHumParams(char* buffer, size_t bufferSize) {
    if (State::sensors.temperature > -100) {
        char encodedSheetName[32];
        urlEncode("温湿度", encodedSheetName, sizeof(encodedSheetName));
        snprintf(buffer, bufferSize, "?sheet=%s&temp=%.1f&hum=%.1f",
                 encodedSheetName,
                 State::sensors.temperature,
                 State::sensors.humidity);
    } else {
        buffer[0] = '\0';
    }
}

void addAuthHeaders(HTTPClient& http) {
    String token = SWITCHBOT_TOKEN;
    String secret = SWITCHBOT_SECRET;
    time_t t_val; time(&t_val);
    char t_str[15]; sprintf(t_str, "%lu", (unsigned long)t_val);
    String t = String(t_str) + "000";
    String nonce = "";
    for(int i=0; i<16; i++) nonce += String(random(16), HEX);
    String dataToSign = token + t + nonce;
    byte hmacResult[32];
    Crypto::hmac_sha256((const uint8_t*)secret.c_str(), secret.length(), (const uint8_t*)dataToSign.c_str(), dataToSign.length(), hmacResult);
    String sign = Crypto::base64_encode(hmacResult, 32);
    http.addHeader("Authorization", token);
    http.addHeader("t", t);
    http.addHeader("nonce", nonce);
    http.addHeader("sign", sign);
}

void sendLineNotification(String message) {
    if (!State::g_wifiConnected) return;
    
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    if (http.begin(client, "https://api.line.me/v2/bot/message/push")) {
        http.addHeader("Content-Type", "application/json");
        http.addHeader("Authorization", "Bearer " + String(LINE_CHANNEL_ACCESS_TOKEN));
        JsonDocument doc;
        doc["to"] = LINE_USER_ID;
        JsonObject msgObj = doc["messages"].to<JsonArray>().add<JsonObject>();
        msgObj["type"] = "text";
        msgObj["text"] = message;
        String requestBody;
        serializeJson(doc, requestBody);
        http.POST(requestBody);
        http.end();
    }
}

void sendSwitchBotCommand(const char* deviceId, const char* command, const char* parameter, const char* commandType) {
    State::lcd.clear();
    if (!State::g_wifiConnected) {
        Display::printLcdLine(0, "WiFi Disconnected");
        delay(ACTION_RESULT_DISPLAY_MS);
        return;
    }
    Display::printLcdLine(0, "Sending Command...");
    Display::printLcdLine(1, "Please wait...");
    
    WiFiClientSecure secureClient;
    secureClient.setInsecure();
    HTTPClient http;
    bool success = false;
    if (http.begin(secureClient, "https://api.switch-bot.com/v1.1/devices/" + String(deviceId) + "/commands")) {
        addAuthHeaders(http);
        http.addHeader("Content-Type", "application/json; charset=utf-8");
        JsonDocument doc;
        doc["command"] = command;
        doc["parameter"] = parameter;
        doc["commandType"] = commandType;
        String jsonBody;
        serializeJson(doc, jsonBody);
        if (http.POST(jsonBody) == HTTP_CODE_OK) {
            success = true;
        }
        http.end();
    }
    
    State::lcd.clear();
    Display::printLcdLine(0, "Command Result");
    Display::printLcdLine(1, success ? "Success!" : "Failed!");
    delay(ACTION_RESULT_DISPLAY_MS);
}

bool sendWakeOnLanPacket(const byte mac[6]) {
    WiFiUDP udp;
    if (udp.begin(9)) {
        byte magicPacket[102];
        memset(magicPacket, 0xFF, 6);
        for (int i = 1; i <= 16; i++) {
            memcpy(&magicPacket[i * 6], mac, 6);
        }
        udp.beginPacket(IPAddress(255, 255, 255, 255), 9);
        udp.write(magicPacket, sizeof(magicPacket));
        
        if (udp.endPacket()) {
            if (DEBUG) Serial.println("WoL packet sent.");
            return true;
        } else {
            if (DEBUG) Serial.println("WoL packet send failed.");
            return false;
        }
    } else {
        if (DEBUG) Serial.println("WoL UDP setup failed.");
        return false;
    }
}

void sendWakeOnLan(const byte mac[6]) {
    State::lcd.clear();
    Display::printLcdLine(0, "Sending WoL Packet...");

    bool success = sendWakeOnLanPacket(mac);

    Display::printLcdLine(1, success ? "Packet Sent!" : "Send Failed!");
    delay(ACTION_RESULT_DISPLAY_MS);
}


void requestDistance() {
    if (!State::g_wifiConnected || State::childIpAddress == "") {
        State::sensors.ultrasonicDistance = -2.0;
        return;
    }
    
    HTTPClient http;
    WiFiClient client;
    if (http.begin(client, "http://" + State::childIpAddress + "/distance")) {
        http.setTimeout(400);
        if (http.GET() == HTTP_CODE_OK) {
            JsonDocument doc;
            if (deserializeJson(doc, http.getString()) == DeserializationError::Ok) {
                State::sensors.ultrasonicDistance = doc["distance"];
            }
        } else {
            State::sensors.ultrasonicDistance = -3.0;
        }
        http.end();
    } else {
        State::sensors.ultrasonicDistance = -4.0;
    }
}

void handleNtpSync() {
    if (!State::g_wifiConnected || State::system.ntpInitialized) return;
        
    if (!State::system.isAutoResync) {
        if (DEBUG) Serial.println("[Core 1] NTP sync started...");
    }
    State::system.isAutoResync = false;

    setenv("TZ", "JST-9", 1);
    tzset();
    
    NTP.begin(NTP_SERVER);

    bool syncSuccess = false;
    unsigned long ntp_timeout = millis();
    while (millis() - ntp_timeout < 15000) {
        if (time(nullptr) > 100000) {
            syncSuccess = true;
            break;
        }
        delay(500);
    }

    if (syncSuccess) {
        if (DEBUG) Serial.println("\n[Core 1] NTP Sync Successful!");
        State::system.ntpInitialized = true;
        State::server.begin();
    } else {
        if (DEBUG) Serial.println("\n[Core 1] NTP Sync Failed! (Timeout)");
    }
    State::system.forceMainScreenRedraw = true;
}


void logDataToGoogleSheet(const char* params) {
    if (!State::g_wifiConnected || String(GAS_URL).startsWith("-----")) {
        if (DEBUG) Serial.println("Cannot log to sheet: WiFi not connected or GAS_URL not set.");
        return;
    }
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    char url[256];
    snprintf(url, sizeof(url), "%s%s", GAS_URL, params);

    if (http.begin(client, url)) {
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        int httpCode = http.GET();
        if (httpCode > 0) {
            if (DEBUG) Serial.printf("[HTTP] GET... code: %d\n", httpCode);
        } else {
            if (DEBUG) Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
        }
        http.end();
    } else {
        if (DEBUG) Serial.printf("[HTTP] Unable to connect to %s\n", url);
    }
}


void resyncNtpTimeFromMenu() {
    State::lcd.clear();
    Display::printLcdLine(0, "Resyncing Time...");
    if (!State::g_wifiConnected) {
        Display::printLcdLine(1, "WiFi Disconnected");
        delay(ACTION_RESULT_DISPLAY_MS);
        return;
    }
    
    resyncNtpTime();
    
    Display::printLcdLine(1, State::system.ntpInitialized ? "Success!" : "Failed!");
    delay(ACTION_RESULT_DISPLAY_MS);
}

void resyncNtpTime() {
    if (!State::g_wifiConnected) {
        if (DEBUG) Serial.println("NTP Resync failed. WiFi not connected.");
        return;
    }
    State::system.ntpInitialized = false;
    State::system.isAutoResync = true;
    if (DEBUG) Serial.println("NTP Resync requested.");
    handleNtpSync();
}

void sendLineTestMessage() {
    State::lcd.clear();
    Display::printLcdLine(0, "Sending LINE Test...");
    if (!State::g_wifiConnected) {
        Display::printLcdLine(1, "WiFi Disconnected");
        delay(ACTION_RESULT_DISPLAY_MS);
        return;
    }
    sendLineNotification("これは雷センサーからのテスト通知です。");
    Display::printLcdLine(1, "Sent!");
    delay(ACTION_RESULT_DISPLAY_MS);
}

void getSwitchBotDeviceList(){
    State::lcd.clear();
    Display::printLcdLine(0, "Getting Device IDs");
    if (!State::g_wifiConnected) {
        Display::printLcdLine(1, "WiFi Disconnected");
        delay(ACTION_RESULT_DISPLAY_MS);
        return;
    }
    Display::printLcdLine(1, "Check Serial Mon.");

    WiFiClientSecure secureClient;
    secureClient.setInsecure();
    HTTPClient http;
    bool success = false;
    if (http.begin(secureClient, "https://api.switch-bot.com/v1.1/devices")) {
        addAuthHeaders(http);
        if (http.GET() == HTTP_CODE_OK) {
            success = true;
            if (DEBUG) {
                Serial.println("\n--- SwitchBot Device List ---");
                Serial.println(http.getString());
            }
        }
        http.end();
    }
    Display::printLcdLine(2, success ? "Success!" : "Failed!");
    delay(ACTION_RESULT_DISPLAY_MS);
}

void showIpAddressAndHold() {
    State::lcd.clear();
    Display::printLcdLine(0, "IP Address:");
    if (State::g_wifiConnected) {
        Display::printLcdLine(1, WiFi.localIP().toString().c_str());
    } else {
        Display::printLcdLine(1, "Disconnected");
    }
    delay(IP_DISPLAY_DURATION_MS);
}

void manualLogToSheet() {
    State::lcd.clear();
    if (!State::g_wifiConnected) {
        Display::printLcdLine(0, "WiFi Disconnected");
        delay(ACTION_RESULT_DISPLAY_MS);
        return;
    }
    Display::printLcdLine(0, "Logging to Sheet...");
    Display::printLcdLine(1, "Please wait...");
    
    char params[128];
    createTempHumParams(params, sizeof(params));

    State::lcd.clear();
    Display::printLcdLine(0, "Logging Result");
    if (strlen(params) > 0) {
        logDataToGoogleSheet(params);
        Display::printLcdLine(1, "Logged!");
    } else {
        Display::printLcdLine(1, "No sensor data!");
    }
    delay(ACTION_RESULT_DISPLAY_MS);
}

void pollGasForWol() {
    if (String(GAS_URL_WOL).startsWith("-----")) return;
    
    WiFiClientSecure client;
    HTTPClient http;
    client.setInsecure();
    
    if (DEBUG) Serial.print("[Core 1] Polling GAS for WoL signal...");

    char signalCheckUrl[128];
    snprintf(signalCheckUrl, sizeof(signalCheckUrl), "%s?action=signal", GAS_URL_WOL);
    
    if (http.begin(client, signalCheckUrl)) {
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        http.setTimeout(3800);

        int httpCode = http.GET();

        if (httpCode == HTTP_CODE_OK) {
            String payload = http.getString();
            payload.trim(); 

            if (payload == "TRIGGER") {
                if (DEBUG) Serial.println("  -> Trigger detected! Getting command...");
                http.end();
                
                delay(500);

                char commandGetUrl[128];
                snprintf(commandGetUrl, sizeof(commandGetUrl), "%s?action=command", GAS_URL_WOL);

                if (http.begin(client, commandGetUrl)){
                    httpCode = http.GET();
                    if (httpCode == HTTP_CODE_OK) {
                        payload = http.getString();
                        payload.trim();
                        if (DEBUG) Serial.printf("  -> Command received: [%s]\n", payload.c_str());
                        
                        if (payload == "デスクトップPC起動") {
                            if (sendWakeOnLanPacket(MAC_DESKTOP)) {
                                State::g_wolTarget = 1;
                                State::g_wolTriggered = true;
                            }
                        } else if (payload == "サーバーPC起動") {
                            if (sendWakeOnLanPacket(MAC_SERVER)) {
                                State::g_wolTarget = 2;
                                State::g_wolTriggered = true;
                            }
                        }
                    } else {
                        if (DEBUG) Serial.printf("  -> HTTP Command GET failed, error: %d\n", httpCode);
                    }
                }
            } else if (DEBUG) {
                 Serial.println(" -> No trigger.");
            }
        } else {
            if (DEBUG) Serial.printf(" -> HTTP Signal GET failed, error: %d (%s)\n", httpCode, http.errorToString(httpCode).c_str());
        }
    } else {
        if (DEBUG) Serial.println(" -> http.begin failed.");
    }
    http.end();
}

}

//================================================================
// センサー管理 (コア0)
//================================================================
namespace Sensors {
void handleLightningInterrupt() {
    State::lightningInterruptFlag = true;
}

void addHistoryRecord(String type, int distance, const char* timestamp) {
    strcpy(State::history.records[State::history.index].timestamp, timestamp);
    State::history.records[State::history.index].type = type;
    State::history.records[State::history.index].distance = distance;
    State::history.index = (State::history.index + 1) % HISTORY_SIZE;
    if(State::history.count < HISTORY_SIZE) {
        State::history.count++;
    }
}

void handleLightning() {
    State::lightningInterruptFlag = false;
    delay(5);
    int intVal = State::lightning.readInterruptReg();
    if (DEBUG) Serial.printf("Interrupt Register: 0x%02X\n", intVal);
    
    char timestamp[20];
    if (intVal == 0x01 || intVal == 0x08) {
        time_t now = time(nullptr);
        if (now > 100000) {
            strftime(timestamp, sizeof(timestamp), "%m/%d %H:%M", localtime(&now));
            strcpy(State::sensors.lastEventTime, timestamp);
        } else {
            strcpy(timestamp, "Time N/A");
            strcpy(State::sensors.lastEventTime, timestamp);
        }
    }

    if (intVal == 0x01) {
        State::sensors.lastEventType = "Noise";
        if (AUTO_NOISE_LEVEL_CONTROL && State::system.currentNoiseLevel < 7) {
            State::system.currentNoiseLevel++;
            State::lightning.setNoiseLevel(State::system.currentNoiseLevel);
             if (DEBUG) Serial.printf("Noise detected. Noise level increased to: %d\n", State::system.currentNoiseLevel);
        }
        addHistoryRecord("Noise", 0, timestamp);

    } else if (intVal == 0x08) {
        Utils::blinkLED("yellow", 2, 80);
        int distance = State::lightning.distanceToStorm();
        State::sensors.lastLightningDistance = distance;
        State::sensors.lastEventType = "Lightning";
        
        addHistoryRecord("Lightning", distance, timestamp);

        State::g_pending_lightning_log_core1 = true;
    }
    State::system.forceMainScreenRedraw = true;
}

void updateDht() {
    float temp_raw = State::dht20.getTemperature();
    float hum_raw = State::dht20.getHumidity();

    if (!isnan(temp_raw) && !isnan(hum_raw)) {
        State::sensors.temperature = round(temp_raw * 10.0) / 10.0;
        
        if (hum_raw >= 0 && hum_raw <= 1.0) {
            hum_raw *= 100.0;
        }
        State::sensors.humidity = round(hum_raw * 10.0) / 10.0;
    } else {
        if (DEBUG) Serial.println("Failed to read from DHT sensor!");
    }
}

void update() {
    if (State::lightningInterruptFlag) {
        handleLightning();
    }
}

void calibrateSensor() {
    State::lcd.clear();
    Display::printLcdLine(0, "Calibrating...");
    if (State::lightning.calibrateOsc()) {
        Display::printLcdLine(1, "Calibration OK!");
        if (DEBUG) Serial.println("Antenna tuning successful.");
    } else {
        Display::printLcdLine(1, "Calibration FAILED!");
        if (DEBUG) Serial.println("Antenna tuning failed.");
    }
    delay(ACTION_RESULT_DISPLAY_MS);
}

void init() {
    // 【修正点】DHT20の初期化に失敗した場合、LCDにエラーを表示して停止するように変更
    if (State::dht20.begin() != 0) {
        if (DEBUG) Serial.println("DHT20 Error! Halting.");
        Display::printLcdLine(1, "DHT20 COM FAILED!");
        Display::printLcdLine(2, "Check wiring & I2C");
        while(1) { delay(10); }
    }
    Display::printLcdLine(1, "DHT20 Sensor OK");
    delay(1000);
    State::lcd.clear();
    Display::printLcdLine(0, "System Starting...");
    
    State::system.currentNoiseLevel = INITIAL_NOISE_LEVEL;
    if (!State::lightning.begin(Wire)) {
        if (DEBUG) Serial.println("AS3935 Error! Halting.");
        Display::printLcdLine(1, "AS3935 COM FAILED!");
        Display::printLcdLine(2, "Check wiring & I2C");
        while(1) { delay(10); }
    } else {
        State::lightning.resetSettings();
        
        State::lightning.setIndoorOutdoor(OUTDOOR);
        State::lightning.setNoiseLevel(State::system.currentNoiseLevel);
        State::lightning.watchdogThreshold(LIGHTNING_WATCHDOG_THRESHOLD);
        State::lightning.spikeRejection(LIGHTNING_SPIKE_REJECTION);
        
        if (DEBUG) {
         Serial.println("--- AS3935 Initial Settings ---");
         Serial.printf("Watchdog Threshold: Set to %d, Read back %d\n", LIGHTNING_WATCHDOG_THRESHOLD, State::lightning.readWatchdogThreshold());
         Serial.printf("Spike Rejection:    Set to %d, Read back %d\n", LIGHTNING_SPIKE_REJECTION, State::lightning.readSpikeRejection());
         Serial.printf("Noise Level:        Set to %d, Read back %d\n", State::system.currentNoiseLevel, State::lightning.readNoiseLevel());
         Serial.println("---------------------------------");
        }
        Display::printLcdLine(1, "Lightning Sensor OK");
        delay(1500);
    }
    attachInterrupt(digitalPinToInterrupt(Pins::LIGHTNING_IRQ), handleLightningInterrupt, RISING);
}
}

//================================================================
// 入力処理 (コア0)
//================================================================
namespace Input {
uint8_t buttonState = HIGH;
uint8_t lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
unsigned long buttonPressTime = 0;
bool longPressTriggered = false;

void handleButton() {
    int reading = digitalRead(Pins::BUTTON);

    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != buttonState) {
            buttonState = reading;

            if (buttonState == LOW) {
                buttonPressTime = millis();
                longPressTriggered = false;
            } else {
                if (!longPressTriggered) {
                    State::timers.lastActivity = millis();
                    State::system.needsRedraw = true;
                    if (State::menu.currentMode == State::MAIN_DISPLAY) {
                        if (!State::system.backlightAlwaysOn) {
                            digitalWrite(Pins::LCD_BACKLIGHT, HIGH);
                            State::timers.backlightOn = millis();
                        }
                    } else if (State::menu.currentMode == State::HISTORY || State::menu.currentMode == State::SENSOR_DIAGNOSTICS) {
                        // これらの画面では短押しは無効
                    } else {
                        int count = 0, currentSelection = 0;
                        Menu::getCurrentMenu(count, currentSelection);
                        if(count > 0) {
                            if (State::menu.currentMode == State::DEVICE_CONTROL) {
                                State::menu.commandSelection = (State::menu.commandSelection + 1) % count;
                            } else {
                                State::menu.menuSelection = (State::menu.menuSelection + 1) % count;
                            }
                        }
                    }
                }
            }
        }
    }

    if (buttonState == LOW && !longPressTriggered && (millis() - buttonPressTime > LONG_PRESS_DURATION_MS)) {
        longPressTriggered = true;
        State::timers.lastActivity = millis();
        State::system.needsRedraw = true;
        switch (State::menu.currentMode) {
            case State::MAIN_DISPLAY:
                Menu::changeMode(State::MENU);
                break;
            case State::HISTORY:
            case State::ULTRASONIC_MONITOR:
            case State::SENSOR_DIAGNOSTICS:
                Menu::changeMode(State::MAIN_DISPLAY);
                break;
            default: {
                int count = 0, currentSelection = 0;
                const Menu::MenuItem* menu = Menu::getCurrentMenu(count, currentSelection);
                if (menu && currentSelection < count) {
                    menu[currentSelection].action();
                }
                break;
            }
        }
    }
    lastButtonState = reading;
}
}

//================================================================
// メイン制御 (コア0)
//================================================================
namespace Menu {
void changeMode(State::Mode newMode) {
    State::lcd.clear(); // モード変更時に必ず画面をクリア
    State::menu.currentMode = newMode;
    State::menu.menuSelection = 0;
    State::menu.commandSelection = 0;
    State::system.needsRedraw = true;
    State::timers.lastActivity = millis();
    if (newMode == State::MAIN_DISPLAY) {
        State::system.forceMainScreenRedraw = true;
    }
}

void performMenuAction(void (*action)(), bool returnToMain) {
    if (action) {
        action(); // アクション関数内でクリア、表示、待機を完結させる
    }
    // アクション後に必ず画面遷移または再描画を行う
    if (returnToMain) {
        changeMode(State::MAIN_DISPLAY);
    } else {
        // 現在のモードを再読み込みしてメニュー画面を再描画する
        changeMode(State::menu.currentMode);
    }
}

void checkInactivity() {
    if (State::menu.currentMode == State::MAIN_DISPLAY || State::menu.currentMode == State::ULTRASONIC_MONITOR || State::menu.currentMode == State::SENSOR_DIAGNOSTICS) {
        return;
    }
    if (millis() - State::timers.lastActivity > INACTIVITY_TIMEOUT_MS) {
        changeMode(State::MAIN_DISPLAY);
    }
}
}

//================================================================
// Setup & Loop (コア0)
//================================================================
void setup() {
    Serial.begin(115200);
    unsigned long setup_start_time = millis();
    while (!Serial && (millis() - setup_start_time < 3000));
    if (DEBUG) Serial.println("\n--- System Booting (Core 0) ---");

    pinMode(Pins::LCD_BACKLIGHT, OUTPUT);
    pinMode(Pins::LED_R, OUTPUT);
    pinMode(Pins::LED_G, OUTPUT);
    pinMode(Pins::LED_B, OUTPUT);
    pinMode(Pins::BUTTON, INPUT_PULLUP);
    pinMode(Pins::LIGHTNING_IRQ, INPUT);
    Wire.setSDA(Pins::I2C_SDA);
    Wire.setSCL(Pins::I2C_SCL);
    Wire.begin();

    // 起動中はバックライトとLEDを点灯
    digitalWrite(Pins::LCD_BACKLIGHT, HIGH);
    Utils::setRGB(255, 255, 255);
    
    Display::init(); // "System Starting..."

    // Wi-Fi接続処理の前に、センサー類を初期化
    Sensors::init();
    
    // Core1を起動して、Wi-Fi接続をバックグラウンドで開始させる
    if(DEBUG) Serial.println("[Core 0] Starting Core 1 for network tasks.");
    rp2040.restartCore1();

    // Core1がWi-Fiに接続するのを待つ（タイムアウト付き）
    State::lcd.clear();
    Display::printLcdLine(0, "Connecting to WiFi");
    Display::printLcdLine(1, "Please wait...");
    unsigned long wifi_wait_start = millis();
    while(!State::g_wifiConnected && millis() - wifi_wait_start < 30000) { // 30秒のタイムアウト
        delay(100);
    }

    // Wi-Fi接続結果をLCDに表示
    State::lcd.clear();
    if (!State::g_wifiConnected) {
        Display::printLcdLine(0, "WiFi Connect FAILED");
        Display::printLcdLine(1, "Check SSID/PASS");
        if (DEBUG) Serial.println("[Core 0] WiFi connection timed out on Core 1.");
        delay(3000); // ユーザーがメッセージを読めるように待機
    } else {
        Display::printLcdLine(0, "WiFi Connected!");
        Display::printLcdLine(1, WiFi.localIP().toString().c_str());
        if (DEBUG) Serial.println("[Core 0] WiFi connection confirmed from Core 1.");
        delay(2000); 
    }
    
    Menu::changeMode(State::MAIN_DISPLAY);
    
    // メイン画面に切り替わったらLEDを消灯し、バックライトモードを適用
    Utils::setRGB(0, 0, 0);
    if (!State::system.backlightAlwaysOn) {
        digitalWrite(Pins::LCD_BACKLIGHT, LOW);
    }
    
    if (DEBUG) Serial.println("--- [Core 0] Boot Complete, Main Screen Active ---");
}

void loop() {
    if (State::g_wolTriggered) {
        State::g_wolTriggered = false;
        Display::triggerWolMessage(State::g_wolTarget);
        State::g_wolTarget = 0;
    }
    
    Input::handleButton();
    Display::update();
    Menu::checkInactivity();
    
    Sensors::update();
    
    if (State::g_trigger_dht_read) {
        State::g_trigger_dht_read = false;
        if (DEBUG) Serial.println("[Core 0] Reading DHT sensor (triggered by Core 1)...");
        Sensors::updateDht();
    }

    static int last_display_second = -1;
    static unsigned long sensor_display_trigger_time = 0;

    if (State::system.ntpInitialized) {
        time_t now = time(nullptr);
        struct tm* timeinfo = localtime(&now);
        int current_second = timeinfo->tm_sec;

        if (current_second != last_display_second) {
            last_display_second = current_second;
            sensor_display_trigger_time = millis() + 500;
        }
    }

    if (sensor_display_trigger_time > 0 && millis() >= sensor_display_trigger_time) {
        sensor_display_trigger_time = 0;
        State::g_update_sensor_display = true;
    }


    if (State::system.illuminationOn) {
        Utils::handleSmoothIllumination();
    }

    if (State::menu.currentMode == State::MAIN_DISPLAY && !State::system.backlightAlwaysOn &&
        State::timers.backlightOn > 0 && (millis() - State::timers.backlightOn > BACKLIGHT_DURATION_MS)) {
        digitalWrite(Pins::LCD_BACKLIGHT, LOW);
        State::timers.backlightOn = 0;
    }
}


//================================================================
// Setup & Loop (コア1)
//================================================================
void setup1() {
    if (DEBUG) Serial.println("[Core 1] Core 1 started. Initializing network...");
    
    WiFi.mode(WIFI_STA);
    if (USE_STATIC_IP) {
        if (DEBUG) Serial.println("[Core 1] Configuring static IP address...");
        IPAddress local_IP(STATIC_IP_BYTES);
        IPAddress gateway(GATEWAY_BYTES);
        IPAddress subnet(SUBNET_BYTES);
        IPAddress primaryDNS(PRIMARY_DNS_BYTES);
        IPAddress secondaryDNS(SECONDARY_DNS_BYTES);
        
        // 【修正点】コンパイルエラーに基づき、Pico W環境で利用可能なWiFi.config関数の引数に合わせます。
        // 書式: config(IPアドレス, DNSサーバー, ゲートウェイ, サブネット)
        WiFi.config(local_IP, primaryDNS, gateway, subnet);
        
        // 【追加】setDNS() を使用して、プライマリおよびセカンダリDNSサーバーを明示的に設定します。
        // これにより、両方のDNSサーバーが確実に設定されます。
        WiFi.setDNS(primaryDNS, secondaryDNS);
    }

    int credIndex = 0;
    unsigned long total_start_time = millis();
    
    // 複数のWiFi設定を順番に試す
    while(WiFi.status() != WL_CONNECTED && (millis() - total_start_time < 60000)) { // 全体で60秒試す
        const char* current_ssid = wifiCredentials[credIndex].ssid;

        if (strlen(current_ssid) > 0 && strcmp(current_ssid, "-----") != 0) {
            if (DEBUG) Serial.printf("[Core 1] Attempting to connect to SSID: %s\n", current_ssid);
            WiFi.begin(current_ssid, wifiCredentials[credIndex].password);
            
            unsigned long startTime = millis();
            // 1つのSSIDあたり15秒試す
            while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
                delay(500);
                if (DEBUG) Serial.print(".");
            }
            if (DEBUG) Serial.println();
        }

        // 接続できなかった場合、次のSSIDへ
        if (WiFi.status() != WL_CONNECTED) {
            if (DEBUG) Serial.printf("[Core 1] Failed to connect to %s. Trying next...\n", current_ssid);
            credIndex = (credIndex + 1) % numWifiCredentials;
            if (credIndex == 0) { // 全て試したら少し待つ
                 delay(1000);
            }
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
      if (DEBUG) Serial.printf("\n[Core 1] WiFi connected! IP address: %s\n", WiFi.localIP().toString().c_str());
      State::g_wifiConnected = true;

      Network::initOTA();
      Network::handleNtpSync();
    } else {
      if (DEBUG) Serial.println("\n[Core 1] Failed to connect to any WiFi network.");
      State::g_wifiConnected = false;
    }
}

void loop1() {
    if (State::g_wifiConnected) {
        ArduinoOTA.handle();

        WiFiClient client = State::server.accept();
        if(client) {
             if (State::childIpAddress == "") Utils::blinkLED("blue", 3, 150);
            State::childIpAddress = client.remoteIP().toString();
            client.stop();
        }

        handlePeriodicTasks_Core1();

    } else {
        // WiFiが切断された場合、再接続を試みる
        if(DEBUG) Serial.println("[Core 1] WiFi connection lost. Rebooting Core 1 to reconnect...");
        delay(5000); // 5秒待ってから再起動
        rp2040.restartCore1();
    }
    
    delay(10);
}


//================================================================
// 定期実行タスク (コア1)
//================================================================
void handlePeriodicTasks_Core1() {
    static unsigned long last_tick_time = 0;
    static int scheduler_step = 0;

    if (millis() - last_tick_time >= 1000) {
        last_tick_time = millis();

        switch (scheduler_step) {
            case 0:
                State::g_trigger_dht_read = true;
                break;
            case 1:
                State::g_trigger_wol_poll = true;
                break;
            case 2:
                State::g_trigger_dht_read = true;
                break;
            case 3:
                break;
        }
        scheduler_step = (scheduler_step + 1) % 4;
    }
    
    if (State::g_trigger_wol_poll) {
        State::g_trigger_wol_poll = false;
        Network::pollGasForWol();
    }

    if (State::g_pending_lightning_log_core1) {
        State::g_pending_lightning_log_core1 = false;
        if (DEBUG) Serial.println("[Core 1] Handling pending lightning log...");

        int distance = State::sensors.lastLightningDistance;
        char msg[60];
        snprintf(msg, sizeof(msg), "雷を検知しました！\n距離: 約%dkm", distance);
        Network::sendLineNotification(msg);
        
        char params[128];
        char encodedSheetName[64];
        Network::urlEncode("雷の受信履歴", encodedSheetName, sizeof(encodedSheetName));
        snprintf(params, sizeof(params), "?sheet=%s&dist=%d", encodedSheetName, distance);
        Network::logDataToGoogleSheet(params);
    }

    if (!State::system.ntpInitialized || time(nullptr) < 100000) {
        return;
    }

    time_t now = time(nullptr);
    struct tm *timeinfo = localtime(&now);
    int currentMinute = timeinfo->tm_min;

    static int lastSyncMinute = -1;
    static int lastLogMinute = -1;

    if (!State::system.initialLogSent) {
        if (State::sensors.temperature > -100) {
            char params[128];
            Network::createTempHumParams(params, sizeof(params));
            if (strlen(params) > 0) {
                Network::logDataToGoogleSheet(params);
                State::system.initialLogSent = true;
                lastLogMinute = currentMinute;
            }
        }
    }

    if (currentMinute % 4 == 0) {
        if (currentMinute != lastSyncMinute) {
            lastSyncMinute = currentMinute;
            Network::resyncNtpTime();
        }
    }

    if (currentMinute % 10 == 0) {
        if (currentMinute != lastLogMinute) {
            lastLogMinute = currentMinute;
            if (State::sensors.temperature > -100) {
                char params[128];
                Network::createTempHumParams(params, sizeof(params));
                if (strlen(params) > 0) {
                    Network::logDataToGoogleSheet(params);
                }
            }
        }
    }
}


