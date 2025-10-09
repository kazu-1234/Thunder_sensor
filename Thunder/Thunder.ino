// Ver 8.8
//================================================================
// ライブラリ
//================================================================
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <time.h>
#include <stdlib.h>
#include <WiFiNTP.h>
#include <DFRobot_DHT20.h>
#include <SparkFun_AS3935.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <LittleFS.h>
#include <math.h>       // for round()
#include <ArduinoOTA.h> // 標準の無線書き込み(OTA)用ライブラリ

// ★追加: 設定ファイルを読み込む
#include "config.h"

//================================================================
// ★★★ 設定項目 (本体の動作設定) ★★★
//================================================================
// --- 雷センサー感度設定 ---
const uint8_t LIGHTNING_WATCHDOG_THRESHOLD = 1; // 雷検知の強さの閾値 (調整範囲: 1～10)
const uint8_t LIGHTNING_SPIKE_REJECTION = 1;    // 電気的ノイズの除去レベル (調整範囲: 1～11)
const uint8_t INITIAL_NOISE_LEVEL = 1;          // 周囲の環境ノイズの初期レベル (調整範囲: 1～7)

// --- 動作設定 ---
const unsigned long LONG_PRESS_DURATION_MS = 1000;
const unsigned long INACTIVITY_TIMEOUT_MS = 5000;
const unsigned long BACKLIGHT_DURATION_MS = 3000;
const unsigned long SENSOR_READ_INTERVAL_MS = 2000; // センサー読み取り間隔
const unsigned long IP_DISPLAY_DURATION_MS = 5000;
const int HISTORY_SIZE = 3;
const int LCD_COLS = 20;
const int LCD_ROWS = 4;

//================================================================
// ピン定義
//================================================================
namespace Pins
{
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
namespace State
{
    // --- 動作モード ---
    enum Mode
    {
        MAIN_DISPLAY,
        MENU,
        HISTORY,
        SWITCHBOT_APPLIANCE_SELECT,
        DEVICE_CONTROL,
        WAKE_ON_LAN,
        ULTRASONIC_MONITOR,
        SENSOR_DIAGNOSTICS
    };

    // --- メニューの状態 ---
    struct MenuState
    {
        Mode currentMode = MAIN_DISPLAY;
        int menuSelection = 0;
        int deviceSelection = 0;
        int commandSelection = 0;
    };

    // --- システムの状態 ---
    struct SystemState
    {
        bool illuminationOn = false;
        bool backlightAlwaysOn = false;
        bool ntpInitialized = false;
        bool isAutoResync = false;
        bool needsRedraw = true;
        bool forceMainScreenRedraw = true;
        uint8_t currentNoiseLevel = 2;
        bool initialLogSent = false;
        bool pendingLightningLog = false;
        bool dht20_initialized = false;
        bool as3935_initialized = false;
    };

    // --- センサーデータ ---
    struct SensorData
    {
        float temperature = -999.0;
        float humidity = -999.0;
        float ultrasonicDistance = -1.0;
        char lastEventTime[20] = "N/A";
        String lastEventType = "None"; // "Lightning" or "Noise"
        int lastLightningDistance = -1;
    };

    // --- 本体メモリに記録する雷履歴 ---
    struct EventRecord
    {
        char timestamp[20];
        String type;
        int distance;
    };

    struct HistoryState
    {
        EventRecord records[HISTORY_SIZE];
        int index = 0;
        int count = 0;
    };

    // --- タイマー ---
    struct TimerState
    {
        unsigned long lastActivity = 0;
        unsigned long backlightOn = 0;
    };

    // --- 暗号化コンテキスト ---
    struct SHA256_CTX
    {
        uint8_t data[64];
        uint32_t datalen;
        uint64_t bitlen;
        uint32_t state[8];
    };

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
}

//================================================================
// 暗号化ユーティリティ (編集不要)
//================================================================
namespace Crypto
{
    const uint32_t K[64] = {0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    uint32_t rotr(uint32_t x, uint32_t n) { return (x >> n) | (x << (32 - n)); }
    void sha256_transform(State::SHA256_CTX *ctx, const uint8_t data[])
    {
        uint32_t a, b, c, d, e, f, g, h, i, j, t1, t2, m[64];
        for (i = 0, j = 0; i < 16; ++i, j += 4)
            m[i] = (data[j] << 24) | (data[j + 1] << 16) | (data[j + 2] << 8) | (data[j + 3]);
        for (; i < 64; ++i)
            m[i] = (((m[i - 2] >> 17) | (m[i - 2] << 15)) ^ ((m[i - 2] >> 19) | (m[i - 2] << 13)) ^ (m[i - 2] >> 10)) + m[i - 7] + (((m[i - 15] >> 7) | (m[i - 15] << 25)) ^ ((m[i - 15] >> 18) | (m[i - 15] << 14)) ^ (m[i - 15] >> 3)) + m[i - 16];
        a = ctx->state[0];
        b = ctx->state[1];
        c = ctx->state[2];
        d = ctx->state[3];
        e = ctx->state[4];
        f = ctx->state[5];
        g = ctx->state[6];
        h = ctx->state[7];
        for (i = 0; i < 64; ++i)
        {
            t1 = h + (((e >> 6) | (e << 26)) ^ ((e >> 11) | (e << 21)) ^ ((e >> 25) | (e << 7))) + ((e & f) ^ (~e & g)) + K[i] + m[i];
            t2 = (((a >> 2) | (a << 30)) ^ ((a >> 13) | (a << 19)) ^ ((a >> 22) | (a << 10))) + ((a & b) ^ (a & c) ^ (b & c));
            h = g;
            g = f;
            f = e;
            e = d + t1;
            d = c;
            c = b;
            b = a;
            a = t1 + t2;
        }
        ctx->state[0] += a;
        ctx->state[1] += b;
        ctx->state[2] += c;
        ctx->state[3] += d;
        ctx->state[4] += e;
        ctx->state[5] += f;
        ctx->state[6] += g;
        ctx->state[7] += h;
    }
    void sha256_init(State::SHA256_CTX *ctx)
    {
        ctx->datalen = 0;
        ctx->bitlen = 0;
        ctx->state[0] = 0x6a09e667;
        ctx->state[1] = 0xbb67ae85;
        ctx->state[2] = 0x3c6ef372;
        ctx->state[3] = 0xa54ff53a;
        ctx->state[4] = 0x510e527f;
        ctx->state[5] = 0x9b05688c;
        ctx->state[6] = 0x1f83d9ab;
        ctx->state[7] = 0x5be0cd19;
    }
    void sha256_update(State::SHA256_CTX *ctx, const uint8_t data[], size_t len)
    {
        for (uint32_t i = 0; i < len; ++i)
        {
            ctx->data[ctx->datalen] = data[i];
            ctx->datalen++;
            if (ctx->datalen == 64)
            {
                sha256_transform(ctx, ctx->data);
                ctx->bitlen += 512;
                ctx->datalen = 0;
            }
        }
    }
    void sha256_final(State::SHA256_CTX *ctx, uint8_t hash[])
    {
        uint32_t i = ctx->datalen;
        if (ctx->datalen < 56)
        {
            ctx->data[i++] = 0x80;
            while (i < 56)
                ctx->data[i++] = 0x00;
        }
        else
        {
            ctx->data[i++] = 0x80;
            while (i < 64)
                ctx->data[i++] = 0x00;
            sha256_transform(ctx, ctx->data);
            memset(ctx->data, 0, 56);
        }
        ctx->bitlen += ctx->datalen * 8;
        ctx->data[63] = ctx->bitlen;
        ctx->data[62] = ctx->bitlen >> 8;
        ctx->data[61] = ctx->bitlen >> 16;
        ctx->data[60] = ctx->bitlen >> 24;
        ctx->data[59] = ctx->bitlen >> 32;
        ctx->data[58] = ctx->bitlen >> 40;
        ctx->data[57] = ctx->bitlen >> 48;
        ctx->data[56] = ctx->bitlen >> 56;
        sha256_transform(ctx, ctx->data);
        for (i = 0; i < 4; ++i)
        {
            hash[i] = (ctx->state[0] >> (24 - i * 8)) & 0xff;
            hash[i + 4] = (ctx->state[1] >> (24 - i * 8)) & 0xff;
            hash[i + 8] = (ctx->state[2] >> (24 - i * 8)) & 0xff;
            hash[i + 12] = (ctx->state[3] >> (24 - i * 8)) & 0xff;
            hash[i + 16] = (ctx->state[4] >> (24 - i * 8)) & 0xff;
            hash[i + 20] = (ctx->state[5] >> (24 - i * 8)) & 0xff;
            hash[i + 24] = (ctx->state[6] >> (24 - i * 8)) & 0xff;
            hash[i + 28] = (ctx->state[7] >> (24 - i * 8)) & 0xff;
        }
    }
    void hmac_sha256(const uint8_t *key, size_t keylen, const uint8_t *data, size_t datalen, uint8_t *out)
    {
        State::SHA256_CTX ctx;
        uint8_t k_ipad[65], k_opad[65], tk[32];
        memset(k_ipad, 0, sizeof(k_ipad));
        memset(k_opad, 0, sizeof(k_opad));
        if (keylen > 64)
        {
            sha256_init(&ctx);
            sha256_update(&ctx, key, keylen);
            sha256_final(&ctx, tk);
            key = tk;
            keylen = 32;
        }
        memcpy(k_ipad, key, keylen);
        memcpy(k_opad, key, keylen);
        for (int i = 0; i < 64; i++)
        {
            k_ipad[i] ^= 0x36;
            k_opad[i] ^= 0x5c;
        }
        sha256_init(&ctx);
        sha256_update(&ctx, k_ipad, 64);
        sha256_update(&ctx, data, datalen);
        sha256_final(&ctx, out);
        sha256_init(&ctx);
        sha256_update(&ctx, k_opad, 64);
        sha256_update(&ctx, out, 32);
        sha256_final(&ctx, out);
    }
    String base64_encode(const uint8_t *data, size_t len)
    {
        static const char *b64_table = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        String ret;
        ret.reserve((len + 2) / 3 * 4);
        for (size_t i = 0; i < len; i += 3)
        {
            uint32_t octet_a = i < len ? data[i] : 0;
            uint32_t octet_b = i + 1 < len ? data[i + 1] : 0;
            uint32_t octet_c = i + 2 < len ? data[i + 2] : 0;
            uint32_t triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;
            ret += b64_table[(triple >> 3 * 6) & 0x3F];
            ret += b64_table[(triple >> 2 * 6) & 0x3F];
            ret += b64_table[(triple >> 1 * 6) & 0x3F];
            ret += b64_table[(triple >> 0 * 6) & 0x3F];
        }
        if (len % 3 == 1)
        {
            ret[ret.length() - 1] = '=';
            ret[ret.length() - 2] = '=';
        }
        if (len % 3 == 2)
        {
            ret[ret.length() - 1] = '=';
        }
        return ret;
    }
}

//================================================================
// プロトタイプ宣言
//================================================================
namespace Menu
{
    void changeMode(State::Mode newMode);
}
namespace Display
{
    void printLcdLine(int line, const char *text);
    void updateMainDisplay();
}
namespace Network
{
    void sendSwitchBotCommand(const char *deviceId, const char *command, const char *parameter, const char *commandType = "command");
    void resyncNtpTime();
    void getSwitchBotDeviceList();
    void sendLineTestMessage();
    void sendWakeOnLan(const char *macStr);
    void requestDistance();
    void showIpAddressAndHold();
    void logDataToGoogleSheet(String params);
    void manualLogToSheet();
    bool connectToWiFi(bool showOnLcd);
    void checkWiFiConnection();
    void handleNtpSync();
}
namespace Utils
{
    void toggleBacklightMode();
    void toggleIlluminationMode();
    void rebootDevice();
}
namespace Sensors
{
    void init();
    void calibrateSensor();
    void updateDht();
}
void handlePeriodicTasks();
void handlePendingNetworkTasks();
void handleFourSecondCycle();

//================================================================
// メニュー定義
//================================================================
namespace Menu
{
    struct MenuItem
    {
        const char *text;
        void (*action)();
    };
    void performMenuAction(void (*action)(), bool returnToMain);

    void enterHistory() { changeMode(State::HISTORY); }
    void enterSwitchbotMenu() { changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }
    void enterWakeOnLan() { changeMode(State::WAKE_ON_LAN); }
    void enterUltrasonic() { changeMode(State::ULTRASONIC_MONITOR); }
    void enterDiagnostics() { changeMode(State::SENSOR_DIAGNOSTICS); }
    void enterDeviceControl()
    {
        State::menu.deviceSelection = State::menu.menuSelection;
        State::menu.commandSelection = 0;
        changeMode(State::DEVICE_CONTROL);
    }

// Small helpers to reduce repetitive lambda boilerplate in menus
#define PM(action, ret) ([]() { performMenuAction(action, ret); })
#define SB(dev, cmd, param, type, ret) ([]() { performMenuAction([]() { Network::sendSwitchBotCommand(dev, cmd, param, type); }, ret); })
#define SB_DEF(dev, cmd, param, ret) SB(dev, cmd, param, "command", ret)
// helper to call arbitrary statement(s) inside performMenuAction
#define PM_CALL(call, ret) ([]() { performMenuAction([]() { call; }, ret); })
    const MenuItem mainMenu[] = {
        {"1. Lightning Log", enterHistory},
        {"2. Device Control", enterSwitchbotMenu},
        {"3. Wake on LAN", enterWakeOnLan},
        {"4. Backlight Mode", PM(Utils::toggleBacklightMode, true)},
        {"5. RGB Illumination", PM(Utils::toggleIlluminationMode, true)},
        {"6. Measure Dist", enterUltrasonic},
        {"7. Resync Time", PM(Network::resyncNtpTime, true)},
        {"8. Show IP Address", PM_CALL(Network::showIpAddressAndHold(); Menu::changeMode(State::MAIN_DISPLAY), false)},
        {"9. Get SwitchBot IDs", PM(Network::getSwitchBotDeviceList, true)},
        {"10. Send LINE Test", PM(Network::sendLineTestMessage, true)},
        {"11. Log Data Manually", PM(Network::manualLogToSheet, true)},
        {"12. Reboot", Utils::rebootDevice},
        {"13. Sensor Diag", enterDiagnostics},
        {"14. Calibrate Sensor", PM(Sensors::calibrateSensor, true)}};
    const int MAIN_MENU_COUNT = sizeof(mainMenu) / sizeof(mainMenu[0]);

    const MenuItem applianceMenu[] = {
        {"1. Light", enterDeviceControl}, {"2. TV", enterDeviceControl}, {"3. Air Conditioner", enterDeviceControl}, {"4. Fan", enterDeviceControl}, {"5. Speaker", enterDeviceControl}, {"6. Others", enterDeviceControl}, {"7. Back", []()
                                                                                                                                                                                                                             { changeMode(State::MENU); }}};
    const int APPLIANCE_MENU_COUNT = sizeof(applianceMenu) / sizeof(applianceMenu[0]);

    const MenuItem wolMenu[] = {
        {"1. Desktop PC", PM_CALL(Network::sendWakeOnLan(MAC_DESKTOP), true)},
        {"2. Server PC", PM_CALL(Network::sendWakeOnLan(MAC_SERVER), true)}};
    const int WOL_MENU_COUNT = sizeof(wolMenu) / sizeof(wolMenu[0]);

    const MenuItem lightControlMenu[] = {
        {"On", SB_DEF(DEVICE_ID_LIGHT, "turnOn", "default", false)},
        {"Off", SB_DEF(DEVICE_ID_LIGHT, "turnOff", "default", false)},
        {"Bright+", SB_DEF(DEVICE_ID_LIGHT, "brightnessUp", "default", false)},
        {"Bright-", SB_DEF(DEVICE_ID_LIGHT, "brightnessDown", "default", false)},
        {"Warm", SB_DEF(DEVICE_ID_LIGHT, "setColorTemperature", "2700", false)},
        {"White", SB_DEF(DEVICE_ID_LIGHT, "setColorTemperature", "6500", false)},
        {"Back", []()
         { changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}};
    const MenuItem tvControlMenu[] = {
        {"Power", SB_DEF(DEVICE_ID_TV, "turnOn", "default", false)},
        {"CH +", SB_DEF(DEVICE_ID_TV, "channelAdd", "default", false)},
        {"CH -", SB_DEF(DEVICE_ID_TV, "channelSub", "default", false)},
        {"Vol +", SB_DEF(DEVICE_ID_TV, "volumeAdd", "default", false)},
        {"Vol -", SB_DEF(DEVICE_ID_TV, "volumeSub", "default", false)},
        {"Back", []()
         { changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}};
    const MenuItem acControlMenu[] = {
        {"Run", SB_DEF(DEVICE_ID_AC, "turnOn", "default", false)},
        {"Stop", SB_DEF(DEVICE_ID_AC, "turnOff", "default", false)},
        {"Temp +", SB_DEF(DEVICE_ID_AC, "setTemperature", "26,auto,1,on", false)},
        {"Temp -", SB_DEF(DEVICE_ID_AC, "setTemperature", "24,auto,1,on", false)},
        {"Cooling", SB_DEF(DEVICE_ID_AC, "setAll", "25,2,1,on", false)},
        {"Heating", SB_DEF(DEVICE_ID_AC, "setAll", "22,5,1,on", false)},
        {"Dehumidify", SB_DEF(DEVICE_ID_AC, "setAll", "25,3,1,on", false)},
        {"Back", []()
         { changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}};
    const MenuItem fanControlMenu[] = {
        {"On", SB_DEF(DEVICE_ID_FAN, "press", "運転", false)},
        {"Off", SB_DEF(DEVICE_ID_FAN, "press", "切/入", false)},
        {"Speed +", SB_DEF(DEVICE_ID_FAN, "press", "風量+", false)},
        {"Speed -", SB_DEF(DEVICE_ID_FAN, "press", "風量-", false)},
        {"Swing", SB_DEF(DEVICE_ID_FAN, "press", "スウィング", false)},
        {"Back", []()
         { changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}};
    const MenuItem speakerControlMenu[] = {
        {"Power", SB_DEF(DEVICE_ID_SPEAKER, "press", "Power", false)},
        {"Vol +", SB_DEF(DEVICE_ID_SPEAKER, "volumeAdd", "default", false)},
        {"Vol -", SB_DEF(DEVICE_ID_SPEAKER, "volumeSub", "default", false)},
        {"Prev", SB_DEF(DEVICE_ID_SPEAKER, "previousTrack", "default", false)},
        {"Next", SB_DEF(DEVICE_ID_SPEAKER, "nextTrack", "default", false)},
        {"Back", []()
         { changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}};
    const MenuItem othersControlMenu[] = {
        {"Command 1", SB_DEF(DEVICE_ID_OTHERS, "turnOn", "default", false)},
        {"Command 2", SB_DEF(DEVICE_ID_OTHERS, "turnOff", "default", false)},
        {"Back", []()
         { changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}};

    const MenuItem *const applianceControlMenus[] = {lightControlMenu, tvControlMenu, acControlMenu, fanControlMenu, speakerControlMenu, othersControlMenu};
    const int applianceControlMenuCounts[] = {7, 6, 8, 6, 6, 3};

    const MenuItem *getCurrentMenu(int &count, int &selection)
    {
        switch (State::menu.currentMode)
        {
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
            if (State::menu.deviceSelection < sizeof(applianceControlMenus) / sizeof(MenuItem **))
            {
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
namespace Utils
{
    void setRGB(int r, int g, int b)
    {
        analogWrite(Pins::LED_R, r);
        analogWrite(Pins::LED_G, g);
        analogWrite(Pins::LED_B, b);
    }

    void blinkLED(String color, int times, int duration)
    {
        for (int i = 0; i < times; i++)
        {
            if (color == "green")
                setRGB(0, 255, 0);
            else if (color == "yellow")
                setRGB(255, 255, 0);
            else if (color == "blue")
                setRGB(0, 0, 255);
            else if (color == "white")
                setRGB(255, 255, 255);
            delay(duration);
            setRGB(0, 0, 0);
            delay(duration);
        }
    }

    void handleSmoothIllumination()
    {
        unsigned long hue = millis() / 10;
        hue %= 360;
        float s = 1.0, v = 1.0, r, g, b;
        int i = floor(hue / 60.0);
        float f = hue / 60.0 - i, p = v * (1.0 - s), q = v * (1.0 - (s * f)), t = v * (1.0 - (s * (1.0 - f)));
        switch (i % 6)
        {
        case 0:
            r = v, g = t, b = p;
            break;
        case 1:
            r = q, g = v, b = p;
            break;
        case 2:
            r = p, g = v, b = t;
            break;
        case 3:
            r = p, g = q, b = v;
            break;
        case 4:
            r = t, g = p, b = v;
            break;
        case 5:
            r = v, g = p, b = q;
            break;
        }
        setRGB(r * 255, g * 255, b * 255);
    }

    void toggleBacklightMode()
    {
        State::system.backlightAlwaysOn = !State::system.backlightAlwaysOn;
        digitalWrite(Pins::LCD_BACKLIGHT, State::system.backlightAlwaysOn ? HIGH : LOW);
        State::lcd.clear();
        Display::printLcdLine(0, "Backlight Mode");
        Display::printLcdLine(1, State::system.backlightAlwaysOn ? "Always ON" : "Auto OFF");
    }

    void toggleIlluminationMode()
    {
        State::system.illuminationOn = !State::system.illuminationOn;
        if (!State::system.illuminationOn)
        {
            setRGB(0, 0, 0);
        }
        State::lcd.clear();
        Display::printLcdLine(0, "Illumination Mode");
        Display::printLcdLine(1, State::system.illuminationOn ? "ON" : "OFF");
    }

    void rebootDevice()
    {
        State::lcd.clear();
        Display::printLcdLine(0, "Rebooting...");
        delay(1000);
        rp2040.reboot();
    }
}

//================================================================
// ディスプレイ管理
//================================================================
namespace Display
{
    // LCDの特定の行にテキストを表示する
    void printLcdLine(int line, const char *text)
    {
        char buf[LCD_COLS + 1];
        snprintf(buf, sizeof(buf), "%-*s", LCD_COLS, text);
        State::lcd.setCursor(0, line);
        State::lcd.print(buf);
    }

    // 画面の各パーツを描画するヘルパー関数
    namespace
    { // 無名名前空間でこのファイル内からのみアクセス可能にする
        void drawTime()
        {
            char buf[LCD_COLS + 1];
            if (State::system.ntpInitialized && time(nullptr) > 100000)
            {
                time_t now = time(nullptr);
                strftime(buf, sizeof(buf), "%m/%d(%a) %H:%M:%S", localtime(&now));
            }
            else
            {
                snprintf(buf, sizeof(buf), "Connecting Network..");
            }
            printLcdLine(0, buf);
        }

        void drawSensors()
        {
            char buf[LCD_COLS + 1];
            if (!State::system.dht20_initialized)
            {
                snprintf(buf, sizeof(buf), "DHT20 Init Failed!");
            }
            else if (State::sensors.temperature > -999.0)
            {
                snprintf(buf, sizeof(buf), "T:%.1fC  H:%.1f%%", State::sensors.temperature, State::sensors.humidity);
            }
            else
            {
                snprintf(buf, sizeof(buf), "Sensor Reading...");
            }
            printLcdLine(1, buf);
        }

        void drawLightningInfo()
        {
            char buf[LCD_COLS + 1];
            if (State::sensors.lastEventType == "Lightning")
            {
                snprintf(buf, sizeof(buf), "Last Evt:%s", State::sensors.lastEventTime);
                printLcdLine(2, buf);
                snprintf(buf, sizeof(buf), "Distance: %d km", State::sensors.lastLightningDistance);
                printLcdLine(3, buf);
            }
            else if (State::sensors.lastEventType == "Noise")
            {
                snprintf(buf, sizeof(buf), "Last Evt:%s", State::sensors.lastEventTime);
                printLcdLine(2, buf);
                printLcdLine(3, "Distance: Noise");
            }
            else
            {
                printLcdLine(2, "Last Event: None");
                if (!State::system.as3935_initialized)
                {
                    printLcdLine(3, "AS3935 Init Failed");
                }
                else
                {
                    printLcdLine(3, "Distance: ---");
                }
            }
        }
    }

    // メイン画面の表示を更新する
    void updateMainDisplay()
    {
        static unsigned long lastTimeUpdate = 0;
        static unsigned long lastSensorDraw = 0;

        // 画面全体を再描画する必要があるか
        if (State::system.forceMainScreenRedraw)
        {
            State::lcd.clear();
            drawTime();
            drawSensors();
            drawLightningInfo();
            lastTimeUpdate = millis();
            lastSensorDraw = millis();
        }
        else
        {
            // 1秒ごとに時刻表示を更新
            if (millis() - lastTimeUpdate > 1000)
            {
                drawTime();
                lastTimeUpdate = millis();
            }
            // センサーの読み取り間隔に合わせて表示も更新
            if (millis() - lastSensorDraw > SENSOR_READ_INTERVAL_MS)
            {
                drawSensors();
                lastSensorDraw = millis();
            }
        }
    }

    // メニュー画面を描画する
    void drawMenu()
    {
        int itemCount = 0, currentSelection = 0;
        const Menu::MenuItem *menuItems = Menu::getCurrentMenu(itemCount, currentSelection);
        if (!menuItems)
            return;

        int page = currentSelection / LCD_ROWS;
        for (int i = 0; i < LCD_ROWS; i++)
        {
            int index = page * LCD_ROWS + i;
            char buf[LCD_COLS + 1];
            if (index < itemCount)
            {
                sprintf(buf, "%s%s", (index == currentSelection ? ">" : " "), menuItems[index].text);
            }
            else
            {
                strcpy(buf, "");
            }
            printLcdLine(i, buf);
        }
    }

    // 雷履歴画面を描画する
    void drawHistoryScreen()
    {
        printLcdLine(0, "--- Lightning Log ---");
        for (int i = 0; i < HISTORY_SIZE; i++)
        {
            char buf[LCD_COLS + 1];
            if (i < State::history.count)
            {
                int idx = (State::history.index - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
                if (State::history.records[idx].type == "Noise")
                {
                    snprintf(buf, sizeof(buf), "%d: %s Noise", i + 1, State::history.records[idx].timestamp);
                }
                else
                {
                    snprintf(buf, sizeof(buf), "%d: %s %2dkm", i + 1, State::history.records[idx].timestamp, State::history.records[idx].distance);
                }
            }
            else
            {
                snprintf(buf, sizeof(buf), "%d: ---", i + 1);
            }
            printLcdLine(i + 1, buf);
        }
    }

    // 超音波センサー監視画面を描画する
    void drawUltrasonicMonitorScreen()
    {
        static unsigned long lastRequestTime = 0;
        if (millis() - lastRequestTime > 500)
        {
            lastRequestTime = millis();
            Network::requestDistance();
        }

        printLcdLine(0, "Ultrasonic Sensor");
        char buf[LCD_COLS + 1];
        if (State::childIpAddress == "")
            snprintf(buf, sizeof(buf), "Child not found");
        else if (State::sensors.ultrasonicDistance < 0)
            snprintf(buf, sizeof(buf), "Requesting data...");
        else
            snprintf(buf, sizeof(buf), "Dist: %.1f cm", State::sensors.ultrasonicDistance);
        printLcdLine(1, buf);
        printLcdLine(2, "");
        printLcdLine(3, "(Long press to exit)");
    }

    // センサー診断画面を描画する
    void drawDiagnosticsScreen()
    {
        static unsigned long lastReadTime = 0;
        if (millis() - lastReadTime > 1000)
        { // 1秒ごとに更新
            lastReadTime = millis();

            printLcdLine(0, "--- Sensor Diag ---");
            char buf[LCD_COLS + 1];

            // 公開されている関数を使ってセンサーの状態を表示
            uint8_t noise = State::lightning.readNoiseLevel();
            uint8_t watchdog = State::lightning.readWatchdogThreshold();
            snprintf(buf, sizeof(buf), "Noise:%d Watchdog:%d", noise, watchdog);
            printLcdLine(1, buf);

            uint8_t spike = State::lightning.readSpikeRejection();
            uint8_t intReg = State::lightning.readInterruptReg(); // 割り込みレジスタは読める
            snprintf(buf, sizeof(buf), "Spike:%d IntReg:0x%02X", spike, intReg);
            printLcdLine(2, buf);

            int irqPinState = digitalRead(Pins::LIGHTNING_IRQ);
            snprintf(buf, sizeof(buf), "IRQ Pin State: %d", irqPinState);
            printLcdLine(3, buf);
        }
    }

    // 現在のモードに応じて画面を更新する
    void update()
    {
        if (State::system.needsRedraw)
        {
            State::lcd.clear();                         // モードが切り替わったら一度画面をクリア
            State::system.forceMainScreenRedraw = true; // メイン画面に戻った時に全体再描画を強制
        }

        switch (State::menu.currentMode)
        {
        case State::MAIN_DISPLAY:
            updateMainDisplay();
            break;
        case State::ULTRASONIC_MONITOR:
            drawUltrasonicMonitorScreen();
            break;
        case State::HISTORY:
            drawHistoryScreen();
            break;
        case State::SENSOR_DIAGNOSTICS:
            drawDiagnosticsScreen();
            break;
        default: // MENU, DEVICE_CONTROLなど
            if (State::system.needsRedraw)
                drawMenu();
            break;
        }

        State::system.needsRedraw = false;
        State::system.forceMainScreenRedraw = false;
    }

    // ディスプレイの初期化
    void init()
    {
        State::lcd.begin(LCD_COLS, LCD_ROWS);
        Display::printLcdLine(0, "System Starting...");
    }
}

//================================================================
// ネットワーク管理
//================================================================
namespace Network
{

    // OTA(無線書き込み)機能の初期化
    void initOTA()
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            // Arduino IDEの「ポート」に表示される名前を設定
            ArduinoOTA.setHostname("pico-lightning-sensor");

            // 書き込み中のイベントをシリアルモニタに表示する設定
            ArduinoOTA.onStart([]()
                               {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
            else // U_SPIFFS
                type = "filesystem";
            if (DEBUG) Serial.println("[OTA] Start updating " + type); });
            ArduinoOTA.onEnd([]()
                             {
            if (DEBUG) Serial.println("\n[OTA] End"); });
            ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                                  {
            if (DEBUG) Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100))); });
            ArduinoOTA.onError([](ota_error_t error)
                               {
            if (DEBUG) {
                Serial.printf("[OTA] Error[%u]: ", error);
                if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
                else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
                else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
                else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
                else if (error == OTA_END_ERROR) Serial.println("End Failed");
            } });

            // OTAを開始
            ArduinoOTA.begin();
            if (DEBUG)
                Serial.println("[OTA] Ready. Hostname: pico-lightning-sensor");
        }
        else
        {
            if (DEBUG)
                Serial.println("[OTA] Init Failed: WiFi not connected.");
        }
    }

    // URLエンコードを行う関数
    String urlEncode(const char *msg)
    {
        const char *hex = "0123456789abcdef";
        String encodedMsg = "";
        while (*msg != '\0')
        {
            if (('a' <= *msg && *msg <= 'z') || ('A' <= *msg && *msg <= 'Z') || ('0' <= *msg && *msg <= '9') || *msg == '-' || *msg == '_' || *msg == '.' || *msg == '~')
            {
                encodedMsg += *msg;
            }
            else
            {
                encodedMsg += '%';
                encodedMsg += hex[*msg >> 4];
                encodedMsg += hex[*msg & 15];
            }
            msg++;
        }
        return encodedMsg;
    }

    String createTempHumParams()
    {
        if (State::sensors.temperature > -100)
        {
            // 日本語のシート名をurlEncode関数でエンコードする
            return "?sheet=" + urlEncode("温湿度") + "&temp=" + String(State::sensors.temperature, 1) + "&hum=" + String(State::sensors.humidity, 1);
        }
        return "";
    }

    void addAuthHeaders(HTTPClient &http)
    {
        String token = SWITCHBOT_TOKEN;
        String secret = SWITCHBOT_SECRET;
        time_t t_val;
        time(&t_val);
        char t_str[15];
        sprintf(t_str, "%lu", (unsigned long)t_val);
        String t = String(t_str) + "000";
        String nonce = "";
        for (int i = 0; i < 16; i++)
            nonce += String(random(16), HEX);
        String dataToSign = token + t + nonce;
        byte hmacResult[32];
        Crypto::hmac_sha256((const uint8_t *)secret.c_str(), secret.length(), (const uint8_t *)dataToSign.c_str(), dataToSign.length(), hmacResult);
        String sign = Crypto::base64_encode(hmacResult, 32);
        http.addHeader("Authorization", token);
        http.addHeader("t", t);
        http.addHeader("nonce", nonce);
        http.addHeader("sign", sign);
    }

    void sendLineNotification(String message)
    {
        if (WiFi.status() != WL_CONNECTED)
            return;
        WiFiClientSecure client;
        client.setInsecure();
        HTTPClient http;
        if (http.begin(client, "https://api.line.me/v2/bot/message/push"))
        {
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

    void sendSwitchBotCommand(const char *deviceId, const char *command, const char *parameter, const char *commandType)
    {
        State::lcd.clear();
        Display::printLcdLine(0, "Sending Command...");
        if (WiFi.status() != WL_CONNECTED)
        {
            Display::printLcdLine(1, "WiFi Disconnected");
            return;
        }
        WiFiClientSecure secureClient;
        secureClient.setInsecure();
        HTTPClient http;
        if (http.begin(secureClient, "https://api.switch-bot.com/v1.1/devices/" + String(deviceId) + "/commands"))
        {
            addAuthHeaders(http);
            http.addHeader("Content-Type", "application/json; charset=utf-8");
            JsonDocument doc;
            doc["command"] = command;
            doc["parameter"] = parameter;
            doc["commandType"] = commandType;
            String jsonBody;
            serializeJson(doc, jsonBody);
            int httpCode = http.POST(jsonBody);
            Display::printLcdLine(1, httpCode == HTTP_CODE_OK ? "Success!" : "Failed!");
            http.end();
        }
        else
        {
            Display::printLcdLine(1, "Begin Failed!");
        }
    }

    // MACアドレス文字列("XX:XX:XX:XX:XX:XX")をbyte配列に変換する関数
    bool parseMacAddress(const char *macStr, byte *macArray)
    {
        // 文字列の長さが17文字でない場合はエラー
        if (strlen(macStr) != 17)
        {
            return false;
        }
        // sscanfを使用して6つの16進数値を読み取る
        int values[6];
        int count = sscanf(macStr, "%x:%x:%x:%x:%x:%x", &values[0], &values[1], &values[2], &values[3], &values[4], &values[5]);
        // 6個の値を正しく読み取れなかった場合はエラー
        if (count != 6)
        {
            return false;
        }
        // 読み取った値をbyte配列に格納
        for (int i = 0; i < 6; ++i)
        {
            macArray[i] = (byte)values[i];
        }
        return true; // 成功
    }

    // MACアドレスを文字列(const char*)で受け取るように変更
    void sendWakeOnLan(const char *macStr)
    {
        State::lcd.clear();
        Display::printLcdLine(0, "Sending WoL Packet...");

        byte mac[6];
        // MACアドレス文字列をbyte配列に変換
        if (!parseMacAddress(macStr, mac))
        {
            Display::printLcdLine(1, "Invalid MAC Addr!");
            if (DEBUG)
                Serial.printf("[WoL] Error: Invalid MAC address format in config.h: %s\n", macStr);
            return; // 変換に失敗した場合は処理を中断
        }

        WiFiUDP udp;
        if (udp.begin(9))
        {
            byte magicPacket[102];
            memset(magicPacket, 0xFF, 6);
            for (int i = 1; i <= 16; i++)
            {
                memcpy(&magicPacket[i * 6], mac, 6);
            }
            udp.beginPacket(IPAddress(255, 255, 255, 255), 9);
            udp.write(magicPacket, sizeof(magicPacket));
            udp.endPacket();
            Display::printLcdLine(1, "Packet Sent!");
            if (DEBUG)
                Serial.println("[WoL] Magic packet sent.");
        }
        else
        {
            Display::printLcdLine(1, "UDP Setup Failed!");
            if (DEBUG)
                Serial.println("[WoL] UDP setup failed.");
        }
    }

    void requestDistance()
    {
        if (State::childIpAddress == "")
        {
            State::sensors.ultrasonicDistance = -2.0;
            return;
        }
        HTTPClient http;
        WiFiClient client;
        if (http.begin(client, "http://" + State::childIpAddress + "/distance"))
        {
            http.setTimeout(400);
            if (http.GET() == HTTP_CODE_OK)
            {
                JsonDocument doc;
                if (deserializeJson(doc, http.getString()) == DeserializationError::Ok)
                {
                    State::sensors.ultrasonicDistance = doc["distance"];
                }
            }
            else
            {
                State::sensors.ultrasonicDistance = -3.0;
            }
            http.end();
        }
        else
        {
            State::sensors.ultrasonicDistance = -4.0;
        }
    }

    void handleNtpSync()
    {
        static unsigned long ntpAttemptStart = 0;
        const unsigned long ntpTimeout = 15000; // 15 seconds timeout

        // Condition to start NTP sync: not already syncing.
        if (ntpAttemptStart == 0)
        {
            if (DEBUG)
                Serial.println("[Network] Starting background NTP sync...");
            setenv("TZ", "JST-9", 1);
            tzset();
            NTP.begin(NTP_SERVER);
            ntpAttemptStart = millis(); // Mark the start time
            return;
        }

        // If a sync is in progress
        if (ntpAttemptStart > 0)
        {
            // Check for success
            if (time(nullptr) > 100000)
            {
                if (DEBUG)
                    Serial.println("\n[Network] NTP Sync Successful!");
                State::system.ntpInitialized = true;
                State::server.begin();
                State::system.forceMainScreenRedraw = true; // Force redraw to show the correct time
                ntpAttemptStart = 0;                        // End the sync process tracking
            }
            // Check for timeout
            else if (millis() - ntpAttemptStart > ntpTimeout)
            {
                if (DEBUG)
                    Serial.println("\n[Network] NTP Sync Failed (Timeout). Will retry.");
                ntpAttemptStart = 0; // Reset to allow a new attempt
            }
        }
    }

    void handleServerClient()
    {
        WiFiClient client = State::server.accept();
        if (client)
        {
            if (State::childIpAddress == "")
            {
                Utils::blinkLED("blue", 3, 150);
            }
            State::childIpAddress = client.remoteIP().toString();
            unsigned long timeout = millis();
            while (!client.available() && millis() - timeout < 500)
            {
                delay(1);
            }
            JsonDocument responseDoc;
            responseDoc["temperature"] = State::sensors.temperature;
            responseDoc["humidity"] = State::sensors.humidity;
            responseDoc["last_event_time"] = State::sensors.lastEventTime;
            responseDoc["last_event_type"] = State::sensors.lastEventType;
            if (State::sensors.lastEventType == "Lightning")
            {
                responseDoc["last_lightning_dist"] = State::sensors.lastLightningDistance;
            }
            String responseBody;
            serializeJson(responseDoc, responseBody);
            client.println("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close");
            client.printf("Content-Length: %d\r\n\r\n", responseBody.length());
            client.print(responseBody);
            client.stop();
        }
    }

    // WiFi接続を行うブロッキング関数
    bool connectToWiFi(bool showOnLcd)
    {
        if (DEBUG)
            Serial.println("[Network] Starting WiFi connection...");
        if (showOnLcd)
        {
            Display::printLcdLine(0, "Connecting WiFi...");
        }

        for (int i = 0; i < numWifiCredentials; i++)
        {
            if (strlen(wifiCredentials[i].ssid) == 0)
                continue;

            if (DEBUG)
                Serial.printf("[Network] Attempting to connect to SSID: %s\n", wifiCredentials[i].ssid);
            if (showOnLcd)
            {
                char lcd_buf[21];
                snprintf(lcd_buf, sizeof(lcd_buf), "SSID: %s", wifiCredentials[i].ssid);
                Display::printLcdLine(1, lcd_buf);
                Display::printLcdLine(2, "Connecting");
                Display::printLcdLine(3, "");
            }

            WiFi.begin(wifiCredentials[i].ssid, wifiCredentials[i].password);

            unsigned long startTime = millis();
            int animationState = 0;
            const char *animFrames[] = {"  ", ". ", "..", " ."};

            while (WiFi.status() != WL_CONNECTED)
            {
                if (millis() - startTime > 15000)
                { // 15秒でタイムアウト
                    if (DEBUG)
                        Serial.println("Connection timed out.");
                    WiFi.disconnect(true);
                    delay(100);
                    break;
                }

                if (showOnLcd)
                {
                    char lcd_buf[21];
                    snprintf(lcd_buf, sizeof(lcd_buf), "Connecting%s", animFrames[animationState]);
                    Display::printLcdLine(2, lcd_buf);
                    animationState = (animationState + 1) % 4;
                }

                delay(500);
                if (DEBUG)
                    Serial.print(".");
            }

            if (WiFi.status() == WL_CONNECTED)
            {
                Utils::blinkLED("green", 2, 100);

                if (DEBUG)
                {
                    Serial.println("\n[Network] WiFi connected!");
                    Serial.printf("  - SSID: %s\n", WiFi.SSID().c_str());
                    Serial.printf("  - IP Address: %s\n", WiFi.localIP().toString().c_str());
                }
                if (showOnLcd)
                {
                    // ★追加: 画面をクリアして接続成功メッセージを明確に表示
                    State::lcd.clear();
                    Display::printLcdLine(0, "WiFi Connected!");
                    Display::printLcdLine(1, WiFi.localIP().toString().c_str());
                    Display::printLcdLine(2, WiFi.SSID().c_str());
                    delay(2000);
                }
                return true;
            }
        }

        if (DEBUG)
            Serial.println("\n[Network] Failed to connect to any WiFi network.");
        if (showOnLcd)
        {
            Display::printLcdLine(0, "WiFi Connect Fail");
            Display::printLcdLine(1, "Check settings.");
            delay(2000);
        }
        return false;
    }

    // WiFiの接続を監視し、切断されたら再接続を試みる関数
    void checkWiFiConnection()
    {
        static unsigned long lastReconnectAttempt = 0;
        const unsigned long reconnectInterval = 60000;

        if (WiFi.status() != WL_CONNECTED)
        {
            if (millis() - lastReconnectAttempt > reconnectInterval)
            {
                if (DEBUG)
                    Serial.println("[Network] WiFi disconnected. Attempting to reconnect...");
                // 再接続はLCDに詳細表示なしで行う
                if (connectToWiFi(false))
                {
                    if (DEBUG)
                        Serial.println("[Network] WiFi reconnected successfully.");
                    State::system.ntpInitialized = false; // 再接続後はNTPも再同期が必要
                    initOTA();                            // OTAも再初期化
                }
                else
                {
                    if (DEBUG)
                        Serial.println("[Network] Failed to reconnect. Will try again in 1 minute.");
                }
                lastReconnectAttempt = millis();
            }
        }
    }

    void logDataToGoogleSheet(String params)
    {
        if (WiFi.status() != WL_CONNECTED || String(GAS_URL).startsWith("-----"))
        {
            if (DEBUG)
                Serial.println("[Network] Cannot log to sheet: WiFi not connected or GAS_URL not set.");
            return;
        }
        WiFiClientSecure client;
        client.setInsecure(); // SSL証明書を検証しない
        HTTPClient http;
        String url = String(GAS_URL) + params;

        if (DEBUG)
        {
            Serial.print("[Network] Requesting Google Sheet URL: ");
            Serial.println(url);
        }

        if (http.begin(client, url))
        {
            http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
            int httpCode = http.GET();

            if (httpCode > 0)
            {
                if (DEBUG)
                    Serial.printf("[Network] Google Sheet GET... code: %d\n", httpCode);
                if (httpCode == HTTP_CODE_OK)
                {
                    String payload = http.getString();
                    if (DEBUG)
                    {
                        Serial.println("[Network] Google Sheet Response payload:");
                        Serial.println(payload);
                    }
                }
            }
            else
            {
                if (DEBUG)
                    Serial.printf("[Network] Google Sheet GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
            }
            http.end();
        }
        else
        {
            if (DEBUG)
                Serial.printf("[Network] Unable to connect\n");
        }
    }

    void resyncNtpTime()
    {
        State::system.ntpInitialized = false;
        if (DEBUG)
            Serial.println("[Network] NTP Resync requested.");
    }

    void sendLineTestMessage()
    {
        State::lcd.clear();
        Display::printLcdLine(0, "Sending LINE Test...");
        sendLineNotification("これは雷センサーからのテスト通知です。");
        Display::printLcdLine(1, "Sent!");
    }

    void getSwitchBotDeviceList()
    {
        State::lcd.clear();
        Display::printLcdLine(0, "Getting Device IDs");
        Display::printLcdLine(1, "Check Serial Mon.");
        if (WiFi.status() != WL_CONNECTED)
        {
            Display::printLcdLine(2, "WiFi Disconnected");
            return;
        }
        WiFiClientSecure secureClient;
        secureClient.setInsecure();
        HTTPClient http;
        if (http.begin(secureClient, "https://api.switch-bot.com/v1.1/devices"))
        {
            addAuthHeaders(http);
            if (http.GET() == HTTP_CODE_OK)
            {
                if (DEBUG)
                {
                    Serial.println("\n--- SwitchBot Device List ---");
                    Serial.println(http.getString());
                }
                Display::printLcdLine(2, "Success!");
            }
            else
            {
                Display::printLcdLine(2, "Request Failed!");
            }
            http.end();
        }
        else
        {
            Display::printLcdLine(2, "Begin Failed!");
        }
    }

    void showIpAddressAndHold()
    {
        State::lcd.clear();
        Display::printLcdLine(0, "IP Address:");
        if (WiFi.status() == WL_CONNECTED)
        {
            Display::printLcdLine(1, WiFi.localIP().toString().c_str());
        }
        else
        {
            Display::printLcdLine(1, "Disconnected");
        }
        delay(IP_DISPLAY_DURATION_MS);
    }

    void manualLogToSheet()
    {
        State::lcd.clear();
        Display::printLcdLine(0, "Logging to Sheet...");
        String params = createTempHumParams();
        if (params != "")
        {
            logDataToGoogleSheet(params);
            Display::printLcdLine(1, "Logged!");
        }
        else
        {
            Display::printLcdLine(1, "No sensor data!");
        }
    }

    void update()
    {
        // WiFiに接続されているかチェック
        if (WiFi.status() == WL_CONNECTED)
        {
            // NTP同期がまだなら、バックグラウンドで同期を試みる
            if (!State::system.ntpInitialized)
            {
                handleNtpSync();
            }
            // Webサーバーのクライアント処理
            handleServerClient();
        }
        // WiFi接続を監視し、切断されていたら再接続を試みる
        checkWiFiConnection();
    }

    void init()
    {
        WiFi.mode(WIFI_STA);

        if (USE_STATIC_IP)
        {
            if (DEBUG)
                Serial.println("[Network] Configuring static IP address...");
            IPAddress local_IP(STATIC_IP_BYTES);
            IPAddress gateway(GATEWAY_BYTES);
            IPAddress subnet(SUBNET_BYTES);
            IPAddress primaryDNS(PRIMARY_DNS_BYTES);
            IPAddress secondaryDNS(SECONDARY_DNS_BYTES);

            if (DEBUG)
            {
                Serial.print("  - IP Address: ");
                Serial.println(local_IP);
                Serial.print("  - Gateway:    ");
                Serial.println(gateway);
                Serial.print("  - Subnet Mask:");
                Serial.println(subnet);
                Serial.print("  - DNS1:       ");
                Serial.println(primaryDNS);
                Serial.print("  - DNS2:       ");
                Serial.println(secondaryDNS);
            }

            WiFi.config(local_IP, gateway, subnet);
            WiFi.setDNS(primaryDNS, secondaryDNS);

            if (DEBUG)
                Serial.println("[Network] Static IP configuration applied.");
        }

        // ★setup()内でWiFi接続を完了させる
        if (connectToWiFi(true))
        {
            initOTA();
        }
    }
}

//================================================================
// センサー管理
//================================================================
namespace Sensors
{
    void handleLightningInterrupt()
    {
        State::lightningInterruptFlag = true;
    }

    void addHistoryRecord(String type, int distance, const char *timestamp)
    {
        strcpy(State::history.records[State::history.index].timestamp, timestamp);
        State::history.records[State::history.index].type = type;
        State::history.records[State::history.index].distance = distance;
        State::history.index = (State::history.index + 1) % HISTORY_SIZE;
        if (State::history.count < HISTORY_SIZE)
        {
            State::history.count++;
        }
    }

    void handleLightning()
    {
        State::lightningInterruptFlag = false;

        delay(5);
        int intVal = State::lightning.readInterruptReg();
        if (DEBUG)
            Serial.printf("[Sensor] Interrupt Register: 0x%02X\n", intVal);

        char timestamp[20];
        if (intVal == 0x01 || intVal == 0x08)
        {
            time_t now = time(nullptr);
            if (now > 100000)
            {
                strftime(timestamp, sizeof(timestamp), "%m/%d %H:%M", localtime(&now));
                strcpy(State::sensors.lastEventTime, timestamp);
            }
            else
            {
                strcpy(timestamp, "Time N/A");
                strcpy(State::sensors.lastEventTime, timestamp);
            }
        }

        if (intVal == 0x01)
        {
            if (DEBUG)
                Serial.println("[Sensor] Noise detected.");
            State::sensors.lastEventType = "Noise";
            if (State::system.as3935_initialized && State::system.currentNoiseLevel < 7)
            {
                State::system.currentNoiseLevel++;
                State::lightning.setNoiseLevel(State::system.currentNoiseLevel);
            }
            addHistoryRecord("Noise", 0, timestamp);
        }
        else if (intVal == 0x08)
        {
            if (DEBUG)
                Serial.println("[Sensor] Lightning detected.");
            Utils::blinkLED("yellow", 2, 80);
            int distance = State::lightning.distanceToStorm();
            State::sensors.lastLightningDistance = distance;
            State::sensors.lastEventType = "Lightning";

            addHistoryRecord("Lightning", distance, timestamp);

            State::system.pendingLightningLog = true;
        }
        State::system.forceMainScreenRedraw = true;
    }

    // 温湿度センサーの値を更新する
    void updateDht()
    {
        if (State::system.dht20_initialized)
        {
            float temp_raw = State::dht20.getTemperature();
            float hum_raw = State::dht20.getHumidity();

            if (!isnan(temp_raw) && !isnan(hum_raw))
            {
                State::sensors.temperature = round(temp_raw * 10.0) / 10.0;

                if (hum_raw >= 0 && hum_raw <= 1.0)
                {
                    hum_raw *= 100.0;
                }
                State::sensors.humidity = round(hum_raw * 10.0) / 10.0;
                if (DEBUG)
                    Serial.printf("[Sensor] DHT20 Read OK: T=%.1fC, H=%.1f%%\n", State::sensors.temperature, State::sensors.humidity);
            }
            else
            {
                if (DEBUG)
                    Serial.println("[Sensor] Error: Failed to read from DHT sensor!");
            }
        }
    }

    void update()
    {
        // 雷割り込みフラグの処理
        if (State::lightningInterruptFlag)
        {
            handleLightning();
        }
    }

    // センサーのキャリブレーションを実行する関数
    void calibrateSensor()
    {
        State::lcd.clear();
        Display::printLcdLine(0, "Calibrating...");
        if (State::system.as3935_initialized)
        {
            if (State::lightning.calibrateOsc())
            {
                Display::printLcdLine(1, "Calibration OK!");
                if (DEBUG)
                    Serial.println("[Sensor] Antenna tuning successful.");
            }
            else
            {
                Display::printLcdLine(1, "Calibration FAILED!");
                if (DEBUG)
                    Serial.println("[Sensor] Antenna tuning failed.");
            }
        }
        else
        {
            Display::printLcdLine(1, "AS3935 not ready.");
        }
    }

    void init()
    {
        if (DEBUG)
            Serial.println("Initializing Sensors...");

        // --- DHT20 Initialization ---
        if (DEBUG)
            Serial.print("  - Scanning for DHT20 at I2C address 0x38... ");
        Wire.beginTransmission(0x38); // DHT20のI2Cアドレス
        byte dht_error = Wire.endTransmission();

        if (dht_error == 0)
        {
            if (DEBUG)
                Serial.println("Device found.");
            if (DEBUG)
                Serial.print("    - Calling dht20.begin()... ");
            if (State::dht20.begin() == 0)
            {
                if (DEBUG)
                    Serial.println("OK.");
                State::system.dht20_initialized = true;
            }
            else
            {
                if (DEBUG)
                    Serial.println("FAILED.");
                Display::printLcdLine(1, "DHT20 Init FAILED");
            }
        }
        else
        {
            if (DEBUG)
            {
                Serial.print("Device NOT found (error code: ");
                Serial.print(dht_error);
                Serial.println(").");
            }
            Display::printLcdLine(1, "DHT20 NOT FOUND!");
        }

        delay(100);

        // --- AS3935 Initialization ---
        if (DEBUG)
            Serial.print("  - Scanning for AS3935 at I2C address 0x03... ");
        Wire.beginTransmission(Pins::AS3935_ADDR);
        byte as3935_error = Wire.endTransmission();

        if (as3935_error == 0)
        {
            if (DEBUG)
                Serial.println("Device found.");
            if (DEBUG)
                Serial.print("    - Calling lightning.begin()... ");
            if (State::lightning.begin(Wire))
            {
                if (DEBUG)
                    Serial.println("OK.");
                State::system.as3935_initialized = true;
                if (DEBUG)
                    Serial.println("    - Configuring AS3935 sensor...");
                State::lightning.resetSettings();
                State::lightning.setIndoorOutdoor(OUTDOOR);
                State::lightning.setNoiseLevel(INITIAL_NOISE_LEVEL);
                State::system.currentNoiseLevel = INITIAL_NOISE_LEVEL;
                State::lightning.watchdogThreshold(LIGHTNING_WATCHDOG_THRESHOLD);
                State::lightning.spikeRejection(LIGHTNING_SPIKE_REJECTION);
                if (DEBUG)
                    Serial.println("    - Configuration complete.");
            }
            else
            {
                if (DEBUG)
                    Serial.println("FAILED.");
                Display::printLcdLine(2, "AS3935 Init FAILED");
            }
        }
        else
        {
            if (DEBUG)
            {
                Serial.print("Device NOT found (error code: ");
                Serial.print(as3935_error);
                Serial.println(").");
            }
            Display::printLcdLine(2, "AS3935 NOT FOUND!");
        }

        attachInterrupt(digitalPinToInterrupt(Pins::LIGHTNING_IRQ), handleLightningInterrupt, RISING);
        if (DEBUG)
            Serial.println("Sensor Initialization Complete.");
    }
}

//================================================================
// 入力処理
//================================================================
namespace Input
{
    uint8_t buttonState = HIGH;
    uint8_t lastButtonState = HIGH;
    unsigned long lastDebounceTime = 0;
    unsigned long debounceDelay = 50;
    unsigned long buttonPressTime = 0;
    bool longPressTriggered = false;

    void handleButton()
    {
        int reading = digitalRead(Pins::BUTTON);

        if (reading != lastButtonState)
        {
            lastDebounceTime = millis();
        }

        if ((millis() - lastDebounceTime) > debounceDelay)
        {
            if (reading != buttonState)
            {
                buttonState = reading;

                if (buttonState == LOW)
                {
                    buttonPressTime = millis();
                    longPressTriggered = false;
                }
                else
                {
                    if (!longPressTriggered)
                    {
                        State::timers.lastActivity = millis();
                        State::system.needsRedraw = true;
                        if (State::menu.currentMode == State::MAIN_DISPLAY)
                        {
                            if (!State::system.backlightAlwaysOn)
                            {
                                digitalWrite(Pins::LCD_BACKLIGHT, HIGH);
                                State::timers.backlightOn = millis();
                            }
                        }
                        else if (State::menu.currentMode == State::HISTORY || State::menu.currentMode == State::SENSOR_DIAGNOSTICS)
                        {
                            // これらの画面では短押しは無効
                        }
                        else
                        {
                            int count = 0, currentSelection = 0;
                            Menu::getCurrentMenu(count, currentSelection);
                            if (count > 0)
                            {
                                if (State::menu.currentMode == State::DEVICE_CONTROL)
                                {
                                    State::menu.commandSelection = (State::menu.commandSelection + 1) % count;
                                }
                                else
                                {
                                    State::menu.menuSelection = (State::menu.menuSelection + 1) % count;
                                }
                            }
                        }
                    }
                }
            }
        }

        if (buttonState == LOW && !longPressTriggered && (millis() - buttonPressTime > LONG_PRESS_DURATION_MS))
        {
            longPressTriggered = true;
            State::timers.lastActivity = millis();
            State::system.needsRedraw = true;
            switch (State::menu.currentMode)
            {
            case State::MAIN_DISPLAY:
                Menu::changeMode(State::MENU);
                break;
            case State::HISTORY:
            case State::ULTRASONIC_MONITOR:
            case State::SENSOR_DIAGNOSTICS:
                Menu::changeMode(State::MAIN_DISPLAY);
                break;
            default:
            {
                int count = 0, currentSelection = 0;
                const Menu::MenuItem *menu = Menu::getCurrentMenu(count, currentSelection);
                if (menu && currentSelection < count)
                {
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
// メイン制御
//================================================================
namespace Menu
{
    void changeMode(State::Mode newMode)
    {
        State::menu.currentMode = newMode;
        State::menu.menuSelection = 0;
        State::menu.commandSelection = 0;
        State::system.needsRedraw = true;
        State::timers.lastActivity = millis();
        if (newMode == State::MAIN_DISPLAY)
        {
            State::system.forceMainScreenRedraw = true;
        }
    }

    void performMenuAction(void (*action)(), bool returnToMain)
    {
        if (action)
        {
            action();
            delay(1500); // 結果表示のために少し待機
        }
        if (returnToMain)
        {
            changeMode(State::MAIN_DISPLAY);
        }
        else
        {
            State::system.needsRedraw = true;
        }
    }

    void checkInactivity()
    {
        if (State::menu.currentMode == State::MAIN_DISPLAY || State::menu.currentMode == State::ULTRASONIC_MONITOR || State::menu.currentMode == State::SENSOR_DIAGNOSTICS)
        {
            return;
        }
        if (millis() - State::timers.lastActivity > INACTIVITY_TIMEOUT_MS)
        {
            changeMode(State::MAIN_DISPLAY);
        }
    }
}

//================================================================
// 定期実行タスク
//================================================================
void handlePeriodicTasks()
{
    static int lastSyncMinute = -1;
    static int lastLogMinute = -1;

    if (!State::system.ntpInitialized || time(nullptr) < 100000)
    {
        return;
    }

    time_t now = time(nullptr);
    struct tm *timeinfo = localtime(&now);
    int currentMinute = timeinfo->tm_min;

    if (!State::system.initialLogSent)
    {
        if (State::sensors.temperature > -100)
        {
            String params = Network::createTempHumParams();
            if (params != "")
            {
                if (DEBUG)
                    Serial.println("[Task] Initial log of Temp/Hum data.");
                Network::logDataToGoogleSheet(params);
                State::system.initialLogSent = true;
                lastLogMinute = currentMinute;
            }
        }
    }

    if (currentMinute % 4 == 0)
    {
        if (currentMinute != lastSyncMinute)
        {
            lastSyncMinute = currentMinute;
            State::system.isAutoResync = true;
            Network::resyncNtpTime();
        }
    }

    // 10分ごとの記録
    if (currentMinute % 10 == 0)
    {
        if (currentMinute != lastLogMinute)
        {
            lastLogMinute = currentMinute;
            String params = Network::createTempHumParams();
            if (params != "")
            {
                if (DEBUG)
                    Serial.println("[Task] 10-minute interval: Logging Temp/Hum data.");
                Network::logDataToGoogleSheet(params);
            }
        }
    }
}

void handlePendingNetworkTasks()
{
    if (State::system.pendingLightningLog && WiFi.status() == WL_CONNECTED)
    {
        if (DEBUG)
            Serial.println("[Task] Pending lightning event found. Sending notifications...");
        int distance = State::sensors.lastLightningDistance;
        char msg[60];
        snprintf(msg, sizeof(msg), "雷を検知しました！\n距離: 約%dkm", distance);
        Network::sendLineNotification(msg);
        String params = "?sheet=" + Network::urlEncode("雷の受信履歴") + "&dist=" + String(distance);
        Network::logDataToGoogleSheet(params);
        State::system.pendingLightningLog = false;
    }
}

// ★追加: 4秒サイクルのタスクを実行する関数
void handleFourSecondCycle()
{
    static unsigned long lastCycleTick = 0;
    static int cycleState = 0; // 0, 1, 2, 3 の4つの状態（1秒ごと）

    if (millis() - lastCycleTick >= 1000)
    { // 1秒ごとに実行
        lastCycleTick = millis();

        switch (cycleState)
        {
        case 0: // 1秒目
            // このタイミングで温湿度センサーを読み取る
            if (DEBUG)
                Serial.println("[Cycle] State 0: Reading Temp/Hum.");
            Sensors::updateDht();
            break;
        case 1: // 2秒目
            // アイドル状態 (旧WoLポーリング)
            if (DEBUG)
                Serial.println("[Cycle] State 1: Idle.");
            break;
        case 2: // 3秒目
            // このタイミングで再度、温湿度センサーを読み取る
            if (DEBUG)
                Serial.println("[Cycle] State 2: Reading Temp/Hum.");
            Sensors::updateDht();
            break;
        case 3: // 4秒目
            // アイドル状態
            if (DEBUG)
                Serial.println("[Cycle] State 3: Idle.");
            break;
        }

        cycleState = (cycleState + 1) % 4; // 次の状態へ遷移
    }
}

//================================================================
// Setup & Loop
//================================================================
void setup()
{
    Serial.begin(115200);
    delay(2000);
    if (DEBUG)
        Serial.println("--- System Booting ---");

    if (DEBUG)
        Serial.println("Initializing Pins...");
    pinMode(Pins::LCD_BACKLIGHT, OUTPUT);
    pinMode(Pins::LED_R, OUTPUT);
    pinMode(Pins::LED_G, OUTPUT);
    pinMode(Pins::LED_B, OUTPUT);
    pinMode(Pins::BUTTON, INPUT_PULLUP);
    pinMode(Pins::LIGHTNING_IRQ, INPUT);
    if (DEBUG)
        Serial.println("Initializing I2C...");
    Wire.setSDA(Pins::I2C_SDA);
    Wire.setSCL(Pins::I2C_SCL);
    Wire.begin();
    if (DEBUG)
        Serial.println("Pin & I2C Initialization Complete.");

    digitalWrite(Pins::LCD_BACKLIGHT, HIGH);
    if (DEBUG)
        Serial.println("Initializing Display...");
    Display::init();
    if (DEBUG)
        Serial.println("Display Initialization Complete.");

    Utils::setRGB(255, 255, 255);
    delay(1000);
    Utils::setRGB(0, 0, 0);
    if (!State::system.backlightAlwaysOn)
    {
        digitalWrite(Pins::LCD_BACKLIGHT, LOW);
    }

    Sensors::init();

    if (DEBUG)
        Serial.println("Initializing Network...");
    Network::init();
    if (DEBUG)
        Serial.println("Network Initialization Complete.");

    Menu::changeMode(State::MAIN_DISPLAY);
    if (DEBUG)
        Serial.println("--- Boot Complete ---");
}

void loop()
{
    // --- 毎ループ実行する必要がある処理 ---
    ArduinoOTA.handle();     // OTA（無線書き込み）の待機
    Input::handleButton();   // ボタン入力の処理
    Network::update();       // WiFi接続と時刻同期をバックグラウンドで処理
    Sensors::update();       // 雷割り込みフラグのチェック
    Display::update();       // 画面描画
    Menu::checkInactivity(); // メニューのタイムアウトチェック

    // --- 4秒サイクルのタスク管理 ---
    handleFourSecondCycle(); // 温湿度読み取りなどを管理

    // --- 定期実行・不定期実行のタスク ---
    handlePeriodicTasks();       // 10分ごとのデータ記録など
    handlePendingNetworkTasks(); // 雷検知時の通知など

    // --- その他の常時処理 ---
    if (State::system.illuminationOn)
    {
        Utils::handleSmoothIllumination();
    }

    if (State::menu.currentMode == State::MAIN_DISPLAY && !State::system.backlightAlwaysOn &&
        State::timers.backlightOn > 0 && (millis() - State::timers.backlightOn > BACKLIGHT_DURATION_MS))
    {
        digitalWrite(Pins::LCD_BACKLIGHT, LOW);
        State::timers.backlightOn = 0;
    }
}
