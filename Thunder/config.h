#ifndef CONFIG_H
#define CONFIG_H

//================================================================
// ★★★ 設定ファイル (このファイルに機密情報を入力してください) ★★★
//================================================================

// --- デバッグ設定 ---
// trueにすると、動作状況がシリアルモニタに詳細に出力されます。
const bool DEBUG = true;

// --- WiFi設定 (優先順位順に3つまで設定) ---
struct WiFiCredential {
  const char* ssid;
  const char* password;
};

const WiFiCredential wifiCredentials[] = {
  {"-----", "-----"}, // 優先度1
  {"-----", "-----"}, // 優先度2
  {"-----", "-----"}  // 優先度3
};
const int numWifiCredentials = sizeof(wifiCredentials) / sizeof(wifiCredentials[0]);

const char* NTP_SERVER = "ntp.nict.jp";

// --- PCのMACアドレス (Wake on LAN用) ---
// ★★★ MACアドレスを "00:1A:2B:3C:4D:5E" の形式で直接入力してください ★★★
const char* MAC_DESKTOP = "00:00:00:00:00:00";
const char* MAC_SERVER  = "00:00:00:00:00:00";

// --- 静的IPアドレス設定 (固定IPを使用する場合) ---
// trueにするとIPアドレスを固定します。falseにするとDHCPから自動取得します。
const bool USE_STATIC_IP = false;
const byte STATIC_IP_BYTES[] = {0, 0, 0, 0};
const byte GATEWAY_BYTES[]   = {0, 0, 0, 0};
const byte SUBNET_BYTES[]    = {0, 0, 0, 0};
const byte PRIMARY_DNS_BYTES[]     = {0, 0, 0, 0};
const byte SECONDARY_DNS_BYTES[]     = {0, 0, 0, 0};

// --- Google Apps Script ---
const char* GAS_URL = "-----";
const char* GAS_URL_WOL = "-----"; // LINE経由のWoL指示を受け取るGASのURL

// --- SwitchBot API設定 ---
const char* SWITCHBOT_TOKEN = "-----";
const char* SWITCHBOT_SECRET = "-----";
const char* DEVICE_ID_LIGHT = "-----";
const char* DEVICE_ID_TV = "-----";
const char* DEVICE_ID_AC = "-----";
const char* DEVICE_ID_FAN = "-----";
const char* DEVICE_ID_SPEAKER = "-----";
const char* DEVICE_ID_OTHERS = "-----";

// --- LINE Messaging API 設定 ---
const char* LINE_CHANNEL_ACCESS_TOKEN = "-----";
const char* LINE_USER_ID = "-----";


#endif // CONFIG_H

