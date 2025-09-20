# 多機能雷センサー for Raspberry Pi Pico 2 W

## 概要

このプロジェクトは、Raspberry Pi Pico W を活用し、高機能な雷・環境モニターを製作するものです。

雷の検知・距離測定、温湿度のモニタリング、各種Webサービスとの連携機能を備えており、本体のLCDとボタンだけでスマートホーム機器の操作も可能です。

## インストール

本プロジェクトを動作させるには、ハードウェアの組み立てとソフトウェアのセットアップが必要です。

1. 必要なもの (依存関係)
■ ハードウェア

Raspberry Pi Pico W

雷センサーモジュール: SparkFun AS3935 Lightning Detector

温湿度センサー: DFRobot DHT20

LCDディスプレイ: I2C接続 20x4 キャラクタ液晶

タクトスイッチ (プッシュボタン) x 1

フルカラーLED、抵抗、ブレッドボード、ジャンパーワイヤー

■ ソフトウェア & ライブラリ

Arduino IDE

Arduino ライブラリ (ライブラリマネージャからインストール)
WiFi.h
WiFiServer.h
WiFiClientSecure.h
HTTPClient.h
Wire.h
LiquidCrystal.h
time.h
stdlib.h
WiFiNTP.h
DFRobot_DHT20.h
SparkFun_AS3935.h
ArduinoJson.h
WiFiUdp.h
LittleFS.h
math.h
ArduinoOTA.h

2. セットアップ
■ ハードウェアの配線
スケッチ内の namespace Pins の定義に従って、各コンポーネントを配線します。I2Cデバイス (AS3935, DHT20, LCD) はすべて同じSDA/SCLピンに接続します。

コンポーネント

Pico W Pin (GP)

I2C SDA

GP0

I2C SCL

GP1

AS3935 IRQ

GP16

ボタン

GP15

LCD バックライト

GP14

LED (R, G, B)

GP28, GP27, GP26

■ Google Apps Script の設定

提供されているGoogle Apps Scriptのコードを、新しいGoogleスプレッドシートのスクリプトエディタに貼り付けます。

スクリプトを「ウェブアプリ」としてデプロイし、アクセスできるユーザーを「全員」に設定します。

デプロイ後に表示される ウェブアプリURL をコピーします。

■ Arduinoコードの設定

pico_w_lightning_sensor_v5.6.ino を開きます。

★★★ 設定項目 ★★★ セクションを、ご自身の環境に合わせて編集します。

GAS_URL: 上記でコピーしたウェブアプリURL

wifiCredentials: Wi-FiのSSIDとパスワード

各種APIキー、デバイストークン、MACアドレスなど
## 使い方

1. プログラムの書き込みと実行
Arduino IDEでボードを 「Raspberry Pi Pico 2 W」 に設定します。

PCとPico WをUSBケーブルで接続し、シリアルポートを選択してプログラムを書き込みます。

書き込みが完了すると、デバイスが自動的に起動し、Wi-Fiへの接続を開始します。

2. 操作方法
本体のボタン1つで全ての操作を行います。

短く押す: メニュー項目を移動します。

長く押す: 項目を決定・実行、または前の画面に戻ります。

メイン画面には日時、温湿度、最新の雷情報が表示されます。長押しでメニュー画面にアクセスし、SwitchBot機器の操作や各種ネットワーク機能の実行が可能です。

## ライセンス

このプロジェクトは、[ライセンス名] ライセンスの下でライセンスされています。
