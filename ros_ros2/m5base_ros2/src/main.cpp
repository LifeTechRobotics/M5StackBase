#include <M5Stack.h>    // CORE2の場合は <M5Core2.h>
#include <ros2arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include "nvs_flash.h"

// ボタンで速度を操作する場合には定義する
#define BTN_TEST_MODE

// WiFi通信設定
#define SERVER_NUM_CLIENT         1           // 接続クライアント台数
#define SERVER_PORT               10200       // 通信用ポート（＊）（適宜変更・・・Windowsソフトも変更）

// 通信タイムアウト
#define COMM_TIMEOUT              1000        // 1000mm secounds

// Prefarence定義
#define PREF_ROOT                 "bmboroot"  // Rot name
#define PREF_IP1                  "ip1"       // IP Address 1
#define PREF_IP2                  "ip2"       // IP Address 2
#define PREF_IP3                  "ip3"       // IP Address 3
#define PREF_IP4                  "ip4"       // IP Address 4
#define PREF_GW1                  "gw1"       // Gateway Address 1
#define PREF_GW2                  "gw2"       // Gateway Address 2
#define PREF_GW3                  "gw3"       // Gateway Address 3
#define PREF_GW4                  "gw4"       // Gateway Address 4

// PWMの分解能
#define PWM_RESOLUTION_LED        8
#define PWM_RES_MAX_LED           ((1<<PWM_RESOLUTION_LED) -1)


// 変数定義
Preferences preferences;          // ESP32の保存情報
int gMotorRSpd = 0;
int gMotorLSpd = 0;
bool gPushOnA = false;
bool gPushOnB = false;
bool gPushOnC = false;

WiFiServer gServer(SERVER_PORT);                // WiFiの自サーバー変数
WiFiClient gClient;                             // 接続クライアント用
bool gAPConnectFlag = false;                    // APへの接続状態
unsigned long gCommTime = 0;                    // 通信タイムアウト用

// LED
int gPWMLed1 = 0;                 // LED1 PWM
int gPWMLed2 = 0;                 // LED2 PWM

// DEMO MODE
bool gDemoMode = false;           // デモモード


//
// motor driver
void motorDriverSend()
{
  // 速度設定の確認
  if(gMotorRSpd > 63){
    gMotorRSpd = 63;
  }
  if(gMotorRSpd < -63){
    gMotorRSpd = -63;
  }
  if(gMotorLSpd > 63){
    gMotorLSpd = 63;
  }
  if(gMotorLSpd < -63){
    gMotorLSpd = -63;
  }
  int spdr = 0;
  int spdl = 0;
  if(gMotorRSpd >= 0 && gMotorRSpd <= 63){
    spdr = gMotorRSpd;
  }
  else {
    spdr = 64 - gMotorRSpd;
  }
  if(gMotorLSpd >= 0 && gMotorLSpd <= 63){
    spdl = 128 + gMotorLSpd;
  }
  else {
    spdl = 192 - gMotorLSpd;
  }

  // 設定速度の表示
  if(!gDemoMode){
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.print("IP address : ");
    M5.Lcd.println(WiFi.localIP());
  }
  M5.Lcd.setCursor(0, 32);
  M5.Lcd.printf("%3d/%3d     ", spdr, spdl);

  // モータドライバへ送信
  Serial1.write(spdr);
  Serial1.write(spdl);
}


//
// ROS2 node
void subscribeString(std_msgs::String* msg, void* arg)
{
  (void)(arg);

  Serial.println("Received: " + String(msg->data));
  int sr = 0, sl = 0;
  sscanf(msg->data, "%d,%d", &sr, &sl);
  gMotorRSpd = sr;
  gMotorLSpd = sl;

  // モータードライバへ送信
  motorDriverSend();
}

class StringSub: public ros2::Node
{
public:
  StringSub()
  : Node("ros2_m5stack_subscriber_node")
  {
    this->createSubscriber<std_msgs::String>("m5stack_listener", (ros2::CallbackFunc)subscribeString, nullptr);
  }
};


//
// WiFi Information Erase
void WiFiInfoErase() {
  Serial.println("--- WiFi Information Erase ---");
  nvs_flash_erase();
  nvs_flash_init();

  preferences.begin(PREF_ROOT, false);
  preferences.putInt(PREF_IP1, -1);
  preferences.putInt(PREF_IP2, -1);
  preferences.putInt(PREF_IP3, -1);
  preferences.putInt(PREF_IP4, -1);

  preferences.putInt(PREF_GW1, -1);
  preferences.putInt(PREF_GW2, -1);
  preferences.putInt(PREF_GW3, -1);
  preferences.putInt(PREF_GW4, -1);
  preferences.end();
}

//
// split関数
int split(String data, char delimiter, String *dst, int dsize){
  int index = 0;
  int datalength = data.length();

  for(int i = 0; i < datalength; i++) {
    char tmp = data.charAt(i);
    if(tmp == delimiter ) {
      index++;
      if(index > (dsize - 1)) {
        return -1;
      }
    }
    else dst[index] += tmp;
  }
  return (index + 1);
}

//
// setuo
void setup() {
  // put your setup code here, to run once:
  char wifiSsid[37] = {};
  char wifiKey[66] = {};
  bool wifiErase = false;

  M5.begin(true,true,true,false);             // 本体初期化（LCD, SD, Serial, I2C）※I2Cのみ無効
  Serial.begin(115200);                       // シリアル通信初期化(USBと共用、初期値は RX=G3, TX=G1)
  Serial1.begin(115200, SERIAL_8N1, 16, 17);  // シリアル通信1初期化(RX, TX) ※BASIC R2/T2

  // Motor Stop
  Serial1.write(0);
  Serial1.write(128);

  // 起動時にボタンCを押していたらWiFiをリセットしてコンフィグモードにはいる
  M5.update();  //本体のボタン状態更新
  M5.Lcd.clearDisplay(BLACK);
  delay(300);
  M5.Lcd.clearDisplay(BLUE);
  delay(300);
  M5.Lcd.clearDisplay(BLACK);
  delay(300);
  M5.Lcd.clearDisplay(GREEN);
  delay(300);
  M5.Lcd.clearDisplay(BLACK);
  delay(300);
  M5.Lcd.clearDisplay(RED);
  delay(300);
  M5.Lcd.clearDisplay(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0, 0);

  // WiFi処理
  if(M5.BtnC.isPressed()){
    WiFiInfoErase();
    wifiErase = true;
  }
  else {
    // WiFi設定の既存設定情報
    M5.Lcd.println("--- WiFi Setup Information ---");
    M5.Lcd.print("MAC  : ");
    M5.Lcd.println(WiFi.macAddress());
    preferences.begin("nvs.net80211", true);
    preferences.getBytes("sta.ssid", wifiSsid, sizeof(wifiSsid));
    preferences.getBytes("sta.pswd", wifiKey, sizeof(wifiKey));
    preferences.end();
    M5.Lcd.printf("SSID : %s\n", &wifiSsid[4]);
    M5.Lcd.printf("PASS : %s\n", wifiKey);

    // 表示確認用に1秒待つ
    delay(1000);

    // Aボタンを押している間は進まない
    while(1) {
      M5.update();  // ボタンの更新
      if(M5.BtnA.isReleased()){
        break;
      }
      delay(10);
    }
  }

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_STA);
  if(!wifiErase){
    // WiFi接続処理
    // IP & Gateway
    preferences.begin(PREF_ROOT, true);
    int ip1 = preferences.getInt(PREF_IP1, -1);
    int ip2 = preferences.getInt(PREF_IP2, -1);
    int ip3 = preferences.getInt(PREF_IP3, -1);
    int ip4 = preferences.getInt(PREF_IP4, -1);
    int gw1 = preferences.getInt(PREF_GW1, -1);
    int gw2 = preferences.getInt(PREF_GW2, -1);
    int gw3 = preferences.getInt(PREF_GW3, -1);
    int gw4 = preferences.getInt(PREF_GW4, -1);
    preferences.end();

    if(ip1 != -1 && ip2 != -1 && ip3 != -1 && ip4 != -1){
      if(gw1 != -1 && gw2 != -1 && gw3 != -1 && gw4 != -1){
        IPAddress ip(ip1, ip2, ip3, ip4);
        IPAddress gw(gw1, gw2, gw3, gw4);
        IPAddress sb(255,255,255,0);
        WiFi.config(ip, gw, sb);
      }
    }
  }
  WiFi.begin();

  // サーバー開始
  gServer.begin();
  gAPConnectFlag = false;

  // DEMO MODE
  M5.Lcd.clearDisplay(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print("TEST MODE PUSH [A]");
  delay(2000);
  M5.update();
  if(M5.BtnA.isPressed()){
    gDemoMode = true;
  }
  M5.Lcd.clearDisplay(BLACK);
}

//
// loop
void loop() {
  // put your main code here, to run repeatedly:
  static StringSub StringNode;  // ROS node
  unsigned long tm = millis();
  M5.update();  //本体のボタン状態更新

  M5.Lcd.setTextColor(WHITE, BLACK);
  if(gDemoMode){
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("TEST MODE");
    M5.Lcd.println("MOTOR +     [A]");
    M5.Lcd.println("MOTOR -     [B]");
    M5.Lcd.println("MOTOR STOP  [C]");

    if(M5.BtnA.isPressed()){
      gPushOnA = true;
    }
    if(M5.BtnB.isPressed()){
      gPushOnB = true;
    }
    if(M5.BtnC.isPressed()){
      gPushOnC = true;
    }

    if(gPushOnA){
      if(M5.BtnA.isReleased()){
        gMotorRSpd ++;
        gMotorLSpd ++;
        gPushOnA = false;
      }
    }
    if(gPushOnB){
      if(M5.BtnB.isReleased()){
        gMotorRSpd --;
        gMotorLSpd --;
        gPushOnB = false;
      }
    }
    if(gPushOnC){
      if(M5.BtnC.isReleased()){
        gMotorRSpd = 0;
        gMotorLSpd = 0;
        gPushOnC = false;
      }
    }
  }



  if(!gDemoMode && (WiFi.status() == WL_CONNECTED)) {
    // 初回APに接続時のみ
    if(!gAPConnectFlag) {
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.println("WiFi connected !!");
      M5.Lcd.print("IP address : ");
      M5.Lcd.println(WiFi.localIP());
      gAPConnectFlag = true;

      char ipbuf[32];
      String ips = WiFi.localIP().toString();
      ips.toCharArray(ipbuf, sizeof(ipbuf));
      ros2::init(&gClient, ipbuf, SERVER_PORT);

      ros2::spin(&StringNode);
    }
    else {
      // APに接続できていない
      // モータ停止
      gMotorRSpd = 0;
      gMotorLSpd = 0;
    }
  }
  else if(!gDemoMode){
    // WiFi APに接続できていない場合
    gAPConnectFlag = false;
    M5.Lcd.clearDisplay(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("WiFi AP Not Connect !");

    // 画面モードかスマートモードか
    int setupmode = 0;
    int cnt = 0;
    M5.Lcd.println("");
    M5.Lcd.println("WiFi Setup !!");
    M5.Lcd.println("");
    M5.Lcd.println("Setup SmartConfig [A]");
    M5.Lcd.println("Setup IP/GATEWAY  [B]");
    M5.Lcd.println("Exit & Reset      [C]");
    M5.Lcd.println("");
    while(1){
      M5.Lcd.setCursor(0, 56);
      if(cnt == 0 || cnt == 4){
        M5.Lcd.print("|");
      }
      else if(cnt == 1 || cnt == 5){
        M5.Lcd.print("/");
      }
      else if(cnt == 2 || cnt == 6){
        M5.Lcd.print("-");
      }
      else if(cnt == 3 || cnt == 7){
        M5.Lcd.print("\\");
      }
      cnt ++;
      cnt %= 4;

      M5.update();
      if(M5.BtnA.isPressed()){
        setupmode = 1;
        break;
      }
      if(M5.BtnB.isPressed()){
        setupmode = 2;
        break;
      }
      if(M5.BtnC.isPressed()){
        ESP.restart();
      }
      delay(200);
    }

    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.clearDisplay(BLACK);
    M5.Lcd.setCursor(0, 0);
    if(setupmode == 1){
      WiFi.mode(WIFI_STA);
      WiFi.beginSmartConfig();

      M5.Lcd.println("Waiting for SmartConfig");
      M5.Lcd.println("Exit & Reset      [C]");
      cnt = 0;
      while (!WiFi.smartConfigDone()) {
        delay(200);
        M5.update();
        if(M5.BtnC.isPressed()){
          ESP.restart();
        }

        M5.Lcd.setCursor(0, 16);
        if(cnt == 0 || cnt == 4){
          M5.Lcd.print("|");
        }
        else if(cnt == 1 || cnt == 5){
          M5.Lcd.print("/");
        }
        else if(cnt == 2 || cnt == 6){
          M5.Lcd.print("-");
        }
        else if(cnt == 3 || cnt == 7){
          M5.Lcd.print("\\");
        }
        cnt ++;
        cnt %= 4;
      }
    }
    else if(setupmode == 2){
      int curs_x = 0;
      int curs_y = 0;
      int aip1[3] = {0, 0, 0};
      int aip2[3] = {0, 0, 0};
      int aip3[3] = {0, 0, 0};
      int aip4[3] = {0, 0, 0};
      int agt1[3] = {0, 0, 0};
      int agt2[3] = {0, 0, 0};
      int agt3[3] = {0, 0, 0};
      int agt4[3] = {0, 0, 0};
      int *pcur = NULL;

      M5.Lcd.println("Setup WiFi IP/GATEWAY");
      M5.Lcd.println("UP/OK [A]");
      M5.Lcd.println("DOWN  [B]");
      M5.Lcd.println("NEXT  [C]");
      M5.Lcd.println("");
      M5.Lcd.setCursor(0, 40);
      M5.Lcd.println("IP:");
      M5.Lcd.setCursor(0, 56);
      M5.Lcd.println("GATEWAY:");

      M5.Lcd.setTextColor(WHITE, BLACK);
      while(1){
        M5.update();
        delay(100);

        if(curs_y == 0 || curs_y == 1){
          M5.Lcd.setTextColor(WHITE, BLACK);

          M5.Lcd.setCursor(0, 80);
          M5.Lcd.print("Set & Exit");
          M5.Lcd.setCursor(0, 88);
          M5.Lcd.print("Reset");

          M5.Lcd.setCursor(64, 40);
          if(curs_y == 0){
            for(int i = 0; i < 3; i++){
              if(curs_x == i){
                M5.Lcd.setTextColor(RED, BLACK);
                pcur = &aip1[i];
              }
              else {
                M5.Lcd.setTextColor(WHITE, BLACK);
              }
              M5.Lcd.printf("%d", aip1[i]);
            }
            M5.Lcd.setTextColor(WHITE, BLACK);
            M5.Lcd.print(".");
            for(int i = 0; i < 3; i++){
              if(curs_x == (i + 3)){
                M5.Lcd.setTextColor(RED, BLACK);
                pcur = &aip2[i];
              }
              else {
                M5.Lcd.setTextColor(WHITE, BLACK);
              }
              M5.Lcd.printf("%d", aip2[i]);
            }
            M5.Lcd.setTextColor(WHITE, BLACK);
            M5.Lcd.print(".");
            for(int i = 0; i < 3; i++){
              if(curs_x == (i + 6)){
                M5.Lcd.setTextColor(RED, BLACK);
                pcur = &aip3[i];
              }
              else {
                M5.Lcd.setTextColor(WHITE, BLACK);
              }
              M5.Lcd.printf("%d", aip3[i]);
            }
            M5.Lcd.setTextColor(WHITE, BLACK);
            M5.Lcd.print(".");
            for(int i = 0; i < 3; i++){
              if(curs_x == (i + 9)){
                M5.Lcd.setTextColor(RED, BLACK);
                pcur = &aip4[i];
              }
              else {
                M5.Lcd.setTextColor(WHITE, BLACK);
              }
              M5.Lcd.printf("%d", aip4[i]);
            }
          }
          else {
            M5.Lcd.setTextColor(WHITE, BLACK);
            M5.Lcd.printf("%d%d%d.", aip1[0], aip1[1], aip1[2]);
            M5.Lcd.printf("%d%d%d.", aip2[0], aip2[1], aip2[2]);
            M5.Lcd.printf("%d%d%d.", aip3[0], aip3[1], aip3[2]);
            M5.Lcd.printf("%d%d%d", aip4[0], aip4[1], aip4[2]);
          }

          M5.Lcd.setCursor(64, 56);
          if(curs_y == 1){
            for(int i = 0; i < 3; i++){
              if(curs_x == i){
                M5.Lcd.setTextColor(RED, BLACK);
                pcur = &agt1[i];
              }
              else {
                M5.Lcd.setTextColor(WHITE, BLACK);
              }
              M5.Lcd.printf("%d", agt1[i]);
            }
            M5.Lcd.setTextColor(WHITE, BLACK);
            M5.Lcd.print(".");
            for(int i = 0; i < 3; i++){
              if(curs_x == (i + 3)){
                M5.Lcd.setTextColor(RED, BLACK);
                pcur = &agt2[i];
              }
              else {
                M5.Lcd.setTextColor(WHITE, BLACK);
              }
              M5.Lcd.printf("%d", agt2[i]);
            }
            M5.Lcd.setTextColor(WHITE, BLACK);
            M5.Lcd.print(".");
            for(int i = 0; i < 3; i++){
              if(curs_x == (i + 6)){
                M5.Lcd.setTextColor(RED, BLACK);
                pcur = &agt3[i];
              }
              else {
                M5.Lcd.setTextColor(WHITE, BLACK);
              }
              M5.Lcd.printf("%d", agt3[i]);
            }
            M5.Lcd.setTextColor(WHITE, BLACK);
            M5.Lcd.print(".");
            for(int i = 0; i < 3; i++){
              if(curs_x == (i + 9)){
                M5.Lcd.setTextColor(RED, BLACK);
                pcur = &agt4[i];
              }
              else {
                M5.Lcd.setTextColor(WHITE, BLACK);
              }
              M5.Lcd.printf("%d", agt4[i]);
            }
          }
          else {
            M5.Lcd.setTextColor(WHITE, BLACK);
            M5.Lcd.printf("%d%d%d.", agt1[0], agt1[1], agt1[2]);
            M5.Lcd.printf("%d%d%d.", agt2[0], agt2[1], agt2[2]);
            M5.Lcd.printf("%d%d%d.", agt3[0], agt3[1], agt3[2]);
            M5.Lcd.printf("%d%d%d", agt4[0], agt4[1], agt4[2]);
          }

          if(M5.BtnC.wasPressed()){
            if(curs_x == 11){
              curs_x = 0;
              curs_y = curs_y + 1;
            }
            else {
              curs_x = curs_x + 1;
            }
          }
          else if(M5.BtnA.wasPressed()){
            *pcur = *pcur + 1;
            *pcur = (*pcur % 10);
          }
          else if(M5.BtnB.wasPressed()){
            *pcur = *pcur - 1;
            if(*pcur < 0){
              *pcur = 9;
            }
          }
        }
        else if(curs_y == 2){
          M5.Lcd.setCursor(64, 56);
          M5.Lcd.setTextColor(WHITE, BLACK);
          M5.Lcd.printf("%d%d%d.", agt1[0], agt1[1], agt1[2]);
          M5.Lcd.printf("%d%d%d.", agt2[0], agt2[1], agt2[2]);
          M5.Lcd.printf("%d%d%d.", agt3[0], agt3[1], agt3[2]);
          M5.Lcd.printf("%d%d%d", agt4[0], agt4[1], agt4[2]);

          M5.Lcd.setTextColor(RED, BLACK);
          M5.Lcd.setCursor(0, 80);
          M5.Lcd.print("Set & Exit");
          M5.Lcd.setTextColor(WHITE, BLACK);
          M5.Lcd.setCursor(0, 88);
          M5.Lcd.print("Reset");

          if(M5.BtnA.wasPressed()){
            int ip1 = aip1[0] * 100 + aip1[1] * 10 + aip1[2];
            int ip2 = aip2[0] * 100 + aip2[1] * 10 + aip2[2];
            int ip3 = aip3[0] * 100 + aip3[1] * 10 + aip3[2];
            int ip4 = aip4[0] * 100 + aip4[1] * 10 + aip4[2];

            int gw1 = agt1[0] * 100 + agt1[1] * 10 + agt1[2];
            int gw2 = agt2[0] * 100 + agt2[1] * 10 + agt2[2];
            int gw3 = agt3[0] * 100 + agt3[1] * 10 + agt3[2];
            int gw4 = agt4[0] * 100 + agt4[1] * 10 + agt4[2];

            preferences.begin(PREF_ROOT, false);
            preferences.putInt(PREF_IP1, ip1);
            preferences.putInt(PREF_IP2, ip2);
            preferences.putInt(PREF_IP3, ip3);
            preferences.putInt(PREF_IP4, ip4);

            preferences.putInt(PREF_GW1, gw1);
            preferences.putInt(PREF_GW2, gw2);
            preferences.putInt(PREF_GW3, gw3);
            preferences.putInt(PREF_GW4, gw4);
            preferences.end();

            ESP.restart();
          }
          else if(M5.BtnC.wasPressed()){
            curs_x = 0;
            curs_y = curs_y + 1;
          }
        }
        else if(curs_y == 3){
          M5.Lcd.setTextColor(WHITE, BLACK);
          M5.Lcd.setCursor(0, 80);
          M5.Lcd.print("Set & Exit");
          M5.Lcd.setTextColor(RED, BLACK);
          M5.Lcd.setCursor(0, 88);
          M5.Lcd.print("Reset");

          if(M5.BtnA.wasPressed()){
            ESP.restart();
          }
          else if(M5.BtnC.wasPressed()){
            curs_x = 0;
            curs_y = 0;
          }
        }

        M5.Lcd.setTextColor(WHITE, BLACK);
//        M5.Lcd.setCursor(0, 88);
//        M5.Lcd.print(curs_x);
//        M5.Lcd.print("/");
//        M5.Lcd.print(curs_y);
      }
    }

    M5.Lcd.clearDisplay(BLACK);

    // モータ停止
    gMotorRSpd = 0;
    gMotorLSpd = 0;
  }

  // モータードライバへ送信
  // 設定中の停止状態
  motorDriverSend();
}
