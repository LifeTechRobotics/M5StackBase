#include <M5Stack.h>    // CORE2の場合は <M5Core2.h>
#include <WiFi.h>
#include <Preferences.h>
#include "nvs_flash.h"

// ボタンで速度を操作する場合には定義する
#define BTN_TEST_MODE

// WiFi通信設定
#define SERVER_NUM_CLIENT         1           // 接続クライアント台数
#define SERVER_PORT               10200       // 通信用ポート（＊）（適宜変更・・・Windowsソフトも変更）

// 通信用のプロトコル定義
#define PROTOCOL_HEADER           "###"
#define PROTOCOL_RESET            "RST"
#define PROTOCOL_IP               "SIP"
#define PROTOCOL_GATEWAY          "SGW"
#define PROTOCOL_WIFI_ERASE       "WES"
#define PROTOCOL_OK               "$$$,OK\r"  // 正常
#define PROTOCOL_NG               "$$$,NG\r"  // エラー

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


// モータ速度用
#define MOTOR_SPEED_MAX           63          // モータ速度最大値
#define MOTOR_SPEED_MIN           -63         // モータ速度最小値


// 変数定義
Preferences preferences;                      // ESP32の保存情報
int gMotorRSpd = 0;                           // モータRの値（-63～+63）
int gMotorLSpd = 0;                           // モータLの値（-63～+63）
bool gPushOnA = false;                        // Aが押されているかどうか
bool gPushOnB = false;                        // Bがおされているかどうか
bool gPushOnC = false;                        // Cがおされているかどうか

WiFiServer gServer(SERVER_PORT);              // WiFiの自サーバー変数
WiFiClient gClient;                           // 接続クライアント用
bool gClientConnectFlag = false;              // クライアントとしての接続状態
bool gAPConnectFlag = false;                  // APへの接続状態
unsigned long gCommTime = 0;                  // 通信タイムアウト用

// LED
int gPWMLed1 = 0;                             // LED1 PWM
int gPWMLed2 = 0;                             // LED2 PWM

// DEMO MODE
bool gDemoMode = false;                       // デモモード

// ROATE COUNTER
int gRotateCount = 0;                         // インジケーター回転用


//
// インジケーター用カウンターの初期化
void IndicatorInit() {
  gRotateCount = 0;
}

//
// インジケーターの表示
void IndicatorPrint(int xpos, int ypos) {
  M5.Lcd.setCursor(xpos, ypos);
  if(gRotateCount == 0){
    M5.Lcd.print("|");
  }
  else if(gRotateCount == 1){
    M5.Lcd.print("/");
  }
  else if(gRotateCount == 2){
    M5.Lcd.print("-");
  }
  else if(gRotateCount == 3){
    M5.Lcd.print("\\");
  }

  gRotateCount ++;
  gRotateCount %= 4;
}

//
// WiFi Information Read ip
void WiFiInfoReadIP(int *pip1, int *pip2, int *pip3, int *pip4) {
  preferences.begin(PREF_ROOT, true);

  *pip1 = preferences.getInt(PREF_IP1, -1);
  *pip2 = preferences.getInt(PREF_IP2, -1);
  *pip3 = preferences.getInt(PREF_IP3, -1);
  *pip4 = preferences.getInt(PREF_IP4, -1);

  preferences.end();
}

// WiFi Information Read gateway
void WiFiInfoReadGateway(int *pgw1, int *pgw2, int *pgw3, int *pgw4) {
  preferences.begin(PREF_ROOT, true);

  *pgw1 = preferences.getInt(PREF_GW1, -1);
  *pgw2 = preferences.getInt(PREF_GW2, -1);
  *pgw3 = preferences.getInt(PREF_GW3, -1);
  *pgw4 = preferences.getInt(PREF_GW4, -1);

  preferences.end();
}

// WiFi Information Write ip
void WiFiInfoWriteIP(int ip1, int ip2, int ip3, int ip4) {
  preferences.begin(PREF_ROOT, false);

  preferences.putInt(PREF_IP1, ip1);
  preferences.putInt(PREF_IP2, ip2);
  preferences.putInt(PREF_IP3, ip3);
  preferences.putInt(PREF_IP4, ip4);

  preferences.end();
}

// WiFi Information Write gateway
void WiFiInfoWriteGateway(int gw1, int gw2, int gw3, int gw4) {
  preferences.begin(PREF_ROOT, false);

  preferences.putInt(PREF_GW1, gw1);
  preferences.putInt(PREF_GW2, gw2);
  preferences.putInt(PREF_GW3, gw3);
  preferences.putInt(PREF_GW4, gw4);

  preferences.end();
}

// WiFi Information Erase
void WiFiInfoErase() {
  Serial.println("--- WiFi Information Erase ---");
  nvs_flash_erase();
  nvs_flash_init();

  // Erace IP
  WiFiInfoWriteIP(-1, -1, -1, -1);

  // Erace Gateway
  WiFiInfoWriteGateway(-1, -1, -1, -1);
}

// Address Display
void IPAddressDisplay(int ap1[3], int ap2[3], int ap3[3], int ap4[3]) {
  M5.Lcd.setTextColor(WHITE, BLACK);

  M5.Lcd.printf("%d%d%d.", ap1[0], ap1[1], ap1[2]);
  M5.Lcd.printf("%d%d%d.", ap2[0], ap2[1], ap2[2]);
  M5.Lcd.printf("%d%d%d.", ap3[0], ap3[1], ap3[2]);
  M5.Lcd.printf("%d%d%d",  ap4[0], ap4[1], ap4[2]);
}

// Address Calc(int ap[3])
int AddressCalc(int ap[3]) {
  int ret = (ap[0] * 100) + (ap[1] * 10) + ap[2];

  return ret;
}

//
// split関数
int split(String data, char delimiter, String *dst, int dsize) {
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
// Motor Set Value
void MotorSetValue(int rspd, int lspd) {
  gMotorRSpd = rspd;
  gMotorLSpd = lspd;
}

// Motor Stop
void MotorStop() {
  MotorSetValue(0, 0);
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
  Serial1.begin(115200, SERIAL_8N1, 16, 17);  // シリアル通信1初期化(RX(R2), TX(T2)) ※BASIC R2/T2

  // Motor Sto
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
    int ip1 = -1;
    int ip2 = -1;
    int ip3 = -1;
    int ip4 = -1;
    int gw1 = -1;
    int gw2 = -1;
    int gw3 = -1;
    int gw4 = -1;
    WiFiInfoReadIP(&ip1, &ip2, &ip3, &ip4);
    WiFiInfoReadGateway(&gw1, &gw2, &gw3, &gw4);

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
  gClientConnectFlag = false;

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
// Motor Demo Mode
// [A] ... 速度を+方向へ加算していく
// [B] ... 速度を-方向へ減算していく
// [C] ... 停止
void MotorDemo() {
  // 内部ループはなしなのでM5.updateはLoop処理の最初は有効
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
      // モータ停止
      MotorStop();
      gPushOnC = false;
    }
  }
}

//
// loop
void loop() {
  // put your main code here, to run repeatedly:
  unsigned long tm = millis();
  M5.update();  //本体のボタン状態更新

  M5.Lcd.setTextColor(WHITE, BLACK);
  if(gDemoMode){
    // モータ動作デモモード
    MotorDemo();
  }



  if(!gDemoMode && (WiFi.status() == WL_CONNECTED)) {
    // 初回APに接続時のみ
    if(!gAPConnectFlag) {
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.println("WiFi : AP connected !!             ");
      M5.Lcd.print("IP address : ");
      M5.Lcd.println(WiFi.localIP());
      gAPConnectFlag = true;
    }

    // クライアントの接続
    if(!gClientConnectFlag) {
      gClient = gServer.available();

      if(gClient.connected()){
        // クライアント接続完了
        gClientConnectFlag = true;
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("WiFi : Client Connected !!       ");

        // 通信タイムアウトの更新
        gCommTime = tm;
      }
      else {
        // 接続失敗 3秒後に再接続
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("WiFi : Client does not Connect !!");
        delay(3000);
        // モータ停止
        MotorStop();
      }
    }

    if(gClientConnectFlag) {
      // サーバーに接続中
      if(gClient.available()) {
        // 受信データあり
        String line = gClient.readStringUntil('\r');
//        char tbuf[16];
//        sprintf(tbuf, "%012d : ", tm);
//        Serial.print(tbuf);
//        Serial.println(line);

        // データ解析
        // ###,RST
        // ###,WES
        // ###,SIP,192,168,0,110
        // ###,SGW,192,168,0,1
        // ###,20,20,000,000
        // ###：ヘッダー（固定）
        // 020：右モーターの指令値
        // 020：左モーターの指令値
        // 000：LED1の輝度
        // 000：LED2の輝度
        String params[6] = {"\0"};
        int cnt = split(line, ',', params, 6);
        bool dataflag = false;
        if(cnt == 6) {
          if(params[0].compareTo(PROTOCOL_HEADER) == 0) {
            if(params[1].compareTo(PROTOCOL_IP) == 0) {
              // IP設定
              int ip1 = params[2].toInt();
              int ip2 = params[3].toInt();
              int ip3 = params[4].toInt();
              int ip4 = params[5].toInt();

              // Write IP
              WiFiInfoWriteIP(ip1, ip2, ip3, ip4);

              dataflag = true;
            }
            else if(params[1].compareTo(PROTOCOL_GATEWAY) == 0) {
              // GATEWAY設定
              int gw1 = params[2].toInt();
              int gw2 = params[3].toInt();
              int gw3 = params[4].toInt();
              int gw4 = params[5].toInt();

              // Write Gateway
              WiFiInfoWriteGateway(gw1, gw2, gw3, gw4);

              dataflag = true;
            }
          }
        }
        else if(cnt == 5){
          if(params[0].compareTo(PROTOCOL_HEADER) == 0) {
            // モータ値設定
            int mrspd = params[1].toInt();
            int mlspd = params[2].toInt();
            int l1pwm = params[3].toInt();
            int l2pwm = params[4].toInt();

            M5.Lcd.setCursor(0, 48);
            M5.Lcd.print(mrspd);
            M5.Lcd.print("/");
            M5.Lcd.print(mlspd);
            M5.Lcd.print("        ");

            M5.Lcd.setCursor(0, 56);
            M5.Lcd.println(params[1]);
            M5.Lcd.println(params[2]);
            M5.Lcd.println(params[3]);
            M5.Lcd.println(params[4]);

            // エラーチェック
            if(l1pwm >= 0 && l1pwm <= PWM_RES_MAX_LED) {
              if(l2pwm >= 0 && l2pwm <= PWM_RES_MAX_LED) {

                // 設定値の更新
                gMotorRSpd = mrspd;
                gMotorLSpd = mlspd;
                gPWMLed1   = l1pwm;
                gPWMLed2   = l2pwm;

                // 正常データ
                dataflag = true;
              }
            }
          }
        }
        else if(cnt == 2){
          if(params[0].compareTo(PROTOCOL_HEADER) == 0) {
            if(params[1].compareTo(PROTOCOL_RESET) == 0) {
              // ソフトリセット
              M5.Lcd.setCursor(0, 0);
              M5.Lcd.println("");
              M5.Lcd.println("Reset");
              ESP.restart();
            }
            else if(params[1].compareTo(PROTOCOL_WIFI_ERASE) == 0) {
              // WiFi情報消去
              WiFiInfoErase();
              M5.Lcd.setCursor(0, 0);
              M5.Lcd.println("");
              M5.Lcd.println("Erase & Reset");
              ESP.restart();
            }
          }
        }

        // 返信データ
        if(dataflag){
          // 正常データの場合
          gClient.print(PROTOCOL_OK);
          // 通信タイムアウトの更新
          gCommTime = tm;
        }
        else {
          // データエラーの場合
          // 設定値は前の値のまま変更しない
          gClient.print(PROTOCOL_NG);
        }
      }
      else {
        // データが受信できない場合
        if(gCommTime > 0 && ((tm - gCommTime) > COMM_TIMEOUT)) {
          // 通信タイムアウト
          gClient.stop();
          gCommTime = 0;
          gClientConnectFlag = false;
          // モータ停止
          MotorStop();
        }
      }
    }
    else {
      // サーバーに接続できていない
      // モータ停止
      MotorStop();
    }
  }
  else if(!gDemoMode){
    // WiFi APに接続できていない場合
    gAPConnectFlag = false;
    M5.Lcd.clearDisplay(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("WiFi AP Not Connect !");

    // サーバーへの接続解除
    if(gClientConnectFlag) {
      M5.Lcd.println("WiFi Disconnect Server");
      gClient.stop();
      gClientConnectFlag = false;
    }

    // 画面モードかスマートモードか
    int setupmode = 0;
    M5.Lcd.println("");
    M5.Lcd.println("WiFi Setup !!");
    M5.Lcd.println("");
    M5.Lcd.println("Setup SmartConfig [A]");
    M5.Lcd.println("Setup IP/GATEWAY  [B]");
    M5.Lcd.println("Exit & Reset      [C]");
    M5.Lcd.println("");
    IndicatorInit();
    while(1){
      // インジケーター表示
      IndicatorPrint(0, 56);

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
      IndicatorInit();
      while (!WiFi.smartConfigDone()) {
        delay(200);
        M5.update();
        if(M5.BtnC.isPressed()){
          ESP.restart();
        }

        // インジケーター表示
        IndicatorPrint(0, 16);
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

      int ai1 = -1;
      int ai2 = -1;
      int ai3 = -1;
      int ai4 = -1;

      WiFiInfoReadIP(&ai1, &ai2, &ai3, &ai4);
      if(!(ai1 == -1 || ai2 == -1 || ai3 == -1 || ai4 == -1)) {
        aip1[0] = ai1 / 100;
        aip1[1] = (ai1 % 100) / 10;
        aip1[2] = ai1 % 10;

        aip2[0] = ai2 / 100;
        aip2[1] = (ai2 % 100) / 10;
        aip2[2] = ai2 % 10;

        aip3[0] = ai3 / 100;
        aip3[1] = (ai3 % 100) / 10;
        aip3[2] = ai3 % 10;

        aip4[0] = ai4 / 100;
        aip4[1] = (ai4 % 100) / 10;
        aip4[2] = ai4 % 10;
      }
      WiFiInfoReadGateway(&ai1, &ai2, &ai3, &ai4);
      if(!(ai1 == -1 || ai2 == -1 || ai3 == -1 || ai4 == -1)) {
        agt1[0] = ai1 / 100;
        agt1[1] = (ai1 % 100) / 10;
        agt1[2] = ai1 % 10;

        agt2[0] = ai2 / 100;
        agt2[1] = (ai2 % 100) / 10;
        agt2[2] = ai2 % 10;

        agt3[0] = ai3 / 100;
        agt3[1] = (ai3 % 100) / 10;
        agt3[2] = ai3 % 10;

        agt4[0] = ai4 / 100;
        agt4[1] = (ai4 % 100) / 10;
        agt4[2] = ai4 % 10;
      }

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
            IPAddressDisplay(aip1, aip2, aip3, aip4);
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
            IPAddressDisplay(agt1, agt2, agt3, agt4); 
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
          IPAddressDisplay(agt1, agt2, agt3, agt4);

          M5.Lcd.setTextColor(RED, BLACK);
          M5.Lcd.setCursor(0, 80);
          M5.Lcd.print("Set & Exit");
          M5.Lcd.setTextColor(WHITE, BLACK);
          M5.Lcd.setCursor(0, 88);
          M5.Lcd.print("Reset");

          if(M5.BtnA.wasPressed()){
            int ip1 = AddressCalc(aip1);
            int ip2 = AddressCalc(aip2);
            int ip3 = AddressCalc(aip3);
            int ip4 = AddressCalc(aip4);

            int gw1 = AddressCalc(agt1);
            int gw2 = AddressCalc(agt2);
            int gw3 = AddressCalc(agt3);
            int gw4 = AddressCalc(agt4);

            // Write IP
            WiFiInfoWriteIP(ip1, ip2, ip3, ip4);

            // Write Gateway
            WiFiInfoWriteGateway(gw1, gw2, gw3, gw4);

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
    MotorStop();
  }

  // 速度設定の確認
  if(gMotorRSpd > MOTOR_SPEED_MAX){
    gMotorRSpd = MOTOR_SPEED_MAX;
  }
  if(gMotorRSpd < MOTOR_SPEED_MIN){
    gMotorRSpd = MOTOR_SPEED_MIN;
  }
  if(gMotorLSpd > MOTOR_SPEED_MAX){
    gMotorLSpd = MOTOR_SPEED_MAX;
  }
  if(gMotorLSpd < MOTOR_SPEED_MIN){
    gMotorLSpd = MOTOR_SPEED_MIN;
  }
  int spdr = 0;
  int spdl = 0;
  if(gMotorRSpd >= 0 && gMotorRSpd <= MOTOR_SPEED_MAX){
    spdr = gMotorRSpd;
  }
  else {
    spdr = 64 - gMotorRSpd;
  }
  if(gMotorLSpd >= 0 && gMotorLSpd <= MOTOR_SPEED_MAX){
    spdl = 128 + gMotorLSpd;
  }
  else {
    spdl = 192 - gMotorLSpd;
  }

  // 設定速度の表示
  if(!gDemoMode){
    M5.Lcd.setCursor(0, 0);
    if(gClient.connected()){
      M5.Lcd.print("Client IP address : ");
      M5.Lcd.println(gClient.remoteIP());
    }
    else {
      M5.Lcd.print("IP address : ");
      M5.Lcd.println(WiFi.localIP());
    }
  }
  M5.Lcd.setCursor(0, 32);
  M5.Lcd.printf("%3d/%3d     ", spdr, spdl);

  // モータドライバへ送信
  Serial1.write(spdr);
  Serial1.write(spdl);
}
