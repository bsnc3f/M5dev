#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <M5Stack.h>
#include <VL53L0X.h>
VL53L0X sensor;

//////////////////////////////////
//レシーバーのMACアドレスに置き換えます
//////////////////////////////////

uint8_t broadcastAddress[] = {0x30, 0xAE, 0xA4, 0x0E, 0x0D, 0x6C};

const uint boardID = 2; // 端末を識別するボードID
const uint productID = 202008;
const uint maxMoni = 4; // モニタ入力の最大値
uint selfMonitor = 0;
uint distance = 65353;
const uint distanceMin = 30;
const uint distanceMax = 250;
boolean is_sleep = false;
boolean sensorHealth = true;
boolean sensorErr = false;
boolean inDistance = false;
boolean notificated = false;

#define timer sensor.setTimeout(500)

// Struct_messageへラッピング
typedef struct struct_message
{
  uint pid = productID;
  uint id = boardID;
  uint moni = selfMonitor;
  int x;
  int y;

} struct_message;

//Create a struct_message called myData
struct_message myData;

// レシーブ用
// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;
struct_message board3;
struct_message board4;
struct_message board5;

// Create an array with all the structures
struct_message boardsStruct[5] = {board1, board2, board3, board4, board5};

// ESP初期化処理
void InitESPNow()
{
  // ESP-NOW初期化
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  { //esp_err_t esp_now_init（ void ） ESPNOW初期化
    Serial.println("ESPNow Init Success");
  }
  else
  {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

// データ送信時のコールバック
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nPacket Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// データ受信時のコールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  // Update the structures with the new incoming data
  boardsStruct[myData.id - 1].pid = myData.pid;
  boardsStruct[myData.id - 1].id = myData.id;
  boardsStruct[myData.id - 1].x = myData.x;
  boardsStruct[myData.id - 1].y = myData.y;
  boardsStruct[myData.id - 1].moni = myData.moni;
  //  boardsStruct[myData.id - 1].reserveNum = myData.reserveNum;
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  Serial.printf("productID: %d \n", boardsStruct[myData.id - 1].pid);
  Serial.printf("x: %d \n", boardsStruct[myData.id - 1].x);
  Serial.printf("y: %d \n", boardsStruct[myData.id - 1].y);
  Serial.printf("Set Monitor Value: %d \n", boardsStruct[myData.id - 1].moni);
  //  Serial.printf("return reserveNum value: %d \n", boardsStruct[myData.id - 1].reserveNum);
  Serial.println();

  if (myData.pid == productID && myData.id != boardID && myData.x <= 50)
  {
    // CallBackのボードIDが自分でなければ
    switchedDisp();
  }
}

// LED Matrixの明るさ（0～100）
#define DISP_BRIGHTNESS_MIN 80  //
#define DISP_BRIGHTNESS_MAX 100 //
uint8_t disp_brightness = DISP_BRIGHTNESS_MIN;

void setup()
{
  // シリアルモニターの初期化
  Serial.begin(115200);
  Wire.begin();

  // デバイスをWi-Fiステーションとして設定します
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.println(WiFi.macAddress());

  /////////////////////////////////////////////////
  //  ESP-NOW
  /////////////////////////////////////////////////

  // ESP-NOWを初期化します
  InitESPNow();

  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESPNow Init Success");
    Serial.println(WiFi.macAddress());
    M5.Lcd.println("ESPNow Init Success\n");
    M5.Lcd.println(WiFi.macAddress());
  }
  else
  {
    Serial.println("ESPNow Init Failed");
    M5.Lcd.print("ESPNow Init Failed\n");
    delay(2000);
    ESP.restart();
  }

  // ピア登録
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // ピア追加
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  /////////////////////////////////////////////////
  //  CallBack
  /////////////////////////////////////////////////

  // 送信コールバックを登録
  esp_now_register_send_cb(OnDataSent);

  // 受信コールバックを登録
  esp_now_register_recv_cb(OnDataRecv);

  /////////////////////////////////////////////////
  //  M5Stack専用
  /////////////////////////////////////////////////

  M5.begin(); // SerialEnable = true, I2CEnable = false, DisplayEnable = true);
  M5.Speaker.begin();          // これが無いとmuteしても無意味です。
  M5.Speaker.mute();           // ノイズ対策
                               //  M5.Power.begin();  // バッテリーチェック関連
                               //  if(!M5.Power.canControl()) {    //can't control
                               //    M5.Lcd.print("Power Control NG");
                               //    return;

  /////////////////////////////////////////////////
  //  LCD Setup
  /////////////////////////////////////////////////

  // LCD display
  M5.Lcd.clear(BLACK);
  //  M5.Lcd.setTextFont(4); // 2 = 16ピクセルASCIIフォント
  M5.Lcd.setBrightness(disp_brightness); // 0 ～ 100
  M5.Lcd.setRotation(0);                 // M5.Lcd.begin()の中で既にsetRotation(1)が実行されている。 1で+90度ずつ回転。 4～7はインバートして回転

  /////////////////////////////////////////////////
  // バッテリー情報表示
  /////////////////////////////////////////////////

  //  show_battery_info();

  /////////////////////////////////////////////////
  //  VL53L0X sensor
  /////////////////////////////////////////////////

  sensor.init();
  sensor.setTimeout(500);

  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    M5.Lcd.setTextSize(3);
    M5.Lcd.println("Sensor NG");
    M5.Lcd.println("!!!!");
    delay(2000);
    M5.Lcd.clear(BLACK);
    sensor.stopContinuous();
    sensorHealth = false;
//    sensorErr = true;
  }
/*  
//#define LONG_RANGE  // 長距離モード センサの感度を高め、潜在的な範囲を広げターゲット以外の物体からの反射も高める。 暗い場所では最も効果的
//#define HIGH_SPEED  // 精度を下げてより高い速度を得る
//#define HIGH_ACCURACY  // 速度を落としてより高い精度を得る
*/
#if defined LONG_RANGE  // 長距離モード センサの感度を高め、潜在的な範囲を広げターゲット以外の物体からの反射も高める。 暗い場所では最も効果的
  sensor.setSignalRateLimit(0.1);  // 戻り信号のレート制限を下げます（デフォルトは0.25 MCPS）
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);  // レーザーパルス周期を長くする（デフォルトは14および10 PCLKs）
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED  // 精度を下げてより高い速度を得る
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);  // タイミングバジェットを20ミリ秒に減らす（デフォルトは約33ミリ秒
#elif defined HIGH_ACCURACY  // 速度を落としてより高い精度を得る
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);  // タイミングバジェットを200ミリ秒に増やす
#endif
}

void LCDReflesh(int lBright, int tSize, int xCursor, int yCursor)
{
  M5.Lcd.wakeup();
  M5.Lcd.clear(BLACK);
  M5.Lcd.setBrightness(lBright);
  M5.Lcd.setTextSize(tSize);
  M5.Lcd.setCursor(xCursor, yCursor);
}

void sensON()
{
  myData.moni = 0;
  selfMonitor = 0;
////  myData.id = boardID; // 他idから制御後、効かない事があるため、idをSlaveにみせる
  esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  M5.update();
  M5.Lcd.setTextColor(TFT_CYAN);            //CYAN
  LCDReflesh(disp_brightness, 26, 60, 120); // Bright, FontSize, x, y
  M5.Lcd.println("NPS");
  notificated = false;
  delay(500);
}

void switchedDisp()
{
  switch (myData.moni)
  {
  case 0:
    M5.Lcd.setTextColor(TFT_CYAN);            //CYAN
    LCDReflesh(disp_brightness, 26, 60, 120); // Bright, FontSize, x, y
    M5.Lcd.println("NPS");
    break;
  case 1:
    M5.Lcd.setTextColor(YELLOW);
    LCDReflesh(disp_brightness, 26, 20, 120); // Bright, FontSize, x, y
    M5.Lcd.println("C Cam");
    break;
  case 2:
    M5.Lcd.setTextColor(YELLOW);
    LCDReflesh(disp_brightness, 26, 20, 120); // Bright, FontSize, x, y
    M5.Lcd.println("B Cam");
    break;
  case 3:
    M5.Lcd.setTextColor(YELLOW);
    LCDReflesh(disp_brightness, 26, 20, 120); // Bright, FontSize, x, y
                                              //    LCDReflesh(80,26,90,120);  // Bright, FontSize, x, y
    M5.Lcd.println("A Cam");
    break;
  default:
    break;
  }
  notificated = false;
//  sensorHealth = true; // センサーエラーを再通知するためのおまじない
}

void loop()
{
//  M5.update();
//  Set values to send
  myData.pid = productID;
  myData.id = boardID;
  myData.moni = selfMonitor;
  myData.x = random(0, 50);
  myData.y = random(0, 50);

//  sensor.setTimeout(500);

  if (sensorHealth)
  {
//  timer = sensor.setTimeout(500);

    distance = sensor.readRangeSingleMillimeters();
    //    sensor.setTimeout(500);
    Serial.println(distance);  //***

    if (sensor.timeoutOccurred())
    {
      sensorHealth = !sensorHealth;
      M5.Lcd.setCursor(10, 10);
      Serial.println("Sensor Timeout");
      sensor.stopContinuous();
  //    clearTimeout();
    }

    if (distance < 250)
    {
      inDistance = true;
      M5.Lcd.setCursor(20, 10);
      M5.Lcd.setTextSize(4);
      M5.Lcd.setTextColor(RED);
      M5.Lcd.println("sens on");
      delay(330);

      if (distance > distanceMin && distance < distanceMax && selfMonitor != 1)
      {
        do {
        sensON();
        selfMonitor = 0;

        switchedDisp();
        delay(300);
        selfMonitor = 1;
        } while(distance > distanceMin && distance < distanceMax && myData.moni != 0);
        //return;
      }
      else if (distance <= 40)
      {
        M5.Lcd.clear(GREEN);
        M5.Lcd.println("Too Near!!");
        delay(1000);
      }
      else
      {
      }
     }
    if (inDistance == true) {
      myData.moni = 0;
      switchedDisp();
      delay(500);
      inDistance = false;
      }
  }
  else
  {
    delay(10);  // 現時点では、バグ対策に必要
//    Serial.println("Sensor Checking");
//    // SensorがNGから復帰した場合の処理を追加したい

      if(!sensorErr)
      {
        if(!notificated)
        {
        M5.Lcd.setCursor(10, 10);
        M5.Lcd.setTextSize(4);
        M5.Lcd.setTextColor(BLUE);
        M5.Lcd.println("Sensor NG");
        Serial.println("Sensor Checking");
        delay(30);  // 現時点では、バグ対策に必要
        sensor.init();
        Serial.println("Sensor Initializing");
        notificated = true;
//        sensorErr = true;
//        clearTimeout(timer);
//        sensor.stopContinuous();
        }
      }
  }

  if (M5.BtnC.wasPressed())
  {
    switch (selfMonitor)
    {
    case 0:
      M5.Lcd.setTextColor(TFT_CYAN);            //CYAN
      LCDReflesh(disp_brightness, 26, 60, 120); // Bright, FontSize, x, y
      M5.Lcd.println("NPS");
      break;
    case 1:
      M5.Lcd.setTextColor(YELLOW);
      LCDReflesh(disp_brightness, 26, 20, 120); // Bright, FontSize, x, y
      M5.Lcd.println("C Cam");
      break;
    case 2:
      M5.Lcd.setTextColor(YELLOW);
      LCDReflesh(disp_brightness, 26, 20, 120); // Bright, FontSize, x, y
      M5.Lcd.println("B Cam");
      break;
    case 3:
      M5.Lcd.setTextColor(YELLOW);
      LCDReflesh(disp_brightness, 26, 20, 120); // Bright, FontSize, x, y
                                                //    LCDReflesh(80,26,90,120);  // Bright, FontSize, x, y
      M5.Lcd.println("A Cam");
      break;
    default:
      break;
    }
    notificated = false;


    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    if (result == ESP_OK)
    {
      Serial.println("Send with success");
    }
    else
    {
      Serial.println("Error sending the data");
    }
    Serial.print("Self Monitor No. is ");
    Serial.println(selfMonitor);
    selfMonitor++;
    if (selfMonitor >= maxMoni)
    {
      selfMonitor = 0;
    }
    delay(100);
  }
  else if (M5.BtnC.pressedFor(600))
  {
    myData.moni = 0;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    M5.Lcd.setTextColor(TFT_CYAN);            //CYAN
    LCDReflesh(disp_brightness, 26, 60, 120); // Bright, FontSize, x, y
    M5.Lcd.println("NPS");
    while (!M5.BtnC.wasReleased())
    { // ボタンが離されるまで待つ
      delay(50);
      M5.update();
      selfMonitor = 1;
    }
    notificated = false;
  }
  if (M5.BtnB.wasPressed())
  {
    // Set values to send
    myData.id = boardID;
    myData.moni = 0;
    myData.x = random(0, 50);
    myData.y = random(0, 50);

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    switchedDisp();
    selfMonitor = 1;
    notificated = false;
  }
  if (M5.BtnA.wasPressed())
  {
    if (is_sleep)
    {
      ; // 復帰するだけで、何もしない
    }
    else
    {
      M5.Lcd.sleep();          // スリープ
      M5.Lcd.setBrightness(0); // スリープとは別に制御をかけている
    }
    is_sleep = !is_sleep;
    //    M5.Lcd.sleep();  // スリープ
    //    M5.Lcd.setBrightness(0);  // スリープとは別に制御をかけている
  }

  M5.update();

}
