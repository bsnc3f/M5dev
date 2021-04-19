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
const uint distanceMin = 40;
const uint distanceMax = 250;
boolean is_sleep = false;
boolean sensorHealth = true;
boolean inDistance = false;
  
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
  Serial.printf("productID is: %d \n", boardsStruct[myData.id - 1].pid);
  Serial.printf("return x value: %d \n", boardsStruct[myData.id - 1].x);
  Serial.printf("return y value: %d \n", boardsStruct[myData.id - 1].y);
  Serial.printf("return moni value: %d \n", boardsStruct[myData.id - 1].moni);
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

static int giBattery = 0;
static int giBatteryOld = 0xFF;
double vbat = 0.0;
int8_t bat_charge_p = 0;

//void show_battery_info(){
//  // バッテリー電圧表示
//  // GetVbatData()の戻り値はバッテリー電圧のステップ数で、
//  // AXP192のデータシートによると1ステップは1.1mV
//  vbat = M5.Power.getBatteryLevel();// * 1.1 / 1000;　M5Stack専用
////  vbat = M5.Power.GetVbatData(); * 1.1 / 1000;  M5StickC専用
//  M5.Lcd.setCursor(0, 5);
//  M5.Lcd.printf("Volt: %.2fV", vbat * 1.1 / 1000);
//
//
//  // バッテリー残量表示
//  // 簡易的に、線形で4.2Vで100%、3.0Vで0%とする
//  bat_charge_p = int8_t((vbat - 3.0) / 1.2 * 100);
//  if(bat_charge_p > 100){
//    bat_charge_p = 100;
//  }else if(bat_charge_p < 0){
//    bat_charge_p = 0;
//  }
//  M5.Lcd.setCursor(0, 25);
//  M5.Lcd.printf("Charge: %3d%%", bat_charge_p);
//
//  // 液晶の明るさ表示
//  M5.Lcd.setCursor(0, 45);
//  M5.Lcd.printf("Brightness:%2d", disp_brightness);
//}

void setup()
{
  // シリアルモニターの初期化
  Serial.begin(115200);
  Wire.begin();

  // デバイスをWi-Fiステーションとして設定します
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.println(WiFi.macAddress());
  InitESPNow();

  /////////////////////////////////////////////////
  //  ESP-NOW
  /////////////////////////////////////////////////

  // ESP-NOWを初期化します
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

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    sensor.setTimeout(1000);
    M5.Lcd.setTextSize(3);
    M5.Lcd.println("Sensor NG!!");
    //   while (1) {}
  }

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
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
//  myData.id = boardID; // 他idから制御後、効かない事があるため、idをSlaveにみせる
  esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  M5.update();
  M5.Lcd.setTextColor(TFT_CYAN);            //CYAN
  LCDReflesh(disp_brightness, 26, 60, 120); // Bright, FontSize, x, y
  M5.Lcd.println("NPS");
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
  sensorHealth = true; // センサーエラーを再通知するためのおまじない
}

void loop()
{
  M5.update();

  // Set values to send
  myData.pid = productID;
  myData.id = boardID;
  myData.moni = selfMonitor;
  myData.x = random(0, 50);
  myData.y = random(0, 50);

  int distance = sensor.readRangeSingleMillimeters();
  distance;

  if (sensor.timeoutOccurred())
  {
    sensorHealth = !sensorHealth; //Serial.print(" TIMEOUT");
    sensor.setTimeout(0);
  }

  if (sensorHealth)
  {
    sensor.setTimeout(500);
    //    distance = sensor.readRangeSingleMillimeters();
    sensor.readRangeSingleMillimeters();
    Serial.println(sensor.readRangeSingleMillimeters());

    // SensorがNGから復帰した場合
    if (!sensorHealth && distance > 100 && distance <= 8190)
    {
      sensorHealth = true;
      switchedDisp();
    }

    if (distance < 250)
    {
      inDistance = true;
      M5.Lcd.setCursor(20, 10);
      M5.Lcd.setTextSize(4);
      M5.Lcd.setTextColor(RED);
      M5.Lcd.println("sens on");
//      M5.Lcd.println(selfMonitor);
      delay(330);

      //      distance = sensor.readRangeSingleMillimeters();
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
      else if (distance <= 30)
      {
        M5.Lcd.clear(GREEN);
        M5.Lcd.println("Too Near!!");
        delay(1000);
        sensor.setTimeout(3000);        
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
    if (distance > distanceMax && distance <= 8200)
    {
      sensorHealth = true;
      switchedDisp();
    }
    else
      {
      M5.Lcd.setCursor(20, 10);
      M5.Lcd.setTextSize(4);
      M5.Lcd.setTextColor(BLUE);
      M5.Lcd.println("Sensor NG!!");
      }
  }

  //  giBattery = M5.Power.getBatteryLevel();
  //  M5.Lcd.setCursor(10, 5);
  //  M5.Lcd.setTextSize(1);
  //  M5.Lcd.setTextColor(WHITE);
  //  M5.Lcd.printf("%3d \%",giBattery);
  //  giBatteryOld = giBattery;   // 前回値更新

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

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    if (result == ESP_OK)
    {
      Serial.println("Sent with success");
    }
    else
    {
      Serial.println("Error sending the data");
    }
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
}
