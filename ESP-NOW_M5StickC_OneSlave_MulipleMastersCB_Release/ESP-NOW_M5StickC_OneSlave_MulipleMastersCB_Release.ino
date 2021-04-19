#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <M5StickC.h>

//////////////////////////////////
//レシーバーのMACアドレスに置き換えます
//////////////////////////////////

uint8_t broadcastAddress[] = {0x30, 0xAE, 0xA4, 0x0E, 0x0D, 0x6C};

const uint boardID = 3; // ボードID
const uint productID = 202008;
const uint maxMoni = 4; // モニタ入力の最大値
uint selfMonitor = 0;
boolean is_sleep = false;

#define BTN_B_PIN 39 //A = 37 B = 39
#define BTN_ON LOW
#define BTN_OFF HIGH
uint8_t prev_btn_b = BTN_OFF;
uint8_t btn_b = BTN_OFF;

#define PWR_BTN_STABLEL 0     // 押していない or 1秒以上押し続ける
#define PWR_BTN_LONG_PRESS 1  // 1秒の長押し発生
#define PWR_BTN_SHORT_PRESS 2 // 1秒以下のボタンクリック

// LED Matrixの明るさ（0～100）
#define DISP_BRIGHTNESS_MIN 8  // 0〜6もセット可能だが、何も見えない
#define DISP_BRIGHTNESS_MAX 12 //
uint8_t disp_brightness = DISP_BRIGHTNESS_MIN;
uint rotete = 3;

double vbat = 0.0;
int8_t bat_charge_p = 0;

// Struct_messageへラッピング
typedef struct struct_message
{
  int pid = productID;
  int id = boardID;
  int moni;
  int x;
  int y;
} struct_message;

//Create a struct_message called myData
struct_message myData;

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
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  Serial.printf("productID is: %d \n", boardsStruct[myData.id - 1].pid);
  Serial.printf("return x value: %d \n", boardsStruct[myData.id - 1].x);
  Serial.printf("return y value: %d \n", boardsStruct[myData.id - 1].y);
  Serial.printf("return moni value: %d \n", boardsStruct[myData.id - 1].moni);
  Serial.println();

  if (myData.pid == productID && myData.id != boardID && myData.x <= 50)
  {
    // CallBackのボードIDが自分でなければ
    switchedDisp();
  }
}

void show_battery_info()
{
  // バッテリー電圧表示
  // GetVbatData()の戻り値はバッテリー電圧のステップ数で、
  // AXP192のデータシートによると1ステップは1.1mV
  vbat = M5.Axp.GetVbatData() * 1.1 / 1000;
  M5.Lcd.setCursor(0, 5);
  M5.Lcd.printf("Volt: %.2fV", vbat);

  // バッテリー残量表示
  // 簡易的に、線形で4.2Vで100%、3.0Vで0%とする
  bat_charge_p = int8_t((vbat - 3.0) / 1.2 * 100);
  if (bat_charge_p > 100)
  {
    bat_charge_p = 100;
  }
  else if (bat_charge_p < 0)
  {
    bat_charge_p = 0;
  }
  M5.Lcd.setCursor(0, 25);
  M5.Lcd.printf("Charge: %3d%%", bat_charge_p);

  // 液晶の明るさ表示
  M5.Lcd.setCursor(0, 45);
  M5.Lcd.printf("Brightness:%2d", disp_brightness);
}

void setup()
{
  // シリアルモニターの初期化
  Serial.begin(115200);
  Wire.begin(); // バッテリー残量表示で使用

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
    //    M5.Lcd.print("ESPNow Init Success\n");
    Serial.println(WiFi.macAddress());
  }
  else
  {
    Serial.println("ESPNow Init Failed");
    //    M5.Lcd.print("ESPNow Init Failed\n");
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

  // 送信コールバックを登録
  esp_now_register_send_cb(OnDataSent);

  // 受信コールバックを登録
  esp_now_register_recv_cb(OnDataRecv);

  /////////////////////////////////////////////////
  //  M5StickC専用
  /////////////////////////////////////////////////

  M5.begin();                       // オプションをつけるとLCDのトラブルが出る
                                    //  M5.begin(true, false, true); // SerialEnable = true, I2CEnable = false, DisplayEnable = true);
  pinMode(BTN_B_PIN, INPUT_PULLUP); // M5StickC
  pinMode(M5_LED, OUTPUT);          // LED ON(GPIO_NUM_10 or M5_LED)
  delay(50);
  digitalWrite(GPIO_NUM_10, HIGH);

  // LCD display
  M5.Lcd.setRotation(3); // ボタンBが上になる向き
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextFont(4); // 4 = 26ピクセルASCIIフォント
  M5.Lcd.setTextSize(1);
  M5.Axp.ScreenBreath(10); // 液晶の明るさ設定

  /////////////////////////////////////////////////
  // バッテリー情報表示
  /////////////////////////////////////////////////
  show_battery_info();
}

void switchedDisp()
{
  switch (myData.moni)
  {
  case 0:
    M5.Lcd.setTextColor(TFT_CYAN);         //CYAN
    LCDReflesh(disp_brightness, 3, 3, 10); // Bright, FontSize, x, y
    M5.Lcd.println("NPS");
    break;
  case 1:
    M5.Lcd.setTextColor(YELLOW);
    LCDReflesh(disp_brightness, 3, 55, 10); // Bright, FontSize, x, y
    M5.Lcd.println("C");
    break;
  case 2:
    M5.Lcd.setTextColor(YELLOW);
    LCDReflesh(disp_brightness, 3, 55, 10); // Bright, FontSize, x, y
    M5.Lcd.println("B");
    break;
  case 3:
    M5.Lcd.setTextColor(YELLOW);
    LCDReflesh(disp_brightness, 3, 55, 10); // Bright, FontSize, x, y
                                            //    LCDReflesh(80,26,90,120);  // Bright, FontSize, x, y
    M5.Lcd.println("A");
    break;
  default:
    break;
  }
}

void LCDReflesh(int lBright, int tSize, int xCursor, int yCursor)
{
  M5.Lcd.fillScreen(BLACK); //  M5StackはM5.Lcd.clear(BLACK);
  M5.Lcd.setTextSize(tSize);
  M5.Lcd.setCursor(xCursor, yCursor);
  M5.Lcd.setTextFont(4); // 4 = 26ピクセルASCIIフォント
}

void loop()
{
  M5.update();

  if (M5.BtnA.wasPressed())
  {
    switch (selfMonitor)
    {
    case 0:
      M5.Lcd.setTextColor(TFT_CYAN);         //CYAN
      LCDReflesh(disp_brightness, 3, 3, 10); // Bright, FontSize, x, y
      M5.Lcd.println("NPS");
      break;
    case 1:
      M5.Lcd.setTextColor(YELLOW);
      LCDReflesh(disp_brightness, 3, 55, 10); // Bright, FontSize, x, y
      M5.Lcd.println("C");
      break;
    case 2:
      M5.Lcd.setTextColor(YELLOW);
      LCDReflesh(disp_brightness, 3, 55, 10); // Bright, FontSize, x, y
      M5.Lcd.println("B");
      break;
    case 3:
      M5.Lcd.setTextColor(YELLOW);
      LCDReflesh(disp_brightness, 3, 55, 10); // Bright, FontSize, x, y
                                              //    LCDReflesh(80,26,90,120);  // Bright, FontSize, x, y
      M5.Lcd.println("A");
      break;
    default:
      break;
    }

    // Set values to send
    myData.pid = productID;
    myData.id = boardID;
    myData.moni = selfMonitor;
    myData.x = random(0, 50);
    myData.y = random(0, 50);

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
    digitalWrite(GPIO_NUM_10, LOW);
    delay(50);
    digitalWrite(GPIO_NUM_10, HIGH);
  }
  else if (M5.BtnA.pressedFor(600))
  {
    myData.moni = 0;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    M5.Lcd.setTextColor(TFT_CYAN);         //CYAN
    LCDReflesh(disp_brightness, 3, 3, 10); // Bright, FontSize, x, y
    M5.Lcd.println("NPS");
    while (!M5.BtnA.wasReleased())
    { // ボタンが離されるまで待つ
      delay(50);
      M5.update();
      selfMonitor = 1;
    }
  }

  btn_b = digitalRead(BTN_B_PIN);

  if (prev_btn_b == BTN_OFF && btn_b == BTN_ON)
  {
    // ボタンBが押されたとき。1回ごとにディスプレイの明るさを上げる。
    // 最小7、最大12。12を超えたら0に戻す
    disp_brightness += 1;
    if (disp_brightness > DISP_BRIGHTNESS_MAX)
    {
      disp_brightness = DISP_BRIGHTNESS_MIN;
    }
    M5.Axp.ScreenBreath(disp_brightness);
    LCDReflesh(7, 1, 0, 0);
    M5.Lcd.setCursor(0, 45);
    M5.Lcd.printf("Brightness:%2d", disp_brightness);
    delay(500);
  }

  if (M5.Axp.GetBtnPress() == PWR_BTN_LONG_PRESS)
  { //  左側の釦が押されたら　戻り値0 その他, 1 1秒以上押した, 2 1秒未満のクリック, *0以外は1度のみ返却
    // 電源ボタンの長押し
    if (is_sleep)
    {
      !is_sleep;
    }
    else
    {
      //    M5.Axp.SetSleep(); // 画面が消えるだけのスリープに入る
    }
  }
  else if (M5.Axp.GetBtnPress() == PWR_BTN_SHORT_PRESS)
  {

    // LCD display
    if (rotete = 3)
    {
      rotete = 1;
    }
    //    else if (rotete = 1){
    //      rotete = 3;
    //      }

    M5.Lcd.setRotation(rotete); // rotete

    digitalWrite(GPIO_NUM_10, LOW);
    delay(50);
    digitalWrite(GPIO_NUM_10, HIGH);

    //    M5.update();
    //    is_sleep = !is_sleep;
  }
}
