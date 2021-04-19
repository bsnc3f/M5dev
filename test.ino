#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <M5Atom.h>
// LED Matrixの明るさ（0～100）
uint8_t brightness = 10;
int moni = 0;

//レシーバーのMACアドレスに置き換えます
uint8_t broadcastAddress[] = {0x30, 0xAE, 0xA4, 0x0E, 0x0D, 0x6C};
const int BoadID = 1; // ボードID

// File URLC:/Github/ESP32/M5/AtomPixTool/M5Atom/output/N123.c
const unsigned char image_N123[302] =
    {
        /* width  020 */ 0x14,
        /* height 005 */ 0x05,
        /* Line   000 */ 0x00, 0xaa, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, //
        /* Line   001 */ 0x00, 0xaa, 0xff, 0x00, 0xaa, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //
        /* Line   002 */ 0x00, 0xaa, 0xff, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xff, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //
        /* Line   003 */ 0x00, 0xaa, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xff, 0x00, 0xaa, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //
        /* Line   004 */ 0x00, 0xaa, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, //
};                                                                                                                                                                                                                                                                                                                                                                                               //M5.dis.displaybuff((uint8_t*)image_N123, -5, 0);

// Struct_messageへラッピング
typedef struct struct_message
{
  int id; // must be unique for each sender board
  int x;
  int y;
  int moni;
} struct_message;

//Create a struct_message called myData
struct_message myData;

// レシーブ用
// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;
struct_message board3;

// レシーブ用
// Create an array with all the structures
struct_message boardsStruct[3] = {board1, board2, board3};

// データ送信時のコールバック
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
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
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  boardsStruct[myData.id - 1].x = myData.x;
  boardsStruct[myData.id - 1].y = myData.y;
  boardsStruct[myData.id - 1].moni = myData.moni;
  Serial.printf("return x value: %d \n", boardsStruct[myData.id - 1].x);
  Serial.printf("return y value: %d \n", boardsStruct[myData.id - 1].y);
  Serial.printf("return moni value: %d \n", boardsStruct[myData.id - 1].moni);
  Serial.println();

  if (myData.id - 1 != BoadID)
  { // CallBackのボードIDが自分でなければ
    switchedDisp();
  }
}

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

void setup()
{
  // シリアルモニターの初期化
  Serial.begin(115200);

  // デバイスをWi-Fiステーションとして設定します
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.println(WiFi.macAddress());
  InitESPNow();

  M5.begin(true, false, true); // SerialEnable = true, I2CEnable = false, DisplayEnable = true);
  delay(10);
  M5.dis.setBrightness(brightness);
  M5.dis.displaybuff((uint8_t *)image_N123, 0, 0);
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
}

void switchedDisp()
{
  switch (myData.moni)
  {
  case 0:
    M5.dis.displaybuff((uint8_t *)image_N123, 0, 0);
    break;
  case 1:
    M5.dis.displaybuff((uint8_t *)image_N123, -5, 0);
    break;
  case 2:
    M5.dis.displaybuff((uint8_t *)image_N123, -10, 0);
    break;
  case 3:
    M5.dis.displaybuff((uint8_t *)image_N123, -15, 0);
    break;
  default:
    break;
  }
  M5.update();
}

void loop()
{

  M5.update();

  if (M5.Btn.wasPressed())
  {
    switch (moni)
    {
    case 0:
      M5.dis.displaybuff((uint8_t *)image_N123, 0, 0);
      break;
    case 1:
      M5.dis.displaybuff((uint8_t *)image_N123, -5, 0);
      break;
    case 2:
      M5.dis.displaybuff((uint8_t *)image_N123, -10, 0);
      break;
    case 3:
      M5.dis.displaybuff((uint8_t *)image_N123, -15, 0);
      break;
    default:
      break;
    }
    // Set values to send
    myData.x = random(0, 50);
    myData.y = random(0, 50);
    myData.id = BoadID;
    myData.moni = moni;

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
    Serial.println(moni);
    moni++;
    if (moni >= 4)
    {
      moni = 0;
    }
  }
  else if (M5.Btn.pressedFor(600))
  {
    myData.moni = 0;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    M5.dis.displaybuff((uint8_t *)image_N123, 0, 0);
    switchedDisp();

    while (!M5.Btn.wasReleased())
    { // ボタンが離されるまで待つ
      delay(50);
      M5.update();
      moni = 1;
    }
  }
}
