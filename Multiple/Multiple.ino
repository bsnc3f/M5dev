#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>

//レシーバーのMACアドレスに置き換えます
uint8_t broadcastAddress [] = {0x30,0xAE,0xA4,0x0E,0x0D,0x6C};

// Define variables to store BME280 readings to be sent
float temperature;
float humidity;
float pressure;

//入ってくる測定値を保存する変数を定義します
float incomingTemp;
float incomingHum;
float incomingPres;

//データの送信が成功した場合に保存する変数
String success;

//データを送信するための構造の例
//レシーバー構造に一致する必要があります
typedef struct struct_message {
    float temp;
    float hum;
    float pres;
} struct_message;

// BME280Readingsというstruct_messageを作成して、センサーの読み取り値を保持します
struct_message BME280Readings;

//入ってくるセンサーの読み取り値を保持するstruct_messageを作成します
struct_message incomingReadings;

//データ送信時のコールバック
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

//データ受信時のコールバック
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incomingTemp = incomingReadings.temp;
  incomingHum = incomingReadings.hum;
  incomingPres = incomingReadings.pres;
}
 
void setup(){
  //シリアルモニターの初期化
  Serial.begin(115200);
 
  //デバイスをWi-Fiステーションとして設定します
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.println(WiFi.macAddress());

  // ESP-NOWを初期化します
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // 送信CBを登録
  esp_now_register_send_cb(OnDataSent);
  
  //ピア登録
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  //ピア追加
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  //受信コールバックを登録
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop(){
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &BME280Readings, sizeof(BME280Readings));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(1000);
}
