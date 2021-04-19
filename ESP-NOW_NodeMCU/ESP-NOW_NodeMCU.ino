#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>

#define bLED 32  // OnRecive LED
#define rLED 33  // PeerRevice LED
#define make1 14  // make1 pin
#define make2 27  // make2 pin
#define make3 25  // make2 pin
#define make4 26  // make3 pin

const int maketime = 250;  // makeタイム
int dPin[32];

// レシーバーのMACアドレスに置き換えます
uint8_t broadcastAddress [] = {0x50,0x02,0x91,0x89,0xf9,0x6c};

// CallBack用ブロードキャストMACアドレス
uint8_t gAddress [] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
const int pid = 202008;

int reserveNum = 0;
int ignoreNum = 1;
int moni = 0;
const int maxMoni = 4; // モニタ入力の最大値
const int maxID = 5;


typedef struct struct_message {
  int id;
  int x;
  int y;
  int moni;
  int productID;
}struct_message;

// Create a struct_message called myData
struct_message myData;

// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;
struct_message board3;
struct_message board4;
struct_message board5;

// Create an array with all the structures
struct_message boardsStruct[5] = {board1, board2, board3, board4, board5};


//データ受信時のコールバック
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  boardsStruct[myData.id-1].productID = myData.productID;
  boardsStruct[myData.id-1].x = myData.x;
  boardsStruct[myData.id-1].y = myData.y;
  boardsStruct[myData.id-1].moni = myData.moni;
  Serial.printf("productID is: %d \n", boardsStruct[myData.id-1].productID);
  Serial.printf("x value: %d \n", boardsStruct[myData.id-1].x);
  Serial.printf("y value: %d \n", boardsStruct[myData.id-1].y);
  Serial.printf("moni value: %d \n", boardsStruct[myData.id-1].moni);
  Serial.println();
  cbLED(bLED,1,20);  // LED No, 回数, Delay量
  
  // 送信元の照合
  if (myData.productID == pid && myData.id <= maxID && myData.moni <= maxMoni){
    reserveNum++;
    esp_now_send(gAddress, incomingData, len);
        //  esp_now_send(gAddress, (uint8_t *) &myData, sizeof(myData));
        //  esp_err_t result = esp_now_send(gAddress, (uint8_t *) &myData, sizeof(myData));
    cbLED(bLED,1,20);  // LED No, 回数, Delay量

      if (reserveNum <= 1) {
       if (myData.moni != moni-1){
          monitor_select(myData.moni);
          cbLED(rLED,2,20);  // LED No, 回数, Delay量    
         }
       else{
         }
        }

      else {
          Serial.print("Reserve Num is Over ");
          Serial.print(reserveNum);
          delay(2000);
          reserveNum--;
      }
  }
}


//データ送信時のコールバック
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nCallback Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.println();
  reserveNum--;
}

// ESP初期化処理
void InitESPNow() {
    // ESP-NOW初期化
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {  //esp_err_t esp_now_init（ void ） ESPNOW初期化
  }
  else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

void cbLED(int LEDs, int num, int dly) {
  for(int i=0; i<num; i++){
  digitalWrite(LEDs, HIGH);
  delay(dly);
  digitalWrite(LEDs, LOW);
  delay(dly);
  }
}

void monitor_select(int selectMonitor) {
  switch (selectMonitor)
    {
    case 0:
    Serial.println("現在のモニター入力は　SDI_A です");
      digitalWrite(make1, HIGH);
      digitalWrite(make2, LOW);
      digitalWrite(make3, LOW);
      digitalWrite(make4, LOW);
     break;
    case 1:
    Serial.println("現在のモニター入力は　SDI_B です");
      digitalWrite(make1, LOW);
      digitalWrite(make2, HIGH);
      digitalWrite(make3, LOW);
      digitalWrite(make4, LOW);
     break;
    case 2:
    Serial.println("現在のモニター入力は　SDI_C です");
      digitalWrite(make1, LOW);
      digitalWrite(make2, LOW);
      digitalWrite(make3, HIGH);
      digitalWrite(make4, LOW);
     break;
    case 3:
    Serial.println("現在のモニター入力は　SDI_D です");
      digitalWrite(make1, LOW);
      digitalWrite(make2, LOW);
      digitalWrite(make3, LOW);
      digitalWrite(make4, HIGH);
     break;
    case 4:
     break;
    default:
     break;
    }
}

void setup(){
  // PIN_NUM で全デジタル出力をLOWにセット
  for (byte i = 0; i < 33; i++)
  {
    pinMode(dPin[i], OUTPUT);
    digitalWrite(dPin[i], LOW);
  }
  
  pinMode(bLED, OUTPUT);
  pinMode(rLED, OUTPUT);
  digitalWrite(bLED, LOW);
  digitalWrite(rLED, LOW);
  pinMode(make1, OUTPUT);  
  pinMode(make2, OUTPUT);  
  pinMode(make3, OUTPUT);  
  pinMode(make4, OUTPUT);  
  digitalWrite(make1, HIGH);
  digitalWrite(make2, LOW);
  digitalWrite(make3, LOW);
  digitalWrite(make4, LOW);
  
  
  //シリアルモニターの初期化
  Serial.begin(115200);
 
  //デバイスをWi-Fiステーションとして設定します
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.println(WiFi.macAddress());
  InitESPNow();
  
  // ESP-NOWを初期化します
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
    Serial.println(WiFi.macAddress());
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }

  //ピア登録
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, gAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  //ピア追加
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // 送信コールバックを登録
  esp_now_register_send_cb(OnDataSent);

  //受信コールバックを登録
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop(){

  myData.id = 0;
  myData.x = random(0,50);
  myData.y = random(0,50);
//  myData.moni = moni;

  if (reserveNum != 0){
    delay(100);
    }
  
  // Acess the variables for each board
  /*int board1X = boardsStruct[0].x;
  int board1Y = boardsStruct[0].y;
  int board2X = boardsStruct[1].x;
  int board2Y = boardsStruct[1].y;
  int board3X = boardsStruct[2].x;
  int board3Y = boardsStruct[2].y;*/

}
