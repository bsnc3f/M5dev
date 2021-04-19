#include <M5Stack.h>

#define STS_OFF 0
#define STS_ON  1

// 図形
const unsigned short guspLight_lev[10][4] = 
{
    // x,    y,     width,  height
    {  35,  160,    20,     20  }, 
    {  60,  150,    20,     30  },
    {  85,  140,    20,     40  },
    {  110, 130,    20,     50  },
    {  135, 120,    20,     60  },
    {  160, 110,    20,     70  },
    {  185, 100,    20,     80  },
    {  210, 90,     20,     90  },
    {  235, 80,     20,     100 },
    {  260, 70,     20,     110 }
};

static unsigned char gucUpdateSts = 0;
static unsigned char gucBackLightCnt = 0;

void setup() {
  // put your setup code here, to run once:
  // M5Stackの初期化
  M5.begin();

  // 文字サイズを変更
  M5.Lcd.setTextSize(2);
  // Aボタン　カウンタ表示
  M5.Lcd.setCursor(50,0);
  M5.Lcd.printf("Button A : bright");
  // Cボタン　カウンタ表示
  M5.Lcd.setCursor(50,40);
  M5.Lcd.printf("Button C : dark");
  // 初期化
  gucBackLightCnt = 5;
  // 表示を更新
  gucUpdateSts = STS_ON;
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned char ucCnt = 0;

  M5.update();
  // Aボタン
  if (M5.BtnA.wasPressed()) {
    // 表示を更新
    gucUpdateSts = STS_ON;
    // カウンタ更新
    if(gucBackLightCnt < 9) {
        gucBackLightCnt++;
    }
  }
  // Bボタン
  if (M5.BtnB.wasPressed()) {
    ;
  }
  // Cボタン
  if (M5.BtnC.wasPressed()) {
    // 表示を更新
    gucUpdateSts = STS_ON;
    // カウンタ更新
    if(gucBackLightCnt > 0) {
        gucBackLightCnt--;
    }
  }

   // 表示更新
  if( STS_ON == gucUpdateSts ){
    // 10段階
    for( ucCnt = 0; ucCnt < 10; ucCnt++ )
    {
        if(gucBackLightCnt >= ucCnt){
            M5.Lcd.fillRect( guspLight_lev[ucCnt][0], guspLight_lev[ucCnt][1], guspLight_lev[ucCnt][2], guspLight_lev[ucCnt][3],  WHITE );
            M5.Lcd.drawRect( guspLight_lev[ucCnt][0], guspLight_lev[ucCnt][1], guspLight_lev[ucCnt][2], guspLight_lev[ucCnt][3],  WHITE );
        } else {
            M5.Lcd.fillRect( guspLight_lev[ucCnt][0], guspLight_lev[ucCnt][1], guspLight_lev[ucCnt][2], guspLight_lev[ucCnt][3],  BLACK );
            M5.Lcd.drawRect( guspLight_lev[ucCnt][0], guspLight_lev[ucCnt][1], guspLight_lev[ucCnt][2], guspLight_lev[ucCnt][3],  WHITE );
        }
    }
    // 表示更新終了
    gucUpdateSts = STS_OFF;
    // バックライト設定
    M5.Lcd.setBrightness(25 * (gucBackLightCnt +1));
  }
}
