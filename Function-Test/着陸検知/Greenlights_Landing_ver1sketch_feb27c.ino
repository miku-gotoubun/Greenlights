//USE I2C   SA0 = GND    ADDRESS = 0x5C
//SIMPLE DISPLAY PROGRAM


#include <Wire.h>
#include<uTimerLib.h>


#define ADDRESS       0x5C /* SA0 -> GND */


#define REF_P_XL      0x08
#define REF_P_L       0x09
#define REF_P_H       0x0A
#define WHO_AM_I      0x0F
#define RES_CONF      0x10
#define CTRL_REG1     0x20
#define CTRL_REG2     0x21
#define CTRL_REG3     0x22
#define CTRL_REG4     0x23
#define INTERRUPT_CFG 0x24
#define INT_SOURCE    0x25
#define STATUS_REG    0x27
#define PRESS_OUT_XL  0x28
#define PRESS_OUT_L   0x29
#define PRESS_OUT_H   0x2A
#define TEMP_OUT_L    0x2B
#define TEMP_OUT_H    0x2C
#define FIFO_CTRL     0x2E
#define FIFO_STATUS   0x2F
#define THS_P_L       0x30
#define THS_P_H       0x31
#define RPDS_L        0x39
#define RPDS_H        0x3A

const int INA1 = 27;
const int INA2 = 14;

const int INB1 = 15;
const int INB2 = 2;


float PRESS_OFFSET  = 0;
float CURRENT_PRESS = 0;
float fdata;

int flytepin=32;
int val=0;

//float first_press=0.00;

#define MOVING_AVERAGE_SIZE 10 // 移動平均のサイズ
#define DETECTION_THRESHOLD 0.5 // 着陸検知の閾値
#define DETECTION_TIME_THRESHOLD 5000 // 着陸検知に必要な時間（ミリ秒）


unsigned long lastPressureUpdateTime = 0; // 最後に気圧が更新された時間
float pressureValues[MOVING_AVERAGE_SIZE]; // 気圧値の配列
int pressureIndex = 0; // 現在の配列の位置
float movingAverage = 0; // 移動平均


#define led 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(flytepin, INPUT);
  pinMode(led, OUTPUT);

  Wire.begin();
  WHO_ARE_YOU();//ＬＰＳ２５ＨＢ接続確認
  DEV_RESET();//ＬＰＳ２５ＨＢリセット
  SET_OFFSET();//オフセット値設定
  //Serial.println(readData(WHO_AM_I),HEX); 

val=digitalRead(flytepin);
  for(val=LOW; val==LOW; ) {
    val=digitalRead(flytepin);
  if (val==LOW) {
    Serial.println("待機中");
    delay(100);

  }
  else if(val==HIGH) {
    Serial.println("フライトピン分離");
    delay(100);
    //TimerLib.setTimeout_us(timerTask,15000000);
    break;
  }
  }
  Serial.println("フライトピン分離");
  GET_PRESS();
  //float first_press=fdata;
    delay(100);
    TimerLib.setTimeout_s(timerTask,60); //フライトピン分離から割り込み判断は1分

for(movingAverage = 0; movingAverage>=1023.2;){
    GET_PRESS();//気圧取得、表示
  Serial.print("  ");
  GET_TEMP();//気温取得、表示
  Serial.println("  ");

  updatePressure();
  
  // 移動平均を計算する
  float sum = 0;
  for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
    sum += pressureValues[i];
  }
  movingAverage = sum / MOVING_AVERAGE_SIZE;

}
delay(5000);
prachute_separate();


}
void timerTask() {
Serial.println("==============Timer ON!===================");
}

void prachute_separate() {
  Serial.println("==============prachute_separate!===================");
  Serial.println("Landing detected!");

}


void loop() {
  // put your main code here, to run repeatedly:
  
}

void SET_OFFSET()//実際の気象庁発表などの気圧公表値から、オフセットを計算、格納
{
  float SENS_DATA = 0;
  long  ldata = 0;
  Serial.println("INPUT CURRENT ATMOSPHERIC PRESSURE");
  Serial.println("     ****.*(hPa) & HIT 'ENTER' KEY");
  Serial.print  ("     ");
  CURRENT_PRESS = 1022.2;//実際の気象庁発表による現在の公表値をぶち込む
  if(CURRENT_PRESS != 0)//０でなければ以下を実行、０の場合はオフセットを設定しない
  {
    Serial.print(CURRENT_PRESS,1);
    Serial.println(" hPa");
    SENS_DATA = SENS_PRESS();//現在の周囲気圧値を測定
    PRESS_OFFSET = CURRENT_PRESS - SENS_DATA;//公表値ー周囲気圧値　（感覚的に解りやすいようにここでは正負反転した値を表示しています）
    Serial.print("OFFSET = ");
    Serial.print(PRESS_OFFSET);
    Serial.println(" hPa");
    ldata = (long)((SENS_DATA - CURRENT_PRESS) * 4096);//（周囲気圧値ー公表値）＊４０９６＝オフセットデータ
    writeData(RPDS_L,(byte)((ldata >> 8) & 0xFF));//オフセットデータの下位８ビットをＲＰＤＳ＿Ｌに格納
    writeData(RPDS_H,(byte)((ldata >> 16) & 0xFF));//オフセットデータの上位８ビットをＲＰＤＳ＿Ｈに格納
  }
}


float SENS_PRESS(void)//現在の周囲気圧を測定して返す
{
  long ldata;
  float fdata;
  ldata = (long)(readData(PRESS_OUT_H));//ＰＲＥＳＳ＿ＯＵＴ＿Ｈを読む
  ldata = (ldata << 8)| (long)(readData(PRESS_OUT_L));//ＰＲＥＳＳ＿ＯＵＴ＿Ｈを８ピット上位にシフトしてＰＲＥＳＳ＿ＯＵＴ＿Ｌを加算する
  ldata = (ldata << 8)| (long)(readData(PRESS_OUT_XL));//ＰＲＥＳＳ＿ＯＵＴ＿Ｈ、ＰＲＥＳＳ＿ＯＵＴ＿Ｌをそれぞれ８ビット上位にシフトしてＰＲＥＳＳ＿ＯＵＴ＿ＸＬを加算する
  fdata = (float)ldata /4096;//浮動小数点表記に変換した後、４０９６で除算し現在の周囲気圧をｈＰａ値で得る
  return fdata;//値を返す  
}


void GET_PRESS(void)//現在の周囲気圧を測定して、表示する
{
  //long ldata;
  
  fdata = SENS_PRESS();
  //Serial.print("Press = ");
  if(fdata <1000)Serial.print(" ");//整数部が３桁の場合、１０００の位に空白を入れる（２桁は考慮しない）
  Serial.print(fdata);//現在の周囲気圧を表示する（オフセットが考慮された値）
  //Serial.print(" hPa");
}


void GET_TEMP(void)//気温を測定、表示する
{
  int idata;
 // float fdata;
  Serial.print("Temp = ");
  idata = (int)readData(TEMP_OUT_H);//ＴＥＭＰ＿ＯＵＴ＿Ｈを読む
  idata = (idata <<8 )| (int)(readData(TEMP_OUT_L));//ＴＥＭＰ＿ＯＵＴ＿Ｈを８ビット上位にシフトしてＴＥＭＰ＿ＯＵＴ＿Ｌを加算する
 //fdata = 42.5 + ((float)idata / 480);//摂氏に変換
 //Serial.print(fdata);//温度を表示する
    Serial.print((float)idata/4096);//温度の表示がおかしい買ったので↑適当に割ってみた
  Serial.print(" deg.");
}


void DEV_RESET(void)//ＬＰＳ２５ＨＢリセットルーティン
{
  writeData(CTRL_REG2,0x04);//ソフトウェアリセット
  while(readData(CTRL_REG2));//リセット確認
  delay(100);
  writeData(CTRL_REG2,0x80);//リブートメモリ
  while(readData(CTRL_REG2));//リブート確認
  delay(100);
  writeData(CTRL_REG1,0xC2);//アクティブモード、アウトプット１Ｈｚ(0x92)→アクティブモード、アウトプット25Ｈｚ(0xC2)
  delay(100);
}


void WHO_ARE_YOU(void)//デバイス確認
{
  if(readData(WHO_AM_I) == 0xBD)//ＷＨＯ＿ＡＭ＿Ｉ読み込み。デバイスが確認できれば以下を実行
  {
    Serial.println("LPS25HB IS READY! (^^)");
  }
  else//デバイスが確認できなければ以下を実行
  {
    Serial.println("LPS25HB IS NOT READY....orz");
    Serial.println("PREASE CHECK DEVICE CONNECTION");
    Serial.println(" & RESET ");
    while(1);
  }
}


void writeData(byte reg, byte data)//Ｉ２Ｃ　１バイト書き込み
{
  Wire.beginTransmission(ADDRESS);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}


byte readData(byte reg)//Ｉ２Ｃ　１バイト読み出し
{
  Wire.beginTransmission(ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDRESS,1);
  return Wire.read();
}

void updatePressure() {
  // 新しい気圧値を取得して配列に追加する
  GET_PRESS();
  pressureValues[pressureIndex] = fdata;
  
  // 配列のインデックスを更新する
  pressureIndex = (pressureIndex + 1) % MOVING_AVERAGE_SIZE;

}
