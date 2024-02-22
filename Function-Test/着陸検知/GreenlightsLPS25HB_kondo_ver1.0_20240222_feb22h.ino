//USE I2C   SA0 = GND    ADDRESS = 0x5C
//SIMPLE DISPLAY PROGRAM


#include <Wire.h>


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


float PRESS_OFFSET  = 0;
float CURRENT_PRESS = 0;
float fdata;

int flytepin=32;
int val=0;

#define MOVING_AVERAGE_SIZE 10 // 移動平均のサイズ
#define DETECTION_THRESHOLD 0.5 // 着陸検知の閾値
#define DETECTION_TIME_THRESHOLD 5000 // 着陸検知に必要な時間（ミリ秒）

unsigned long lastPressureUpdateTime = 0; // 最後に気圧が更新された時間
float pressureValues[MOVING_AVERAGE_SIZE]; // 気圧値の配列
int pressureIndex = 0; // 現在の配列の位置
float movingAverage = 0; // 移動平均

void setup()
{
  Serial.begin(9600);
  pinMode(flytepin, INPUT);

  Wire.begin();
  WHO_ARE_YOU();//ＬＰＳ２５ＨＢ接続確認
  DEV_RESET();//ＬＰＳ２５ＨＢリセット
  SET_OFFSET();//オフセット値設定
    Serial.println(readData(WHO_AM_I),HEX);
}


float RX_ALL(void)//キーボードからの手入力ルーティン（整数＋小数１桁）数値を入力されずに改行された場合は０を返す
{
  float rx_result = 0;
  byte  rx_flag   = 0;
  byte  rx_data   = 0;
  byte  under_dp  = 0;
  while(rx_flag == 0)//受信終了フラグが立つまで以下を実行
  {
    if(Serial.available() > 0)//文字があれば以下を実行
    {
      rx_data = Serial.read();//文字取得
      if((rx_data == 0x0D)||(rx_data == 0x0A))rx_flag = 1;//「復帰」または「改行」ならば受信終了フラグを立てる
      else//文字なら以下を実行
      {
        if((rx_data >= '0')&&(rx_data <= '9'))//数字ならば以下を実行
        {
          if(under_dp == 0)//小数点以下でなければ以下を実行
          {
            rx_result = rx_result * 10 + (float)(rx_data - 0x30);//受信文字を数字に変換して、受信毎に桁を上げる
          }
          else//小数点以下であれば以下を実行
          {
            rx_result = rx_result + ((float)(rx_data - 0x30))/10;//受信文字を数字に変換して、１／１０して加算
            rx_flag = 1;//受信終了フラグを立てる
          }
        }
        else//数字でなければ以下を実行
        {
          if(rx_data == '.')//小数点であれば以下を実行
          {
            under_dp = 1;//小数点フラグを立て、以後の数字は小数点以下の値と判断
          }
        }
      }
    }
  }
  rx_data = Serial.read();//復帰または改行を全て受信するためのＤＵＭＭＹ
  return rx_result;//受け取った数字を返す
}


void SET_OFFSET()//実際の気象庁発表などの気圧公表値から、オフセットを計算、格納
{
  float SENS_DATA = 0;
  long  ldata = 0;
  Serial.println("INPUT CURRENT ATMOSPHERIC PRESSURE");
  Serial.println("     ****.*(hPa) & HIT 'ENTER' KEY");
  Serial.print  ("     ");
  CURRENT_PRESS = RX_ALL();//手入力された現在の公表値を得る
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
  //Serial.print(fdata);//現在の周囲気圧を表示する（オフセットが考慮された値）
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
  
  // 最後に気圧が更新された時間を記録する
  lastPressureUpdateTime = millis();
}

bool isLandingDetected() {
  // 最後の気圧値と現在の移動平均との差を計算し、閾値より小さい場合に着陸検知とする
  return abs(fdata - movingAverage) < DETECTION_THRESHOLD;
}

void loop()
{
  delay(200);
  Serial.print("Press = ");
  GET_PRESS();//気圧取得、表示
  Serial.print(fdata);//現在の周囲気圧を表示する（オフセットが考慮された値）
  Serial.print(" hPa");
  Serial.print("  ");
  GET_TEMP();//気温取得、表示
  Serial.print("  ");

  
 
  val=digitalRead(flytepin);
  if (val==LOW) {
    Serial.println("待機中");
    //delay(100);

  }
  else if(val==HIGH) {
    Serial.println("フライトピン分離");
    //delay(100);

    updatePressure();
  
  // 移動平均を計算する
  float sum = 0;
  for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
    sum += pressureValues[i];
  }
  movingAverage = sum / MOVING_AVERAGE_SIZE;
  
  // 気圧の変化を監視し、一定時間変化が見られなければ着陸したと判断する
  unsigned long currentTime = millis();
  if (currentTime - lastPressureUpdateTime > DETECTION_TIME_THRESHOLD && isLandingDetected()) {
    Serial.println("Landing detected!");
    // 何らかの処理を行う

  }
  }
  
}


