#include <SoftwareSerial.h>
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

#include <uTimerLib.h>


#define goalpoint_Y 130.9600492  //目的地のY座標 tanekon(30.37429784,130.9600492)
#define goalpoint_X 30.37429784
//目的地のX座標

#define goal_distance 2  //ゴールと判定する距離_注意!!要測定!!値が非常に小さく,単位が不明瞭,現地で確認すべき

#define challenge 10  //モータで角度調整する回数これを越えるとスタック判定

int challenge_count = 0;

const int INA1 = 26;
const int INA2 = 27;

const int INB1 = 0;
const int INB2 = 4;

#include <math.h>
//ヒュベニの公式を使って距離を求めるためのグローバル変数
double Dy = 0.00;
double Dx = 0.00;
double P = 0.00;
double Rx = 6378137.000;     //長半径（赤道半径）
double Ry = 6356752.314245;  //短半径（極半径）
double E = 0.08181919083;    //離心率
double W = 0.00;
double M = 0.00;             //子午線曲率半径
double N = 0.00;             //卯酉曲線半径
double DirectionGPS = 0.00;  //現在位置からゴールの方角
double DistanceGPS = 0.00;   //ゴールまでの距離


#include <Wire.h>
// BMX055 加速度センサのI2Cアドレス
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Openの時)
// BMX055 ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13  // (JP1,JP2,JP3 = Openの時)

// センサーの値を保存するグローバル変数
float xAccl = 0.00;
float yAccl = 0.00;
float zAccl = 0.00;
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
int xMag = 0;
int yMag = 0;
int zMag = 0;
double xCal = 0;  //補正後の地磁気値
double yCal = 0;
double zCal = 0;
double roll = 0;       //静止時のロール角
double pitch = 0;      //静止時のピッチ角
double Azimuth = 0;    //方位角
double Azimuth_a = 0;  //x,yのみを使った方位角
double r = 0;          //地磁気値の球半径
//-----MATLABで求めた校正値を入力---
//double cal_b[3] = { 10.3168, 40.8539, -77.7989 };
//double cal_A[3] = { 0.9627, 0.9826, 1.0572 };
double cal_b[3] = { -78.3827, 129.9656, -49.5987 };
double cal_A[3] = { 0.9731, 1.0115, 1.0163 };


double Difference = 0;  //角度変換後の地磁気とGPSの相対角度



// rxPin = 2  txPin = 3
SoftwareSerial mySerial(16, 17);

// NMEAの緯度経度を「度分秒」(DMS)の文字列に変換する
String NMEA2DMS(float val) {
  int d = val / 100;
  int m = ((val / 100.0) - d) * 100.0;
  float s = ((((val / 100.0) - d) * 100.0) - m) * 60;
  return String(d) + "度" + String(m) + "分" + String(s, 1) + "秒";
}

// (未使用)NMEAの緯度経度を「度分」(DM)の文字列に変換する
String NMEA2DM(float val) {
  int d = val / 100;
  float m = ((val / 100.0) - d) * 100.0;
  return String(d) + "度" + String(m, 4) + "分";
}

// NMEAの緯度経度を「度」(DD)の文字列に変換する
String NMEA2DD(float val) {
  int d = val / 100;
  int m = (((val / 100.0) - d) * 100.0) / 60;
  float s = (((((val / 100.0) - d) * 100.0) - m) * 60) / (60 * 60);
  return String(d + m + s, 6);
}

// UTC時刻から日本の標準時刻に変換する(GMT+9:00)
String UTC2GMT900(String str) {
  int hh = (str.substring(0, 2).toInt()) + 9;
  if (hh > 24) hh = hh - 24;

  return String(hh, DEC) + ":" + str.substring(2, 4) + ":" + str.substring(4, 6);
}

void Forced_stop() {
  //停止
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, LOW);
  ledcWrite(0, 0);
  digitalWrite(INB1, LOW);
  digitalWrite(INB2, LOW);
  ledcWrite(1, 0);
  delay(1000);
  for (;;) {
    SerialBT.print("TIME UP !! STOP!!");
    digitalWrite(2, HIGH);
    delay(200);
    digitalWrite(2, LOW);
    delay(200);
  }
}

void setup() {
  SerialBT.begin("Greenlights2_ESP32");  //Bluetooth device name
  SerialBT.print("device start");
  mySerial.begin(9600);
  Serial.begin(9600);

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);


  //Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバッグ用シリアル通信は9600bps
  Serial.begin(9600);
  //BMX055 初期化
  BMX055_Init();
  delay(300);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);

  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  //pwmの設定。最初の引数がchannel,次が周波数,最後がduty比（ここでは8bit = 256段階）
  ledcSetup(0, 12800, 8);  //Aのモーター右
  ledcSetup(1, 12800, 8);  //Bのモーター左
  // ledPinをチャネル0へ接続
  ledcAttachPin(INA2, 0);
  ledcAttachPin(INB1, 1);
  //ゴール探索開始してから18分後(1080秒)に強制終了
  TimerLib.setTimeout_s(Forced_stop, 1000);
}

void loop() {
  // 1つのセンテンスを読み込む
  String line = mySerial.readStringUntil('\n');

  if (line != "") {
    int i, index = 0, len = line.length();
    String str = "";

    // StringListの生成(簡易)
    String list[30];
    for (i = 0; i < 30; i++) {
      list[i] = "";
    }

    // 「,」を区切り文字として文字列を配列にする
    for (i = 0; i < len; i++) {
      if (line[i] == ',') {
        list[index++] = str;
        str = "";
        continue;
      }
      str += line[i];
    }

    // $GPGGAセンテンスのみ読み込む
    if (list[0] == "$GPGGA") {

      // ステータス
      if (list[6] != "0") {

        SerialBT.println("--------------------------------------");

        // 現在時刻
        SerialBT.print(UTC2GMT900(list[1]));

        // 緯度
        Serial.print(" ,Lat:,");
        Serial.print(NMEA2DMS(list[2].toFloat()));
        Serial.print(",");


        // 経度
        Serial.print(" Lot:,");
        Serial.print(NMEA2DMS(list[4].toFloat()));
        Serial.print(",");


        // 海抜
        Serial.print(" ,Altitude above sea level:,");
        Serial.print(list[9]);
        list[10].toLowerCase();
        Serial.print(list[10]);



        Serial.println("");
        Serial.println("");

        //NMEA形式の値を文字列変数に代入後,float形式の変数に代入_これでGPSの情報を変数に代入できる
        String X = NMEA2DD(list[2].toFloat());
        String Y = NMEA2DD(list[4].toFloat());


        float YA = goalpoint_Y, YB = Y.toFloat();  //点A・BのY座標を宣言(-0.7071)
        float XA = goalpoint_X, XB = X.toFloat();  //点A・BのX座標を宣言

        SerialBT.print(",Lat:,");
        SerialBT.print(XB, 5);
        SerialBT.print(" ,Lot:,");
        SerialBT.print(YB, 5);
        SerialBT.print(" ,  ");

        Dy = XB * PI / 180 - (XA * PI / 180);  //２点の緯度（ラジアン）の差
        Dx = YB * PI / 180 - (YA * PI / 180);  //２点の経度（ラジアン）の差

        P = ((XB * PI / 180) + (XA * PI / 180)) / 2;  //２点の緯度の平均
        double W = sqrt(1 - E * E * sin(P) * sin(P));
        double M = Rx * (1 - E * E) / (W * W * W);  //子午線曲率半径
        double N = Rx / W;                          //卯酉曲線半径

        DistanceGPS = sqrt((Dy * M * Dy * M + Dx * N * cos(P) * Dx * N * cos(P)));  //２点の距離計算de目的地A迄の距離(m)



        // SerialBT.println(XB,6);
        // SerialBT.println(YB,6);

        // SerialBT.println(XB-XA,6);
        // SerialBT.println(YB-YA,6);
        SerialBT.print("DistanceGPS = ");  //目的地A迄の距離(m)
        SerialBT.print(DistanceGPS);

        double DirectionGPS = atan2((YB - YA), (XB - XA)) * 180.0 / PI;  //GPSの現在地と目的地の角度

        DirectionGPS += 180;  //0~180を超えると符号が変わりややこしいため0~360に角度を変換する



        SerialBT.print(" : DirectionGPS = ");  //目的地Aの方角(°）
        SerialBT.println(DirectionGPS);




        //ここでゴールを判定する
        //ゴール判定LED

        if (DistanceGPS < goal_distance) {
          for (;;) {
            SerialBT.println("STOP and GOAL!!");
            digitalWrite(2, HIGH);
            delay(200);
            digitalWrite(2, LOW);
            delay(200);
          }
        }






        //BMX055 加速度の読み取り
        BMX055_Accl();
        SerialBT.print("Accl= ");
        SerialBT.print(xAccl);
        SerialBT.print(",");
        SerialBT.print(yAccl);
        SerialBT.print(",");
        SerialBT.print(zAccl);
        SerialBT.println("");

        //BMX055 ジャイロの読み取り
        BMX055_Gyro();
        SerialBT.print("Gyro= ");
        SerialBT.print(xGyro);
        SerialBT.print(",");
        SerialBT.print(yGyro);
        SerialBT.print(",");
        SerialBT.print(zGyro);
        SerialBT.println("");

        //BMX055 磁気の読み取り
        BMX055_Mag();
        SerialBT.print("Mag= ");
        SerialBT.print(xMag);
        SerialBT.print(",");
        SerialBT.print(yMag);
        SerialBT.print(",");
        SerialBT.print(zMag);
        SerialBT.println("");



        //地磁気値の補正
        xCal = (xMag - cal_b[0]) * cal_A[0];
        yCal = (yMag - cal_b[1]) * cal_A[1];
        zCal = (zMag - cal_b[2]) * cal_A[2];



        //補正用角度を加速度から算出
        roll = atan(yAccl / zAccl);
        pitch = atan(-xAccl / (yAccl * sin(roll) + zAccl * cos(roll)));

        r = sqrt(pow(xCal, 2) + pow(yCal, 2) + pow(zCal, 2));

        //方位角算出
        Azimuth_a = atan2(yCal, xCal);
        Azimuth = atan2((zCal * sin(-roll) + yCal * cos(roll)), (xCal * cos(pitch) + yCal * sin(-roll) * sin(-pitch) - zCal * sin(-pitch) * cos(roll)));



        double DirectionMag = Azimuth * 180.0 / PI;  //地磁気の値をradから度に変換



        //0~180を超えると符号が変わりややこしいため0~360に角度を変換する
        if (DirectionMag < 0) {
          DirectionMag += 360;
        }






        for (i = 0; i <= 15; i++) {
          Difference = abs(DirectionMag - DirectionGPS);
          SerialBT.print("DirectionMag = ");  //目的地Aの方角(°）
          SerialBT.println(DirectionMag);
          SerialBT.print("DirectionGPS = ");  //目的地Aの方角(°）
          SerialBT.println(DirectionGPS);
          SerialBT.print("GOAL degree=");
          SerialBT.println(Difference);



          if (DirectionMag > DirectionGPS) {
            if (Difference >= 180) {
              //左のモーターで機体を少し右回転
              SerialBT.println("Turn Right");
              digitalWrite(INB1, HIGH);
              digitalWrite(INB2, LOW);
              ledcWrite(1, 255);
              digitalWrite(INA1, LOW);
              digitalWrite(INA2, LOW);
              ledcWrite(0, 0);
              delay(300);


            }

            else {
              //右のモーターで機体を少し左回転
              SerialBT.println("Turn Left");
              digitalWrite(INA1, LOW);
              digitalWrite(INA2, HIGH);
              ledcWrite(0, 255);
              digitalWrite(INB1, LOW);
              digitalWrite(INB2, LOW);
              ledcWrite(1, 0);
              delay(300);
            }
          }




          if (DirectionMag < DirectionGPS) {
            if (Difference >= 180) {

              //右のモーターで機体を少し左回転
              SerialBT.println("Turn Left");
              digitalWrite(INA1, LOW);
              digitalWrite(INA2, HIGH);
              ledcWrite(0, 255);
              digitalWrite(INB1, LOW);
              digitalWrite(INB2, LOW);
              ledcWrite(1, 0);
              delay(300);


            }

            else {
              //左のモーターで機体を少し右回転
              SerialBT.println("Turn Right");
              digitalWrite(INB1, HIGH);
              digitalWrite(INB2, LOW);
              ledcWrite(1, 255);
              digitalWrite(INA1, LOW);
              digitalWrite(INA2, LOW);
              ledcWrite(0, 0);
              delay(300);
            }
          }


          if (Difference >= 180) Difference = 360 - Difference;


          //モーターを止める
          //停止
          digitalWrite(INA1, LOW);
          digitalWrite(INA2, LOW);
          ledcWrite(0, 0);
          digitalWrite(INB1, LOW);
          digitalWrite(INB2, LOW);
          ledcWrite(1, 0);
          delay(2000);


          //BMX055 磁気の読み取り
          BMX055_Accl();
          BMX055_Gyro();
          BMX055_Mag();

          //地磁気値の補正
          xCal = (xMag - cal_b[0]) * cal_A[0];
          yCal = (yMag - cal_b[1]) * cal_A[1];
          zCal = (zMag - cal_b[2]) * cal_A[2];
          //補正用角度を加速度から算出
          roll = atan(yAccl / zAccl);
          pitch = atan(-xAccl / (yAccl * sin(roll) + zAccl * cos(roll)));
          r = sqrt(pow(xCal, 2) + pow(yCal, 2) + pow(zCal, 2));
          //  Azimuth_a = atan2(yCal, xCal);
          Azimuth = atan2((zCal * sin(-roll) + yCal * cos(roll)), (xCal * cos(pitch) + yCal * sin(-roll) * sin(-pitch) - zCal * sin(-pitch) * cos(roll)));
          DirectionMag = Azimuth * 180.0 / PI;


          //0~180を超えると符号が変わりややこしいため0~360に角度を変換する
          if (DirectionMag < 0) {
            DirectionMag += 360;
          }

          //GPSと地磁気の相対角度計算
          Difference = abs(DirectionMag - DirectionGPS);


          SerialBT.print("After moving Mag_degree=");
          SerialBT.println(DirectionMag);

          if (Difference >= 180) Difference = 360 - Difference;

          if (Difference < 20) {
            SerialBT.print("Goal degree=");
            SerialBT.println(Difference);

            //モーター直進数秒
            SerialBT.print("Go straight for 10s");
            digitalWrite(INA1, LOW);
            digitalWrite(INA2, HIGH);
            ledcWrite(0, 255);

            digitalWrite(INB1, HIGH);
            digitalWrite(INB2, LOW);
            ledcWrite(1, 255);
            delay(10000);
            //モーターを止める
            digitalWrite(INA1, LOW);
            digitalWrite(INA2, LOW);
            ledcWrite(0, 0);
            digitalWrite(INB1, LOW);
            digitalWrite(INB2, LOW);
            ledcWrite(1, 0);
            delay(1000);


            break;
          }


          //スタック処理
          challenge_count += 1;
          if (challenge_count >= challenge) {
            //モーター直進数秒
            SerialBT.println(" Stacked!! Go straight for 3s ");
            digitalWrite(INA1, LOW);
            digitalWrite(INA2, HIGH);
            ledcWrite(0, 255);

            digitalWrite(INB1, HIGH);
            digitalWrite(INB2, LOW);
            ledcWrite(1, 255);
            delay(3000);
            //delay(1000);
            //モーターを止める
            digitalWrite(INA1, LOW);
            digitalWrite(INA2, LOW);
            ledcWrite(0, 0);
            digitalWrite(INB1, LOW);
            digitalWrite(INB2, LOW);
            ledcWrite(1, 0);
            delay(1000);
            challenge_count = 0;
            break;
          }



          SerialBT.println("");
          SerialBT.println("");

          delay(500);
        }












        delay(500);

        SerialBT.println("");
        SerialBT.println("");
        SerialBT.println("");
        SerialBT.println("");

      } else {
        SerialBT.print("Don't catch GPS Data!!");
      }

      SerialBT.println("");
    }
  }
}







//=====================================================================================//
void BMX055_Init() {
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x0F);  // Select PMU_Range register
  Wire.write(0x03);  // Range = +/- 2g
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x10);  // Select PMU_BW register
  Wire.write(0x08);  // Bandwidth = 7.81 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x11);  // Select PMU_LPW register
  Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F);  // Select Range register
  Wire.write(0x04);  // Full scale = +/- 125 degree/s
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10);  // Select Bandwidth register
  Wire.write(0x07);  // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11);  // Select LPM1 register
  Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x83);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x01);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C);  // Select Mag register
  Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-Axis enabled
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52);  // Select Mag register
  Wire.write(16);    // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}
//=====================================================================================//
void BMX055_Accl() {
  unsigned int data[6];
  for (int i = 0; i < 6; i++) {
    Wire.beginTransmission(Addr_Accl);
    Wire.write((2 + i));  // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Accl, 1);  // Request 1 byte of data
    // Read 6 bytes of data
    // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data to 12-bits
  xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
  if (xAccl > 2047) xAccl -= 4096;
  yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
  if (yAccl > 2047) yAccl -= 4096;
  zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
  if (zAccl > 2047) zAccl -= 4096;
  xAccl = xAccl * 0.00098;  // range = +/-2g
  yAccl = yAccl * 0.00098;  // range = +/-2g
  zAccl = zAccl * 0.00098;  // range = +/-2g
}
//=====================================================================================//
void BMX055_Gyro() {
  unsigned int data[6];
  for (int i = 0; i < 6; i++) {
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((2 + i));  // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1);  // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xGyro = (data[1] * 256) + data[0];
  if (xGyro > 32767) xGyro -= 65536;
  yGyro = (data[3] * 256) + data[2];
  if (yGyro > 32767) yGyro -= 65536;
  zGyro = (data[5] * 256) + data[4];
  if (zGyro > 32767) zGyro -= 65536;

  xGyro = xGyro * 0.0038;  //  Full scale = +/- 125 degree/s
  yGyro = yGyro * 0.0038;  //  Full scale = +/- 125 degree/s
  zGyro = zGyro * 0.0038;  //  Full scale = +/- 125 degree/s
}
//=====================================================================================//
void BMX055_Mag() {
  unsigned int data[8];
  for (int i = 0; i < 8; i++) {
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i));  // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1);  // Request 1 byte of data
    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xMag = ((data[1] << 5) | (data[0] >> 3));
  if (xMag > 4095) xMag -= 8192;
  yMag = ((data[3] << 5) | (data[2] >> 3));
  if (yMag > 4095) yMag -= 8192;
  zMag = ((data[5] << 7) | (data[4] >> 1));
  if (zMag > 16383) zMag -= 32768;
}
