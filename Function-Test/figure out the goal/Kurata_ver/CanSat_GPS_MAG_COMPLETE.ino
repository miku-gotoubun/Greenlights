#include <SoftwareSerial.h>

#define goalpoint_Y 133.719696  //目的地のY座標
#define goalpoint_X 33.620403   //目的地のX座標

#define goal_distance 12345  //ゴールと判定する距離_注意!!要測定!!値が非常に小さく,単位が不明瞭,現地で確認すべき

#define challenge 30  //モータで角度調整する回数これを越えるとスタック判定

int challenge_count = 0;


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
double cal_b[3] = { 10.3168, 40.8539, -77.7989 };
double cal_A[3] = { 0.9627, 0.9826, 1.0572 };


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

void setup() {
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

        Serial.println("--------------------------------------");

        // 現在時刻
        Serial.print(UTC2GMT900(list[1]));

        // 緯度
        Serial.print(" 緯度:");
        Serial.print(NMEA2DMS(list[2].toFloat()));
        Serial.print("(");
        Serial.print(NMEA2DD(list[2].toFloat()));
        Serial.print(")");

        // 経度
        Serial.print(" 経度:");
        Serial.print(NMEA2DMS(list[4].toFloat()));
        Serial.print("(");
        Serial.print(NMEA2DD(list[4].toFloat()));
        Serial.print(")");

        // 海抜
        Serial.print(" 海抜:");
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


        // Serial.println(XB,6);
        // Serial.println(YB,6);

        // Serial.println(XB-XA,6);
        // Serial.println(YB-YA,6);
        Serial.print("DistanceGPS = ");  //目的地A迄の距離(m)
        Serial.println(sqrt(pow(XB - XA, 2) + pow(YB - YA, 2)), 11);
        //Serial.print(" : DirectionGPS = ");  //目的地Aの方角(°）
        // Serial.println(atan2((YB - YA), (XB - XA)) * 180.0 / PI);


        double DirectionGPS = atan2((YB - YA), (XB - XA)) * 180.0 / PI;  //GPSの現在地と目的地の角度
        double DistanceGPS = sqrt(pow(XB - XA, 2) + pow(YB - YA, 2));    //GPSの現在地と目的地の距離



        DirectionGPS += 180;  //0~180を超えると符号が変わりややこしいため0~360に角度を変換する



        Serial.print(" : DirectionGPS = ");  //目的地Aの方角(°）
        Serial.println(DirectionGPS);




        //ここでゴールを判定する
        //ゴール判定LED

        if (DistanceGPS < goal_distance) {
          for (;;) {
            digitalWrite(2, HIGH);
            delay(200);
            digitalWrite(2, LOW);
            delay(200);
          }
        }






        //BMX055 加速度の読み取り
        BMX055_Accl();
        // Serial.print("Accl= ");
        // Serial.print(xAccl);
        // Serial.print(",");
        // Serial.print(yAccl);
        // Serial.print(",");
        // Serial.print(zAccl);
        // Serial.println("");

        //BMX055 ジャイロの読み取り
        BMX055_Gyro();
        // Serial.print("Gyro= ");
        // Serial.print(xGyro);
        // Serial.print(",");
        // Serial.print(yGyro);
        // Serial.print(",");
        // Serial.print(zGyro);
        // Serial.println("");

        //BMX055 磁気の読み取り
        BMX055_Mag();
        //Serial.print("Mag= ");
        // Serial.print(xMag);
        // Serial.print(",");
        // Serial.print(yMag);
        // Serial.print(",");
        // Serial.print(zMag);
        // Serial.println("");



        //地磁気値の補正
        xCal = (xMag - cal_b[0]) * cal_A[0];
        yCal = (yMag - cal_b[1]) * cal_A[1];
        zCal = (zMag - cal_b[2]) * cal_A[2];

        // Serial.print("cal= ");
        // Serial.print(xCal);
        // Serial.print(",");
        // Serial.print(yCal);
        // Serial.print(",");
        // Serial.println(zCal);
        // Serial.println("");

        //補正用角度を加速度から算出
        roll = atan(yAccl / zAccl);
        pitch = atan(-xAccl / (yAccl * sin(roll) + zAccl * cos(roll)));
        // Serial.print("roll=");
        // Serial.println(roll);
        // Serial.print("pitch=");
        // Serial.println(pitch);
        r = sqrt(pow(xCal, 2) + pow(yCal, 2) + pow(zCal, 2));
        // Serial.print("r=");
        // Serial.println(r);
        //方位角算出
        Azimuth_a = atan2(yCal, xCal);
        Serial.print("方位角(平面)_DirectionMag= ");
        //Serial.println(Azimuth_a);
        // Azimuth= atan2((zCal*sin(roll)+yCal*cos(roll)),(xCal*cos(pitch)+yCal*sin(roll)*sin(pitch)-zCal*sin(pitch)*cos(roll)));
        Azimuth = atan2((zCal * sin(-roll) + yCal * cos(roll)), (xCal * cos(pitch) + yCal * sin(-roll) * sin(-pitch) - zCal * sin(-pitch) * cos(roll)));
        //Serial.println(Azimuth);


        double DirectionMag = Azimuth * 180.0 / PI;  //地磁気の値をradから度に変換
        Serial.println(DirectionMag);


        //0~180を超えると符号が変わりややこしいため0~360に角度を変換する
        if (DirectionMag < 0) {
          DirectionMag += 360;
        }


        // DirectionMag = (DirectionMag + 360) % 360;
        // DirectionGPS = (DirectionGPS + 360) % 360;




        for (;;) {
          Difference = abs(DirectionMag - DirectionGPS);
          // if (Difference >= 180) Difference = 360-Difference;

          Serial.print("DirectionMag = ");  //目的地Aの方角(°）
          Serial.println(DirectionMag);
          Serial.print("DirectionGPS = ");  //目的地Aの方角(°）
          Serial.println(DirectionGPS);
          Serial.print("ゴールまでの角度=");
          Serial.println(Difference);



          if (DirectionMag > DirectionGPS) {
            if (Difference >= 180) {

              //右のモーターで機体を少し左回転

              delay(300);
            }

            else {

              //左のモーターで機体を少し右回転
              delay(300);
            }
          }




          if (DirectionMag < DirectionGPS) {
            if (Difference >= 180) {

              //モーターで機体を少し右回転
              delay(300);
            }

            else {

              //モーターで機体を少し左回転
              delay(300);
            }
          }


          if (Difference >= 180) Difference = 360 - Difference;
          Serial.print("移動前のゴールまでの相対角度=");
          Serial.println(Difference);


          //モーターを止める


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


          Serial.print("移動後のMagの角度=");
          Serial.println(DirectionMag);

          if (Difference >= 180) Difference = 360 - Difference;

          if (Difference < 20) {
            Serial.print("ゴールの角度=");
            Serial.println(Difference);

            //モーター直進数秒

            delay(10000);
            break;
          }


          //スタック処理
          challenge_count += 1;
          if (challenge_count >= challenge) {
            //モーター直進数秒
            delay(1000);
            //モーターを止める
            delay(1000);
            challenge_count=0;
            break;
          }



          Serial.println("");
          Serial.println("");

          delay(500);
        }












        delay(1000);

        Serial.println("");
        Serial.println("");
        Serial.println("");
        Serial.println("");

      } else {
        Serial.print("測位できませんでした。");
      }

      Serial.println("");
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
