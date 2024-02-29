#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>
#include<uTimerLib.h>


// BMX055 加速度センサのI2Cアドレス
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Openの時)
// BMX055 ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13  // (JP1,JP2,JP3 = Openの時)

const int INA1 = 26;
const int INA2 = 27;

const int INB1 = 0;
const int INB2 = 4;

// センサーの値を保存するグローバル変数
float xAccl = 0.00;
float yAccl = 0.00;
float zAccl = 0.00;
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
int16_t xMag = 0;
int16_t yMag = 0;
int16_t zMag = 0;

float X = 0.00;
float Y = 0.00;

float Mag_Degree = 0.00;
float gps_ang = 0.00;

SoftwareSerial gpsSerial(16, 17);  // GPSモジュールとの通信ピン (RX, TX)

TinyGPSPlus gps;
double Latitude;
double Longitude;

double Dy = 0.00;
double Dx = 0.00;
double P = 0.00;
double Rx = 6378137.000;     //長半径（赤道半径）
double Ry = 6356752.314245;  //短半径（極半径）
double E = 0.08181919083;    //離心率
double W = 0.00;
double M = 0.00;  //子午線曲率半径
double N = 0.00;  //卯酉曲線半径
double Distance = 0.00;

//ゴールの設定
double golelat = 33.620076;
double golelot = 133.719697;  //33.62007650806248, 133.71969754391964

float control_degree = 0.00;
int Greenlights_go=0;


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
  Wire.write(0x16);  // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}

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
  xAccl = xAccl * 0.0098;  // range = +/-2g
  yAccl = yAccl * 0.0098;  // range = +/-2g
  zAccl = zAccl * 0.0098;  // range = +/-2g
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

void Forced_stop() {
  //停止
  digitalWrite(INA1,LOW);
  digitalWrite(INA2,LOW);
  ledcWrite(0,0);
  digitalWrite(INB1,LOW);
  digitalWrite(INB2,LOW);
  ledcWrite(1,0);
}

void setup() {
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

  //BMX055 GPS
  Wire.begin();
  Serial.begin(9600);
  gpsSerial.begin(9600);
  BMX055_Init();
  delay(300);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);

  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  delay(300);
}

void loop() {
  TimerLib.setTimeout_s(Forced_stop,900); //ゴール探索開始してから18分後(1080秒)に強制終了
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // GPSデータが正常に解析された場合
      Serial.println("--------------------------------");

      // シリアルモニターに表示
      //時刻（JST）
      int hh = gps.time.hour() + 9;
      if (hh > 24) hh = hh - 24;
      Serial.print(hh);  // Hour (0-23) (u8)
      Serial.print("時");
      Serial.print(gps.time.minute());  // Minute (0-59) (u8)
      Serial.print("分");
      Serial.print(gps.time.second());  // Second (0-59) (u8)
      Serial.print("秒");

      Serial.print("Latitude: ");
      Serial.print(gps.location.lat(), 7);
      Serial.print(", Longitude: ");
      Serial.println(gps.location.lng(), 7);

      Latitude = gps.location.lat();  //代入
      Longitude = gps.location.lng();

      Dy = Latitude * PI / 180 - (golelat * PI / 180);   //２点の緯度（ラジアン）の差
      Dx = Longitude * PI / 180 - (golelot * PI / 180);  //２点の経度（ラジアン）の差

      P = ((Latitude * PI / 180) + (golelat * PI / 180)) / 2;  //２点の緯度の平均
      double W = sqrt(1 - E * E * sin(P) * sin(P));
      double M = Rx * (1 - E * E) / (W * W * W);  //子午線曲率半径
      double N = Rx / W;                          //卯酉曲線半径

      Distance = sqrt((Dy * M * Dy * M + Dx * N * cos(P) * Dx * N * cos(P)));  //２点の距離計算

      Serial.print("目的地までの距離：");
      Serial.println(Distance);


      //北を0度としたときの目的地の角度 float a_ang=atan2((gps_lng-def_lng),(gps_lat-def_lat))*180/PI;
      float gps_ang = atan2((golelot - gps.location.lng()), (golelat - (gps.location.lat()))) * 180 / PI;
      Serial.print("gps angle = ");  //目的地Aの方角(°）
      Serial.println(gps_ang, 2);


      //Serial.println("--------------------------------------");

      //BMX055 加速度の読み取り
      BMX055_Accl();
      Serial.print("Accl= ");
      Serial.print(xAccl);
      Serial.print(",");
      Serial.print(yAccl);
      Serial.print(",");
      Serial.print(zAccl);
      Serial.println("");

      //BMX055 ジャイロの読み取り
      BMX055_Gyro();
      Serial.print("Gyro= ");
      Serial.print(xGyro);
      Serial.print(",");
      Serial.print(yGyro);
      Serial.print(",");
      Serial.print(zGyro);
      Serial.println("");

      //BMX055 磁気の読み取り
      BMX055_Mag();
      Serial.print("Mag= ");
      Serial.print(xMag);
      Serial.print(",");
      Serial.print(yMag);
      Serial.print(",");
      Serial.print(zMag);
      Serial.println("");

      X = xMag - 232.5;
      Y = yMag - 160.5;
      float Mag_Degree = (atan2(Y, X) * 180 / PI)- 7.5; //磁力線の偏角　高知工科大学→7.5 種子島→6.167
      Serial.print("Mag Degree=");
      Serial.println(Mag_Degree); //磁力線の偏角　高知工科大学→7.5 種子島→6.167
      
      int i=0;
      for(i=0;i<=4;i++) {
      BMX055_Mag();
      X = xMag - 232.5;
      Y = yMag - 160.5;
      float Mag_Degree = (atan2(Y, X) * 180 / PI)- 7.5; //磁力線の偏角　高知工科大学→7.5 種子島→6.167
      Serial.print("Mag Degree=");
      Serial.println(Mag_Degree); //磁力線の偏角　高知工科大学→7.5 種子島→6.167
      if((gps_ang>=0) && (Mag_Degree<0)) {
    control_degree=abs(abs(Mag_Degree)-abs(gps_ang));
    if((abs(gps_ang) > abs(Mag_Degree)) && (control_degree>30)) {
      //左回転
    digitalWrite(INB1, HIGH);
    digitalWrite(INB2, LOW);
    ledcWrite(1, 255);
    digitalWrite(INA1, LOW);
    digitalWrite(INA2, LOW);
    ledcWrite(0, 0);
    delay(abs(control_degree)*1+10);

    //停止
      digitalWrite(INA1, LOW);
      digitalWrite(INA2, LOW);
      ledcWrite(0, 0);
      digitalWrite(INB1, LOW);
      digitalWrite(INB2, LOW);
      ledcWrite(1, 0);
      delay(1000);
    }
    else if((abs(gps_ang) < abs(Mag_Degree)) && (control_degree>30)) {
      //右回転
    digitalWrite(INA1, LOW);
    digitalWrite(INA2, HIGH);
    ledcWrite(0, 255);
    digitalWrite(INB1, LOW);
    digitalWrite(INB2, LOW);
    ledcWrite(1, 0);
    delay(abs(control_degree)*1+10);

    //停止
      digitalWrite(INA1, LOW);
      digitalWrite(INA2, LOW);
      ledcWrite(0, 0);
      digitalWrite(INB1, LOW);
      digitalWrite(INB2, LOW);
      ledcWrite(1, 0);
      delay(1000);
    }
    else if(control_degree<=30) {
      i=4;
    }
  }
  else if((gps_ang>=0 && Mag_Degree>=0)) {
      control_degree=abs(Mag_Degree)+abs(gps_ang);
      if((control_degree<330) && (control_degree>30)) {
        //左回転
    digitalWrite(INB1, HIGH);
    digitalWrite(INB2, LOW);
    ledcWrite(1, 255);
    digitalWrite(INA1, LOW);
    digitalWrite(INA2, LOW);
    ledcWrite(0, 0);
    delay(abs(control_degree)*1+10);

    //停止
      digitalWrite(INA1, LOW);
      digitalWrite(INA2, LOW);
      ledcWrite(0, 0);
      digitalWrite(INB1, LOW);
      digitalWrite(INB2, LOW);
      ledcWrite(1, 0);
      delay(1000);
        //Greenlights_go=2;
      }
      else if((control_degree<=30) || (control_degree>=330)) {

      i=4;
      
    }
  }
  else if(gps_ang<0 && Mag_Degree>=0) {
    control_degree=abs(abs(Mag_Degree)-abs(gps_ang));
    if((abs(gps_ang) > abs(Mag_Degree)) && (control_degree>30)) {
      //Greenlights_go=1;
      //右回転
    digitalWrite(INA1, LOW);
    digitalWrite(INA2, HIGH);
    ledcWrite(0, 255);
    digitalWrite(INB1, LOW);
    digitalWrite(INB2, LOW);
    ledcWrite(1, 0);
    delay(abs(control_degree)*1+10);

    //停止
      digitalWrite(INA1, LOW);
      digitalWrite(INA2, LOW);
      ledcWrite(0, 0);
      digitalWrite(INB1, LOW);
      digitalWrite(INB2, LOW);
      ledcWrite(1, 0);
      delay(1000);
    }
    else if((abs(gps_ang) < abs(Mag_Degree)) && (control_degree>30)) {
      //左回転
    digitalWrite(INB1, HIGH);
    digitalWrite(INB2, LOW);
    ledcWrite(1, 255);
    digitalWrite(INA1, LOW);
    digitalWrite(INA2, LOW);
    ledcWrite(0, 0);
    delay(abs(control_degree)*1+10);

    //停止
      digitalWrite(INA1, LOW);
      digitalWrite(INA2, LOW);
      ledcWrite(0, 0);
      digitalWrite(INB1, LOW);
      digitalWrite(INB2, LOW);
      ledcWrite(1, 0);
      delay(1000);
    }
    else if(control_degree<=30) {
      i=4;
      
    }
  }
  else if((gps_ang<0 && Mag_Degree<0)) {
    control_degree=abs(Mag_Degree)+abs(gps_ang);
      if((control_degree<330) && (control_degree>30)) {
        //右回転
    digitalWrite(INA1, LOW);
    digitalWrite(INA2, HIGH);
    ledcWrite(0, 255);
    digitalWrite(INB1, LOW);
    digitalWrite(INB2, LOW);
    ledcWrite(1, 0);
    delay(abs(control_degree)*1+10);

    //停止
      digitalWrite(INA1, LOW);
      digitalWrite(INA2, LOW);
      ledcWrite(0, 0);
      digitalWrite(INB1, LOW);
      digitalWrite(INB2, LOW);
      ledcWrite(1, 0);
      delay(1000);
      }
      
      else if((control_degree<=30) || (control_degree>=330)) {
        i=4;
      }
    }
  } //for終わり(方向修正終わり)
  if(Distance > 10) { //ゴールまで5m以上離れている
      //直進
    digitalWrite(INA1, LOW);
      digitalWrite(INA2, HIGH);
      ledcWrite(0, 255);

      digitalWrite(INB1, HIGH);
      digitalWrite(INB2, LOW);
      ledcWrite(1, 255);
      delay(10000);

      //停止
      digitalWrite(INA1, LOW);
      digitalWrite(INA2, LOW);
      ledcWrite(0, 0);
      digitalWrite(INB1, LOW);
      digitalWrite(INB2, LOW);
      ledcWrite(1, 0);
      delay(1000);
      }
      else if(Distance <= 10 && Distance > 5) {
        digitalWrite(INA1, LOW);
      digitalWrite(INA2, HIGH);
      ledcWrite(0, 200);

      digitalWrite(INB1, HIGH);
      digitalWrite(INB2, LOW);
      ledcWrite(1, 200);
      delay(5000);

      //停止
      digitalWrite(INA1, LOW);
      digitalWrite(INA2, LOW);
      ledcWrite(0, 0);
      digitalWrite(INB1, LOW);
      digitalWrite(INB2, LOW);
      ledcWrite(1, 0);
      delay(1000);
      }
      else if(Distance <= 5) {
        //停止
      digitalWrite(INA1, LOW);
      digitalWrite(INA2, LOW);
      ledcWrite(0, 0);
      digitalWrite(INB1, LOW);
      digitalWrite(INB2, LOW);
      ledcWrite(1, 0);
      delay(1000);
      exit(0);
      }

      }//gps.encode

      

    } //while (gpsSerial.available() > 0) {
  }