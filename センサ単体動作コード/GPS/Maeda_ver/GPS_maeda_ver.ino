#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>

SoftwareSerial gpsSerial(16, 17); // GPSモジュールとの通信ピン (RX, TX)
TinyGPSPlus gps;

const int chipSelect = 5; // SDカードモジュールのCSピン

double Latitude;
double Longitude;
double Dy = 0.00;
double Dx = 0.00;
double P = 0.00;
double Rx = 6378137.000;//長半径（赤道半径）
double Ry = 6356752.314245;//短半径（極半径）
double E= 0.08181919083;//離心率
double W;// = 0.00;
double M;// = 0.00;//子午線曲率半径
double N;// = 0.00;//卯酉曲線半径
double D;// = 0.00;

double golelat = 33.6195483330;
double golelot = 133.7200183330;
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);

  if (!SD.begin(chipSelect)) {
    Serial.println("SDカード初期化に失敗しました");
    return;
  }

  Serial.println("SDカード初期化成功");
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // GPSデータが正常に解析された場合
      Serial.println("--------------------------------");
      
      // シリアルモニターに表示
      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 10);
      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 10);
      
      //double Latitude;
      //double Longitude; 

      Latitude = gps.location.lat();//代入
      Longitude = gps.location.lng();
      
      //Serial.print("Latitude");//確認
      //Serial.println(Latitude,6);
      //Serial.print("Longitude");
      //Serial.println(Longitude,6);

    

      Dy = Latitude * PI / 180 - (golelat * PI / 180);//２点の緯度（ラジアン）の差
      Dx = Longitude * PI / 180 - (golelot * PI / 180);//２点の経度（ラジアン）の差

      P = ((Latitude * PI /180 )+ (golelat * PI /180))/2;//２点の緯度の平均
      double W = sqrt(1 - E*E*sin(P)*sin(P));//
      double M = Rx*(1 - E*E)/(W*W*W);//子午線曲率半径
      double N = Rx/W;//卯酉曲線半径

      D = sqrt((Dy*M*Dy*M + Dx*N*cos(P)*Dx*N*cos(P)));//２点の距離計算
    
      
      /*Serial.print("Dy: ");//確認
      Serial.println(Dy,10);
      Serial.print("Dx: ");
      Serial.println(Dx,10);
      Serial.print("P: ");
      Serial.println(P,10);
      Serial.print("W: ");
      Serial.println(W,10);
      Serial.print("M: ");
      Serial.println(M,10);
      Serial.print("N: ");
      Serial.println(N,10);
      Serial.print("cos(P): ");
      Serial.println(cos(P),10);*/

      Serial.print("Distance: ");//距離表示
      Serial.println(D,10);
/*
      // SDカードに書き込み
      File dataFile = SD.open("/gpsdata_2023_11_08.csv", FILE_APPEND);
​
      if (dataFile) {
        //dataFile.print("Latitude: ");
        dataFile.print(gps.location.lat(), 10);
      
        dataFile.print(",");
        //dataFile.print("Longitude: ");
        dataFile.println(gps.location.lng(), 10);
        //dataFile.print("Speed: ");
        //dataFile.println(gps.speed.kmph());
        //dataFile.print("Altitude: ");
        //dataFile.println(gps.altitude.meters());
        dataFile.print(",");
        dataFile.println(D, 10);
​
        dataFile.close(); // ファイルを閉じる
      } else {
        Serial.println("ファイルの開封に失敗しました");
      }
      */
    }
  }
}
