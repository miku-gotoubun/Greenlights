#include <SoftwareSerial.h>

#include <math.h>
#include <stdio.h>
 
// rxPin = 16  txPin = 17
SoftwareSerial mySerial(16, 17); //ピン番号は16,17
 
// NMEAの緯度経度を「度分秒」(DMS)の文字列に変換する
String NMEA2DMS(float val) {
  int d = val / 100;
  int m = ((val / 100.0) - d) * 100.0;
  float s = ((((val / 100.0) - d) * 100.0) - m) * 60;
  return String(d) + "度" + String(m) + "分" + String(s, 1) + "秒";
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
  int hh = (str.substring(0,2).toInt()) + 9;
  if(hh > 24) hh = hh - 24;
 
  return String(hh,DEC) + ":" + str.substring(2,4) + ":" + str.substring(4,6);  
}
 
void setup() {
  mySerial.begin(9600);
  Serial.begin(115200);
}
void loop() {
  // 1つのセンテンスを読み込む
  String line = mySerial.readStringUntil('\n');
 
  if(line != ""){
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

     
   

     // 2点の中心角(ラジアン)を求める
    /*float a =
      (sin(rlat1) * sin(rlat2)) +
      (cos(rlat1) * cos(rlat2) *
      cos(rlng1 - rlng2));
      */
    


   
    // $GPGGAセンテンスのみ読み込む
    if (list[0] == "$GPGGA") {
      
      // ステータス
      if(list[6] != "0"){      
        // 現在時刻
        //Serial.print(UTC2GMT900(list[1]));
        
        // 緯度
        Serial.print(" 緯度(正規):");
        Serial.print(NMEA2DMS(list[2].toFloat()));
        Serial.print("(");
        Serial.print(NMEA2DD(list[2].toFloat()));
        Serial.print(")");
 
        // 経度
        Serial.print(" 経度(正規):");
        Serial.print(NMEA2DMS(list[4].toFloat()));
        Serial.print("(");
        Serial.print(NMEA2DD(list[4].toFloat()));
        Serial.println(")");


        //lat1,lng1を現在地、２を目的地とする
    float distance;
    float lat1;
    float lng1;
    float lat2;
    float lng2;

    
    //current location
    lat1 = NMEA2DD(list[2].toFloat()).toFloat();
    lng1 = NMEA2DD(list[4].toFloat()).toFloat();
    Serial.print("緯度:");
    Serial.print(lat1);
    Serial.print(",経度:");
    Serial.println(lng1);
    

    
    //goalpoint (clock tower)
    lat2 = 33.6212150;
    lng2 = 133.720246;

    // 円周率
    const float pi = 3.1415;

    // 緯度経度をラジアンに変換
    float rlat1 = lat1 * (pi / 180);
    Serial.print("rlat1:");
    Serial.print(rlat1);

    float rlng1 = lng1 * (pi / 180);
    Serial.print(",rlng1:");
    Serial.println(rlng1);

    float rlat2 = lat2 * ( pi / 180);
    Serial.print(",rlat2:");
    Serial.print(rlat2);

    float rlng2 = lng2 * ( pi / 180);
    Serial.print(",rlng2:");
    Serial.println(rlng2);
    


    //2点の緯度差
    float Dy=rlat1-rlat2;
    Serial.print("Dy:");
    Serial.print(Dy);
    


    //2点の経度差
    float Dx=rlng1-rlng2;
    Serial.print(",Dx:");
    Serial.println(Dx);
    


    //2点の緯度の平均
    float P=(rlat1+rlat2)/2;
    Serial.print(",P:");
    Serial.println(P);
    

    

    //地球の離心率
    const float E=0.081819190;

    float W=sqrt(1-(pow(E,2)*(pow(sin(P),2))));
    Serial.print(",W:");
    Serial.println(W);
    


    //子午線曲率半径
    float M=(6378137.00*(1-(E*E)))/ pow(W,3);
    Serial.print(",M:");
    Serial.println(M);
    


    //卯酉線曲線半径
    float N=6378137.00 / W ;
    Serial.print(",N:");
    Serial.print(N);
    


    // 2点間の距離(メートル)
    distance = sqrt(pow(Dy*M,2)+pow(Dx*N*cos(P),2));
    Serial.print(",distance:");
    Serial.print(distance);

    //目的地の方角
    float angle =0.00;
    
    angle = atan2((lat2-lat1),(lng2-lng1))*(180/pi);
    Serial.print(" ,angle=");
    Serial.print(angle);

        // 距離
       /* Serial.print("目的地までの距離:");
        Serial.print(distance);
        Serial.print("m");
        */
 
       
      }else{
        Serial.print("測位できませんでした。");
      }
      
      Serial.println("");
    }
  }
}
