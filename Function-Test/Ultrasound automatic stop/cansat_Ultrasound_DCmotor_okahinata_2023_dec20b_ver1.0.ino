#define trigPin 33 // トリガーピンをD8に
#define echoPin 25 // エコーピンをD9に
float Duration = 0;  // 計測した時間
float Distance = 0;  // 距離
//Aのモーター
const int M_F = 27;
const int M_R = 14;
//Bのモーター
const int MT_F = 15;
const int MT_R = 2;
void setup() {
  // put your setup code here, to run once:
    //Aのモーター
    pinMode( M_F, OUTPUT );
    pinMode( M_R, OUTPUT );
    //Bのモーター
    pinMode( MT_F, OUTPUT );
    pinMode( MT_R, OUTPUT );
Serial.begin(9600);  // シリアルモニタの開始
  pinMode(echoPin,INPUT);   // エコーピンを入力に
  pinMode(trigPin,OUTPUT);  // トリガーピンを出力に
}
void loop() {
  // put your main code here, to run repeatedly:
//超音波
digitalWrite(trigPin,LOW);              // 計測前に一旦トリガーピンをLowに
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);             // トリガーピンを10μsの時間だけHighに
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  Duration = pulseIn(echoPin,HIGH);      // エコーピンからの入力
  Duration = Duration / 2;               // 距離を1/2に
  Distance = Duration*340*100/1000000;   // 音速を掛けて単位をcmに
  if(Distance <= 40){  // ゴールまでの距離が40cmになれば停止、そうでなければ進む
analogWrite( 15, 0);
   analogWrite( 2, 0);
  // delay(1000);
   analogWrite( 27, 0);
   analogWrite( 14, 0);
   delay(1000);
  }
  else{
analogWrite( 15, 0);
   analogWrite( 2, 255);
   //delay(1000);
   analogWrite( 27, 255);
   analogWrite( 14, 0);
   delay(1000);
  }
     //delay(100);
}