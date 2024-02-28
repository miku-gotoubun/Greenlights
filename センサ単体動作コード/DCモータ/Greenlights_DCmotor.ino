const int INA1=26;
const int INA2=27;

const int INB1=0;
const int INB2=4;

void setup() {
  pinMode(INA1,OUTPUT);
  pinMode(INA2,OUTPUT);

  pinMode(INB1,OUTPUT);
  pinMode(INB2,OUTPUT);
  //pwmの設定。最初の引数がchannel,次が周波数,最後がduty比（ここでは8bit = 256段階）
  ledcSetup(0, 12800, 8); //Aのモーター右
  ledcSetup(1, 12800, 8); //Bのモーター左
  // ledPinをチャネル0へ接続
  ledcAttachPin(INA2, 0);
  ledcAttachPin(INB1, 1);
} 
void loop() {
  //前進
  digitalWrite(INA1,LOW);
  digitalWrite(INA2,HIGH);
  ledcWrite(0,255);

  digitalWrite(INB1,HIGH);
  digitalWrite(INB2,LOW);
  ledcWrite(1,255);
  delay(1000);

  //停止
  digitalWrite(INA1,LOW);
  digitalWrite(INA2,LOW);
  ledcWrite(0,0);
  digitalWrite(INB1,LOW);
  digitalWrite(INB2,LOW);
  ledcWrite(1,0);
  delay(1000);

   //後退
  
  digitalWrite(INA1,HIGH);
  digitalWrite(INA2,LOW);
  ledcWrite(0,0);

  digitalWrite(INB1,LOW);
  digitalWrite(INB2,HIGH);
  ledcWrite(1,0);
  delay(3000);

  //停止
  digitalWrite(INA1,LOW);
  digitalWrite(INA2,LOW);
  ledcWrite(0,0);
  digitalWrite(INB1,LOW);
  digitalWrite(INB2,LOW);
  ledcWrite(1,0);
  delay(1000);

}
