#include <PS4Controller.h>
#include <ps4.h>
#include <ps4_int.h>

const int INA1=27;
const int INA2=14;

const int INB1=15;
const int INB2=2;

int value;

void setup() {
  Serial.begin(115200);
  PS4.begin("c4:de:e2:c7:1b:e6"); //C4:DE:E2:C7:1B:E6
  Serial.println("Ready.");

  pinMode(INA1,OUTPUT);
  pinMode(INA2,OUTPUT);

  pinMode(INB1,OUTPUT);
  pinMode(INB2,OUTPUT);

  pinMode(PS4.isConnected(),INPUT_PULLUP);
}

void loop() {

  

  
  // Below has all accessible outputs from the controller
  if (PS4.isConnected()) {

    

    //十字ボタンの動作  
    if (PS4.Right()==1) {
      Serial.println("Right Button");
      analogWrite(INA1, 0);
  analogWrite(INA2, 0);
  analogWrite(INB1, 200);
  analogWrite(INB2, 0);
    }
    else if(PS4.Right()==0){
      //delay(50);
      analogWrite(INA1, 0);
  analogWrite(INA2, 0);
  analogWrite(INB1, 0);
  analogWrite(INB2, 0);

    }
    
    if (PS4.Down()==1) {
      Serial.println("Down Button");
      analogWrite(INA1, 200);
  analogWrite(INA2, 0);
  analogWrite(INB1, 0);
  analogWrite(INB2, 200);
    }

    else if(PS4.Down()==0){
      //delay(50);
      analogWrite(INA1, 0);
  analogWrite(INA2, 0);
  analogWrite(INB1, 0);
  analogWrite(INB2, 0);

    }
    if (PS4.Up()==1) {
      Serial.println("Up Button");
      analogWrite(INA1, 0);
  analogWrite(INA2, 200);
  analogWrite(INB1, 200);
  analogWrite(INB2, 0);
    }

    else if(PS4.Up()==0){
      //delay(50);
      analogWrite(INA1, 0);
  analogWrite(INA2, 0);
  analogWrite(INB1, 0);
  analogWrite(INB2, 0);

    }
    if (PS4.Left()==1) {
      Serial.println("Left Button");
      analogWrite(INA1, 0);
  analogWrite(INA2, 200);
  analogWrite(INB1, 0);
  analogWrite(INB2, 0);

    }

    else if(PS4.Left()==0){
      //delay(50);
      analogWrite(INA1, 0);
  analogWrite(INA2, 0);
  analogWrite(INB1, 0);
  analogWrite(INB2, 0);

    } 

     if (PS4.UpRight()) Serial.println("Up Right");
    if (PS4.DownRight()) Serial.println("Down Right");
    if (PS4.UpLeft()) Serial.println("Up Left");
    if (PS4.DownLeft()) Serial.println("Down Left");

    
    if (PS4.Cross()==1) {
      Serial.println("Cross Button");
      analogWrite(INA1, 0);
  analogWrite(INA2, 0);
  analogWrite(INB1, 0);
  analogWrite(INB2, 0);

    }
    
    if (PS4.Circle()==1) {
      Serial.println("Circle Button");
      digitalWrite(INA1, LOW);
  digitalWrite(INA2, HIGH);
  digitalWrite(INB1, HIGH);
  digitalWrite(INB2, LOW);
    }

     else if (PS4.Circle()==0){
      //delay(200);
      analogWrite(INA1, 0);
      analogWrite(INA2, 0);
    }

    

    if (PS4.L1()) {
      Serial.println("L1 Button");
      digitalWrite(INA1, LOW);
  digitalWrite(INA2, LOW);
  digitalWrite(INB1, HIGH);
  digitalWrite(INB2, LOW);
    }
    
    if (PS4.R1()) {
      Serial.println("R1 Button");
      digitalWrite(INA1, LOW);
  digitalWrite(INA2, HIGH);
  digitalWrite(INB1, LOW);
  digitalWrite(INB2, LOW);
      
    }

    if (PS4.L2()>0) {
      Serial.printf("L2 button at %d\n", PS4.L2Value());
      analogWrite(INB1, PS4.L2Value());
      analogWrite(INB2, 0);
    }
    else if (PS4.L2()==0){
      //delay(200);
      analogWrite(INB1, 0);
      analogWrite(INB2, 0);
    }
    if (PS4.R2()>0) {
      Serial.printf("R2 button at %d\n", PS4.R2Value());
      analogWrite(INA1, 0);
      analogWrite(INA2, PS4.R2Value());
    }

    else if (PS4.R2()==0){
      //delay(200);
      analogWrite(INA1, 0);
      analogWrite(INA2, 0);
    }

   
    


    Serial.printf("Battery Level : %d\n", PS4.Battery());

    Serial.println(); 
    // This delay is to make the output more human readable
    // Remove it when you're not trying to see the output
    delay(50);
  }
}
