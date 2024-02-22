int flytepin=32;
int val=0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(flytepin, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  val=digitalRead(flytepin);
  if (val==LOW) {
    Serial.println("待機中");
    delay(100);

  }
  else if(val==HIGH) {
    Serial.println("フライトピン分離");
    delay(500);
  }

}
