#include<SD.h>
#include<SPI.h>
#include<Wire.h>

//I2Cアドレス (JP1,JP2,JP3 = Openの時)
// BMX055　加速度センサのI2Cアドレス 
#define Addr_Accl 0x19
// BMX055　ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69
// BMX055　磁気センサのI2Cアドレス
#define Addr_Mag 0x13

// センサー値保持用
float xAccl=0.00;
float yAccl=0.00;
float zAccl=0.00;
float xGyro=0.00;
float yGyro=0.00;
float zGyro=0.00;
int   xMag=0;
int yMag=0;
int zMag=0;

float xMagc=0.00;
float yMagc=0.00;
float zMagc=0.00;

File f;

void setup() {
  // put your setup code here, to run once:

  Wire.begin();   // Wire(I2C)の初期化
  Serial.begin(9600);
  
  BMX055_Init();  //BMX055 初期化
  delay(300);
  
f=SD.open("/BMX055_adjust_test02_20240109.csv",FILE_APPEND);
if(SD.begin()){
  if(f){
    f.println("AX,AY,AZ,GX,GY,GZ,MX,MY,MZ,MXc,MYc");
    f.close();
}
else{
  Serial.print("Set up error");
  }

}

}


void loop() {
    

  // put your main code here, to run repeatedly:
if(!SD.begin()){
  Serial.println("SD failed");  
}
else{
  Serial.println("SD good");
}

f=SD.open("/BMX055_adjust_test02_20240109.csv",FILE_APPEND);
if(SD.begin()){
  if(f){
    //BMX055 加速度の読み取り
  BMX055_Accl();
  Serial.print("Accl= ");
  Serial.print(xAccl);
  Serial.print(",");
  Serial.print(yAccl);
  Serial.print(",");
  Serial.print(zAccl);
  Serial.println(""); 


  f.print(xAccl);
  f.print(",");
  f.print(yAccl);
  f.print(",");
  f.print(zAccl);
  f.print(","); 

  //BMX055 ジャイロの読み取り

 BMX055_Gyro();
 Serial.print("Gyro= ");
  Serial.print(xGyro);
  Serial.print(",");
  Serial.print(yGyro);
  Serial.print(",");
  Serial.print(zGyro);
  Serial.println(""); 
  f.print(xGyro);
  f.print(",");
  f.print(yGyro);
  f.print(",");
  f.print(zGyro);
  f.print(","); 
  
  //BMX055 磁気の読み取り
 BMX055_Mag();

  Serial.print("Mag= ");
  Serial.print(xMag);
  Serial.print(",");
  Serial.print(yMag);
  Serial.print(",");
  Serial.print(zMag);
  Serial.println("");
  f.print(xMag);
  f.print(",");
  f.print(yMag);
  f.print(",");
  f.print(zMag);
  f.println(",");
  

    f.close();
    Serial.println("write OK");    
    
  
  
  
  
  
/*
  Serial.print("Magc=");
  
  Serial.print(xMagc);
  Serial.print(",");
  
  Serial.print(yMagc);
  
  Serial.print("");
  */

  }
}
else{
  Serial.println("write failed");
}

delay(100);

}



void BMX055_Init()
{
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x0F); // Select PMU_Range register
  Wire.write(0x03);   // Range = +/- 2g
  Wire.endTransmission();
  delay(50);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x10);  // Select PMU_BW register
  Wire.write(0x08);  // Bandwidth = 7.81 Hz
  Wire.endTransmission();
  delay(50);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x11);  // Select PMU_LPW register
  Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
  Wire.endTransmission();
  delay(50);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F);  // Select Range register
  Wire.write(0x04);  // Full scale = +/- 125 degree/s
  Wire.endTransmission();
  delay(50);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10);  // Select Bandwidth register
  Wire.write(0x07);  // ODR = 100 Hz
  Wire.endTransmission();
  delay(50);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11);  // Select LPM1 register
  Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay(50);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x83);  // Soft reset
  Wire.endTransmission();
  delay(50);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x01);  // Soft reset
  Wire.endTransmission();
  delay(50);
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
  Wire.write(0x0e);  // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}
//=====================================================================================//
void BMX055_Accl()
{
  int data[6];
  for (int i = 0; i < 6; i++)
  {
    data[i]=0;
    Wire.beginTransmission(Addr_Accl);
    Wire.write((2 + i));// Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Accl, 1);// Request 1 byte of data
    // Read 6 bytes of data
    //data[]= xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
    if (Wire.available() == 1) data[i] = Wire.read();
  }
  // データ変換
  xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
  if (xAccl > 2047)  xAccl -= 4096;
  yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
  if (yAccl > 2047)  yAccl -= 4096;
  zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
  if (zAccl > 2047)  zAccl -= 4096;
  xAccl = xAccl * 0.0098; // renge +-2g
  yAccl = yAccl * 0.0098; // renge +-2g
  zAccl = zAccl * 0.0098; // renge +-2g
}
//=====================================================================================//
void BMX055_Gyro()
{
  int data[6];
  for (int i = 0; i < 6; i++)
  {
    data[i]=0;
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((2 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // データ変換
  xGyro = (data[1] * 256) + data[0];
  if (xGyro > 32767)  xGyro -= 65536;
  yGyro = (data[3] * 256) + data[2];
  if (yGyro > 32767)  yGyro -= 65536;
  zGyro = (data[5] * 256) + data[4];
  if (zGyro > 32767)  zGyro -= 65536;

  xGyro = xGyro * 0.0038; //  Full scale = +/- 125 degree/s
  yGyro = yGyro * 0.0038; //  Full scale = +/- 125 degree/s
  zGyro = zGyro * 0.0038; //  Full scale = +/- 125 degree/s
}
//=====================================================================================//
void BMX055_Mag()
{
  byte data[8];
  for (int i = 0; i < 8; i++)
  {
    data[i]=0;
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // データ変換
  xMag = * ((int16_t * ) &data[0]) / 8;
  yMag = ((data[3] * 256) + data[2]) / 8;
  zMag = ((data[5] * 256) + data[4]) / 2;
}
