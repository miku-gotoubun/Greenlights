# DC-moter
このDCモータはanalogWriteを使用していない。ledcWriteを使用している。ESP32の場合は後者の方がDCモータのPWM制御に適している。
以下参考にしたサイト↓  
https://rikoubou.hatenablog.com/entry/2017/06/05/172953  
https://100kinsat.github.io/posts/control-motor/  

## 2024-01-24補足  
DCモーターで後退する時は

digitalWrite(INA1,HIGH);  
digitalWrite(INA2,LOW);  
ledcWrite(0,0);  

上のようにledcwriteのデューティ比は０の時が後退時の最高速度になる。


## [2024-02-28](https://github.com/miku-gotoubun/Greenlights/blob/main/%E3%82%BB%E3%83%B3%E3%82%B5%E5%8D%98%E4%BD%93%E5%8B%95%E4%BD%9C%E3%82%B3%E3%83%BC%E3%83%89/DC%E3%83%A2%E3%83%BC%E3%82%BF/Greenlights_DCmotor.ino)
GreenlightsのDCモータの基本動作ができるプログラムをアップした
