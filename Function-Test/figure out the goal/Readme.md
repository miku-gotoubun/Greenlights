# figure out the goal
cansat_BMX055_and_GPS_success20231204_ver2.0はBMX055とGPSの統合処理をしたものである。こいつの欠点はGPS受信時のみBMX055の値が出力されないことである。
これを条件分岐を使ってできんことはないと思うが、まだやっていない。

### [2023-12-20](https://github.com/miku-gotoubun/Greenlights/blob/main/Function-Test/figure%20out%20the%20goal/cansat_BMX055_and_GPS_success_2023_dec20a_ver4.ino)  
cansat_BMX055_and_GPS_success_2023_dec20a_ver4　ではGPSを受信していなくてもBMX055の値が表示されるようにしたが、GPSとBMX055の値の出力が同時にできない.
GPSの出力にタイムラグがありそうな気がする(未確認)  

当分の間はver2で動作確認する。
