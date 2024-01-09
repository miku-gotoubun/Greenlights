# figure out the goal
cansat_BMX055_and_GPS_success20231204_ver2.0はBMX055とGPSの統合処理をしたものである。こいつの欠点はGPS受信時のみBMX055の値が出力されないことである。
これを条件分岐を使ってできんことはないと思うが、まだやっていない。

### [2023-12-20](https://github.com/miku-gotoubun/Greenlights/blob/main/Function-Test/figure%20out%20the%20goal/cansat_BMX055_and_GPS_success_2023_dec20a_ver4.ino)  
cansat_BMX055_and_GPS_success_2023_dec20a_ver4　ではGPSを受信していなくてもBMX055の値が表示されるようにしたが、GPSとBMX055の値の出力が同時にできない.
GPSの出力にタイムラグがありそうな気がする(未確認)  

当分の間はver2で動作確認する。

### [2023-12-22](https://github.com/miku-gotoubun/Greenlights/blob/main/Function-Test/figure%20out%20the%20goal/cansat_BMX055_and_GPS_degree_trying_ver1.0_2023_dec22a.ino )  
cansat_BMX055_and_GPS_degree_trying_ver1.0_2023_dec22a.ino ではゴールまでの距離と北からの角度と時刻を計算するコードを作った。

次回はGPSでゴールの方角を編み出すコードを作る。

### [2024-01-09](https://github.com/miku-gotoubun/Greenlights/blob/main/Function-Test/figure%20out%20the%20goal/Greenlights_BMX055_GPS_autonomous_control_2024_jan8a.ino)  
Greenlights_BMX055_GPS_autonomous_control_2024_jan8a.ino では、GPSでゴールの方角の検出に成功したが、どうしても９軸センサの値が合わない。
これは恐らく建物内で測定したからだと思う。オフセット補正を自動でやるにしても落下地点の場所によって大きくずれる可能性がある。それだけでなく、DCモータの磁石によってずれることが懸念される。オフセット補正の自動化については再考する必要がある。
