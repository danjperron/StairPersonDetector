This is the current version to detect somebody on stair.<br>
It uses Vl53L0 for detection and send MQTT signal to turn ON<br>
the light via tasmota wall switches.<br>

The switches will replace the 3 way switches using 2 ways mode.<br>
The tasmota switches use group topic to synchronize themself.<br>

The PicoW on detection send POWER on via MQTT.<br>
If any detection after sometimes and the Light is ON it will turn it off (defautl 120 secs)<br>
<br>
Convert Martin & jerry to tasmota
ref:https://www.youtube.com/watch?v=XuyEj1WNMiM
<br><br>
Sync two tasmota<br>
On each of my tasmota from the console on both tasmota devices I type<br>
grouptopic escalier  
setoption85 1<br>
(B.T.W.escalier is stair in french.)<br>
<br>
ref: https://www.youtube.com/watch?v=Bp1BsTe2zyY&t=322s
<br><br>
