# StairPersonDetector
<img src="pico_mpu6050.jpg" height=320><img src="mpu6050FFT.jpg" height=320><br>
Using a Pico  to detect if somebody  goes up or down the stair<br>
This is a proof of concept <br>

Demonstration link to youtube.com <a href=https://youtu.be/534miBv5ut0>https://youtu.be/534miBv5ut0</a><br>

- First trial<br>
 Use A MPU6050 to detect vibration on the stair.<br>
 I create the code using kiss FFT from https://github.com/AlexFWulff/awulff-pico-playground <br>
 The MPU6050 functions  are base on the Pico-SDK. I don't use the fifo since it is only the accelerometer <br>
 I'm doing an 512 points  FFT on 500 samples/sec by interleave of 256 samples. This way I do have faster response.<br>
 <br>In conclusion.<br>
  Walking around the stair trigger the mpu6050 so it is hard to find a good threshold to discriminate.
  
- Second and final trial<br>
  This version doesn't use the mpu6050.<br>
  It uses an I.R. distance indicator check https://github.com/danjperron/VL53L0X_pico<br>
  folder pico_escalier.   This is the curren version using the VL53L0 sensor to turn Light ON/OFF via MQTT<br>

- File Description<br>
  mpu6050_udp folder.  code source to transfer FFT result from mpu6050 via udp packet<br>
 
- How to compile<br>
  from the folder mpu6050_udp<br>
  mkdir build<br>
  cd build<br>
  cmake -DPICO_BOARD=pico_w -DWIFI_SSID="your essid" -DWIFI_PASSWORD="your password" ..<br>
  make<br>
  copy over the file mpu6050_udp.uf2 to the pico in flash mode<br>
  or use the SWD with openocd on a raspberry Pi<br>
  openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program mpu6050_udp.elf verify reset exit"<br>
  <br>
  Don't forget to change the define. in the main file mpu6050_udp.c<br>
  <br>#define  SEND_TO_IP  "10.11.12.104"  this should be the IP of the computer which run mpuPlot.py<br>
  #define  SEND_TO_PORT 6001  this is the port<br>
  #define  threshold  0.0   Minimum threshold to send data via udp  (value is in milli g) 1g = 9.8m/s*s<br>
  #define FSAMP 500         Sample frequency of the mpu6050. The mpu6050 divider will use this (8000/FSAMP) -1<br>
  #define NSAMP 512         Number of sample points for FFT<br>       
                    
  <br><br>The python script mpuPlot.py will received the data in UDP mode and display the chart.<br>
  The python library numpy and matplotlib need to be installed.<br>
  N.B. FSAMP and NSAMP in mpuPlot.py. should be the same as the mpu6050_udp.c<br>
  <br>
  
B.T.W. I added a working code "test_mqtt" using the SDK to handle the MQTT.<br>
This is just an example on how to handle MQTT<br>
the CMakeLists.txt file in pico-sdk/src/rp2_common/pico_lwip needs to be modified by adding after line   # MQTT client files<br>
    <blockquote>add_library(pico_lwip_mqtt INTERFACE)
    target_sources(pico_lwip_mqtt INTERFACE<br>
            ${PICO_LWIP_PATH}/src/apps/mqtt/mqtt.c
            )
</blockquote>  
  
  
  
  
 
