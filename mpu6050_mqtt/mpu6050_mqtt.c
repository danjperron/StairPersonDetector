
/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/apps/mqtt.h"
#include "kiss_fftr.h"
#include "hardware/i2c.h"
#include "hardware/rtc.h"
#include <time.h>
#include "lwip/dns.h"
#include "pico/util/datetime.h"
#include "settings.h"
#include "mpu6050_i2c.h"
#include "mpu6050_ntp.h"

char udpHostIP[256];

// UDP PORT
#define  SEND_TO_PORT 6001


// MQTT

#define MQTT_HOST_IP "10.11.12.192"
#define MQTT_HOST_PORT 1883
#define MQTT_CLIENT_NAME "escalier2"




#define TOPIC_ESCALIER_BAS   "stat/escalierBas/POWER"
// we only need the stat of one tasmota switch
//#define TOPIC_ESCALIER_HAUT  "stat/escalierHaut/POWER" 

#define TOPIC_STAT "stat/"MQTT_CLIENT_NAME
#define TOPIC_CMND "cmnd/"MQTT_CLIENT_NAME

#define TOPIC_ESCALIER_STATUS        TOPIC_STAT"/info"
#define TOPIC_ESCALIER_GOT_TRIGGER   TOPIC_STAT"/gotTrigger"
#define TOPIC_CMND_ESCALIER          TOPIC_CMND"/#"
#define TOPIC_ESCALIER_POWER         TOPIC_CMND"/POWER"
#define TOPIC_ESCALIER_THRESHOLD     TOPIC_CMND"/threshold"
#define TOPIC_ESCALIER_UDPTHRESHOLD  TOPIC_CMND"/udpthreshold"
#define TOPIC_ESCALIER_PEAKTHRESHOLD TOPIC_CMND"/peakthreshold"
#define TOPIC_ESCALIER_ENABLE        TOPIC_CMND"/enable"
#define TOPIC_ESCALIER_DELAY         TOPIC_CMND"/delay"
#define TOPIC_ESCALIER_INFO          TOPIC_CMND"/info"
#define TOPIC_ESCALIER_UDP_HOST_IP   TOPIC_CMND"/udphostip"
#define TOPIC_ESCALIER_CALIBRATE     TOPIC_CMND"/calibrate"



// threshold  in  0.1mG  1G=10000
int8_t lightStatus=0;
int8_t mpuEnable=1;
int8_t mpuCalibrate=0;
absolute_time_t startOnTime=0;  //  timestamp for ligt on delay
int8_t weSetLightOn=0;
float LatestPeakGt=0;
float LatestPower=0;

// mpu6050 is set to +/- 2G
// g factor will set 1G=10000.0 (0.1 mG)
float gFactor = 20000.0 / 32767.0;




// FFT stuff  500samples/sec take 512 data points
// my sensor is at 45 degree so  I use YZ vector
// choice  sqrt(x*x + y*y + z*z)  USE_XYZ_VECTOR
// or    .707 Y  - .707 Z

#define USE_XYZ_VECTOR
#undef USE_XYZ_VECTOR


// FFT DECLARATION
    volatile int current_in=1;
    kiss_fft_scalar fft_in1[NSAMP]; // kiss_fft_scalar is a float
    kiss_fft_scalar fft_in2[NSAMP]; // kiss_fft_scalar is a float
    kiss_fft_cpx fft_out[NSAMP];
    kiss_fftr_cfg cfg;
    float  max_power = 0;
    int max_idx = 0;
    float peak1Gt;
    float peak2Gt;



float freqs[NSAMP/2];
// data to record mpu6050 data
// spectrum output to send via UDP
// N.B. spectrum[0] is the max idx instead of the sum of all
// last spectrum position will record the peakdiff before the fft
unsigned short spectrum[NSAMP/2+1];


/////// UDP FUNCTION

struct udp_pcb  * send_udp_pcb;


void SendUDP(char * IP , int port, void * data, int data_size)
{
      ip_addr_t   destAddr;
      ip4addr_aton(IP,&destAddr);
      struct pbuf * p = pbuf_alloc(PBUF_TRANSPORT,data_size+1,PBUF_RAM);
      char *pt = (char *) p->payload;
      memcpy(pt,data,data_size);
      pt[data_size]='\0';
      cyw43_arch_lwip_begin();
      udp_sendto(send_udp_pcb,p,&destAddr,port);
      cyw43_arch_lwip_end();
      pbuf_free(p);
}

///////// MQTT STUFF
typedef struct MQTT_CLIENT_DATA_T_ {
    mqtt_client_t* mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    uint8_t data[MQTT_OUTPUT_RINGBUF_SIZE];
    uint8_t topic[100];
    uint32_t len;
} MQTT_CLIENT_DATA_T;

void publish(mqtt_client_t *client, char * topic, char *payload, int payload_size);


MQTT_CLIENT_DATA_T *mqtt;
struct mqtt_connect_client_info_t mqtt_client_info=
{
  MQTT_CLIENT_NAME,
  NULL, /* user */
  NULL, /* pass */
  0,  /* keep alive */
  NULL, /* will_topic */
  NULL, /* will_msg */
  0,    /* will_qos */
  0     /* will_retain */
#if LWIP_ALTCP && LWIP_ALTCP_TLS
  , NULL
#endif
};

void publishStatus(void)
{
      char datestamp[32];
      char info[1024];
      sprintf(info,"%s Light:%s Delay:%d Enable:%s trigger Threshold  mpu:%.1f  udp:%.1f  peak:%.1f "
                   "raw offset  x:%.1f y:%.1f z:%.1f Latest Peak:%.1f FFT:%.1f"
                   " UDP_host_IP: '%s'",
                stampDate(datestamp),
                lightStatus ? "ON" : "OFF",
                mpuSettings.lightDelay,
                mpuEnable ? "ON" : "OFF",
                mpuSettings.mpuThreshold,
                mpuSettings.udpThreshold,
                mpuSettings.peakThreshold,
                mpuSettings.Gx_offset ,
                mpuSettings.Gy_offset ,
                mpuSettings.Gz_offset ,
                LatestPeakGt,
                LatestPower,
                udpHostIP);
      publish(mqtt->mqtt_client_inst,TOPIC_ESCALIER_STATUS,info,strlen(info));
      printf("%s\n",info);
}


static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    int _itemp;
    float _ftemp;
    MQTT_CLIENT_DATA_T* mqtt_client = (MQTT_CLIENT_DATA_T*)arg;
    LWIP_UNUSED_ARG(data);

    strncpy(mqtt_client->data, data, len);
    mqtt_client->len=len;
    mqtt_client->data[len]='\0';
    bool flag = false;
    if (strcmp(mqtt_client->topic, TOPIC_ESCALIER_BAS)==0)
    {
      if (strcmp(mqtt_client->data,"ON")==0)
         lightStatus = 1;
      else if(strcmp(mqtt_client->data,"OFF")==0)
         lightStatus = 0;
      printf("LightStatus : %d\n",lightStatus);
    }
// we only use one response from one tasmota
/*    else
    if (strcmp(mqtt_client->topic, TOPIC_ESCALIER_HAUT)==0)
    {
      if (strcmp(mqtt_client->data,"ON")==0)
         lightStatus = 1;
      else if(strcmp(mqtt_client->data,"OFF")==0)
         lightStatus = 0;
      printf("LightStatus : %d\n",lightStatus);
    }
*/
    else
    if (strcmp(mqtt_client->topic, TOPIC_ESCALIER_THRESHOLD)==0)
    {
      _ftemp = atof(mqtt_client->data);
      if(_ftemp < 0.0)
          _ftemp=1.0;
      mpuSettings.mpuThreshold= _ftemp;
      Write_Settings();
      printf("mpu6050 FFT Trigger Threshold set to %.1f (0.1 x mG)\n",mpuSettings.mpuThreshold);
    }
    else
    if (strcmp(mqtt_client->topic, TOPIC_ESCALIER_UDPTHRESHOLD)==0)
    {
      _ftemp = atof(mqtt_client->data);
      if(_ftemp < 0.0)
          _ftemp=1.0;
      mpuSettings.udpThreshold=(float) _ftemp;
      Write_Settings();
      printf("mpu6050 FFT Trigger  Threshold on udp set to %.1f (0.1 x mG)\n",mpuSettings.udpThreshold);
    }
    else
    if (strcmp(mqtt_client->topic, TOPIC_ESCALIER_PEAKTHRESHOLD)==0)
    {
      _ftemp = atof(mqtt_client->data);
      if(_ftemp < 0.0)
          _ftemp=1.0;
      mpuSettings.peakThreshold=(float) _ftemp;
      Write_Settings();
      printf("mpu6050 signal Peak Trigger Threshold set to %.1f (0.1 x mG)\n",mpuSettings.peakThreshold);
    }
    else
    if (strcmp(mqtt_client->topic, TOPIC_ESCALIER_DELAY)==0)
    {
      _itemp = atoi(mqtt_client->data);
      if(_itemp < 1)
          _itemp=120;
      mpuSettings.lightDelay=_itemp;
      Write_Settings();
      printf("Light ON delay set to %d\n",mpuSettings.lightDelay);

    }
    else
    if (strcmp(mqtt_client->topic, TOPIC_ESCALIER_ENABLE)==0)
    {
      _itemp=1;
      if (strcmp(mqtt_client->data,"ON")==0)
         mpuEnable = 1;
      else if (strcmp(mqtt_client->data,"OFF")==0)
         mpuEnable = 0;
      else if (strcmp(mqtt_client->data,"0")==0)
         mpuEnable = 0;
      else if (strcmp(mqtt_client->data,"1")==0)
         mpuEnable = 1;
      else _itemp=0;
      if(_itemp)
          printf("MPU6050 set enable to %d\n",mpuEnable);
    }
    else
    if (strcmp(mqtt_client->topic, TOPIC_ESCALIER_INFO)==0)
    {
      publishStatus();
    }
    else
    if (strcmp(mqtt_client->topic, TOPIC_ESCALIER_UDP_HOST_IP)==0)
    {
      strncpy(udpHostIP,mqtt_client->data,255);
      udpHostIP[255]=0;
      printf("UDP host IP set to '%s'\n",udpHostIP);
    }
    else
    if (strcmp(mqtt_client->topic, TOPIC_ESCALIER_CALIBRATE)==0)
    {
      mpuCalibrate=1;
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
  MQTT_CLIENT_DATA_T* mqtt_client = (MQTT_CLIENT_DATA_T*)arg;
  strcpy(mqtt_client->topic, topic);
}
static void mqtt_request_cb(void *arg, err_t err) {
  MQTT_CLIENT_DATA_T* mqtt_client = ( MQTT_CLIENT_DATA_T*)arg;

  LWIP_PLATFORM_DIAG(("MQTT client \"%s\" request cb: err %d\n", mqtt_client->mqtt_client_info.client_id, (int)err));
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
  MQTT_CLIENT_DATA_T* mqtt_client = (MQTT_CLIENT_DATA_T*)arg;
  LWIP_UNUSED_ARG(client);

LWIP_PLATFORM_DIAG(("MQTT client \"%s\" connection cb: status %d\n", mqtt_client->mqtt_client_info.client_id, (int)status));
  if (status == MQTT_CONNECT_ACCEPTED) {
    printf("MQTT client accepted\n");
    mqtt_sub_unsub(client,
            TOPIC_ESCALIER_BAS, 0,
            mqtt_request_cb, arg,
            1);
/*    mqtt_sub_unsub(client,
            TOPIC_ESCALIER_HAUT, 0,
            mqtt_request_cb, arg,
            1);
*/
    mqtt_sub_unsub(client,
            TOPIC_CMND_ESCALIER, 0,
            mqtt_request_cb, arg,
            1);
  }
}
/* Called when publish is complete either with success or failure */
static void mqtt_pub_request_cb(void *arg, err_t result)
{
  if(result != ERR_OK) {
    printf("Publish result: %d\n", result);
  }
}
void publish(mqtt_client_t *client, char * topic, char *payload, int payload_size)
{
  err_t err;
  u8_t qos = 2; /* 0 1 or 2, see MQTT specification */
  u8_t retain = 0; /* No don't retain such crappy payload... */
  err = mqtt_publish(client, topic, payload, payload_size, qos, retain, mqtt_pub_request_cb,NULL);
  if(err != ERR_OK) {
    printf("Publish err: %d\n", err);
  }
}

// DO_FFT this is the thread to read MPU6050 on second cpu and send UDP
void Do_FFT()
{
         char datestamp[32];
         char buffer[256];
         float peakGt;
          while(1)
          {
          // do fft

          int check_in = multicore_fifo_pop_blocking();
          if(check_in==1)
               kiss_fftr(cfg , fft_in1, fft_out);
          else
               kiss_fftr(cfg , fft_in2, fft_out);

          // find max freq
          // compute power and calculate max freq component
          max_power = 0;
          max_idx = 0;
          // any frequency bin over NSAMP/2 is aliased (nyquist sampling theorum)

          for (int i = 1; i < NSAMP/2; i++) {
              float power = sqrt(fft_out[i].r*fft_out[i].r+fft_out[i].i*fft_out[i].i);
              power/= (NSAMP/2);
              if(power>655535.0)
                  spectrum[i]= 65535;
              else
                  spectrum[i]= (unsigned short) power;
              if (power>max_power) {
	          max_power=power;
	          max_idx = i;
              }
           }


            LatestPeakGt= check_in == 1 ? peak1Gt: peak2Gt;
            LatestPower=max_power;

            stampDate(datestamp);
            sprintf(buffer,"%s Idx=%d  Freq: %.1f   max FFT value:%.03f  signal Peak:%0.3f\n\0",datestamp,max_idx, freqs[max_idx],LatestPower,LatestPeakGt);
            printf(buffer);

            peakGt = (check_in == 1) ?  peak1Gt : peak2Gt;

          if(strlen(udpHostIP)!=0)
          {
          if(max_power > mpuSettings.udpThreshold)
          {
            spectrum[0]=max_idx;
            spectrum[NSAMP/2]= peakGt;
            SendUDP(udpHostIP,SEND_TO_PORT,spectrum,sizeof(unsigned short) *  (NSAMP/2));
          }
          }

         if(mpuEnable)
             { // are we in the nigth
              if((max_power > mpuSettings.mpuThreshold) && ( peakGt > mpuSettings.peakThreshold))
                  {  // ok we got someting
                   if(lightStatus==0)
                       { // light is off then turn it on

                        publish(mqtt->mqtt_client_inst,TOPIC_ESCALIER_GOT_TRIGGER,buffer, strlen(buffer));
                        publish(mqtt->mqtt_client_inst,TOPIC_ESCALIER_POWER,"ON\0",3);
                        weSetLightOn=1;
                       }
                       // reset startOnTime
                   startOnTime=get_absolute_time();
                  }
              else
                 if(lightStatus==1)
                    {
                     if(weSetLightOn)
                      if((get_absolute_time() - startOnTime) > (1000000 * mpuSettings.lightDelay))
                         {
                          publish(mqtt->mqtt_client_inst,TOPIC_ESCALIER_POWER,"OFF\0",4);
                          startOnTime=get_absolute_time();
                         }
                    }
                  else
                    weSetLightOn=0;
             }
          multicore_fifo_push_blocking(check_in);
          }
}


void mpu6050_Calibrate(void)
{
  int16_t acceleration[3];
  int  total5seconds= 5 * FSAMP;
  float Gx_sum=0;
  float Gy_sum=0;
  float Gz_sum=0;

  for(int i=0;i<total5seconds;i++)
  {
     while(true)
     {
       if(mpu6050_status() & 1)
         {
          mpu6050_read_acc(acceleration);
          Gx_sum += (float) acceleration[0];
          Gy_sum += (float) acceleration[1];
          Gz_sum += (float) acceleration[2];
          break;
         }
       sleep_us(100);
     }
  }

  mpuSettings.Gx_offset=  Gx_sum / (float) total5seconds;
  mpuSettings.Gy_offset=  Gy_sum / (float) total5seconds;
  mpuSettings.Gz_offset=  Gz_sum / (float) total5seconds;

  Write_Settings();

  publishStatus();
}


int main() {
    stdio_init_all();

    // convert  mpu6050 digital value to 0.1mg  (gravity 1g = 10000)
    // for best integer output to udp host

    int first=1;  // first record not done yet
    int loop=0;
    int recordIdx=0;
    extern struct netif gnetif;
    int16_t acceleration[3];
    float g[3];
    float Gt=0;
    float peakGt=0;

    // first load settings
    Load_Settings();

    // On boot No UDP
    strcpy(udpHostIP,"\0");

    // create Frequency Table
    for(loop=1;loop<NSAMP/2;loop++)
        freqs[loop]= (((float) FSAMP / NSAMP) *loop);


    // allocate FFT
    cfg = kiss_fftr_alloc(NSAMP,false,0,0);


    // initialize wifi
    if (cyw43_arch_init()) {
        printf("Init failed!\n");
        return 1;
    }
    cyw43_pm_value(CYW43_NO_POWERSAVE_MODE,200,1,1,10);
    cyw43_arch_enable_sta_mode();

    printf("WiFi ... ");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed!\n");
        return 1;
    } else {
        printf("Connected.n");
        printf("IP: %s\n",ipaddr_ntoa(((const ip_addr_t *)&cyw43_state.netif[0].ip_addr)));
    }

    rtc_init();
    get_ntp_time();

    sleep_ms(1000);
    // set I2C pins
    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    // create UDP
    send_udp_pcb = udp_new();

        mqtt=(MQTT_CLIENT_DATA_T*)calloc(1, sizeof(MQTT_CLIENT_DATA_T));

    if (!mqtt) {
        printf("mqtt client instant ini error\n");
        return 0;
    }

   mqtt->mqtt_client_info = mqtt_client_info;
   ip_addr_t addr;
   if (!ip4addr_aton(MQTT_HOST_IP, &addr)) {
        printf("ip error\n");
        return 0;
    }

    mqtt->mqtt_client_inst = mqtt_client_new();
    mqtt_set_inpub_callback(mqtt->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, mqtt);

    err_t err = mqtt_client_connect(mqtt->mqtt_client_inst, &addr, MQTT_HOST_PORT,
         &mqtt_connection_cb, mqtt, &mqtt->mqtt_client_info);

    if (err != ERR_OK) {
      printf("connect error\n");
    }
    else
    printf("Client connected\n");

    // reset MPU6050
    mpu6050_reset();


    // launch thread to calculate FFT and publish via UDP
    multicore_launch_core1(Do_FFT);

    publishStatus();

    while (1) {

       if(mpuCalibrate)
        {
          mpu6050_Calibrate();
          mpuCalibrate=0;
          recordIdx=0;
          continue;
        }
       // do we have MPU6050 data
       if(mpu6050_status() & 1)
        {
          mpu6050_read_acc(acceleration);
          g[0] = (float) acceleration[0] - mpuSettings.Gx_offset;
          g[1] = (float) acceleration[1] - mpuSettings.Gy_offset;
          g[2] = (float) acceleration[2] - mpuSettings.Gz_offset;



#ifdef USE_XYZ_VECTOR
          Gt = sqrt(g[0]*g[0]+ g[1]*g[1] + g[2]*g[2]) * gFactor;
#else
          // my sensor is  45 degree  the horizontal is on x axis
          Gt =  0.707 * ( g[1] - g[2]) * gFactor;
#endif

          if(Gt > peakGt)
             peakGt = Gt;

          if(current_in == 1)
            fft_in1[recordIdx] = Gt;
          else
            fft_in2[recordIdx] = Gt;
          recordIdx++;
        }
        // did we fill completely the record
        if(recordIdx>=NSAMP)
        {
          // is it the first record
          // if no  then unblock
          if(first==0)
                multicore_fifo_pop_blocking();
//          printf("Gx:%5.0f Gy:%5.0f  Gz:%5.0f Gt:%5.0f\n",g[0] * gFactor, g[1], g[2],
//                  gFactor * 0.7 * (g[1]-g[2]));
          if(current_in == 1)
            {
             // copy second half to next fft_in1
               peak1Gt= peakGt;
               memcpy(fft_in2,&fft_in1[NSAMP/2],sizeof(float)*NSAMP/2);
             }
           else
            {
               peak2Gt= peakGt;
               memcpy(fft_in1,&fft_in2[NSAMP/2],sizeof(float)*NSAMP/2);
            }

          // ok ready for fft thread specify shich  record
          // and push blocking (this enable the thread to start calculating)
          first=0;
          multicore_fifo_push_blocking(current_in);

          // prepare to record next dat on alternate record
          recordIdx=NSAMP/2;
          current_in = (current_in) == 1 ? 2 :1;
          peakGt=0;
        }
    }
    kiss_fft_free(cfg);
    return 0;
}

