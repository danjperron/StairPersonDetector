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
#include "mpu6050.h"
#include "kiss_fftr.h"
#include "hardware/i2c.h"
#include <time.h>

//  UDP SEND TO HOST IP/POR
#define ENABLE_UDP
#undef ENABLE_UDP
#define  SEND_TO_IP  "10.11.12.104"
#define  SEND_TO_PORT 6001


// MQTT
#define ENABLE_MQTT
//#undef ENABLE_MQTT

#define MQTT_HOST_IP "10.11.12.192"
#define MQTT_HOST_PORT 1883

#define TOPIC_ESCALIER_BAS   "stat/escalierBas/POWER"
#define TOPIC_ESCALIER_HAUT  "stat/escalierHaut/POWER"
#define TOPIC_CMND_ESCALIER  "cmnd/escalier/#"
#define TOPIC_ESCALIER_POWER  "cmnd/escalier/POWER"
#define TOPIC_ESCALIER_THRESHOLD "cmnd/escalier/threshold"
#define TOPIC_ESCALIER_DELAY "cmnd/escalier/delay"
#define TOPIC_ESCALIER_ENABLE "cmnd/escalier/enable"


#define LIGHT_WAIT_DELAY  120

// threshold  in milli G (gravity)
float  mpuThreshold=1500;
int8_t lightStatus=0;
int8_t mpuEnable=1;
absolute_time_t startOnTime=0;  //  timestamp for ligt on delay
int8_t weSetLightOn=0;
int lightDelay = LIGHT_WAIT_DELAY;
// FFT stuff  500samples/sec take 512 data points

#define FSAMP 500
#define NSAMP 512

float freqs[NSAMP/2];
// data to record mpu6050 data
// spectrum output to send via UDP
// N.B. spectrum[0] is the max idx instead of the sum of all
unsigned short spectrum[NSAMP/2];


/////// UDP FUNCTION
#ifdef ENABLE_UDP
#define BUF_SIZE 1024
char UDP_buffer[BUF_SIZE];

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
#endif

#ifdef ENABLE_MQTT
///////// MQTT STUFF
typedef struct MQTT_CLIENT_DATA_T_ {
    mqtt_client_t* mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    uint8_t data[MQTT_OUTPUT_RINGBUF_SIZE];
    uint8_t topic[100];
    uint32_t len;
} MQTT_CLIENT_DATA_T;
MQTT_CLIENT_DATA_T *mqtt;
struct mqtt_connect_client_info_t mqtt_client_info=
{
  "picoClient",
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

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    int _itemp;
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
    else
    if (strcmp(mqtt_client->topic, TOPIC_ESCALIER_HAUT)==0)
    {
      if (strcmp(mqtt_client->data,"ON")==0)
         lightStatus = 1;
      else if(strcmp(mqtt_client->data,"OFF")==0)
         lightStatus = 0;
      printf("LightStatus : %d\n",lightStatus);
    }
    else
    if (strcmp(mqtt_client->topic, TOPIC_ESCALIER_THRESHOLD)==0)
    {
      _itemp = atoi(mqtt_client->data);
      if(_itemp < 1)
          _itemp=500;
      mpuThreshold=(float) _itemp;
      printf("mpu6050 FFT Peak Threshold set to %0.f\n",mpuThreshold);
    }
    else
    if (strcmp(mqtt_client->topic, TOPIC_ESCALIER_DELAY)==0)
    {
      _itemp = atoi(mqtt_client->data);
      if(_itemp < 1)
          _itemp=120;
      lightDelay=_itemp;
      printf("Light ON delay set to %d\n",lightDelay);

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
    mqtt_sub_unsub(client,
            TOPIC_ESCALIER_HAUT, 0,
            mqtt_request_cb, arg,
            1);
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

#endif
///////// MPU6050 FUNCTION  & I2C
// By default these devices  are on bus address 0x68

#define FIFO_ENABLE

static int addr = 0x68;

int I2C_WByte(uint8_t reg, uint8_t value)
{
  uint8_t buf[2];
  /* i2c_default, addr, and nostop = true (master retains bus control) */
  buf[0]=reg;
  buf[1]=value;
  return i2c_write_blocking (i2c_default, addr, buf,2, false);
}

int  I2C_RByte(uint8_t reg, uint8_t * value)
{
  uint8_t buf[1];
  /* i2c_default, addr, and nostop = true (master retains bus control) */
  buf[0]=reg;
  i2c_write_blocking (i2c_default, addr, buf,1, true);
  return i2c_read_blocking(i2c_default,addr,value,1,false);
}


uint8_t mpu6050_status()
{
  uint8_t value;
  I2C_RByte(MPU6050_RA_INT_STATUS,&value);
  return value;
}


static void mpu6050_reset() {
    int loop;
    int TheReg;

    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t tbuf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, addr, tbuf, 2, false);

    sleep_ms(1000);
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    //Sets sample rate to 8000/1+15 = 500Hz
    //I2C_WByte(MPU6050_RA_SMPLRT_DIV,15);
    //Sets sample rate to 8000/1+7 = 1000Hz
    I2C_WByte(MPU6050_RA_SMPLRT_DIV,(8000/FSAMP) - 1);
    //Disable gyro self tests, scale of 500 degrees/s
    I2C_WByte(MPU6050_RA_GYRO_CONFIG, 0b00001000);
//    //Disable accel self tests, scale of +-2g, no DHPF
//    I2C_WByte(MPU6050_RA_ACCEL_CONFIG, 0x0);
    // set +/- 4g
    I2C_WByte(MPU6050_RA_ACCEL_CONFIG, 0x08);


    //Sets clock source to gyro reference w/ PLL
    I2C_WByte(MPU6050_RA_PWR_MGMT_1, 0b00000010);
    //Controls frequency of wakeups in accel low power mode plus the sensor standby modes
    I2C_WByte(MPU6050_RA_PWR_MGMT_2, 0x00);
    //MPU6050_RA_WHO_AM_I             //Read-only, I2C address
    I2C_WByte(MPU6050_RA_INT_ENABLE, 0x01);

    printf("\r\nMPU6050 Setup Complete\r\n");
}

static void mpu6050_read_acc(int16_t accel[3])
{
   uint8_t buffer[6];
   uint8_t val = 0x3B;
   i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
   i2c_read_blocking(i2c_default, addr, buffer, 6, false);
   for(int i=0;i<3;i++)
       accel[i] = buffer[i*2]<<8 | buffer[(i*2)+1];
}


// FFT DECLARATION

    volatile int current_in=1;
    kiss_fft_scalar fft_in1[NSAMP]; // kiss_fft_scalar is a float
    kiss_fft_scalar fft_in2[NSAMP]; // kiss_fft_scalar is a float
    kiss_fft_cpx fft_out[NSAMP];
    kiss_fftr_cfg cfg;
    float  max_power = 0;
    int max_idx = 0;


// DO_FFT this is the thread to read MPU6050 on second cpu and send UDP
void Do_FFT()
{
         char buffer[128];
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
              spectrum[i]= (unsigned short) power;
              if (power>max_power) {
	          max_power=power;
	          max_idx = i;
              }
           }
            sprintf(buffer," Freq: %.1f   G:%.03f\n\0", freqs[max_idx],max_power);
            printf(buffer);
#ifdef ENABLE_UDP
          if(max_power > mpuThreshold)
          {
             spectrum[0]=max_idx;
            int ln = strlen(UDP_buffer);
            SendUDP(SEND_TO_IP,SEND_TO_PORT,spectrum,sizeof(unsigned short) *  (NSAMP/2));
          }
#endif
#ifdef ENABLE_MQTT
         if(mpuEnable)
             { // are we in the nigth
              if(max_power > mpuThreshold)
                  {  // ok we got someting
                   if(lightStatus==0)
                       { // light is off then turn it on
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
                      if((get_absolute_time() - startOnTime) > (1000000 * lightDelay))
                         {
                          publish(mqtt->mqtt_client_inst,TOPIC_ESCALIER_POWER,"OFF\0",4);
                          startOnTime=get_absolute_time();
                         }
                    }
                  else
                    weSetLightOn=0;
             }
#endif
          multicore_fifo_push_blocking(check_in);
          }
}

int main() {
    stdio_init_all();

    // convert  mpu6050 digital value to milli g  (gravity)
    float gFactor = 4000.0 / 32767.0;

    int first=1;  // first record not done yet
    int loop=0;
    int recordIdx=0;
    extern struct netif gnetif;
    int16_t acceleration[3];
    float g[4];

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


    // set I2C pins
    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

#ifdef ENABLE_UDP
    // create UDP
    send_udp_pcb = udp_new();
#endif

#ifdef ENABLE_MQTT
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
#endif

    // reset MPU6050
    mpu6050_reset();


    // launch thread to calculate FFT and publish via UDP
    multicore_launch_core1(Do_FFT);

    while (1) {

       // do we have MPU6050 data
       if(mpu6050_status() & 1)
        {
          mpu6050_read_acc(acceleration);
          g[0] = (float) acceleration[0];
          g[1] = (float) acceleration[1];
          g[2] = (float) acceleration[2];
          g[3] = sqrt(g[0]*g[0]+ g[1]*g[1] + g[2]*g[2]) * gFactor;
          // alternate data record  for the thread FFT calculation
          //  calculate FFT on one and store on the other
          //   g[3]= sintable[recordIdx];   pi test purpose
          if(current_in == 1)
            fft_in1[recordIdx] = g[3];
          else
            fft_in2[recordIdx] = g[3];
          recordIdx++;
        }
        // did we fill completely the record
        if(recordIdx>=NSAMP)
        {
          // is it the first record
          // if no  then unblock
          if(first==0)
                multicore_fifo_pop_blocking();
          if(current_in == 1)
            {
             // copy second half to next fft_in1
               memcpy(fft_in2,&fft_in1[NSAMP/2],sizeof(float)*NSAMP/2);
             }
           else
            {
               memcpy(fft_in1,&fft_in2[NSAMP/2],sizeof(float)*NSAMP/2);
            }

          // ok ready for fft thread specify shich  record
          // and push blocking (this enable the thread to start calculating)
          first=0;
          multicore_fifo_push_blocking(current_in);

          // prepare to record next dat on alternate record
          recordIdx=NSAMP/2;
          current_in = (current_in) == 1 ? 2 :1;
        }
    }
    kiss_fft_free(cfg);
    return 0;
}
