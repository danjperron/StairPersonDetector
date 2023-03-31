/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/cyw43_arch.h"
//#include "pico/multicore.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
//#include "pi.h"  // test purpose
#include "tof.h"
#include "hardware/i2c.h"
#include <time.h>
#include "lwip/apps/mqtt.h"



#define MQTT_HOST_IP "10.11.12.192"
#define MQTT_HOST_PORT 1883

#define TOPIC_ESCALIER_BAS   "stat/escalierBas/POWER"
#define TOPIC_ESCALIER_HAUT  "stat/escalierHaut/POWER"
#define TOPIC_ESCALIER_POWER  "cmnd/escalier/POWER"
// let the light on for 2 mins (120 secs)
#define LIGHT_WAIT_DELAY  120


#define ENABLE_UDP
#undef ENABLE_UDP

int thresholdDistance= 2000;
int8_t lightStatus=0;
int8_t dayStatus=0;


absolute_time_t startOnTime=0;  //  timestamp for ligt on delay

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


#ifdef ENABLE_UDP
//  UDP SEND TO HOST IP/POR

#define  SEND_TO_IP  "10.11.12.104"
#define  SEND_TO_PORT 6001


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


#endif

int main() {
    int i;
    int iDistance;
    int model, revision;
    char buffer[256];
    startOnTime=get_absolute_time();


    stdio_init_all();


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
    i2c_init(i2c_default, 100 * 1000);
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
    // create mqtt
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


    // get model revision
    tofGetModel(&model, &revision);
    printf("Model ID - %d\n", model);
    printf("Revision ID - %d\n", revision);

    i = tofInit(0, 0x29, 1); // set long range mode (up to 2m)
    if (i != 1)
    {
        printf("Error accessing VL53L0X device\n");
        return -1; // problem - quit
    }
    printf("VL53L0X device successfully opened.\n");

    while (1) 
        {
         iDistance = tofReadDistance();
//       printf("\nDistance = %dmm\n", iDistance);
#ifdef ENABLE_UDP
         sprintf(buffer,"%d\n",iDistance);
         SendUDP(SEND_TO_IP,SEND_TO_PORT,buffer,strlen(buffer));
#endif
         if(dayStatus ==0)
             { // are we in the nigth
              if(iDistance < thresholdDistance)
                  {  // ok we got someting
                   if(lightStatus==0)
                       { // light is off then turn it on
                        publish(mqtt->mqtt_client_inst,TOPIC_ESCALIER_POWER,"ON\0",3);
                       }
                       // reset startOnTime
                   startOnTime=get_absolute_time();
                  }
              else
                  {
                   // ok no movemenent for n times we should turn i off
                   if(lightStatus==1)
                      if((get_absolute_time() - startOnTime) > (1000000 * LIGHT_WAIT_DELAY))
                         {
                          publish(mqtt->mqtt_client_inst,TOPIC_ESCALIER_POWER,"OFF\0",4);
                          startOnTime=get_absolute_time();
                         }
                  }
             }
//        else
//            printf(".");
        sleep_ms(50); // 50ms
      }
    return 0;
}
