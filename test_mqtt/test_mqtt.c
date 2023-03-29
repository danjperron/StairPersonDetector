 #include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/apps/mqtt.h"
//#include "ntp_time.h"

#define WIFI_SSID "yourESSID"
#define WIFI_PASSWORD "your passwor"
#define MQTT_HOST_IP "BROKER IP"
#define MQTT_HOST_PORT BORKER_PORT

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
    printf(mqtt_client->topic);
    printf("\n data:");
    printf(mqtt_client->data);
    printf("\n");
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
    printf("MQTT clien accepted\n");
    mqtt_sub_unsub(client,
            "mypico/#", 0,
            mqtt_request_cb, arg,
            1);

/*    mqtt_sub_unsub(client,
            "mypico/start", 0,
            mqtt_request_cb, arg,
            1);
    mqtt_sub_unsub(client,
            "mypico/stop", 0,
            mqtt_request_cb, arg,
            1);
*/
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



int main()
{
    stdio_init_all();

    mqtt=(MQTT_CLIENT_DATA_T*)calloc(1, sizeof(MQTT_CLIENT_DATA_T));

    if (!mqtt) {
        printf("mqtt client instant ini error\n");
        return 0;
    }
    mqtt->mqtt_client_info = mqtt_client_info;

    if (cyw43_arch_init())
    {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_pm_value(CYW43_NO_POWERSAVE_MODE,200,1,1,10);
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("failed to connect\n");
        return 1;
    }
//    ntp_time_init();
//    get_ntp_time();

    printf("connected\n");
    ip_addr_t addr;
    if (!ip4addr_aton(MQTT_HOST_IP, &addr)) {
        printf("ip error\n");
        return 0;
    }


    mqtt->mqtt_client_inst = mqtt_client_new();
    mqtt_set_inpub_callback(mqtt->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, mqtt);

    err_t err = mqtt_client_connect(mqtt->mqtt_client_inst, &addr, MQTT_HOST_PORT, &mqtt_connection_cb, mqtt, &mqtt->mqtt_client_info);
    if (err != ERR_OK) {
      printf("connect error\n");
      return 0;
    }
    printf("Client connected\n");


    publish(mqtt->mqtt_client_inst,"mypico/state","ON\0",3);
    while(1) {
      sleep_ms(500);
      putchar('.');
    }
    return 0;
}
