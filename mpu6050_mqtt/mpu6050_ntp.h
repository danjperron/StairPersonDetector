
#ifndef MPU6050_NTP
#define MPU6050_NTP
#include "pico/util/datetime.h"



typedef struct NTP_T_ {
    ip_addr_t ntp_server_address;
    bool dns_request_sent;
    struct udp_pcb *ntp_pcb;
    absolute_time_t ntp_test_time;
    alarm_id_t ntp_resend_alarm;
} NTP_T;



#define NTP_SERVER "pool.ntp.org"
#define NTP_MSG_LEN 48
#define NTP_PORT 123
#define NTP_DELTA 2208988800 // seconds between 1 Jan 1900 and 1 Jan 1970
#define NTP_TEST_TIME (30 * 1000)
#define NTP_RESEND_TIME (10 * 1000)
#define NTP_MY_TIMEZONE (-4.0 * 60 ) // in minutessecond


char * stampDate_dt(char * datestamp, datetime_t * dt);
char * stampDate(char * datestamp);
void get_ntp_time();


#endif
