#ifndef TUNING_H
#define TUNING_h

#include <stdio.h>
#include <math.h>
#include <time.h>

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include <stdlib.h>

#define EXAMPLE_WIFI_SSID "SUA"
#define EXAMPLE_WIFI_PASS "12345678"

esp_err_t event_handler(void *ctx, system_event_t *event);
void initialise_wifi();
void http_server_netconn_serve(struct netconn *conn,float *setpoint,float *pitchKp,float *pitchKd,float *pitchKi,float *yaw_kP,float *yaw_kD,float *yaw_kI);

#endif
