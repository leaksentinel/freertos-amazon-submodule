/*
 * Copyright (C) 2020 LeakSentinel, Inc. or its affiliates.  All Rights Reserved.
 *
 */

#ifndef _LS_DEVICE_STATE_
#define _LS_DEVICE_STATE_

#include "driver/timer.h"

#define TIMER_DIVIDER          4
#define TIMER_SCALE            ( TIMER_BASE_CLK / TIMER_DIVIDER )
#define TIMER_GROUP_1          1
#define TIMER_NUM_0            0
#define TIMER_NUM_1            1

/* for sleeping... */
#define SLEEP_AFTER_NOPROV     180
#define SLEEP_AFTER_PROV       20
#define REPORT_SHADOW_AWAKE    60
#define REPORT_SHADOW_SLEEP    30
#define DEEP_SLEEP_TIME        360

#define SLEEP                  1 << 0
#define VALVE_OPENING          1 << 1
#define VALVE_CLOSING          1 << 2
#define MOTOR_IDLE             1 << 3
#define MOTOR_TIMEOUT          1 << 4
#define SHADOW_UPDATE          1 << 5
#define TIME_UPDATE            1 << 6
#define OTA_UPDATE             1 << 7

#define RESTART                1 << 8
#define DEVICE_ERROR           1 << 9
#define SAFE_RESTART           1 << 10

#define BATTERY_DISCHARGING    1 << 11
#define BATTERY_FULL           1 << 12
#define BATTERY_CHARGING       1 << 13
#define BATTERY_EMPTY          1 << 14
#define SOURCE_UNPLUGGED       1 << 15
#define SOURCE_GOOD            1 << 16
#define SOURCE_POOR            1 << 17
#define SOURCE_DONTKNOW        1 << 18

#define PROVISIONING           1 << 19
#define PROVISIONED            1 << 20
#define WIFI_CONNECTED         1 << 21

#define ALL_STATES_TO_CLEAR    ( SLEEP | VALVE_OPENING | VALVE_CLOSING | MOTOR_IDLE | MOTOR_TIMEOUT | SHADOW_UPDATE | TIME_UPDATE | OTA_UPDATE )
#define ALL_ERROR_STATES       ( RESTART | DEVICE_ERROR | SAFE_RESTART )
#define ALL_BATTERY_STATES     ( BATTERY_DISCHARGING | BATTERY_FULL | BATTERY_CHARGING | BATTERY_EMPTY )
#define ALL_SOURCE_STATES      ( SOURCE_UNPLUGGED | SOURCE_GOOD | SOURCE_POOR | SOURCE_DONTKNOW )

#define DEVICE_NUM_STATES      22

QueueHandle_t state_handle;
TaskHandle_t xDeviceState;

void device_state_task();

#endif /* _LS_DEVICE_STATE_ */
