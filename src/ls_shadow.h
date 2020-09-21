/*
 * Copyright (C) 2020 LeakSentinel, Inc. or its affiliates.  All Rights Reserved.
 *
 */

#ifndef _LS_SHADOW_
#define _LS_SHADOW_

#include "queue.h"
#include "iot_mqtt.h"

#define SHADOW_MAJOR_VER    1
#define SHADOW_MINOR_VER    3

/* DEVICE */
/* shadow report only values */
#define DEVICE_MAC_ADDRESS                "mac_addr"        /* device.my_mac */
#define DEVICE_FW_VERSION                 "fw_version"      /* device.fw_version */
#define DEVICE_PROVISIONED                "dev_provisioned" /* ? device.dev_provisioned */
#define DEVICE_INSTALLED                  "dev_installed"   /* ? device.dev_installed */
#define DEVICE_TESTED                     "dev_tested"      /* ? device.dev_tested */
#define DEVICE_STATE                      "dev_status"      /* ? device.dev_tested */
/* VALVE */
#define VALVE_TYPE                        "valve_type"      /* my_valve_type */
#define VALVE_STATE                       "valve_state"

/* BATTERY read-only */
#define BATTERY_STATE                     "battery_state"
#define SOURCE_STATE                      "source_state"
#define BATTERY_VOLTAGE                   "battery_voltage"
#define BUS_VOLTAGE                       "bus_voltage"
#define BUS_CURRENT                       "bus_current"
#define BATTERY_CURRENT                   "battery_current"
#define SYS_VOLTAGE                       "sys_voltage"
#define TS_TEMP                           "ts_temperature"
#define DIE_TEMP                          "die_temperature"
#define CHARGE_VOLTAGE                    "charge_voltage"
#define CHARGE_CURRENT                    "charge_current"

/* ADC - ESP32 sampling  read-only */
#define ADC_SAMP_NUM                      "adc_numsamples" /* lsADC_NumSmpls cloud writeable value */

/* MOTOR read-only variables */
#define MOTOR_STATE                       "motor_state"
#define MOTOR_DELAY_TIME                  "delayT"  /* lsMTR_delayStartMS */
#define MOTOR_MAX_CURRENT                 "maxI"    /* lsMTR_currentLimHigh */
#define MOTOR_MAX_TIME                    "maxT"    /* lsMTR_maxTimeS */
#define MOTOR_BACK_TIME                   "backT"   /* lsMTR_turnBackmS */
#define MOTOR_RUN_TIME                    "runTime" /* lsMTR_runTimeMS */

/* cloud writeable values */
#define VALVE_STATE_REQ                   "valve_state_req"
#define VALVE_STATE_COUNTER               "valve_state_counter"

/* WIFI read-only */
#define WIFI_SSID                         "ssid"
#define WIFI_RSSI                         "rssi"
#define WIFI_DISCONNECTED_TIME            "disconnect_time"
/* #define WIFI_CONNECT_FREQ_NOT_SETUP             "connect_not_setup" */
/* #define WIFI_CONNECT_FREQ_FLOWING               "connect_flowing" */
#define WIFI_CONNECT_AWAKE_NOT_FLOWING    "connect_not_flowing"
#define WIFI_CONNECT_SLEEP_MULTIPLIER     "connect_sleep_multiplier"
/* to locally record provisioned status */
#define WIFI_PROVISIONED                  "wifi_provisioned"
/* at the end of the shadow report */
#define DEVICE_UPTIME                     "uptime"     /* xTaskGetTickCount()/100 */
#define SHADOW_VERSION                    "shadow_ver" /* SHADOW_VER */
#define HEAP_SIZE                         "free_heap"
#define LAST_OTA_UPDATED                  "ota_checked"

typedef struct
{
    /* checksum */
    uint8_t checkSum; /* this checksum will make the sum of the structure == 0 (modulo 256) */
                      /* shadow */
    uint8_t shadowVersionMajor;
    uint8_t shadowVersionMinor;
    /* valve */
    uint8_t valveType;
    uint8_t valveState;
    /* adc */
    uint8_t adcNumSamp;
    /* motor */
    uint16_t mtrDelayTime;
    uint16_t mtrMaxI; /* divide by 100 to get correct floating point value */
    uint8_t mtrMaxT;
    uint16_t mtrBackT;
    /* requested */
    uint8_t valveStateReq;
    uint16_t valveStateCounter;
    /* wifi */
    uint16_t connect_not_flowing;
    uint16_t connect_sleep_multiplier;
    uint8_t provisioned;
} shadowLSStruct_t;

typedef struct
{
    char ssid[ 64 ];
    int8_t rssi;
    uint64_t disconnected_time;
    bool initialized;
    bool provisioned;
    bool connected;
    bool time_updated;
} wifi_state_t;

typedef struct
{
    char my_mac[ 25 ];
    char fw_version[ 50 ];
    /* char dsp_fw_version[50]; */
    bool dev_provisioned;
    bool dev_setup;
    bool dev_installed;
    bool dev_tested;
    uint8_t dev_status; /* bit fields to be defined */
} device_info_t;

/**
 * @brief The expected size of #SHADOW_REPORTED_JSON.
 *
 * Because all the format specifiers in #SHADOW_REPORTED_JSON include a length,
 * its full size is known at compile-time.
 */
#define EXPECTED_REPORTED_JSON_SIZE    1000

#define shadowREPORT_VALVE_REQUESTED                 \
    "{\"state\":{\"desired\":{"                      \
    "\"requested\": {\"%s\":\"%d\",\"%s\":\"%d\"}}," \
    "\"reported\":{"                                 \
    "\"valve\":{\"%s\":\"%d\"}}"                     \
    "},"                                             \
    "\"clientToken\":\"token-%d\""                   \
    "}"

#define shadowREPORT_NULL_DESIRED   \
    "{\"state\":{\"desired\":null," \
    "\"reported\":{"                \
    "\"device\":{"                  \
    "\"%s\":\"%s\","                \
    "\"%s\":\"%s\","                \
    "\"%s\":\"%d\","                \
    "\"%s\":\"%d\","                \
    "\"%s\":\"%d\","                \
    "\"%s\":\"%d\""                 \
    "},\"valve\":{"                 \
    "\"%s\":\"%d\","                \
    "\"%s\":\"%d\""                 \
    "},\"adc\":{"                   \
    "\"%s\":\"%d\""                 \
    "},\"motor\":{"                 \
    "\"%s\":\"%d\","                \
    "\"%s\":\"%d\","                \
    "\"%s\":\"%d\","                \
    "\"%s\":\"%d\","                \
    "\"%s\":\"%d\","                \
    "\"%s\":\"%d\""                 \
    "},\"requested\":{"             \
    "\"%s\":\"%d\","                \
    "\"%s\":\"%d\""                 \
    "},\"wifi\":{"                  \
    "\"%s\":\"%s\","                \
    "\"%s\":\"%3d\","               \
    "\"%s\":\"%lld\","              \
    "\"%s\":\"%d\","                \
    "\"%s\":\"%d\""                 \
    "},"                            \
    "\"%s\":\"%d\","                \
    "\"%s\":\"%lld\","              \
    "\"%s\":\"%ld\","               \
    "\"%s\":\"%d.%d\""              \
    "}},"                           \
    "\"clientToken\":\"token-%d\""  \
    "}"

#if USE_SHADOW_DESIRED_SECTION

    #define shadowREPORT_JSON      \
    "{\"state\":{\"desired\":{"    \
    "\"requested\":{"              \
    "\"%s\":\"%d\"}},"             \
    "\"reported\":{"               \
    "\"device\":{"                 \
    "\"%s\":\"%s\","               \
    "\"%s\":\"%s\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\""                \
    "},\"valve\":{"                \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\""                \
    "},\"battery\":{"              \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\""                \
    "},\"adc\":{"                  \
    "\"%s\":\"%d\""                \
    "},\"motor\":{"                \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\""                \
    "},\"requested\":{"            \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\""                \
    "},\"wifi\":{"                 \
    "\"%s\":\"%s\","               \
    "\"%s\":\"%3d\","              \
    "\"%s\":\"%lld\","             \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\""                \
    "},"                           \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%lld\","             \
    "\"%s\":\"%ld\","              \
    "\"%s\":\"%d.%d\""             \
    "}},"                          \
    "\"clientToken\":\"token-%d\"" \
    "}"
#else /* !USE_SHADOW_DESIRED_SECTION */

    #define shadowREPORT_JSON      \
    "{\"state\":{"                 \
    "\"reported\":{"               \
    "\"device\":{"                 \
    "\"%s\":\"%s\","               \
    "\"%s\":\"%s\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\""                \
    "},\"valve\":{"                \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\""                \
    "},\"adc\":{"                  \
    "\"%s\":\"%d\""                \
    "},\"motor\":{"                \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\""                \
    "},\"requested\":{"            \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\""                \
    "},\"wifi\":{"                 \
    "\"%s\":\"%s\","               \
    "\"%s\":\"%3d\","              \
    "\"%s\":\"%lld\","             \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%d\""                \
    "},"                           \
    "\"%s\":\"%d\","               \
    "\"%s\":\"%lld\","             \
    "\"%s\":\"%ld\","              \
    "\"%s\":\"%d.%d\""             \
    "}},"                          \
    "\"clientToken\":\"token-%d\"" \
    "}"
#endif /* !USE_SHADOW_DESIRED_SECTION */

#endif /* _LS_SHADOW_ */
