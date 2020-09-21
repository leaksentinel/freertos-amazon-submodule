/*
 * Copyright (C) 2020 LeakSentinel, Inc. or its affiliates.  All Rights Reserved.
 *
 */

/* global defines for LeakSentinel firmware */

#ifndef _LS_TYPEDEFS_
#define _LS_TYPEDEFS_

#include "driver/adc.h" /* for adc */

/* enumerated states for the valve - starts as unknown, eh? */
typedef enum
{
    VUNKNOWN = 0,
    VOPEN = 1,
    VCLOSED = 2,
    VOPENING = 3,
    VCLOSING = 4
} valve_state;

typedef enum
{
    MUNKNOWN = 0,
    MIDLE = 1,
    MCW = 2,
    MCCW = 3,
    MSTALLED = 4,
    MTIMEDOUT = 5
} motor_state;

typedef enum
{
    TUNKNOWN = 0,
    TGATE = 1,
    TBALL = 2,
    TTEE = 3,
    TOTHER = 4
} valve_type;

typedef enum
{
    BUNKNOWN = 0,
    BCHARGED = 1,
    BDISCHARGING = 2,
    BCHARGING = 3,
    BLOW = 4
} battery_state_simple;

typedef enum
{
    BDUNKNOWN = 0,
    BDIDLE = 1,
    BDTRICKLE = 2,
    BDPRE = 3,
    BDTAPER = 4,
    BDTOPOFF = 5,
    BDDONE = 6,
    BDFAST = 7,
    BDRESERVED = 8
} battery_state;

typedef enum
{
    SNO_INPUT = 0,
    SUSB_HOST_SDP,
    SUSB_CDP_1P5A,
    SUSB_DCP_3P0A,
    SPOORSRC,
    SUNKNOWN_ADAPTER,
    SNONSTANDARD_ADAPTER,
    SOTG,
    SUNKNOWN
} source_state;

/* typedef enum { */
/*     WUNKNOWN = 0, */
/*     WCONNECTED = 1, */
/*     WUNCONNECTED = 2, */
/*     WCONNECTING = 3, */
/*     WDISCONNECTED = 4 */
/* } wifi_state; */

typedef enum
{
    FUNKNOWN = 0,
    FNOFLOW = 1,
    FFLOW = 2
} flow_state;

typedef enum
{
    STAY_AWAKE = 0,
    ASLEEP = 1,
    WAKING_UP = 2,
    CAN_SLEEP = 3
} sleep_state;

typedef enum
{
    PWR_LED = 0,
    WIFI_LED = 1
} enumerate_leds;

typedef enum
{
    BUTTON_ACK = 0,
    BUTTON_PRESSED = 1,
    BUTTON_UNKNOWN = 2
} button_state;

/* status bits */
#define STATUS_BUTTON_WAKE                   ( 1 << 0 )
#define STATUS_TIMER_WAKE                    ( 1 << 1 )
#define STATUS_UNEXPECTED_WAKE               ( 1 << 2 )
#define STATUS_UNKNOWN_WAKE                  ( 1 << 3 )
#define STATUS_BUTTON_STAY_AWAKE             ( 1 << 4 )
#define STATUS_MOTOR_STAY_AWAKE              ( 1 << 5 )
#define STATUS_BATTERY_ERRORS                ( 1 << 6 )
#define STATUS_BATTERY_ISSUES                ( 1 << 7 )


#define USE_BUTTON                           true
#define USE_NETWORK                          true
#define LOAD_STATE_FROM_NVS                  true
#define ALLOW_SLEEPING                       true
#define USE_SINGLE_CURRENT_THRESHOLD         true

/* how long to wait for semaphore ready */
#define SEMAPHORE_LONG_TIME                  0xffff
#define SEMAPHORE_MEDIUM_TIME                0x00ff
#define SEMAPHORE_SHORT_TIME                 0x0001
#define SEMAPHORE_POLL_TIME                  0x0000

#define configEXPECTED_NETWORKS              ( AWSIOT_NETWORK_TYPE_WIFI )
#define USE_SINGLE_CURRENT_THRESHOLD         true
#define MAX_RETRY_CONNECTION                 4 /* number of times to try to connect to cloud mqtt broker */
#define MAX_RETRY_BUTTON_WAIT                6
#define OTA_HR_CHECK                         1 /* check every hour for now */
#define EXTEND_AWAKE_MULTIPLIER              1 /* extend awake timer check when wifi not connected */
#define MIN_ACTIVE_TIMER                     1.0
#define WAIT_COUNT                           50
#define WAIT_FOR_SHADOW_SYNC                 30000    /* ms to wait for shadow sync semaphore */
#define KEEP_ALIVE_SECONDS                   ( 60 )
#define OTA_KEEP_ALIVE_SECONDS               ( 1200 ) /* per OTA demo??? */
#define DELAY_ADC_SAMPLES                    0
#define ADD_DELAY_ADC_SAMPLES                0
#define ADDED_ADC_SAMPLE_DELAY               1                   /* pdMS_TO_TICKS(4) */
#define MOTOR_OPEN_BIAS                      0.0108              /* bias for close direction needing higher threshold */
#define WAIT_FOR_UPDATE_SHADOW_TASK          5000                /* ms to wait for update updateShadowTask semaphore */
#define BUTTON_SAMPLE_DELAY                  pdMS_TO_TICKS( 25 ) /* 50/portTICK_PERIOD_MS */
#define SLEEP_EXTENSION_MIN                  5                   /* minutes to extend sleep time when activity is detected (shadow/button) */
#define SLEEP_CYCLE_COUNT_OTA                10

/**
 * @brief The timeout for Shadow and MQTT operations in this demo.
 */
#define AWS_TIMEOUT_MS                       ( 60000 )
#define MAX_SHADOW_CONNECTION_ERROR_COUNT    2
#define MAX_MQTT_CONNECTION_ERROR_COUNT      2
#define MAX_MQTT_RETRY_COUNT                 2
#define MAX_RETRY_SHADOW_COUNT               2
#define MAX_SHADOW_SKIPPED_COUNT             2
#define MAX_WIFI_ERROR_COUNT                 10

/**
 * @brief Number of LEDs to iterate over
 */
#define NUM_LEDS                             2

/* task priority (up to 25) */
#define PRIORITY_SHADOW                      10
#define PRIORITY_OTA                         10
#define PRIORITY_MOTOR                       20
#define PRIORITY_BUTTON                      10
#define PRIORITY_CONNECTION                  10
#define PRIORITY_LED                         10
#define PRIORITY_BATTERY                     5
#define PRIORITY_DEVICE_STATE                10
#define PRIORITY_SLEEP                       10

/* convert ADC voltage reading to motor current */
/* 375 (scaling factor in 33887) * voltage in mV / 200 (ohms) = current in mA */
/* so current in A is 375 * adc (mV) / 1000 mv/V / 200 (ohms) */
/* only changes when we change resistor on the board */
#define CONST_MTR_V2I      1.875e-3
#define CONST_MTR_V2I_U    1875 /* e-6 */

/* for battery voltage measurement */
/* battery voltage * DIV_RATIO within 3.3v? */
/* 2.6586 * (3.3/4095 * ADC.read(BATTV)); */
/* 1/DIV_RATIO = 2.658536585365854 */
/* #define LS_BATT_DIV_RATIO 2.65854 */

/* we only use ADC_1 for all of our measurements */
#define LS_ADC_UNIT      ADC_UNIT_1

/* minimum battery voltage for calculation purposes */
#define LS_BATT_MIN_V    6.4

#endif /* _LS_TYPEDEFS_ */
