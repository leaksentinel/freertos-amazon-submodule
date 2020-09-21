/*
 * Copyright (C) 2020 LeakSentinel, Inc. or its affiliates.  All Rights Reserved.
 *
 */

#ifndef _LS_BOARD_SETUP_
#define _LS_BOARD_SETUP_

#include "driver/adc.h"     /* for adc */

#define LS_ESPI_IO36     36 /* input to ESP32 connection to TI DSP C5535 */
#define LS_ESPI_IO39     39 /* input to ESP32 connection to TI DSP C5535 */
#define LS_V1P8_OK_N     25 /* VCC 1V8 OK# */
#define LS_V3P3_OK_N     26 /* VCC 3V3 OK# */
#define LS_BATTM_N       32 /* BATTERY VOLTAGE MEAS ENABLE (ACTIVE LOW) */
#define LS_MTRI          33 /* MTR CURRENT INPUT */
#define LS_ESPO_IO27     27 /* output from ESP32 connection to TI DSP C5535 */
#define LS_ESPO_IO14     14 /* output from ESP32 connection to TI DSP C5535 */
#define LS_MTREN         12 /* MTR EN OUTPUT */
#define LS_MTR2          13 /* MTR2 GPIO */
#define LS_MTR1          15 /* MTR1 GPIO */
#define LS_ESPO_IO2      2  /* output from ESP32 connection to TI DSP C5535 */
#define LS_ESPO_IO4      4  /* output from ESP32 connection to TI DSP C5535 */
#define LS_CHRG_INT_N    0  /* connection USB charger */

#define LS_BTN1_N        37 /* BTN1 INPUT */
#define LS_BTN2_N        38 /* BTN2 INPUT */
#define LS_BATTV         34 /* BATTERY VOLTAGE MEASURE */
#define LS_ESPI_IO35     35 /* input to ESP32 connection to TI DSP C5535 */
#define LS_ESP_TX        1  /* serial connection to USB & TI DSP 5535 */
#define LS_ESP_RX        3  /* serial connection to USB & TI DSP 5535 */
#define LS_SYS_SDA       9  /* I2C to battery charger */
#define LS_SYS_SCL       10 /* I2C to battery charger */
#define LS_MTRFLT_N      5  /* MTRFLT STATUS INPUT */
#define LS_CHRG_PG_N     18 /* BATTCHRG STATUS INPUT */
#define LS_LEDBTN2C      23 /* SW2 RED LED CATHODE */
#define LS_LEDBTN2A      19 /* SW2 RED LED ANODE */
#define LS_LEDBTN1C      22 /* SW1 RED LED CATHODE */
#define LS_LEDBTN1A      21 /* SW1 RED LED ANODE */


/* for motor current we use ADC_CHANNEL_5 */
#define LS_MTRI_CHANNEL                ADC_CHANNEL_5

/* for battery voltage we use ADC_CHANNEL_6 */
#define LS_BATTV_CHANNEL               ADC_CHANNEL_6

/* for i2c controller to communicate with bq25882 */
#define CONFIG_I2C_MASTER_PORT_NUM     I2C_NUM_0
/* #define CONFIG_I2C_MASTER_FREQUENCY 400000 */
#define CONFIG_I2C_MASTER_FREQUENCY    100000
#define CONFIG_BQ2558X_ADDR            0x6B
#define CONFIG_BQ2558X_OPMODE          0x00

/* ADC MAPPING */

/*
 * ADC1_CH0 IO36/SENSOR_VP
 * ADC1_CH1 IO37/SENSOR_CAPP
 * ADC1_CH2 IO38/SENSOR_CAPN
 * ADC1_CH3 IO39/SENSOR_VN
 * ADC1_CH4 IO32/32K_XP
 * ADC1_CH5 IO33/32K_XN
 * ADC1_CH6 IO34/VDET_1
 * ADC1_CH7 IO35/VDET_2
 * ADC2_CH0 IO4
 * ADC2_CH1 IO0
 * ADC2_CH2 IO2
 * ADC2_CH3 IO15/MTDO
 * ADC2_CH4 IO13/MTCK
 * ADC2_CH5 IO12/MTDI
 * ADC2_CH6 IO14/MTMS
 * ADC2_CH7 IO27
 * ADC2_CH8 IO25
 * ADC2_CH9 IO26
 */

#endif /* _LS_BOARD_SETUP_ */
