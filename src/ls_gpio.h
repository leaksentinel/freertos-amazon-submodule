/*
 * Copyright (C) 2020 LeakSentinel, Inc. or its affiliates.  All Rights Reserved.
 *
 */

#ifndef _LS_GPIO_
#define _LS_GPIO_


#include "ls_board_setup.h"

#define ESP_INTR_FLAG_DEFAULT    0

#define GPIO_OUT_LEDS_PIN        ( 1ULL << LS_LEDBTN2C ) | ( 1ULL << LS_LEDBTN2A ) | ( 1ULL << LS_LEDBTN1C ) | ( 1ULL << LS_LEDBTN1A )
#define GPIO_OUT_MTR_PIN         ( 1ULL << LS_MTREN ) | ( 1ULL << LS_MTR1 ) | ( 1ULL << LS_MTR2 )
#define GPIO_OUT_BATT_PIN        ( 1ULL << LS_BATTM_N )
#define GPIO_IN_PWR_PIN          ( 1ULL << LS_V1P8_OK_N ) | ( 1ULL << LS_V3P3_OK_N )
#define GPIO_IN_BQSTAT_PIN       ( 1ULL << LS_CHRG_INT_N ) | ( 1ULL << LS_CHRG_PG_N )
#define GPIO_IN_BTN_PIN          ( 1ULL << LS_BTN1_N ) | ( 1ULL << LS_BTN2_N )
#define GPIO_IN_MTRI_PIN         ( 1ULL << LS_MTRI )
#define GPIO_IN_BATTV_PIN        ( 1ULL << LS_BATTV )
#define GPIO_IN_UARTRX_PIN       ( 1ULL << LS_ESP_RX )

#define GPIO_OUT_DSP_PIN         ( 1ULL << LS_ESPO_IO27 ) | ( 1ULL << LS_ESPO_IO14 ) | ( 1ULL << LS_ESPO_IO2 ) | ( 1ULL << LS_ESPO_IO4 )
#define GPIO_IN_DSP_PIN          ( 1ULL << LS_ESPI_IO36 ) | ( 1ULL << LS_ESPI_IO39 ) | ( 1ULL << LS_ESPI_IO35 )

/* #define GPIO_OUT_UARTTX_PIN (1ULL<<LS_ESP_TX) */
/* #define GPIO_OUT_I2CSCL_PIN (1ULL<<LS_SYS_SCL) */
/* #define GPIO_OUT_I2CSDA_PIN (1ULL<<LS_SYS_SDA) */

void setup_hardware( void );

#endif /* _LS_GPIO_ */
