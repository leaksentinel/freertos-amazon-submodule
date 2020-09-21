/*
 * Copyright (C) 2020 LeakSentinel, Inc. or its affiliates.  All Rights Reserved.
 *
 */

#include "esp_system.h"
#include "esp_interface.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_attr.h"         /* for pwm */
#include "queue.h"            /* for gpio */
#include "driver/adc.h"       /* for adc */
#include "driver/mcpwm.h"     /* for pwm */
#include "soc/mcpwm_reg.h"    /* for pwm */
#include "soc/mcpwm_struct.h" /* for pwm */
#include "esp_log.h"
#include "iot_network_manager_private.h"
#include "driver/rtc_io.h" /* for wakeup trigger defines */
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "driver/timer.h"
#include "soc/timer_group_struct.h"
#include "esp_intr_alloc.h"
/* #include "iot_ble_wifi_provisioning.h" */
#include "iot_wifi.h"

#include "ls_debug.h"
#include "ls_typedefs.h"
#include "ls_board_setup.h"
#include "ls_gpio.h"
#include "ls_shadow.h"
#include "ls_device_state.h"
#include "ls_globals.h"

#define TAG    "GPIO"

/* -------------------------------------------------------------------------------------- */

// static void btn_handler( void * );

// extern void bqstat_handler( void * arg );

/* static button_event_t ev; */
// static QueueHandle_t button_events;
// static QueueHandle_t ls_mtr_queue;
xQueueHandle gpio_evt_queue;

static void IRAM_ATTR gpio_isr_handler( void * arg )
{
    uint32_t gpio_num = ( uint32_t ) arg;

    xQueueSendFromISR( gpio_evt_queue, &gpio_num, NULL );
}

void enable_bqstat_intr( void )
{
    esp_err_t ret;

    ESP_LOGI( TAG, "enable_bqstat_intr: enabling interrupts" );
    ret = gpio_intr_enable( LS_CHRG_INT_N );
    ret += gpio_intr_enable( LS_CHRG_PG_N );

    if( ret != ESP_OK )
    {
        ESP_LOGI( TAG, "enable_bqstat_intr: could not enable interrupts" );
    }
}

void disable_bqstat_intr( void )
{
    esp_err_t ret;

    ESP_LOGI( TAG, "enable_bqstat_intr: disabling interrupts" );
    ret = gpio_intr_disable( LS_CHRG_INT_N );
    ret += gpio_intr_disable( LS_CHRG_PG_N );

    if( ret != ESP_OK )
    {
        ESP_LOGI( TAG, "disable_bqstat_intr: could not disable interrupts" );
    }
}

void ls_mtr_send_event( int64_t timestamp,
                        int direction )
{
    // motor_event_t mtr_event =
    // {
    //     .timestamp = timestamp,
    //     .direction = direction,
    // };

    ESP_LOGI( TAG, "ls_mtr_send_event: direction(%d)", direction );
    // xQueueSend( ls_mtr_queue, &mtr_event, portMAX_DELAY );
}

/* button handler (task runs in the background and listens to the event queue) */
// static void btn_handler( void * arg )
// {
//     button_event_t ev;

//     for( ; ; )
//     {
//         if( xQueueReceive( button_events, &ev, portMAX_DELAY ) )
//         {
//             start_sleepExtTimer(); /* we got a button action - let's stay awake now for 5 min */
//             ESP_LOGI( TAG, "btn_handler: Received button %d event %d", ev.pin, ev.event );
//             device.dev_status &= (~STATUS_BUTTON_STAY_AWAKE);
//             device.dev_status |= (STATUS_BUTTON_STAY_AWAKE);
//             /* ESP_LOGI(TAG, "btn_handler: waiting for button lockout semaphore"); */
//             /* IotSemaphore_Wait(&buttonLockoutSemaphore); */
//             if( ( ev.pin == LS_BTN1_N ) && ( ev.event == BUTTON_DOWN_LONG ) )
//             {
//                 if( ( my_valve_type != TUNKNOWN ) && ( ( my_valve_state == VUNKNOWN ) || ( my_valve_state == VOPEN ) || ( my_valve_state == VOPENING ) ) )
//                 {
//                     my_button_state = BUTTON_PRESSED;
//                     my_valve_pending = true;
//                     ESP_LOGI( TAG, "btn_handler: Current valve state %s, will now close", valve_state_str[ my_valve_state ] );
//                     // ls_mtr_send_event( esp_timer_get_time(), LS_CW_VALVE );
//                 }
//                 else
//                 {
//                     ESP_LOGI( TAG, "btn_handler: Current valve state %s, will do nothing new", valve_state_str[ my_valve_state ] );
//                 }
//             }
//             else if( ( ev.pin == LS_BTN2_N ) && ( ev.event == BUTTON_DOWN_LONG ) )
//             {
//                 if( ( my_valve_type != TUNKNOWN ) && ( ( my_valve_state == VUNKNOWN ) || ( my_valve_state == VCLOSED ) || ( my_valve_state == VCLOSING ) ) )
//                 {
//                     my_button_state = BUTTON_PRESSED;
//                     my_valve_pending = true;
//                     ESP_LOGI( TAG, "btn_handler: Current valve state %s, will now open", valve_state_str[ my_valve_state ] );
//                     // ls_mtr_send_event( esp_timer_get_time(), LS_CCW_VALVE );
//                 }
//                 else
//                 {
//                     ESP_LOGI( TAG, "btn_handler: Current valve state %s, will do nothing new", valve_state_str[ my_valve_state ] );
//                 }
//             }
//             else if( ( ev.pin == LS_BTN1_N ) && ( ev.event == BUTTON_UP ) )
//             {
//                 if( ( my_valve_state == VCLOSING ) || ( my_valve_state == VOPENING ) )
//                 {
//                     ESP_LOGI( TAG, "btn_handler: Current valve state %s, will now stop", valve_state_str[ my_valve_state ] );
//                     my_button_state = BUTTON_PRESSED;
//                     my_valve_pending = true;
//                     ls_mtr_send_event( esp_timer_get_time(), LS_STOP_VALVE );
//                 }
//                 else
//                 {
//                     ESP_LOGI( TAG, "btn_handler: Current valve state %s, will do nothing new", valve_state_str[ my_valve_state ] );
//                 }
//             }
//             else if( ( ev.pin == LS_BTN2_N ) && ( ev.event == BUTTON_UP ) )
//             {
//                 if( ( my_valve_state == VCLOSING ) || ( my_valve_state == VOPENING ) )
//                 {
//                     ESP_LOGI( TAG, "btn_handler: Current valve state %s, will now stop", valve_state_str[ my_valve_state ] );
//                     my_button_state = BUTTON_PRESSED;
//                     my_valve_pending = true;
//                     ls_mtr_send_event( esp_timer_get_time(), LS_STOP_VALVE );
//                 }
//                 else
//                 {
//                     ESP_LOGI( TAG, "btn_handler: Current valve state %s, will do nothing new", valve_state_str[ my_valve_state ] );
//                 }
//             }

//             /* IotSemaphore_Post(&buttonLockoutSemaphore); // let's make the semaphore active again (== 1) and available to grab */
//         }
//     }
// }

void setup_hardware( void )
{
    // esp_err_t bqerr;
    gpio_config_t io_conf;

    /* esp_err_t err_ret; */

    /* Output setup */
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;      /* disable interrupt */
    io_conf.mode = GPIO_MODE_OUTPUT;                /* set as output mode */
    io_conf.pull_down_en = 0;                       /* disable pull-down mode */
    io_conf.pull_up_en = 0;                         /* disable pull-up mode */
    io_conf.pin_bit_mask = GPIO_OUT_LEDS_PIN;       /* bit mask of the pins that you want to set,e.g.GPIO18/19 */
    gpio_config( &io_conf );                        /* configure GPIO with the given settings */
    io_conf.pin_bit_mask = GPIO_OUT_MTR_PIN;        /* do the same for the motor pins */
    gpio_config( &io_conf );
    io_conf.pull_up_en = 1;                         /* enable pull-up mode */
    io_conf.pin_bit_mask = ( GPIO_OUT_BATT_PIN );   /* do the same for the battery measure control pin */
    gpio_config( &io_conf );
    gpio_set_level( ( gpio_num_t ) LS_BATTM_N, 1 ); /* disable the battery measurement */
    /* io_conf.pin_bit_mask = GPIO_OUT_UARTTX_PIN;  // do the same for the uarttx pins */
    /* gpio_config(&io_conf); */
    /* io_conf.pin_bit_mask = GPIO_OUT_I2CSCL_PIN;  // do the same for the i2c pins */
    /* gpio_config(&io_conf); */
    /* io_conf.pin_bit_mask = GPIO_OUT_DSP_PIN;     // do the same for the DSP comm pins */
    /* gpio_config(&io_conf); */

    /* Output OD setup */
    /* io_conf.intr_type = GPIO_PIN_INTR_DISABLE;   // disable interrupt */
    /* io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;    // set as output mode */
    /* io_conf.pull_down_en = 0;                    //disable pull-down mode */
    /* io_conf.pull_up_en = 0;                      //disable pull-up mode */
    /* io_conf.pin_bit_mask = GPIO_OUT_I2CSDA_PIN;  // do the same for the i2c pins */
    /* done elsewhere */
    /* gpio_config(&io_conf); */

    /* Input setup */
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE; /* disable interrupt */
    io_conf.mode = GPIO_MODE_INPUT;            /* set as input mode */
    io_conf.pull_down_en = 0;                  /* disable pull-down mode */
    io_conf.pull_up_en = 0;                    /* disable pull-up mode */
    io_conf.pin_bit_mask = GPIO_IN_PWR_PIN;    /* do the same for the motor pins */
    gpio_config( &io_conf );
    io_conf.pin_bit_mask = GPIO_IN_MTRI_PIN;   /* do the same for the motor voltage measure pin */
    gpio_config( &io_conf );
    io_conf.pin_bit_mask = GPIO_IN_BATTV_PIN;  /* do the same for the battery voltage measure pin */
    gpio_config( &io_conf );
    io_conf.pin_bit_mask = GPIO_IN_DSP_PIN;    /* do the same for the DSP comm pins */
    gpio_config( &io_conf );
    /* io_conf.pin_bit_mask = GPIO_IN_UARTRX_PIN;   // do the same for the uart rx pin */
    /* gpio_config(&io_conf); */

    // bqerr = setup_batt_charger(); /* setup i2c for bq2588x */

    // if( bqerr != ESP_OK )
    // {
    //     ESP_LOGE( TAG, "setup_hardware: Battery charger not detected" );
    // }

    /* configure BQ status pins as input with interrupts enabled */
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE; /* enable interrupt on neg edge not any edge? */
    io_conf.mode = GPIO_MODE_INPUT;            /* set as input mode */
    io_conf.pull_down_en = 0;                  /* disable pull-down mode */
    io_conf.pull_up_en = 1;                    /* enable pull-up mode but we have an external resistor */
    io_conf.pin_bit_mask = GPIO_IN_BQSTAT_PIN;
    gpio_config( &io_conf );

    xTaskCreate( device_state_task, "devStateT", 16 * 512, NULL, PRIORITY_DEVICE_STATE, &xDeviceState );

    gpio_evt_queue = xQueueCreate( 6, sizeof( uint32_t ) );                                  /* create a queue to handle gpio event from isr */
    // xTaskCreate( bqstat_handler, "bqstat_handlerT", 5 * 512, NULL, PRIORITY_BATTERY, NULL ); /* start battery monitor task */

    gpio_install_isr_service( ESP_INTR_FLAG_DEFAULT );                                       /* install gpio isr service */
    gpio_isr_handler_add( LS_CHRG_INT_N, gpio_isr_handler, ( void * ) LS_CHRG_INT_N );       /* hook isr handler for specific gpio pin */
    gpio_isr_handler_add( LS_CHRG_PG_N, gpio_isr_handler, ( void * ) LS_CHRG_PG_N );         /* hook isr handler for specific gpio pin */

    // setup_adc_mtri();

    /* Input setup */
    // ls_mtr_queue = xQueueCreate( 4, sizeof( motor_event_t ) );
    // mtr_init( &ls_mtr_queue );

    /* this function sets up the inputs to not use interrupts */
    // button_events = button_init( GPIO_IN_BTN_PIN );
    /*start gpio task */
    // xTaskCreate( btn_handler, "btnHandT", 4 * 512, NULL, PRIORITY_BUTTON, NULL );

    // setupLED();
}
