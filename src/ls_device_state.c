/*
 * Copyright (C) 2020 LeakSentinel, Inc. or its affiliates.  All Rights Reserved.
 *
 */

#include "time.h"
#include "sys/time.h"

#include "esp_system.h"
#include "esp_interface.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_attr.h"         /* for pwm */
#include "queue.h"            /* for gpio */
#include "esp_log.h"
#include "driver/rtc_io.h"    /* for wakeup trigger defines */
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "driver/timer.h"
#include "soc/timer_group_struct.h"
#include "esp_intr_alloc.h"

/* #include "iot_wifi.h" */
#include "iot_ble.h"
#include "iot_ble_config.h"
#include "iot_ble_wifi_provisioning.h"
#include "iot_ble_numericComparison.h"

#include "esp_log.h"
#include "aws_iot_shadow.h"
#include "iot_network_manager_private.h"
/* #include "aws_ota_codesigner_certificate.h" */
/* #include "aws_wifi_connect_task.h" */

#include "ls_debug.h"
#include "ls_typedefs.h"
#include "ls_board_setup.h"
#include "ls_gpio.h"
#include "ls_shadow.h"
#include "ls_device_state.h"
#include "ls_globals.h"

#define TAG    "DEVICE"

/* -------------------------------------------------------------------------------------- */

static timer_isr_handle_t timer_handle;
static bool is_usb_connected = false;

static uint32_t last_sleepTimer_start = 0;
static bool timeSet = false;
static TaskHandle_t xSleepHandle = NULL;

const char * device_state_str[] =
{
    "SLEEP              ",
    "VALVE_OPENING      ",
    "VALVE_CLOSING      ",
    "MOTOR_IDLE         ",
    "MOTOR_TIMEOUT      ",
    "SHADOW_UPDATE      ",
    "TIME_UPDATE        ",
    "OTA_UPDATE         ",
    "RESTART            ",
    "DEVICE_ERROR       ",
    "SAFE_RESTART       ",
    "BATTERY_DISCHARGING",
    "BATTERY_FULL       ",
    "BATTERY_CHARGING   ",
    "BATTERY_EMPTY      ",
    "SOURCE_UNPLUGGED   ",
    "SOURCE_GOOD        ",
    "SOURCE_POOR        ",
    "SOURCE_DONTKNOW    ",
    "PROVISIONING       ",
    "PROVISIONED        ",
    "WIFI_CONNECTED     "
};

static bool isWiFiProvisionedNVS()
{
    uint8_t ctr = 0;
    uint8_t num_networks = 0;

    /* ESP_LOGI(TAG, "isWiFiProvisionedNVS: dumping nvs table"); */
    /* nvs_dump("nvs"); */
    /* ESP_LOGI(TAG, "isWiFiProvisionedNVS: dumping storage table"); */
    /* nvs_dump("storage"); */

    while( !current_wifi_state.initialized && ctr++ < MAX_RETRY_CONNECTION << 2 )
    {
        ESP_LOGI( TAG, "isWiFiProvisionedNVS: wait 2.5s*(1-16) for current_wifi_state.initialized (%d)", current_wifi_state.initialized );
        vTaskDelay( pdMS_TO_TICKS( 2500 ) );
    }

    for( ctr = 0; ctr < 8; ctr++ )
    {
        num_networks = IotBleWifiProv_GetNumNetworks();
        ESP_LOGI( TAG, "isWiFiProvisionedNVS: try #%d IotBleWifiProv_GetNumNetworks(%d)", ctr, num_networks );

        if( num_networks > 0 )
        {
            ESP_LOGI( TAG, "isWiFiProvisionedNVS: found %d WiFi networks", num_networks );
            current_wifi_state.provisioned = true;
            return true;
        }

        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }

    ESP_LOGW( TAG, "isWiFiProvisionedNVS: found no WiFi networks" );
    return false;
}

bool isWiFiConnected()
{
    ESP_LOGI( TAG, "isWiFiConnected: WiFi %d", current_wifi_state.connected );
    return current_wifi_state.connected;
}

void setWiFiStatus( bool connected )
{
    ESP_LOGI( TAG, "setWiFiStatus: WiFi %d", connected );

    if( connected )
    {
        current_wifi_state.provisioned = true;
    }

    current_wifi_state.connected = connected;
    wifi_pwr_state();
}

/**
 * Timer interrupt handler to initiate SLEEP mode
 */
static void IRAM_ATTR timer_intr_handler( void * p )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    TIMERG1.int_clr_timers.t0 = 1;
    /* TIMERG1.hw_timer[0].config.alarm_en = false; */
    timer_set_alarm( TIMER_GROUP_1, TIMER_NUM_0, TIMER_ALARM_DIS );

    xTaskNotifyFromISR( xDeviceState, SLEEP, eSetBits, &xHigherPriorityTaskWoken );

    if( xHigherPriorityTaskWoken )
    {
        portYIELD_FROM_ISR();
    }
}

/**
 * Re-start timer on receiving device state change
 */
static void _start_sleep_timer( double newtime )
{
    double _time = newtime;

    if( my_motor_state != MIDLE )
    {
        _time = REPORT_SHADOW_AWAKE;
    }
    else
    {
        if( !isWiFiConnected() )
        {
            ESP_LOGI( TAG, "_start_sleep_timer: extending awake time by %dX, new time => %fs", EXTEND_AWAKE_MULTIPLIER, _time * EXTEND_AWAKE_MULTIPLIER );
            _time *= ( double ) EXTEND_AWAKE_MULTIPLIER; /* extend awake time as a function of the configurable awake time */

            /* our wifi clean-up seems to be getting stuck */
            /* so we use this timer loop to record 5 min of being stuck */
            if( wifi_cleanup_in_progress )
            {
                ESP_LOGI( TAG, "_start_sleep_timer: wifi_cleanup_in_progress, my_wifi_error_count => %d", my_wifi_error_count + 3 );
                my_wifi_error_count++;
                my_wifi_error_count++;
                my_wifi_error_count++;
            }
        }
    }

    ESP_LOGI( TAG, "_start_sleep_timer: sleep evaluate timer set to => %f seconds", _time );
    timer_pause( TIMER_GROUP_1, TIMER_NUM_0 ); /* may not be needed but it's better to be safe */
    timer_set_alarm_value( TIMER_GROUP_1, TIMER_NUM_0, ( uint64_t ) _time * TIMER_SCALE );
    timer_set_counter_value( TIMER_GROUP_1, TIMER_NUM_0, 0x00000000ULL );
    timer_set_alarm( TIMER_GROUP_1, TIMER_NUM_0, TIMER_ALARM_EN );
    timer_start( TIMER_GROUP_1, TIMER_NUM_0 );
}

/**
 * Init sleep mode timer
 */
static void _init_timer( void )
{
    timer_config_t config;

    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.auto_reload = TIMER_AUTORELOAD_DIS;
    config.alarm_en = true;
    config.intr_type = TIMER_INTR_LEVEL;
    ESP_ERROR_CHECK( timer_init( TIMER_GROUP_1, TIMER_NUM_0, &config ) );

    timer_set_counter_value( TIMER_GROUP_1, TIMER_NUM_0, 0x00000000ULL );
    timer_isr_register( TIMER_GROUP_1, TIMER_NUM_0, timer_intr_handler, NULL, ESP_INTR_FLAG_IRAM, &timer_handle );
    timer_enable_intr( TIMER_GROUP_1, TIMER_NUM_0 );
    ESP_LOGI( TAG, "init_timer: init SLEEP timer" );
}

/* for sleep extension when activity has happened */
void start_sleepExtTimer()
{
    struct timeval now;

    /* have we advanced beyond the last checkin time? */
    gettimeofday( &now, NULL );

    if( last_sleepTimer_start > 0 )
    {
        ESP_LOGI( TAG, "start_sleepExtTimer: SLEEP EXT timer (%d) already active", last_sleepTimer_start );
    }

    last_sleepTimer_start = now.tv_sec;
    ESP_LOGI( TAG, "start_sleepExtTimer: init SLEEP EXT timer (%d)", last_sleepTimer_start );
}

bool is_sleepTimerExt_active()
{
    struct timeval now;

    if( last_sleepTimer_start == 0 )
    {
        ESP_LOGI( TAG, "is_sleepTimerExt_active: not running" );
        return false;
    }

    gettimeofday( &now, NULL );

    if( now.tv_sec > last_sleepTimer_start + SLEEP_EXTENSION_MIN * 60 )
    {
        /* we are beyond sleep timer expiration */
        last_sleepTimer_start = 0;
        ESP_LOGI( TAG, "is_sleepTimerExt_active: now (%d) > SLEEP timer++ (%d)", ( uint32_t ) now.tv_sec, last_sleepTimer_start + SLEEP_EXTENSION_MIN * 60 );
        return false;
    }
    ESP_LOGI( TAG, "is_sleepTimerExt_active: running" );
    return true;
}

static bool _is_timer_active( void )
{
    uint64_t time_elapsed_count;
    double time_remaining;
    uint64_t time_remaining_count;
    uint64_t time_alarm_count;
    esp_err_t xErr;

    timer_pause( TIMER_GROUP_1, TIMER_NUM_0 ); /* may not be needed but it's better to be safe */
    xErr = timer_get_alarm_value( TIMER_GROUP_1, TIMER_NUM_0, &time_alarm_count );
    xErr += timer_get_counter_value( TIMER_GROUP_1, TIMER_NUM_0, &time_elapsed_count );
    timer_start( TIMER_GROUP_1, TIMER_NUM_0 );

    /* xErr = timer_get_counter_time_sec(TIMER_GROUP_1, TIMER_NUM_0, &time_elapsed); */
    if( xErr != ESP_OK )
    {
        ESP_LOGE( TAG, "_is_timer_active: unable to retrieve timer_get_counter_time_sec" );
        return false;
    }

    if( time_alarm_count > time_elapsed_count )
    {
        time_remaining_count = time_alarm_count - time_elapsed_count;
    }
    else
    {
        ESP_LOGI( TAG, "_is_timer_active: expired timer counter %lld > %lld", time_elapsed_count, time_alarm_count );
        return false;
    }

    time_remaining = ( double ) time_remaining_count / ( double ) TIMER_SCALE;

    ESP_LOGI( TAG, "_is_timer_active: time_remaining(%f)", time_remaining );

    if( time_remaining > MIN_ACTIVE_TIMER )
    {
        return true;
    }

    return false;
}

/* evaluate the controller state so that we can either sleep or not */
void eval_sleep_state( void )
{
    if( is_usb_connected )
    {
        my_sleep_state = STAY_AWAKE;
        ESP_LOGI( TAG, "eval_sleep_state: USB powered -> stay awake" );
    }
    else if( my_valve_pending )
    {
        my_sleep_state = STAY_AWAKE;
        ESP_LOGI( TAG, "eval_sleep_state: valve move pending -> stay awake" );
    }
    else if( my_motor_state != MIDLE )
    {
        my_sleep_state = STAY_AWAKE;
        ESP_LOGI( TAG, "eval_sleep_state: motor not idle -> stay awake" );
    }
    else if( check_for_ota )
    {
        my_sleep_state = STAY_AWAKE;
        ESP_LOGI( TAG, "eval_sleep_state: check_for_ota -> stay awake" );
    }
    else if( ota_job_in_progress )
    {
        my_sleep_state = STAY_AWAKE;
        ESP_LOGI( TAG, "eval_sleep_state: ota active -> stay awake" );
    }
    else if( is_sleepTimerExt_active() )
    {
        my_sleep_state = STAY_AWAKE;
        ESP_LOGI( TAG, "eval_sleep_state: sleep extension active -> stay awake" );
    }
    else if( null_desired )
    {
        my_sleep_state = STAY_AWAKE;
        ESP_LOGI( TAG, "eval_sleep_state: null_desired -> stay awake" );
    }
    else if( IotSemaphore_GetCount( &shadowSyncSemaphore ) == 0 )
    {
        my_sleep_state = STAY_AWAKE;
        ESP_LOGI( TAG, "eval_sleep_state: shadow sync active -> stay awake" );
    }
    else if( !isShadowConnected() )
    {
        ESP_LOGI( TAG, "eval_sleep_state: shadow done or not connected -> can sleep" );
        my_sleep_state = CAN_SLEEP;
    }
    else
    {
        ESP_LOGI( TAG, "eval_sleep_state: default -> can sleep" );
        my_sleep_state = CAN_SLEEP;
    }
}

/* determine if the controller is busy so that we can skip sleeping among other things */
bool isBusy( void )
{
    bool xRet = false;

    if( my_valve_pending )
    {
        xRet = true;
        ESP_LOGI( TAG, "eval_sleep_state: valve move pending -> true" );
    }
    else if( my_motor_state != MIDLE )
    {
        xRet = true;
        ESP_LOGI( TAG, "eval_sleep_state: motor not idle -> true" );
    }
    else if( check_for_ota )
    {
        xRet = true;
        ESP_LOGI( TAG, "eval_sleep_state: check_for_ota -> true" );
    }
    else if( ota_job_in_progress )
    {
        xRet = true;
        ESP_LOGI( TAG, "eval_sleep_state: ota active -> true" );
    }
    else if( null_desired )
    {
        xRet = true;
        ESP_LOGI( TAG, "eval_sleep_state: null_desired -> true" );
    }
    else if( IotSemaphore_GetCount( &shadowSyncSemaphore ) == 0 )
    {
        xRet = true;
        ESP_LOGI( TAG, "eval_sleep_state: shadow sync active -> true" );
    }

    return xRet;
}

/**
 * Control SLEEP state if a charger is attached
 */
void status_charger_attached( bool attached )
{
    is_usb_connected = attached;
    eval_sleep_state();
}

bool isUSBConnected()
{
    return is_usb_connected;
}

void ota_update_state( bool ongoing )
{
    ESP_LOGI( TAG, "ota_update_state: current OTA state(%d)", ongoing );
    otaCheckAgentStatus();
    ota_job_in_progress = ongoing;
    eval_sleep_state();
}

bool isShadowAvailable()
{
    bool xRet = false;

    ESP_LOGI( TAG, "isShadowAvailable: shadow job check" );

    if( isShadowConnected() && ( IotSemaphore_GetCount( &shadowSyncSemaphore ) == 1 ) )
    /* if (IotSemaphore_GetCount(&shadowSyncSemaphore) == 1) */
    {
        xRet = true;
    }

    ESP_LOGI( TAG, "isShadowAvailable: shadow job or connection status(%d)", xRet );
    eval_sleep_state();
    return xRet;
}

/**
 * Helper function to print device state flags
 */
static void printDeviceState( char * str,
                              uint32_t state )
{
    uint8_t index;
    char states[ 256 ];
    uint8_t pos = 0;
    uint8_t lenstr = 0;

    for( index = 0; index < DEVICE_NUM_STATES; index++ )
    {
        if( ( state >> index ) & 0x1 )
        {
            /* append device_state_str[index] */
            lenstr = strlen( device_state_str[ index ] );
            strncpy( states + pos, device_state_str[ index ], lenstr );
            pos += lenstr;
            strncpy( states + pos, ",", 1 );
            pos++;
        }
    }

    strncpy( states + pos, "", 1 );
    ESP_LOGI( TAG, "%s: %s", str, states );
}

// void // set_led_state( int led,
//                     int on,
//                     int off,
//                     int delay )
// {
//     // my_leds[ led ].color_on = on;
//     // my_leds[ led ].color_off = off;
//     // my_leds[ led ].delay = delay;
//     // setLEDAuto();
// }

static void doDeepSleep()
{
    // // set_led_state( PWR_LED, LED_OFF, LED_OFF, 1000 );
    // // set_led_state( WIFI_LED, LED_OFF, LED_OFF, 1000 );
    adc_power_off();
    esp_sleep_pd_config( ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF );
    rtc_gpio_deinit( GPIO_NUM_37 );
    rtc_gpio_deinit( GPIO_NUM_38 );
    ESP_LOGI( TAG, "doDeepSleep: DEEP SLEEP MODE executing now" );

    if( isWiFiConnected() )
    {
        WIFI_Disconnect();

    }
    WIFI_Off();
    esp_wifi_stop();
    esp_deep_sleep_start();
}

/**
 * Safe restart using deepsleep wakeup
 */
static void safeRestart()
{
    ESP_LOGW( TAG, "safeRestart: Will enable restart setup in 1s" );
    recordLocalStateAndSave( initLSData );
    esp_sleep_enable_timer_wakeup( 1000 * 1000 ); /* in uS == 1 sec */
    
    doDeepSleep();
}

/**
 * Delete stored wifi credentials and reset (factory reset?), reprovisioning
 */
static void deleteWiFiAndRestart()
{
    uint8_t retryCount = 0;

    ESP_LOGW( TAG, "deleteWiFiAndRestart: DELETE WIFI & RESTART MODE" );

    if( current_wifi_state.provisioned )
    {
        /* let's forget our local state of being provisioned */
        current_wifi_state.provisioned = false;
        ESP_LOGW( TAG, "deleteWiFiAndRestart: Calling IotBleWifiProv_EraseAllNetworks()" );

        while( !IotBleWifiProv_EraseAllNetworks() && retryCount++ < MAX_RETRY_CONNECTION )
        {
            ESP_LOGW( TAG, "deleteWiFiAndRestart: Waiting 1s to delete WiFi networks" );
            vTaskDelay( 1000 );
        }
    }

    ESP_LOGW( TAG, "deleteWiFiAndRestart: SAFE RESTART MODE w/ WIFI DELETED" );
    safeRestart();
}

/**
 * Check WIFI and PWR leds and setup accordingly
 */
void wifi_pwr_state()
{
    uint32_t wifiState = PROVISIONING;

    /* need to short cut just to check NVS status first: */
    if( current_wifi_state.connected )
    {
        wifiState = WIFI_CONNECTED;
    }
    else if( current_wifi_state.provisioned )
    {
        wifiState = PROVISIONED;
    }
    else if( current_wifi_state.initialized )
    { /* need to make sure that WiFi is initialized and ok -- otherwise core dump with WIFI_IsConnected() */
        if( isWiFiProvisionedNVS() )
        {
            wifiState = PROVISIONED;
        }
        else
        {
            wifiState = PROVISIONING;
        }
    }

/*
    ESP_LOGI( TAG, "wifi_pwr_state: 1st WiFi/PWR: pwr:%s_%s delay0:%d wifi:%s_%s delay1:%d",
              led_state_str[ my_leds[ 0 ].color_on ], led_state_str[ my_leds[ 0 ].color_off ], \
              my_leds[ 0 ].delay,                                                              \
              led_state_str[ my_leds[ 1 ].color_on ], led_state_str[ my_leds[ 1 ].color_off ], \
              my_leds[ 1 ].delay );
*/

    /* made it to this point which means that we can show status for pwr and wifi (independently) */
    /* WIFI LED STATE */
    switch( wifiState )
    {
        case WIFI_CONNECTED:
            // set_led_state( WIFI_LED, LED_GREEN, LED_GREEN, 500 );
            break;

        case PROVISIONED:
            // set_led_state( WIFI_LED, LED_GREEN, LED_OFF, 500 );
            break;

        case PROVISIONING:
            // set_led_state( WIFI_LED, LED_RED, LED_OFF, 500 );
            break;

        // default:
            // set_led_state( WIFI_LED, LED_OFF, LED_OFF, 500 );
    }

    switch( my_battery_status )
    {
        case BATTERY_FULL:
            // set_led_state( PWR_LED, LED_GREEN, LED_GREEN, 500 );
            break;

        case BATTERY_EMPTY:
            // set_led_state( PWR_LED, LED_RED, LED_OFF, 500 );
            break;

        case BATTERY_CHARGING:
            // set_led_state( PWR_LED, LED_GREEN, LED_OFF, 500 );
            break;

        case BATTERY_DISCHARGING:
            // set_led_state( PWR_LED, LED_OFF, LED_OFF, 500 );
            break;

        // default:
            // set_led_state( PWR_LED, LED_OFF, LED_OFF, 500 );
            // break;
    }

/*
    ESP_LOGI( TAG, "wifi_pwr_state: 2nd WiFi/PWR: pwr:%s_%s delay0:%d wifi:%s_%s delay1:%d",
              led_state_str[ my_leds[ 0 ].color_on ], led_state_str[ my_leds[ 0 ].color_off ], \
              my_leds[ 0 ].delay,                                                              \
              led_state_str[ my_leds[ 1 ].color_on ], led_state_str[ my_leds[ 1 ].color_off ], \
              my_leds[ 1 ].delay );
*/
}

void doUpdateShadow( const char * msg )
{
    uint8_t ctr = 0;

    /* don't need to create it if the shadow process is already active? */
    while( ctr++ < MAX_RETRY_SHADOW_COUNT )
    {
        ESP_LOGI( TAG, "doUpdateShadow: (%s) about to run isShadowAvailable", msg );

        if( isShadowAvailable() && !ota_job_in_progress )
        {
            ESP_LOGI( TAG, "doUpdateShadow: (%s) about to run updateShadowTask", msg );

            if( !updateShadowTask( NULL ) )
            {
                ESP_LOGE( TAG, "doUpdateShadow: (%s) updateShadowTask failed", msg );
                directTasksStatus();
                safeRestart();
            }
            else
            {
                my_shadow_skipped_count = 0;
                return;
            }
        }
        else
        {
            ESP_LOGW( TAG, "doUpdateShadow: (%s) did not run updateShadowTask as !isShadowAvailable/!ota_job_in_progress iter %d", msg, ctr );
            vTaskDelay( pdMS_TO_TICKS( 1000 ) ); /* if we were trying to connect, let's wait here... timeout?? */
            my_shadow_skipped_count++;
        }
    }
}

void sleepHandler()
{
    uint64_t sleep_time;
    uint8_t ctr = 0;

    if( !isBusy() )
    {
        ESP_LOGI( TAG, "sleepHandler: eval_sleep_state" );
        eval_sleep_state();
        ESP_LOGI( TAG, "sleepHandler: recordLocalStateAndSave()" );
        recordLocalStateAndSave( initLSData );

        if( my_sleep_state == CAN_SLEEP )
        {
            disable_bqstat_intr();
            ESP_LOGI( TAG, "sleepHandler: SLEEP soon" );

            if( isWiFiConnected() )
            {
                while( isMQTTConnecting() && ctr++ < MAX_RETRY_CONNECTION )
                {
                    ESP_LOGI( TAG, "sleepHandler: SLEEP - waiting b/c isMQTTConnecting is true" );
                    vTaskDelay( pdMS_TO_TICKS( 2500 ) ); /* if we were trying to connect, let's wait here... timeout?? */
                }

                if( isMQTTConnected() )
                {
                    ESP_LOGI( TAG, "sleepHandler: calling doUpdateShadow(SLEEP)" );
                    doUpdateShadow( "SLEEP" );
                }
            }

            /* let's check sleep state once more after the possible shadow sync */
            eval_sleep_state();

            if( my_sleep_state == CAN_SLEEP )
            {
                ESP_ERROR_CHECK( esp_sleep_enable_ext1_wakeup( ( 1ULL << LS_BTN2_N ), ESP_EXT1_WAKEUP_ALL_LOW ) );

                if( current_wifi_state.provisioned )
                {
                    sleep_time = my_awake_time * my_sleep_multiplier * 1000 * 1000; /* sleep mode in us */
                    ESP_LOGW( TAG, "sleepHandler: SLEEP will sleep for %d(s)", my_awake_time * my_sleep_multiplier );
                    esp_sleep_enable_timer_wakeup( sleep_time );                    /* in uS */
                }
                else
                {
                    ESP_LOGE( TAG, "sleepHandler: SLEEP can only wake up with button press" );
                }
                doDeepSleep();
            }
            else
            {
                goto NO_SLEEP;
            }
        }
        else
        {
NO_SLEEP:
            ESP_LOGI( TAG, "sleepHandler: NO SLEEP" );

            if( isWiFiConnected() )
            {
                my_wifi_error_count = 0;

                if( !isMQTTConnected() && !my_valve_pending )
                {
                    vTaskDelay( pdMS_TO_TICKS( 2000 ) ); /* if we were trying to connect, let's wait here... timeout?? */
                    ESP_LOGI( TAG, "sleepHandler: NO SLEEP: reconnectMQTT" );
                    reconnectMQTT();
                    vTaskDelay( pdMS_TO_TICKS( 2000 ) ); /* if we were trying to connect, let's wait here... timeout?? */
                }

                if( isMQTTConnected() && !ota_job_in_progress && isShadowAvailable() && ( my_motor_state == MIDLE ) && !my_valve_pending )
                {
                    ESP_LOGI( TAG, "sleepHandler: NO SLEEP - doUpdateShadow" );
                    doUpdateShadow( "NO SLEEP" );
                }
                else
                {
                    ESP_LOGI( TAG, "sleepHandler: NO SLEEP did not run doUpdateShadow" );
                    ESP_LOGI( TAG, "isMQTTConnected(%d) ota_job_in_progress(%d) isShadowAvailable(%d) my_motor_state(%d) my_valve_pending(%d)", \
                              isMQTTConnected(), ota_job_in_progress, isShadowAvailable(), my_motor_state, my_valve_pending );

                    if( ( !isMQTTConnected() && !my_valve_pending ) || !isShadowAvailable() )
                    {
                        my_shadow_skipped_count++;
                    }
                }
            }
            else
            {
                my_wifi_error_count++;
            }
        }
    }

    xSleepHandle = NULL;
    vTaskDelete( NULL );
}

/**
 * Device state main task to control LEDs
 */
void device_state_task()
{
    static uint32_t myState = 0;
    BaseType_t xReturned;
    static bool checkOTARecently = true;
    uint8_t sleepCycleCount = 0;
    uint8_t maxWaitForShadow = 20;
    uint8_t waitCycles = 0;

    _init_timer();
    _start_sleep_timer( my_awake_time ); /* seed initial timer for sleep now */

    for( ; ; )
    {
        ESP_LOGI( TAG, "device_state_task: Waiting for xTaskNotifyWait to be triggered" );

        if( xTaskNotifyWait( myState, ALL_STATES_TO_CLEAR, &myState, portMAX_DELAY ) )
        {
            ESP_LOGI( TAG, "device_state_task: xTaskNotifyWait event recvd" );
            printDeviceState( "xTaskNotifyWait", myState ); /*  use device_state_str */

            /* ESP_LOGI(TAG, PRINTF_BINARY_PATTERN_INT32, PRINTF_BYTE_TO_BINARY_INT32(myState)); */
            if( !_is_timer_active() )
            {
                ESP_LOGI( TAG, "device_state_task: !timer_active" );

                if( my_sleep_state != CAN_SLEEP )
                {
                    _start_sleep_timer( my_awake_time << 1 ); /* re-start the awake timer even if we are going to stay awake */
                }
                else
                {
                    _start_sleep_timer( my_awake_time ); /* re-start the awake timer even if we are going to sleep */
                }
            }

            /* we want to take care of global states first like */
            /* DEVICE_ERROR, MOTOR_IDLE, VALVE_OPENING, VALVE_CLOSING, SLEEP */
            /* todo Reset DEVICE_ERROR state on user action only, where? */
            if( myState & RESTART )
            {
                printDeviceState( "RESTART STATE", myState ); /*  use device_state_str */
                // set_led_state( PWR_LED, LED_RED, LED_RED, 75 );
                // set_led_state( WIFI_LED, LED_RED, LED_RED, 75 );
                /* remove all WiFi configs and reset device */
                deleteWiFiAndRestart();
            }
            else if( myState & SAFE_RESTART ||                                        \
                     ( my_shadow_error_count > MAX_SHADOW_CONNECTION_ERROR_COUNT ) || \
                     ( my_mqtt_error_count > MAX_MQTT_CONNECTION_ERROR_COUNT ) ||     \
                     ( my_shadow_skipped_count > MAX_SHADOW_SKIPPED_COUNT ) ||        \
                     ( my_wifi_error_count > MAX_WIFI_ERROR_COUNT ) )
            {
                printDeviceState( "SAFE_RESTART", myState ); /*  use device_state_str */
                ESP_LOGI( TAG, "device_state_task: SAFE RESTART cause:" );
                ESP_LOGI( TAG, "device_state_task: my_shadow_error_count(%d):(%d)", my_shadow_error_count, MAX_SHADOW_CONNECTION_ERROR_COUNT );
                ESP_LOGI( TAG, "device_state_task: my_mqtt_error_count(%d):(%d)", my_mqtt_error_count, MAX_MQTT_CONNECTION_ERROR_COUNT );
                ESP_LOGI( TAG, "device_state_task: my_shadow_skipped_count(%d):(%d)", my_shadow_skipped_count, MAX_SHADOW_SKIPPED_COUNT );
                ESP_LOGI( TAG, "device_state_task: my_wifi_error_count(%d):(%d)", my_wifi_error_count, MAX_WIFI_ERROR_COUNT );
                // set_led_state( PWR_LED, LED_RED, LED_RED, 75 );
                // set_led_state( WIFI_LED, LED_RED, LED_RED, 75 );
                directTasksStatus();
                safeRestart();
            }
            else if( myState & DEVICE_ERROR )
            {
                printDeviceState( "DEVICE_ERROR", myState ); /*  use device_state_str */
                // set_led_state( PWR_LED, LED_RED, LED_RED, 75 );
                // set_led_state( WIFI_LED, LED_RED, LED_RED, 75 );
                my_sleep_state = STAY_AWAKE;
                continue;
            }
            else if( ( myState & VALVE_OPENING ) || ( myState & VALVE_CLOSING ) )
            {
                printDeviceState( "VALVE_MOVING", myState ); /*  use device_state_str */
                myState &= ~( SLEEP | MOTOR_IDLE | MOTOR_TIMEOUT );
                disable_bqstat_intr();

                if( myState & VALVE_OPENING )
                {
                    ESP_LOGI( TAG, "device_state_task: VALVE OPENING" );
                    // set_led_state( PWR_LED, LED_GREEN, LED_OFF, 200 );
                    // set_led_state( WIFI_LED, LED_GREEN, LED_OFF, 200 );
                }
                else
                {
                    ESP_LOGI( TAG, "device_state_task: VALVE CLOSING" );
                    // set_led_state( PWR_LED, LED_RED, LED_OFF, 200 );
                    // set_led_state( WIFI_LED, LED_RED, LED_OFF, 200 );
                }

                /* is our network up and available? Need to check and if not, wait */
                if( isMQTTConnecting() || isMQTTConnected() )
                {
                    waitCycles = 0;

                    while( needsTimeUpdate() && ( ++waitCycles < maxWaitForShadow ) )
                    {
                        ESP_LOGI( TAG, "device_state_task: VALVE_MOVING: waiting 1s(%d) for time update", waitCycles );
                        vTaskDelay( pdMS_TO_TICKS( 1000 ) ); /* give some time to get LEDs going? */
                    }

                    if( waitCycles != maxWaitForShadow )
                    {
                        /* sync shadow here? But we are still in the middle of this function?? */
                        doUpdateShadow( "VALVE_MOVING" );
                        waitCycles = 0;

                        while( !isShadowAvailable() && ( ++waitCycles < maxWaitForShadow ) )
                        {
                            ESP_LOGI( TAG, "device_state_task: VALVE_MOVING: waiting 1s(%d) for doUpdateShadow", waitCycles );
                            vTaskDelay( pdMS_TO_TICKS( 1000 ) ); /* give some time to get LEDs going? */
                        }
                    }

                    disconnectMQTT();
                }
                else
                {
                    ESP_LOGI( TAG, "device_state_task: VALVE_MOVING: no MQTT connection available" );
                }

                continue;
            }
            else if( myState & MOTOR_IDLE || myState & MOTOR_TIMEOUT )
            {
                printDeviceState( "MOTOR_IDLE/TIMEOUT", myState ); /*  use device_state_str */
                my_valve_pending = false;

                if( myState & MOTOR_TIMEOUT )
                {
                    null_desired = true;
                }

                enable_bqstat_intr();

                if( isWiFiConnected() && !isMQTTConnecting() && !isMQTTConnected() )
                {
                    reconnectMQTT();
                }

                myState &= ~( SLEEP | MOTOR_IDLE | MOTOR_TIMEOUT );
                // set_led_state( PWR_LED, LED_OFF, LED_OFF, 100 );
                // set_led_state( WIFI_LED, LED_OFF, LED_OFF, 100 );
                xTaskNotify( xDeviceState, SHADOW_UPDATE, eSetBits );
            }
            else if( myState & SHADOW_UPDATE )
            {
                printDeviceState( "SHADOW_UPDATE", myState ); /*  use device_state_str */
                myState &= ~( SLEEP | SHADOW_UPDATE );
                doUpdateShadow( "SHADOW_UPDATE" );
                xTaskNotify( xDeviceState, SLEEP, eSetBits );
            }
            else if( myState & OTA_UPDATE || ( isShadowAvailable() && !checkOTARecently && !needsTimeUpdate() && needsOTACheck() ) )
            {
                /* let's check for OTA here? We might skip one sleep sync/cycle, but that's likely ok */
                printDeviceState( "OTA_UPDATE", myState ); /*  use device_state_str */
                myState &= ~( SLEEP | OTA_UPDATE );
                checkOTARecently = true;
                getOTAUpdate();
                xTaskNotify( xDeviceState, SHADOW_UPDATE, eSetBits );
            }
            else if( myState & TIME_UPDATE )
            {
                printDeviceState( "TIME_UPDATE", myState ); /*  use device_state_str */
                myState &= ~( SLEEP | TIME_UPDATE );

                if( !my_valve_pending )
                {
                    if( !timeSet )
                    {
                        getTimeFromShadow();
                        timeSet = true;
                    }

                    xTaskNotify( xDeviceState, OTA_UPDATE, eSetBits );
                }
                else
                {
                    continue;
                }
            }
            else if( myState & SLEEP )
            {
                printDeviceState( "SLEEP", myState ); /*  use device_state_str */

                if( ( my_motor_state != MIDLE ) || my_valve_pending )
                {
                    /* we might get a SLEEP state while we are processing motor movement */
                    /* so let's skip the sleep state processing for now */
                    continue;
                }

                /* task to sleep or stay awake but it needs to be detached so that we can respond to LEDs */
                if( xSleepHandle == NULL )
                {
                    xReturned = xTaskCreate( sleepHandler, "sleepHandler", 4 * 1024, NULL, PRIORITY_SLEEP, &xSleepHandle );
                }

                if( ( xSleepHandle == NULL ) || ( xReturned != pdPASS ) )
                {
                    ESP_LOGE( TAG, "device_state_task: bad xSleepHandle or return in create sleepHandler" );
                    directTasksStatus();
                    safeRestart();
                }

                if( ++sleepCycleCount == SLEEP_CYCLE_COUNT_OTA )
                {
                    ESP_LOGI( TAG, "device_state_task: resetting sleepCycleCount" );
                    checkOTARecently = false;
                    sleepCycleCount = 0;

                    if( !ota_job_in_progress )
                    {
                        /* kill OTA task now */
                        disconnectOTA();
                    }
                }
            }
            else
            {
                printDeviceState( "UNHANDLED DEVICE STATE", myState ); /*  use device_state_str */
            }

            if( my_motor_state == MIDLE )
            {
                printDeviceState( "BATTERY", myState ); /*  use device_state_str */
                /* if we are as far, we can check and set LEDs for wifi and power state */
                wifi_pwr_state();
            }
            else
            {
                ESP_LOGI( TAG, "device_state_task: Not processing WiFi/PWR LED" );
            }
        }
    }
}
