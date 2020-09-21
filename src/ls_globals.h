/*
 * Copyright (C) 2020 LeakSentinel, Inc. or its affiliates.  All Rights Reserved.
 *
 */

/* global defines for LeakSentinel firmware */

#ifndef _LS_GLOBALS_
#define _LS_GLOBALS_

extern const char * valve_state_str[];
extern const char * led_state_str[];
/* extern const char *motor_state_str[]; */
/* extern const char *valve_type_str[]; */
/* extern const char *battery_state_simple_str[]; */
/* extern const char *battery_state_str[]; */
/* extern const char *source_state_str[]; */
/* extern const char* flow_state_str[]; */
/* extern const char *wifi_state_str[]; */

extern valve_type my_valve_type;
extern motor_state my_motor_state;
extern battery_state my_battery_state;
extern source_state my_source_state;
extern device_info_t device;

extern struct bq2588x bq;

extern const char * flow_state_str[];

extern float wifi_rssi;

extern uint16_t my_awake_time;
extern uint16_t my_sleep_multiplier;

extern uint8_t lsADC_NumSmpls;

extern float lsMTR_maxCurrent;

extern uint16_t lsMTR_currentLimHigh; /* need to adjust limit per voltage supply @ 6v = 2.5A, @ 8.5v = 3.4A */

extern uint16_t lsMTR_delayStartMS;

extern uint8_t lsMTR_maxTimeS;
extern uint16_t lsMTR_turnBackmS;
extern uint32_t lsMTR_runTimeMS;

extern int my_battery_status;
extern int my_source_status;
extern xQueueHandle gpio_evt_queue;

extern void directTasksStatus( void );
extern void pauseNetTasks( void );
extern void resumeNetTasks( void );

extern void status_charger_attached( bool attached );
extern esp_err_t bq2588x_get_charge_voltage( int16_t * volt );
extern esp_err_t bq2588x_get_charge_current( int16_t * curr );
extern esp_err_t bq2588x_set_wdt_timer( int time );
extern bool bq2588x_get_wdt_int( void );
extern int bq2588x_set_adc_oneshot_mode( bool oneshot );
extern esp_err_t bq2588x_enable_adc_scan( bool enable );
extern bool bq2588x_wait_adc_scan_done( void );
extern void bq2588x_get_full_status( struct bq2588x * bq );
extern void enable_bqstat_intr( void );
extern void disable_bqstat_intr( void );

extern valve_state my_valve_state;
extern button_state my_button_state;
extern IotSemaphore_t buttonLockoutSemaphore;

extern void device_state_task();

extern void ota_update_state( bool ongoing );
extern char my_thing_name[ 64 ];

extern void nvs_dump( const char * partName );

extern IotSemaphore_t shadowSyncSemaphore;
extern IotSemaphore_t shadowSemaphore;
extern bool ota_job_in_progress;
extern uint16_t my_valve_state_counter;
extern shadowLSStruct_t initLSData;
extern shadowLSStruct_t nvmState;
extern wifi_state_t current_wifi_state;
extern uint16_t my_shadow_error_count;
extern uint16_t my_wifi_error_count;
extern uint16_t my_mqtt_error_count;
extern bool my_valve_pending;
extern uint16_t my_shadow_skipped_count;
extern long lastTimeOTAChecked;

extern void tasksStatus();
extern bool isMQTTConnected();
extern bool isMQTTConnecting();
extern bool isShadowConnected();
extern bool needsTimeUpdate();
extern bool isWiFiConnected();
extern void wifi_pwr_state();

extern void set_timestamp( long int timestamp );
extern void updateLastTimeOTAChecked( long int timestamp );
extern void getTimeFromShadow();


extern void ls_mtr_send_event( int64_t timestamp,
                               int direction );

extern sleep_state my_sleep_state;
bool is_sleepTimerExt_active();
void start_sleepExtTimer();

extern bool isOTAConnected();
extern bool updateShadowTask( void * pDeltaSemaphore );
extern void IRAM_ATTR esp_restart( void );
extern esp_err_t recordLocalStateAndSave( shadowLSStruct_t inputD );

extern void disconnectMQTT();
extern void disconnectOTA();
extern void reconnectMQTT();
extern void getOTAUpdate();
extern bool needsOTACheck();
extern void otaCheckAgentStatus();

/* extern xTaskHandle conTask; */
extern bool null_desired;
extern bool check_for_ota;
extern bool wifi_cleanup_in_progress;

#endif /* _LS_GLOBALS_ */
