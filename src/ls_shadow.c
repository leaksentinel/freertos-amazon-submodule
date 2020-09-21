/*
 * Copyright (C) 2020 LeakSentinel, Inc. or its affiliates.  All Rights Reserved.
 *
 */

/**
 * @file aws_iot_demo_shadow.c
 * @brief Demonstrates usage of the Thing Shadow library.
 *
 * This program demonstrates the using Shadow documents to toggle a state called
 * "powerOn" in a remote device.
 */

/* The config header is always included first. */
#include "iot_config.h"

/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"
#include "sys/time.h"

/* Set up logging for this demo. */
#include "iot_demo_logging.h"

/* Platform layer includes. */
#include "platform/iot_clock.h"
#include "platform/iot_threads.h"

/* MQTT include. */
#include "iot_mqtt.h"

/* Shadow include. */
#include "aws_iot_shadow.h"
#include "iot_network_manager_private.h"

/* JSON utilities include. */
#include "iot_json_utils.h"

#include "aws_application_version.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "iot_wifi.h"

#include "ls_debug.h"
#include "ls_typedefs.h"
#include "ls_board_setup.h"
#include "ls_gpio.h"
#include "ls_shadow.h"
#include "ls_device_state.h"
#include "ls_globals.h"

#define TAG    "SHADOW"

/**
 * @cond DOXYGEN_IGNORE
 * Doxygen should ignore this section.
 *
 * Provide default values for undefined configuration settings.
 */
#ifndef AWS_IOT_DEMO_SHADOW_UPDATE_COUNT
    #define AWS_IOT_DEMO_SHADOW_UPDATE_COUNT        ( 1 )
#endif
#ifndef AWS_IOT_DEMO_SHADOW_UPDATE_PERIOD_MS
    #define AWS_IOT_DEMO_SHADOW_UPDATE_PERIOD_MS    ( 3000 )
#endif
/** @endcond */

/* Validate Shadow demo configuration settings. */
#if AWS_IOT_DEMO_SHADOW_UPDATE_COUNT <= 0
    #error "AWS_IOT_DEMO_SHADOW_UPDATE_COUNT cannot be 0 or negative."
#endif
#if AWS_IOT_DEMO_SHADOW_UPDATE_PERIOD_MS <= 0
    #error "AWS_IOT_DEMO_SHADOW_UPDATE_PERIOD_MS cannot be 0 or negative."
#endif

/**
 * @brief The timeout for Shadow and MQTT operations in this demo.
 */

static IotSemaphore_t deltaSemaphore;
static IotMqttConnection_t shadowMqttConnection;

bool null_desired = false;
static bool ignoreValveStateReq = false;
static bool varChanged = false;
static esp_err_t _get_wifi_info();

/* xQueueHandle MyShadowQueue; */
void debugPrintDeviceState( const char * msg,
                            shadowLSStruct_t debug );
esp_err_t save_state_to_nvs( shadowLSStruct_t data );
void xferCurrentStateToVar( shadowLSStruct_t * data );
uint16_t checkSanity( shadowLSStruct_t shPtr );

/*-----------------------------------------------------------*/

/* Declaration of demo function. */
static int _sendShadowUpdates( IotSemaphore_t * pDeltaSemaphore,
                               IotMqttConnection_t mqttConnection );

static void _operationComplete( void * pArgument,
                                AwsIotShadowCallbackParam_t * pOperation );

/*-----------------------------------------------------------*/

#define shadowBUFFER_LENGTH    8 * 256

bool valveTypeIsUnknown(shadowLSStruct_t state )
{
    if( ( state.valveType == VUNKNOWN ) )
    {
        ESP_LOGE( TAG, "valveTypeIsUnknown: valveType is unknown" );
        return true;
    }
    return false;
}

/* sanity check on state of NVS data */
bool localStateIsBad( shadowLSStruct_t state )
{
    nvs_handle xNvsHandle = NULL;
    size_t num_entries = 0;

    /* first calculate sanity factor? */
    uint16_t sanity = checkSanity( state );

    if( ( sanity & 0xFF ) != 0 )
    {
        ESP_LOGI( TAG, "localStateIsBad: Checking sanity aka checksum (nvs): %x, (0x100) & -> %x failed", sanity, sanity & 0x0100 );
    }
    else if( ( state.shadowVersionMajor != ( uint8_t ) SHADOW_MAJOR_VER ) || ( state.shadowVersionMinor != ( uint8_t ) SHADOW_MINOR_VER ) )
    {
        ESP_LOGE( TAG, "localStateIsBad: SHADOW_MAJOR_VER SHADOW_MINOR_VER" );
    }
    else if( ( state.adcNumSamp == 0 ) )
    {
        ESP_LOGE( TAG, "localStateIsBad: adcNumSamp" );
    }
    else if( state.mtrDelayTime == 0 )
    {
        ESP_LOGE( TAG, "localStateIsBad: mtrDelayTime" );
    }
    else if( ( state.mtrMaxI == 0 ) || ( state.mtrMaxT == 0 ) )
    {
        ESP_LOGE( TAG, "localStateIsBad: mtrMaxI mtrMaxT" );
    }
    else if( state.mtrBackT == 0 )
    {
        ESP_LOGE( TAG, "localStateIsBad: mtrBackT" );
    }
    else if( ( state.connect_not_flowing == 0 ) || ( state.valveState == 3 ) || ( state.valveState == 4 ) )
    {
        ESP_LOGE( TAG, "localStateIsBad: connect_not_flowing valveState" );
    }
    else if( ( state.connect_sleep_multiplier == 0 ) || ( state.valveStateCounter == 0 ) )
    {
        ESP_LOGE( TAG, "localStateIsBad: connect_sleep_multiplier valveStateCounter" );
    }
    else
    {
        /* all good because we did not fall into any of the above clauses */
        return false;
    }

    /* let's delete the data from NVS then */
    ESP_LOGI( TAG, "localStateIsBad: Erasing LSData in NVS" );
    esp_err_t xRet = nvs_flash_init_partition( "nvs" );

    if( xRet != ESP_OK )
    {
        ESP_LOGE( TAG, "localStateIsBad: error nvs_flash_init_partition" );
    }
    else
    {
        xRet = nvs_open_from_partition( "storage", "ourdata", NVS_READWRITE, &xNvsHandle );

        if( xRet != ESP_OK )
        {
            ESP_LOGE( TAG, "localStateIsBad: nvs_open_from_partition error" );
        }
        else
        {
            /* nvs_dump("nvs"); */
            xRet = nvs_get_used_entry_count( xNvsHandle, &num_entries );
            ESP_LOGI( TAG, "localStateIsBad: nvs_get_used_entry_count returned %d entries", num_entries );

            if( xRet != ESP_OK )
            {
                ESP_LOGE( TAG, "localStateIsBad: nvs_get_used_entry_count error" );
            }
            else
            {
                xRet = nvs_erase_key( xNvsHandle, "LSData" );

                if( xRet != ESP_OK )
                {
                    ESP_LOGE( TAG, "localStateIsBad: nvs_erase_key error ret(%d)", xRet );
                    ESP_LOGE( TAG, "localStateIsBad: Error is %s", esp_err_to_name( xRet ) );
                }
                else
                {
                    xRet = nvs_commit( xNvsHandle );
                }
            }
        }
    }

    if( xNvsHandle != ( nvs_handle ) NULL )
    {
        nvs_close( xNvsHandle );
    }

    return true;
}

esp_err_t recordLocalStateAndSave( shadowLSStruct_t inputD )
{
    shadowLSStruct_t data;

    memcpy( &data, &inputD, sizeof( shadowLSStruct_t ) );
    ESP_LOGI( TAG, "recordLocalStateAndSave: starting to xferCurrentStateToVar" );
    /* let's save the state to NVS even if we have already done a sync just in case network issues */
    xferCurrentStateToVar( &data );
    ESP_LOGI( TAG, "recordLocalStateAndSave: starting to save_state_to_nvs" );
    save_state_to_nvs( data );
    return ESP_OK;
}

bool compareNVM( shadowLSStruct_t nvm,
                 shadowLSStruct_t data )
{
    bool xRet = true;

    if( nvm.checkSum != data.checkSum )
    {
        ESP_LOGI( TAG, "compareNVM: checkSum" );
        xRet = false;
    }
    else if( nvm.shadowVersionMajor != data.shadowVersionMajor )
    {
        ESP_LOGI( TAG, "compareNVM: shadowVersionMajor" );
        xRet = false;
    }
    else if( nvm.shadowVersionMinor != data.shadowVersionMinor )
    {
        ESP_LOGI( TAG, "compareNVM: shadowVersionMinor" );
        xRet = false;
    }
    else if( nvm.valveType != data.valveType )
    {
        ESP_LOGI( TAG, "compareNVM: valveType" );
        xRet = false;
    }
    else if( nvm.adcNumSamp != data.adcNumSamp )
    {
        ESP_LOGI( TAG, "compareNVM: adcNumSamp" );
        xRet = false;
    }
    else if( nvm.valveState != data.valveState )
    {
        ESP_LOGI( TAG, "compareNVM: valveState" );
        xRet = false;
    }
    else if( nvm.mtrDelayTime != data.mtrDelayTime )
    {
        ESP_LOGI( TAG, "compareNVM: mtrDelayTime" );
        xRet = false;
    }
    else if( nvm.mtrMaxI != data.mtrMaxI )
    {
        ESP_LOGI( TAG, "compareNVM: mtrMaxI" );
        xRet = false;
    }
    else if( nvm.mtrMaxT != data.mtrMaxT )
    {
        ESP_LOGI( TAG, "compareNVM: mtrMaxT" );
        xRet = false;
    }
    else if( nvm.mtrBackT != data.mtrBackT )
    {
        ESP_LOGI( TAG, "compareNVM: mtrBackT" );
        xRet = false;
    }
    else if( nvm.valveStateReq != data.valveStateReq )
    {
        ESP_LOGI( TAG, "compareNVM: valveStateReq" );
        xRet = false;
    }
    else if( nvm.valveStateCounter != data.valveStateCounter )
    {
        ESP_LOGI( TAG, "compareNVM: valveStateCounter" );
        xRet = false;
    }
    else if( nvm.connect_not_flowing != data.connect_not_flowing )
    {
        ESP_LOGI( TAG, "compareNVM: connect_not_flowing" );
        xRet = false;
    }
    else if( nvm.connect_sleep_multiplier != data.connect_sleep_multiplier )
    {
        ESP_LOGI( TAG, "compareNVM: connect_sleep_multiplier" );
        xRet = false;
    }
    else if( nvm.provisioned != data.provisioned )
    {
        ESP_LOGI( TAG, "compareNVM: provisioned" );
        xRet = false;
    }

    return xRet;
}

esp_err_t get_state_from_nvs( shadowLSStruct_t * state )
{
    uint32_t ulSize = sizeof( shadowLSStruct_t );
    nvs_handle xNvsHandle = NULL;

    ESP_LOGI( TAG, "get_state_from_nvs: getting state from nvs" );
    esp_err_t xRet = nvs_flash_init_partition( "nvs" );

    if( xRet != ESP_OK )
    {
        ESP_LOGE( TAG, "get_state_from_nvs: error nvs_flash_init_partition" );
        goto err_exit;
    }
    else
    {
        xRet = nvs_open_from_partition( "storage", "ourdata", NVS_READONLY, &xNvsHandle );
    }

    if( xRet != ESP_OK )
    {
        ESP_LOGE( TAG, "get_state_from_nvs: nvs_open_from_partition error" );
        goto err_exit;
    }
    else
    {
        xRet = nvs_get_blob( xNvsHandle, "LSData", state, &ulSize );
    }

    if( xRet != ESP_OK )
    {
        ESP_LOGE( TAG, "get_state_from_nvs: nvs_get_blob error 1" );
        goto err_exit;
    }

    ESP_LOG_BUFFER_HEX( TAG, state, ulSize );
    ESP_LOGI( TAG, "get_state_from_nvs: finished retrieving state, printing..." );
    debugPrintDeviceState( "get_state_from_nvs", *state );
err_exit:

    if( xNvsHandle != ( nvs_handle ) NULL )
    {
        nvs_close( xNvsHandle );
    }

    return xRet;
}

/*
 *  // checksum
 *  uint8_t checkSum; // this checksum will make the sum of the structure == 0 (modulo 256)
 *  uint8_t shadowVersionMajor;
 *  uint8_t shadowVersionMinor;
 *  // valve
 *  uint8_t valveType;
 *  uint8_t valveState;
 *  // adc
 *  uint8_t adcNumSamp;
 *  // motor
 *  uint16_t mtrDelayTime;
 *  uint16_t mtrMaxI; // divide by 100 to get correct floating point value
 *  uint8_t mtrMaxT;
 *  uint16_t mtrBackT;
 *  // requested
 *  uint8_t valveStateReq;
 *  uint16_t valveStateCounter;
 *  // wifi
 *  uint16_t connect_not_flowing;
 *  uint16_t connect_sleep_multiplier;
 *  uint8_t provisioned;
 */
void debugPrintDeviceState( const char * msg,
                            shadowLSStruct_t debug )
{
    ESP_LOGI( TAG, "debugPrintDeviceState:(%s) checkSum => %d, shadowVersionMajor => %d, shadowVersionMinor => %d",
              msg,
              debug.checkSum,
              debug.shadowVersionMajor,
              debug.shadowVersionMinor );
    ESP_LOGI( TAG, "debugPrintDeviceState:(%s) ValveType => %d, ValveState => %d, ValveStateReq => %d, valveStateCounter => %d",
              msg,
              debug.valveType,
              debug.valveState,
              debug.valveStateReq,
              debug.valveStateCounter );
    ESP_LOGI( TAG, "debugPrintDeviceState:(%s) adcNumSamp => %d, mtrDelayTime => %d",
              msg,
              debug.adcNumSamp,
              debug.mtrDelayTime );

    ESP_LOGI( TAG, "debugPrintDeviceState:(%s) mtrMaxI => %d, mtrMaxT => %d, mtrBackT => %d",
              msg,
              debug.mtrMaxI,
              debug.mtrMaxT,
              debug.mtrBackT );
    ESP_LOGI( TAG, "debugPrintDeviceState:(%s) connect_not_flowing => %d",
              msg,
              debug.connect_not_flowing );
    ESP_LOGI( TAG, "debugPrintDeviceState:(%s) connect_sleep_multiplier => %d, provisioned => %d",
              msg,
              debug.connect_sleep_multiplier,
              debug.provisioned );
}

uint16_t checkSanity( shadowLSStruct_t shPtr )
{
    uint16_t sum = 0;
    uint8_t * p = ( uint8_t * ) &shPtr;

    for( uint8_t i = 0; i < sizeof( shadowLSStruct_t ); i++ )
    {
        sum += p[ i ];
    }

    return sum;
}

esp_err_t save_state_to_nvs( shadowLSStruct_t data )
{
    nvs_handle xNvsHandle = NULL;

    /* let's make sure that the valve state req is not different from the current valve state */
    /* otherwise we may move the valve on boot up, and it's only a local assignment */
    /* we only want to store a good state, right? */

    if( ( data.valveState == VOPENING ) || ( data.valveState == VCLOSING ) )
    {
        data.valveState = VUNKNOWN;
    }

    /* only save valveState when it is VUNKNOWN, VOPEN, VCLOSED? */
    data.valveStateReq = data.valveState;

    #if 0
        /* let's calculate the required checksum now (b/c we might have made changes) */
        /* we need to calculate the checksum for the in-memory state as well */
        uint16_t sanity = checkSanity( data ) - data.checkSum;
        ESP_LOGI( TAG, "save_state_to_nvs: checkSanity(%x) - old data.checkSum(%x) = new sanity(%x)", checkSanity( data ), data.checkSum, sanity );
        data.checkSum = 0x100 - ( sanity & 0xFF );
    #endif /* 0 */
    /* let's retrieve current NVS state because it might have changed since we started up the program */
    esp_err_t deltaerr = get_state_from_nvs( &nvmState );

    if( deltaerr != ESP_OK )
    {
        ESP_LOGI( TAG, "save_state_to_nvs: error retrieving nvmState" );
    }

    ESP_LOGI( TAG, "save_state_to_nvs: printing nvmState" );
    debugPrintDeviceState( "nvmState", nvmState );
    ESP_LOGI( TAG, "save_state_to_nvs: printing data" );
    debugPrintDeviceState( "save_state_to_nvs", data );

    /* let's check the equality of structures (nvm versus operating) then */
    if( compareNVM( nvmState, data ) )
    {
        /* don't need to write this structure at this time as the memory is the same already */
        ESP_LOGI( TAG, "save_state_to_nvs: consistent data - no write to flash needed" );
        return ESP_OK;
    }

    esp_err_t xRet = nvs_flash_init_partition( "nvs" );

    if( xRet != ESP_OK )
    {
        ESP_LOGE( TAG, "save_state_to_nvs: error nvs_flash_init_partition" );
    }
    else
    {
        xRet = nvs_open_from_partition( "storage", "ourdata", NVS_READWRITE, &xNvsHandle );
    }

    if( xRet != ESP_OK )
    {
        ESP_LOGE( TAG, "save_state_to_nvs: error nvs_open_from_partition" );
    }
    else
    {
        xRet = nvs_set_blob( xNvsHandle, "LSData", ( void * ) &data, sizeof( shadowLSStruct_t ) );
    }

    if( xRet != ESP_OK )
    {
        ESP_LOGE( TAG, "save_state_to_nvs: nvs_set_blob fail" );
    }
    else
    {
        xRet = nvs_commit( xNvsHandle );
    }

    if( xRet != ESP_OK )
    {
        ESP_LOGE( TAG, "save_state_to_nvs: nvs_commit fail" );
    }
    else
    {
        ESP_LOGI( TAG, "save_state_to_nvs: nvs_commit succeeded" );
        ESP_LOG_BUFFER_HEX( TAG, &data, sizeof( shadowLSStruct_t ) );
    }

    if( xNvsHandle != ( nvs_handle ) NULL )
    {
        nvs_close( xNvsHandle );
    }

    return xRet;
}

/* variables to be stored:
 * uint8_t checkSum;
 * uint8_t shadowVersionMajor;
 * uint8_t shadowVersionMinor;
 * uint8_t valveType;
 * uint8_t valveState;
 * uint8_t adcNumSamp;
 * uint16_t mtrDelayTime;
 * uint16_t mtrMaxI;
 * uint8_t mtrMaxT;
 * uint16_t mtrBackT;
 * uint8_t valveStateReq;
 * uint16_t valveStateCounter;
 * uint16_t connect_not_flowing;
 * uint16_t connect_sleep_multiplier;
 * uint8_t provisioned;
 */

void xferCurrentStateToVar( shadowLSStruct_t * data )
{
    uint16_t sanity;
    uint16_t calcSanity;

    if( data == NULL )
    {
        ESP_LOGE( TAG, "xferCurrentStateToVar: data NULL" );
        return;
    }

    /* do we wait here for consistent state? Perhaps... while the motor is moving, we should just wait... */
    /* while (my_motor_state != MIDLE || my_valve_state == VCLOSING || my_valve_state == VOPENING) */
    /* { */
    /*     vTaskDelay(pdMS_TO_TICKS(1000)); */
    /*     if (wait++ > 30) */
    /*     { */
    /*         ESP_LOGW(TAG, "xferCurrentStateToVar: waiting for motor idle"); */
    /*     } */
    /* } */
    ESP_LOGI( TAG, "xferCurrentStateToVar: starting xfer" );
    data->valveType = ( uint8_t ) my_valve_type;
    data->valveState = ( uint8_t ) my_valve_state;
    data->valveStateReq = ( uint8_t ) my_valve_state;
    data->valveStateCounter = ( uint16_t ) my_valve_state_counter;
    /* adc */
    data->adcNumSamp = ( uint8_t ) lsADC_NumSmpls;
    /* motor */
    data->mtrDelayTime = ( uint16_t ) lsMTR_delayStartMS;
    /* wifi */
    data->connect_not_flowing = ( uint16_t ) my_awake_time;
    data->connect_sleep_multiplier = ( uint16_t ) my_sleep_multiplier;
    data->provisioned = current_wifi_state.provisioned;

    data->mtrMaxI = lsMTR_currentLimHigh;
    data->mtrMaxT = ( uint8_t ) lsMTR_maxTimeS;
    data->mtrBackT = ( uint16_t ) lsMTR_turnBackmS;
    /* we need to calculate the checksum for the in-memory state as well */
    ESP_LOGI( TAG, "xferCurrentStateToVar: vars xferred, chksum sanity now" );

    calcSanity = checkSanity( *data );

    if( calcSanity >= data->checkSum )
    {
        ESP_LOGI( TAG, "xferCurrentStateToVar: sanity diff positive" );
        sanity = calcSanity - data->checkSum;
    }
    else
    {
        ESP_LOGI( TAG, "xferCurrentStateToVar: sanity negative" );
        sanity = checkSanity( *data ) - data->checkSum;
    }

    ESP_LOGI( TAG, "xferCurrentStateToVar: checkSanity(%x) - old data.checkSum(%x) = new sanity(%x)", calcSanity, data->checkSum, sanity );
    data->checkSum = 0x100 - ( sanity & 0xFF );

    debugPrintDeviceState( "xferCurrentStateToVar", *data );
    ESP_LOGI( TAG, "xferCurrentStateToVar: done" );
}

/**
 * @brief Parses a key in the "state" section of a Shadow delta document.
 *
 * @param[in] pDeltaDocument The Shadow delta document to parse.
 * @param[in] deltaDocumentLength The length of `pDeltaDocument`.
 * @param[in] pDeltaKey The key in the delta document to find. Must be NULL-terminated.
 * @param[out] pDelta Set to the first character in the delta key.
 * @param[out] pDeltaLength The length of the delta key.
 *
 * @return `true` if the given delta key is found; `false` otherwise.
 */
static bool _getDelta( const char * pDeltaDocument,
                       size_t deltaDocumentLength,
                       const char * pDeltaKey,
                       const char ** pDelta,
                       size_t * pDeltaLength )
{
    bool stateFound = false, deltaFound = false;
    const size_t deltaKeyLength = strlen( pDeltaKey );
    const char * pState = NULL;
    size_t stateLength = 0;

    /* Find the "state" key in the delta document. */
    stateFound = IotJsonUtils_FindJsonValue( pDeltaDocument,
                                             deltaDocumentLength,
                                             "state",
                                             5,
                                             &pState,
                                             &stateLength );

    if( stateFound )
    {
        /* Find the delta key within the "state" section. */
        deltaFound = IotJsonUtils_FindJsonValue( pState,
                                                 stateLength,
                                                 pDeltaKey,
                                                 deltaKeyLength,
                                                 pDelta,
                                                 pDeltaLength );
    }
    else
    {
        ESP_LOGI( TAG, "_getDelta: Failed to find \"state\" in shadow delta document." );
    }

    return deltaFound;
}

/*-----------------------------------------------------------*/
bool updateShadowTask( void * pDeltaSemaphore )
{
    ESP_LOGI( TAG, "updateShadowTask: START and wait for shadowSyncSemaphore" );

    if( IotSemaphore_TimedWait( &shadowSyncSemaphore, WAIT_FOR_SHADOW_SYNC ) )
    {
        int status = _sendShadowUpdates( pDeltaSemaphore, shadowMqttConnection );

        if( status == EXIT_SUCCESS )
        {
            ESP_LOGI( TAG, "updateShadowTask: _sendShadowUpdates successful" );
            my_shadow_error_count = 0;
        }
        else
        {
            ESP_LOGE( TAG, "updateShadowTask: _sendShadowUpdates unsuccessful" );
            my_shadow_error_count++;
            return false;
        }
    }
    else
    {
        ESP_LOGE( TAG, "updateShadowTask: timed out waiting for shadowSyncSemaphore" );
        return false;
    }

    ESP_LOGI( TAG, "updateShadowTask: posting/releasing shadowSyncSemaphore" );
    IotSemaphore_Post( &shadowSyncSemaphore ); /* if we have to wait more than 15sec something's wrong */
    ESP_LOGI( TAG, "updateShadowTask: posting/releasing shadowSemaphore" );
    IotSemaphore_Post( &shadowSemaphore );
    return true;
}
/*-----------------------------------------------------------*/

void LSSeedReqs( shadowLSStruct_t * data )
{
    ESP_LOGI( TAG, "LSSeedReqs: making valveStateReq = valveState" );
    data->valveStateReq = data->valveState;
}

void LSSyncVars( shadowLSStruct_t * data )
{
    /* we want to sync the standard variables here */
    /* starting with initLSData structure */
    ESP_LOGI( TAG, "LSSyncVars: starting sync" );
    my_valve_type = data->valveType;
    my_valve_state_counter = data->valveStateCounter;
    lsADC_NumSmpls = data->adcNumSamp;
    lsMTR_delayStartMS = data->mtrDelayTime;

    my_awake_time = data->connect_not_flowing;
    my_sleep_multiplier = data->connect_sleep_multiplier;
    lsMTR_currentLimHigh = data->mtrMaxI;
    lsMTR_maxTimeS = data->mtrMaxT;
    lsMTR_turnBackmS = data->mtrBackT;

    if( !ignoreValveStateReq )
    {
        if( my_valve_type != TUNKNOWN )
        {
            if( ( data->valveStateReq != VUNKNOWN ) && ( data->valveStateReq != my_valve_state ) )
            {
                ESP_LOGI( TAG, "LSSyncVars: valveStateReq(%d) != my_valve_state(%d)", data->valveStateReq, my_valve_state );

                /* now we need to change the state of the valve here */
                if( data->valveStateReq == VCLOSED )
                {
                    if( ( my_valve_state == VUNKNOWN ) || ( my_valve_state == VOPEN ) || ( my_valve_state == VOPENING ) )
                    {
                        my_valve_pending = true;
                        ESP_LOGI( TAG, "LSSyncVars: Current valve state %d, will now wait/close", my_valve_state );
                        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
                        // ls_mtr_send_event( esp_timer_get_time(), LS_CW_VALVE );
                    }
                    else
                    {
                        ESP_LOGI( TAG, "LSSyncVars: Current valve state %d, will do nothing", my_valve_state );
                    }
                }
                else if( data->valveStateReq == VOPEN )
                {
                    if( ( my_valve_state == VUNKNOWN ) || ( my_valve_state == VCLOSED ) || ( my_valve_state == VCLOSING ) )
                    {
                        my_valve_pending = true;
                        ESP_LOGI( TAG, "LSSyncVars: Current valve state %d, will now open", my_valve_state );
                        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
                        // ls_mtr_send_event( esp_timer_get_time(), LS_CCW_VALVE );
                    }
                    else
                    {
                        ESP_LOGI( TAG, "LSSyncVars: Current valve state %d, will do nothing", my_valve_state );
                    }
                }
                else
                {
                    ESP_LOGI( TAG, "LSSyncVars: Recvd (%d) not open/close, but current valve state %d", data->valveStateReq, my_valve_state );
                }
            }
            else
            {
                ESP_LOGI( TAG, "LSSyncVars: Recvd valveStateReq(%d), but current valve state %d", data->valveStateReq, my_valve_state );
            }
        }
        else
        {
            ESP_LOGI( TAG, "LSSyncVars: unknown valveType(%d) valveState(%d), valveStateReq(%d)", data->valveType, data->valveState, data->valveStateReq );
        }
    }
    else
    {
        ESP_LOGI( TAG, "LSSyncVars: ignoring ValveStateReq b/c counter(%d) unchanged valveState(%d) valveStateReq(%d)", data->valveStateCounter, data->valveState, data->valveStateReq );
        data->valveStateReq = data->valveState;
        null_desired = true;
    }

    if( varChanged )
    {
        xTaskNotify( xDeviceState, SHADOW_UPDATE, eSetBits );
        varChanged = false;
    }
}

/* after a reset we can load the state from NVS */
void LoadNVMVars( bool loadValveState,
                  shadowLSStruct_t data )
{
    /* we want to sync the standard variables here */
    /* starting with initLSData */
    ESP_LOGI( TAG, "LoadNVMVars: load valve state (%d) orig ValveType => %d, ValveState => %d, adcNumSamp => %d",
              ( int ) loadValveState,
              ( int ) my_valve_type,
              ( int ) my_valve_state,
              lsADC_NumSmpls );
    my_valve_type = data.valveType;
    my_awake_time = data.connect_not_flowing;
    my_sleep_multiplier = data.connect_sleep_multiplier;

    /* can't assume that the valve state is the same as it was when we slept/rebooted (should only be VUNKNOWN, VOPEN, VCLOSED) */
    /* if (loadValveState) */
    /* { */
    /*     if (data.valveState == VOPENING || data.valveState == VCLOSING) */
    /*     { */
    /*         data.valveState = VUNKNOWN; */
    /*     } */
    /*     my_valve_state = data.valveState; */
    /*     ESP_LOGI(TAG, "LoadNVMVars: Loaded valve state(%d)", my_valve_state); */
    /* } */

    if( data.valveState == VOPENING )
    {
        data.valveState = VOPEN;
    }
    else if( data.valveState == VCLOSING )
    {
        data.valveState = VCLOSED;
    }

    /* if it's unknown it will stay unknown... */
    my_valve_state = data.valveState;

    my_valve_state_counter = data.valveStateCounter;
    lsADC_NumSmpls = data.adcNumSamp;
    lsMTR_delayStartMS = data.mtrDelayTime;

    lsMTR_currentLimHigh = data.mtrMaxI;
    lsMTR_maxTimeS = data.mtrMaxT;
    lsMTR_turnBackmS = data.mtrBackT;

    current_wifi_state.provisioned = data.provisioned;

    debugPrintDeviceState( "LoadNVMVars", data );
}

/**
 * @brief Shadow delta callback, invoked when the desired and updates Shadow
 * states differ.
 *
 * @param[in] pCallbackContext Not used.
 * @param[in] pCallbackParam The received Shadow delta document.
 */
static void _shadowDeltaCallback( void * pCallbackContext,
                                  AwsIotShadowCallbackParam_t * pCallbackParam )
{
    shadowLSStruct_t deltaTmp;

    /* IotSemaphore_t * pDeltaSemaphore = pCallbackContext; */
    ESP_LOGI( TAG, "_shadowDeltaCallback: starting processing" );
    /* we assume that a variable changed (not valve state req) */
    memcpy( &deltaTmp, &initLSData, sizeof( shadowLSStruct_t ) );

    debugPrintDeviceState( "_shadowDeltaCallback", deltaTmp );

    bool deltaFound = false;
    const char * pDelta = NULL;
    size_t deltaLength = 0;
    char tmp[ 50 ];
    /* bool update_shadow_now = true; */

    ESP_LOGI( TAG, "_shadowDeltaCallback: [%d] %s", pCallbackParam->u.callback.documentLength, pCallbackParam->u.callback.pDocument );

    /* my_valve_state_counter */
    deltaFound = _getDelta( pCallbackParam->u.callback.pDocument, pCallbackParam->u.callback.documentLength, VALVE_STATE_COUNTER, &pDelta, &deltaLength );

    if( deltaFound )
    {
        memset( tmp, 0, 50 );
        snprintf( tmp, deltaLength - 1, "%s", ( pDelta + 1 ) );
        uint16_t shadowCounter = atoi( tmp );
        /* check for change in the counter to tell us whether to follow the ValveStateReq delta */
        ignoreValveStateReq = true;
        varChanged = false;

        if( shadowCounter == initLSData.valveStateCounter )
        {
            ESP_LOGI( TAG, "_shadowDeltaCallback: Found same valve state counter shadow(%d) local(%d)", shadowCounter, initLSData.valveStateCounter );
        }
        else if( shadowCounter == initLSData.valveStateCounter + 1 )
        {
            ESP_LOGI( TAG, "_shadowDeltaCallback: Found valid change valve state counter shadow(%d) local(%d)", shadowCounter, initLSData.valveStateCounter );
            ignoreValveStateReq = false;
            deltaTmp.valveStateCounter = shadowCounter;
        }
        else if( shadowCounter < initLSData.valveStateCounter )
        {
            ESP_LOGI( TAG, "_shadowDeltaCallback: Found << inconsistency in valve state counter shadow(%d) local(%d)", shadowCounter, initLSData.valveStateCounter );
        }
        else
        {
            deltaTmp.valveStateCounter = shadowCounter;
            ESP_LOGI( TAG, "_shadowDeltaCallback: Found >> inconsistency in valve state counter shadow(%d) local(%d)", shadowCounter, initLSData.valveStateCounter );
        }
    }
    else
    {
        /* did not the shadow valve state req counter which means no change to valve state req */
        ESP_LOGI( TAG, "_shadowDeltaCallback: No delta found for ValveStateCounter which means no update for ValveStateReq" );
        ignoreValveStateReq = true;
    }

    varChanged = false;

    deltaFound = _getDelta( pCallbackParam->u.callback.pDocument, pCallbackParam->u.callback.documentLength, VALVE_STATE_REQ, &pDelta, &deltaLength );

    if( deltaFound )
    {
        memset( tmp, 0, 50 );
        snprintf( tmp, deltaLength - 1, "%s", ( pDelta + 1 ) );
        deltaTmp.valveStateReq = atoi( tmp );
    }

    deltaFound = _getDelta( pCallbackParam->u.callback.pDocument, pCallbackParam->u.callback.documentLength, VALVE_TYPE, &pDelta, &deltaLength );

    if( deltaFound )
    {
        memset( tmp, 0, 50 );
        snprintf( tmp, deltaLength - 1, "%s", ( pDelta + 1 ) );
        deltaTmp.valveType = atoi( tmp );
        varChanged = true;
    }

    deltaFound = _getDelta( pCallbackParam->u.callback.pDocument, pCallbackParam->u.callback.documentLength, ADC_SAMP_NUM, &pDelta, &deltaLength );

    if( deltaFound )
    {
        memset( tmp, 0, 50 );
        snprintf( tmp, deltaLength - 1, "%s", ( pDelta + 1 ) );
        deltaTmp.adcNumSamp = atoi( tmp );
        varChanged = true;
    }

    deltaFound = _getDelta( pCallbackParam->u.callback.pDocument, pCallbackParam->u.callback.documentLength, MOTOR_DELAY_TIME, &pDelta, &deltaLength );

    if( deltaFound )
    {
        memset( tmp, 0, 50 );
        snprintf( tmp, deltaLength - 1, "%s", ( pDelta + 1 ) );
        deltaTmp.mtrDelayTime = atoi( tmp );
        varChanged = true;
    }

    deltaFound = _getDelta( pCallbackParam->u.callback.pDocument, pCallbackParam->u.callback.documentLength, MOTOR_MAX_CURRENT, &pDelta, &deltaLength );

    if( deltaFound )
    {
        memset( tmp, 0, 50 );
        snprintf( tmp, deltaLength - 1, "%s", ( pDelta + 1 ) );
        deltaTmp.mtrMaxI = atoi( tmp );
        varChanged = true;
    }

    deltaFound = _getDelta( pCallbackParam->u.callback.pDocument, pCallbackParam->u.callback.documentLength, MOTOR_MAX_TIME, &pDelta, &deltaLength );

    if( deltaFound )
    {
        memset( tmp, 0, 50 );
        snprintf( tmp, deltaLength - 1, "%s", ( pDelta + 1 ) );
        deltaTmp.mtrMaxT = atoi( tmp );
        varChanged = true;
    }

    deltaFound = _getDelta( pCallbackParam->u.callback.pDocument, pCallbackParam->u.callback.documentLength, MOTOR_BACK_TIME, &pDelta, &deltaLength );

    if( deltaFound )
    {
        memset( tmp, 0, 50 );
        snprintf( tmp, deltaLength - 1, "%s", ( pDelta + 1 ) );
        deltaTmp.mtrBackT = atoi( tmp );
        varChanged = true;
    }

    deltaFound = _getDelta( pCallbackParam->u.callback.pDocument, pCallbackParam->u.callback.documentLength, WIFI_CONNECT_AWAKE_NOT_FLOWING, &pDelta, &deltaLength );

    if( deltaFound )
    {
        memset( tmp, 0, 50 );
        snprintf( tmp, deltaLength - 1, "%s", ( pDelta + 1 ) );
        deltaTmp.connect_not_flowing = atoi( tmp );
        varChanged = true;
    }

    deltaFound = _getDelta( pCallbackParam->u.callback.pDocument, pCallbackParam->u.callback.documentLength, WIFI_CONNECT_SLEEP_MULTIPLIER, &pDelta, &deltaLength );

    if( deltaFound )
    {
        memset( tmp, 0, 50 );
        snprintf( tmp, deltaLength - 1, "%s", ( pDelta + 1 ) );
        deltaTmp.connect_sleep_multiplier = atoi( tmp );
        varChanged = true;
    }

    memcpy( &initLSData, &deltaTmp, sizeof( shadowLSStruct_t ) );

    ESP_LOGI( TAG, "_shadowDeltaCallback: Delta updated varChanged(%d)", varChanged );
    debugPrintDeviceState( "initLSData", initLSData );
    LSSyncVars( &initLSData );
}

/*-----------------------------------------------------------*/

/**
 * @brief Initialize the Shadow library.
 *
 * @return `EXIT_SUCCESS` if all libraries were successfully initialized;
 * `EXIT_FAILURE` otherwise.
 */
static int _initializeShadowLib( void )
{
    char tmp[ 24 ] = { 0 };

    AwsIotShadowError_t shadowInitStatus = AWS_IOT_SHADOW_SUCCESS;
    int status = EXIT_SUCCESS;

    sprintf( tmp, "afr(%d.%d.%d)", APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_BUILD );
    ESP_LOGI( TAG, "_initializeShadowLib: starting %s", tmp );
    strcpy( device.fw_version, tmp );

    uint8_t mac[ 6 ];
    WIFI_GetMAC( mac );
    sprintf( &device.my_mac[ 0 ], "%02X:%02X:%02X:%02X:%02X:%02X", mac[ 0 ], mac[ 1 ], mac[ 2 ], mac[ 3 ], mac[ 4 ], mac[ 5 ] );

    /* Use the default MQTT timeout. */
    shadowInitStatus = AwsIotShadow_Init( 0 );

    if( shadowInitStatus != AWS_IOT_SHADOW_SUCCESS )
    {
        status = EXIT_FAILURE;
        IotMqtt_Cleanup();
    }

    return status;
}

/**
 * @brief Set the Shadow callback functions used in this demo.
 *
 * @param[in] pDeltaSemaphore Used to synchronize Shadow updates with the delta
 * callback.
 * @param[in] mqttConnection The MQTT connection used for Shadows.
 * @param[in] pThingName The Thing Name for Shadows in this demo.
 * @param[in] thingNameLength The length of `pThingName`.
 *
 * @return `EXIT_SUCCESS` if all Shadow callbacks were set; `EXIT_FAILURE`
 * otherwise.
 */
static int _setShadowCallbacks( IotSemaphore_t * pDeltaSemaphore,
                                IotMqttConnection_t mqttConnection )
{
    int status = EXIT_SUCCESS;
    AwsIotShadowError_t callbackStatus = AWS_IOT_SHADOW_STATUS_PENDING;
    AwsIotShadowCallbackInfo_t deltaCallback = AWS_IOT_SHADOW_CALLBACK_INFO_INITIALIZER;

    ESP_LOGI( TAG, "_setShadowCallbacks: starting" );
    /* Set the functions for callbacks. */
    deltaCallback.pCallbackContext = pDeltaSemaphore;
    deltaCallback.function = _shadowDeltaCallback;

    /* Set the delta callback, which notifies of different desired and reported
     * Shadow states. */
    callbackStatus = AwsIotShadow_SetDeltaCallback( mqttConnection,
                                                    my_thing_name,
                                                    strlen( my_thing_name ),
                                                    0,
                                                    &deltaCallback );

    if( callbackStatus != AWS_IOT_SHADOW_SUCCESS )
    {
        /* ESP_LOGE(TAG, "_setShadowCallbacks: Failed to set demo shadow callback, error %s.", */
        /*  AwsIotShadow_strerror(callbackStatus)); */
        status = EXIT_FAILURE;
    }

    IotSemaphore_Post( pDeltaSemaphore );

    return status;
}

/*-----------------------------------------------------------*/

/**
 * @brief Send the Shadow updates that will trigger the Shadow callbacks.
 *
 * @param[in] pDeltaSemaphore Used to synchronize Shadow updates with the delta
 * callback.
 * @param[in] mqttConnection The MQTT connection used for Shadows.
 * @param[in] pThingName The Thing Name for Shadows in this demo.
 * @param[in] thingNameLength The length of `pThingName`.
 *
 * @return `EXIT_SUCCESS` if all Shadow updates were sent; `EXIT_FAILURE`
 * otherwise.
 */

static int _sendShadowUpdates( IotSemaphore_t * pDeltaSemaphore,
                               IotMqttConnection_t mqttConnection )
{
    int status = EXIT_SUCCESS;
    esp_err_t err_wifi;
    uint32_t ulInitialReportLength = 0;

    ESP_LOGI( TAG, "_sendShadowUpdates: starting w/isMQTTConnected(%d) isShadowConnected(%d)", isMQTTConnected(), isShadowConnected() );

    AwsIotShadowError_t updateStatus = AWS_IOT_SHADOW_STATUS_PENDING;
    AwsIotShadowDocumentInfo_t updateDocument = AWS_IOT_SHADOW_DOCUMENT_INFO_INITIALIZER;

    /* A buffer containing the update document. It has static duration to prevent
     * it from being placed on the call stack. */

    static char pUpdateDocument[ EXPECTED_REPORTED_JSON_SIZE + 1 ] = { 0 };

    /* Set the common members of the Shadow update document info. */
    updateDocument.pThingName = my_thing_name;
    updateDocument.thingNameLength = strlen( my_thing_name );
    updateDocument.u.update.pUpdateDocument = pUpdateDocument;

    /* Generate a Shadow desired state document, using a timestamp for the client
     * token. To keep the client token within 6 characters, it is modded by 1000000. */
    xferCurrentStateToVar( &initLSData );
    // ESP_LOGI( TAG, "_sendShadowUpdates: about to run updateBatteryStats" );
    // updateBatteryStats( &batteryLSData );

    ESP_LOGI( TAG, "_sendShadowUpdates: about to run _get_wifi_info" );
    err_wifi = _get_wifi_info();

    if( err_wifi != ESP_OK )
    {
        ESP_LOGI( TAG, "_sendShadowUpdates: _get_wifi_info returned %d error", err_wifi );
    }

    ESP_LOGI( TAG, "_sendShadowUpdates: valve state => %d, valve state req => %d, valve state counter => %d, lastTimeOTAChecked => %ld",
              initLSData.valveState, initLSData.valveStateReq, initLSData.valveStateCounter, lastTimeOTAChecked );

    if( ( my_button_state == BUTTON_PRESSED ) && ( my_motor_state == MIDLE ) )
    {
        ESP_LOGI( TAG, "_sendShadowUpdates: branch BUTTON_PRESSED && MIDLE" );
        /* my_button_state = BUTTON_ACK; // postpone this until we have a successful shadow sync */
        ulInitialReportLength = snprintf(
            pUpdateDocument,
            EXPECTED_REPORTED_JSON_SIZE,
            shadowREPORT_VALVE_REQUESTED,
            VALVE_STATE_REQ, initLSData.valveStateReq,
            VALVE_STATE_COUNTER, initLSData.valveStateCounter,
            VALVE_STATE, initLSData.valveState,
            ( int ) xTaskGetTickCount() );
    }
    else if( null_desired )
    {
        ESP_LOGI( TAG, "_sendShadowUpdates: branch null_desired" );
        ulInitialReportLength = snprintf(
            pUpdateDocument,
            EXPECTED_REPORTED_JSON_SIZE,
            shadowREPORT_NULL_DESIRED,
            /* device info */
            DEVICE_MAC_ADDRESS, device.my_mac,
            DEVICE_FW_VERSION, device.fw_version,
            DEVICE_PROVISIONED, device.dev_provisioned,
            DEVICE_INSTALLED, device.dev_installed,
            DEVICE_TESTED, device.dev_tested,
            DEVICE_STATE, device.dev_status,
            /* valve */
            VALVE_TYPE, initLSData.valveType,
            VALVE_STATE, initLSData.valveState,
            /* battery */
            // BATTERY_STATE, batteryLSData.batteryState,
            // SOURCE_STATE, batteryLSData.sourceState,
            // BATTERY_VOLTAGE, batteryLSData.batteryVoltage,
            // BUS_VOLTAGE, batteryLSData.busVoltage,
            // BUS_CURRENT, batteryLSData.busCurrent,
            // BATTERY_CURRENT, batteryLSData.batteryCurrent,
            // SYS_VOLTAGE, batteryLSData.sysVoltage,
            // TS_TEMP, batteryLSData.tsTemp,
            // DIE_TEMP, batteryLSData.dieTemp,
            // CHARGE_VOLTAGE, batteryLSData.chargeVoltage,
            // CHARGE_CURRENT, batteryLSData.chargeCurrent,
            /* adc */
            ADC_SAMP_NUM, initLSData.adcNumSamp,
            /* ADC_SAMP_CUMULATIVE,    initLSData.adcCumSamp, */
            /* motor */
            MOTOR_STATE, my_motor_state,
            MOTOR_DELAY_TIME, initLSData.mtrDelayTime,
            MOTOR_MAX_CURRENT, initLSData.mtrMaxI,
            MOTOR_MAX_TIME, initLSData.mtrMaxT,
            MOTOR_BACK_TIME, initLSData.mtrBackT,
            MOTOR_RUN_TIME, lsMTR_runTimeMS,
            /* requested */
            VALVE_STATE_REQ, initLSData.valveStateReq,
            VALVE_STATE_COUNTER, initLSData.valveStateCounter,
            /* wifi */
            WIFI_SSID, current_wifi_state.ssid,
            WIFI_RSSI, current_wifi_state.rssi,
            WIFI_DISCONNECTED_TIME, ( uint64_t ) current_wifi_state.disconnected_time / ( 1000 / portTICK_RATE_MS ),
            WIFI_CONNECT_AWAKE_NOT_FLOWING, initLSData.connect_not_flowing,
            WIFI_CONNECT_SLEEP_MULTIPLIER, initLSData.connect_sleep_multiplier,
            /* uptime */
            HEAP_SIZE, heap_caps_get_minimum_free_size( MALLOC_CAP_DEFAULT ),
            DEVICE_UPTIME, ( uint64_t ) xTaskGetTickCount() / ( 1000 / portTICK_RATE_MS ),
            LAST_OTA_UPDATED, lastTimeOTAChecked,
            SHADOW_VERSION, SHADOW_MAJOR_VER, SHADOW_MINOR_VER,
            ( int ) xTaskGetTickCount() );
    }
    else
    {
        ESP_LOGI( TAG, "_sendShadowUpdates: branch normal" );
        ulInitialReportLength = snprintf(
            pUpdateDocument,
            EXPECTED_REPORTED_JSON_SIZE,
            shadowREPORT_JSON,
            #if USE_SHADOW_DESIRED_SECTION
                /* desired section */
                VALVE_STATE_REQ, initLSData.valveStateReq,
            #endif /* USE_SHADOW_DESIRED_SECTION */
            /* device info */
            DEVICE_MAC_ADDRESS, device.my_mac,
            DEVICE_FW_VERSION, device.fw_version,
            DEVICE_PROVISIONED, device.dev_provisioned,
            /* DEVICE_SETUP,           device.dev_setup, */
            DEVICE_INSTALLED, device.dev_installed,
            DEVICE_TESTED, device.dev_tested,
            DEVICE_STATE, device.dev_status,
            /* valve */
            VALVE_TYPE, initLSData.valveType,
            VALVE_STATE, initLSData.valveState,
            /* battery */
            // BATTERY_STATE, batteryLSData.batteryState,
            // SOURCE_STATE, batteryLSData.sourceState,
            // BATTERY_VOLTAGE, batteryLSData.batteryVoltage,
            // BUS_VOLTAGE, batteryLSData.busVoltage,
            // BUS_CURRENT, batteryLSData.busCurrent,
            // BATTERY_CURRENT, batteryLSData.batteryCurrent,
            // SYS_VOLTAGE, batteryLSData.sysVoltage,
            // TS_TEMP, batteryLSData.tsTemp,
            // DIE_TEMP, batteryLSData.dieTemp,
            // CHARGE_VOLTAGE, batteryLSData.chargeVoltage,
            // CHARGE_CURRENT, batteryLSData.chargeCurrent,
            /* adc */
            ADC_SAMP_NUM, initLSData.adcNumSamp,
            /* motor */
            MOTOR_STATE, my_motor_state,
            MOTOR_DELAY_TIME, initLSData.mtrDelayTime,
            MOTOR_MAX_CURRENT, initLSData.mtrMaxI,
            MOTOR_MAX_TIME, initLSData.mtrMaxT,
            MOTOR_BACK_TIME, initLSData.mtrBackT,
            MOTOR_RUN_TIME, lsMTR_runTimeMS,
            /* requested */
            VALVE_STATE_REQ, initLSData.valveStateReq,
            VALVE_STATE_COUNTER, initLSData.valveStateCounter,
            /* wifi */
            WIFI_SSID, current_wifi_state.ssid,
            WIFI_RSSI, current_wifi_state.rssi,
            WIFI_DISCONNECTED_TIME, ( uint64_t ) current_wifi_state.disconnected_time / ( 1000 / portTICK_RATE_MS ),
            WIFI_CONNECT_AWAKE_NOT_FLOWING, initLSData.connect_not_flowing,
            WIFI_CONNECT_SLEEP_MULTIPLIER, initLSData.connect_sleep_multiplier,
            /* uptime */
            HEAP_SIZE, heap_caps_get_minimum_free_size( MALLOC_CAP_DEFAULT ),
            DEVICE_UPTIME, ( uint64_t ) xTaskGetTickCount() / ( 1000 / portTICK_RATE_MS ),
            LAST_OTA_UPDATED, lastTimeOTAChecked,
            SHADOW_VERSION, SHADOW_MAJOR_VER, SHADOW_MINOR_VER,
            ( int ) xTaskGetTickCount() );
    }

    ESP_LOGI( TAG, "_sendShadowUpdates: Size of raw shadow update: (%d)", ulInitialReportLength );

    updateDocument.u.update.updateDocumentLength = strlen( pUpdateDocument );

    ESP_LOGI( TAG, "_sendShadowUpdates: Calling AwsIotShadow_TimedUpdate: %s", pUpdateDocument );

    updateStatus = AwsIotShadow_TimedUpdate( mqttConnection,
                                             &updateDocument,
                                             0,
                                             AWS_TIMEOUT_MS );

    /* Check the status of the Shadow update. */
    if( updateStatus != AWS_IOT_SHADOW_SUCCESS )
    {
        ESP_LOGE( TAG, "_sendShadowUpdates: Failed to send Shadow update, error %s.", AwsIotShadow_strerror( updateStatus ) );
        status = EXIT_FAILURE;
    }
    else
    {
        ESP_LOGI( TAG, "_sendShadowUpdates: Successfully sent shadow update" );
        status = EXIT_SUCCESS;

        if( my_button_state == BUTTON_PRESSED )
        {
            ESP_LOGI( TAG, "_sendShadowUpdates: cleared my_button_state == BUTTON_PRESSED flag" );
            my_button_state = BUTTON_ACK;
        }

        if( null_desired )
        {
            ESP_LOGI( TAG, "_sendShadowUpdates: cleared null_desired flag" );
            null_desired = false;
        }
    }

    recordLocalStateAndSave( initLSData );
    /* do we free any memory here, like */
    return status;
}

/*-----------------------------------------------------------*/
int setupShadow( IotMqttConnection_t mqttConnection )
{
    /* handleDelta = true; */

    /* Return value of this function and the exit status of this program. */
    int status = EXIT_SUCCESS;

    /* Flags for tracking which cleanup functions must be called. */
    bool deltaSemaphoreCreated = false;

    ESP_LOGI( TAG, "setupShadow: starting" );
    /* start_shadow_update_state(); */

    shadowMqttConnection = mqttConnection;

    /* Initialize the libraries required for this demo. */
    if( status == EXIT_SUCCESS )
    {
        status = _initializeShadowLib();
        ESP_LOGI( TAG, "setupShadow: _initializeShadowLib returned" );
    }

    if( status == EXIT_SUCCESS )
    {
        /* Create the semaphore that synchronizes with the delta callback. */
        if( deltaSemaphoreCreated == false )
        {
            deltaSemaphoreCreated = IotSemaphore_Create( &deltaSemaphore, 0, 1 );
        }

        if( deltaSemaphoreCreated == false )
        {
            status = EXIT_FAILURE;
        }
    }
    else
    {
        ESP_LOGE( TAG, "setupShadow: Failed to init shadow" );
    }

    if( status == EXIT_SUCCESS )
    {
        /* Set the Shadow callbacks for this demo. */
        status = _setShadowCallbacks( &deltaSemaphore, mqttConnection );
        ESP_LOGI( TAG, "setupShadow: _setShadowCallbacks returned %d", status );
    }
    else
    {
        ESP_LOGE( TAG, "setupShadow: Failed to set callbacks shadow" );
    }

    return status;
}

void getTimeFromShadow()
{
    ESP_LOGI( TAG, "getTimeFromShadow: setup for AwsIotShadow_Get" );

    size_t thingNameLength = 0;
    thingNameLength = strlen( my_thing_name );

    static char pUpdateDocument[ EXPECTED_REPORTED_JSON_SIZE + 1 ] = { 0 };
    AwsIotShadowCallbackInfo_t callbackInfo = AWS_IOT_SHADOW_CALLBACK_INFO_INITIALIZER;
    AwsIotShadowOperation_t operation = ( AwsIotShadowOperation_t ) AWS_IOT_SHADOW_GET_COMPLETE;
    AwsIotShadowError_t updateStatus = AWS_IOT_SHADOW_STATUS_PENDING;
    AwsIotShadowDocumentInfo_t updateDocument = AWS_IOT_SHADOW_DOCUMENT_INFO_INITIALIZER;

    callbackInfo.function = _operationComplete;
    callbackInfo.pCallbackContext = &deltaSemaphore;
    updateDocument.pThingName = my_thing_name;
    updateDocument.thingNameLength = thingNameLength;
    updateDocument.u.update.pUpdateDocument = pUpdateDocument;

    IotSemaphore_Wait( &deltaSemaphore );
    ESP_LOGI( TAG, "getTimeFromShadow: locked deltaSemaphore 1st" );
    updateStatus = AwsIotShadow_Get( shadowMqttConnection, &updateDocument, 0, &callbackInfo, &operation );

    ESP_LOGI( TAG, "getTimeFromShadow: AwsIotShadow_Get returned %d", updateStatus );
    IotSemaphore_Wait( &deltaSemaphore );
    ESP_LOGI( TAG, "getTimeFromShadow: locked deltaSemaphore 2nd" );
    AwsIotShadow_RemovePersistentSubscriptions( shadowMqttConnection, my_thing_name, thingNameLength, AWS_IOT_SHADOW_FLAG_REMOVE_GET_SUBSCRIPTIONS );
}

#define USE_DUMMY_WIFI_INFO    0

static esp_err_t _get_wifi_info()
{
    esp_err_t err;

    memset( current_wifi_state.ssid, 0, 64 );
    wifi_ap_record_t ap_info;
    err = esp_wifi_sta_get_ap_info( &ap_info );
    /* *    - ESP_OK: succeed */
    /* *    - ESP_ERR_WIFI_CONN: The station interface don't initialized */
    /* *    - ESP_ERR_WIFI_NOT_CONNECT: The station is in disconnect status */

    if( err == ESP_OK )
    {
        ESP_LOGI( TAG, "_get_wifi_info: RSSI => %d", ap_info.rssi );
        current_wifi_state.rssi = ap_info.rssi;

        strncpy( current_wifi_state.ssid, ( char * ) ap_info.ssid, strlen( ( char * ) ap_info.ssid ) );

        if( ap_info.rssi == 0 )
        {
            ESP_LOGI( TAG, "_get_wifi_info: we are disconnected!" );
        }
    }
    else if( err == ESP_ERR_WIFI_NOT_CONNECT )
    {
        strncpy( current_wifi_state.ssid, "WIFI_NOCONN", 11 );
        ap_info.rssi = 0;
        ESP_LOGI( TAG, "_get_wifi_info: WiFI AP not connected" );
    }
    else if( err == ESP_ERR_WIFI_CONN )
    {
        strncpy( current_wifi_state.ssid, "WIFI_NOINIT", 11 );
        ap_info.rssi = 0;
        ESP_LOGI( TAG, "_get_wifi_info: WiFI not initialized" );
    }

    return err;
}

/*-----------------------------------------------------------*/

/**
 * Shadow GET callback used to get date time from timestamp
 */

static char * last_strstr( const char * haystack,
                           const char * needle )
{
    if( *needle == '\0' )
    {
        return ( char * ) haystack;
    }

    char * result = NULL;

    for( ; ; )
    {
        char * p = strstr( haystack, needle );

        if( p == NULL )
        {
            break;
        }

        result = p;
        haystack = p + 1;
    }

    return result;
}

static void _operationComplete( void * pArgument,
                                AwsIotShadowCallbackParam_t * pOperation )
{
    ESP_LOGI( TAG, "_operationComplete: get completed result => %d", pOperation->u.operation.result );

    if( pOperation->u.operation.result == AWS_IOT_SHADOW_SUCCESS )
    {
        const char * pDelta = NULL;
        size_t deltaLength = 0;

        /* Check the retrieved Shadow document. */
        if( pOperation->callbackType == AWS_IOT_SHADOW_GET_COMPLETE )
        {
            char * pJsonValueA = NULL;
            char * pJsonValueB = NULL;
            /* size_t jsonValueLength = 0; */
            unsigned long int timestamp = 0;
            ESP_LOGI( TAG, "_operationComplete: processing callback for AWS_IOT_SHADOW_GET_COMPLETE" );

            pJsonValueA = last_strstr( pOperation->u.operation.get.pDocument, "timestamp" );

            if( pJsonValueA != NULL )
            {
                pJsonValueB = strstr( pJsonValueA, "1" ); /* look for leading '1' in the timestamp I guess */

                if( pJsonValueB != NULL )
                {
                    timestamp = atol( pJsonValueB );
                    ESP_LOGI( TAG, "_operationComplete: json(%.*s) => timestamp(%ld)", 10, pJsonValueB, timestamp );
                    set_timestamp( timestamp );
                }
                else
                {
                    ESP_LOGI( TAG, "_operationComplete: no timestamp found in json(%s)", pJsonValueA );
                }
            }

            if( _getDelta( pOperation->u.operation.get.pDocument, pOperation->u.operation.get.documentLength, LAST_OTA_UPDATED, &pDelta, &deltaLength ) )
            {
                char tmp[ 50 ];
                long temp_timeOTAChecked;
                memset( tmp, 0, 50 );
                snprintf( tmp, deltaLength - 1, "%s", ( pDelta + 1 ) );
                temp_timeOTAChecked = atol( tmp );
                ESP_LOGI( TAG, "_operationComplete: pJsonValue %.*s => timestamp(%ld) temp_lastTimeOTAChecked(%ld)", 10, pJsonValueB, timestamp, temp_timeOTAChecked );

                if( temp_timeOTAChecked == 0 )
                {
                    ESP_LOGI( TAG, "_operationComplete: timestamp(%ld) = lastTimeOTAChecked", timestamp );
                    temp_timeOTAChecked = timestamp;
                }

                updateLastTimeOTAChecked( temp_timeOTAChecked );
            }
        }
        else
        {
            ESP_LOGE( TAG, "_operationComplete: got the wrong callback type: %d", pOperation->callbackType );
        }
    }

    IotSemaphore_Post( pArgument );
}
