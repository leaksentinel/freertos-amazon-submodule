/*
 * Copyright (C) 2020 LeakSentinel, Inc. or its affiliates.  All Rights Reserved.
 *
 */

#ifdef IOT_CONFIG_FILE
    #include IOT_CONFIG_FILE
#endif

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "time.h"
#include "sys/time.h"
/* Demo includes */
#include "iot_demo_runner.h"
/* #include "aws_dev_mode_key_provisioning.h" */
#include "aws_demo.h"

/* AWS System includes. */
#include "bt_hal_manager.h"
#include "iot_system_init.h"
#include "iot_logging_task.h"
#include "aws_iot_ota_agent.h"

#include "nvs_flash.h"
#if !AFR_ESP_LWIP
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#endif

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_interface.h"
#include "esp_bt.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_ota_ops.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "errno.h"


#if CONFIG_NIMBLE_ENABLED == 1
    #include "esp_nimble_hci.h"
#else
    #include "esp_gap_ble_api.h"
    #include "esp_bt_main.h"
#endif

#include "driver/uart.h"
#include "driver/gpio.h"
#include "aws_application_version.h"
#include "iot_network_manager_private.h"
#include "iot_mqtt.h"
#include "mbedtls/ssl.h"       /* for debugging purposes */
#include "mbedtls/debug.h"     /* for debugging purposes */
#include "mbedtls/esp_debug.h" /* for debugging purposes */
#include "iot_demo_logging.h"

#include "bt_hal_manager_adapter_ble.h"
#include "bt_hal_manager.h"
#include "bt_hal_gatt_server.h"
#include "tcpip_adapter.h"

#include "iot_ble.h"
#include "iot_ble_config.h"
#include "iot_ble_wifi_provisioning.h"
#include "iot_ble_numericComparison.h"

#include "aws_iot_shadow.h"
#include "iot_network_manager_private.h"
#include "aws_ota_codesigner_certificate.h"

#include "ls_typedefs.h"
#include "ls_main.h"
#include "ls_board_setup.h"
#include "ls_gpio.h"
#include "ls_shadow.h"
#include "ls_device_state.h"
esp_err_t recordLocalStateAndSave( shadowLSStruct_t inputD );

#define TAG    "MAIN"

/* Globals to identify the state of the system! */
sleep_state my_sleep_state = CAN_SLEEP;
device_info_t device = { 0 };

bool isMQTTAttempted = false;
uint16_t my_shadow_error_count = 0;
uint16_t my_wifi_error_count = 0;
uint16_t my_mqtt_error_count = 0;
uint16_t my_shadow_skipped_count = 0;
bool wifi_cleanup_in_progress = false;

shadowLSStruct_t initLSData = { 0 };
shadowLSStruct_t nvmState = { 0 };
wifi_state_t current_wifi_state;
static bool deinitBLE = false;
bool ota_job_in_progress = false;
IotSemaphore_t shadowSemaphore;
IotSemaphore_t shadowSyncSemaphore;
char my_thing_name[ 64 ] = { 0 };

static struct timeval tv;
static uint32_t ourConnectedNetwork = AWSIOT_NETWORK_TYPE_WIFI;
static const IotNetworkInterface_t * pNetworkInterface = NULL;
static void * pConnectionParams = NULL, * pCredentials = NULL;
static bool isOTAConnectedVar = false;
static bool isShadowConnectedVar = false;
long lastTimeOTAChecked = 1574184790;
bool check_for_ota = false;
/* static RTC_DATA_ATTR struct timeval sleep_enter_time; */
IotMqttNetworkInfo_t networkInfo = IOT_MQTT_NETWORK_INFO_INITIALIZER;
IotMqttConnectInfo_t connectInfo = IOT_MQTT_CONNECT_INFO_INITIALIZER;
static IotMqttConnection_t myMQTTConnection = IOT_MQTT_CONNECTION_INITIALIZER;
QueueHandle_t spp_uart_queue = NULL;
static uint8_t connect_retry = MAX_MQTT_RETRY_COUNT;
static bool mqttInitialized = false;
static bool connectMQTTAttempt = false;

static void _connectMqttShadow();
esp_err_t get_thing_name();
void directTasksStatus();

/**
 * @brief Application task startup hook.
 */
void vApplicationDaemonTaskStartupHook( void );

/**
 * @brief Initializes the board.
 */
static void prvMiscInitialization( void );

#if BLE_ENABLED
/* Initializes bluetooth */
    static esp_err_t prvBLEStackInit( void );
    static void spp_uart_init( void );
#endif

extern char clientcredentialIOT_THING_NAME[ 64 ];
extern xTaskHandle OTAHandle;

extern esp_err_t get_state_from_nvs( shadowLSStruct_t * state );
extern bool localStateIsBad( shadowLSStruct_t );
extern void device_provisioned();
extern void setWiFiStatus( bool connected );
extern int setupShadow( IotMqttConnection_t mqttConnection );
extern int vStartOTAUpdate( IotMqttNetworkInfo_t networkInfo,
                            IotMqttConnection_t mqttConnection );
extern void LoadNVMVars( bool loadValveState,
                         shadowLSStruct_t data );
extern void ota_update_state( bool ongoing );
extern void wifi_pwr_state();
extern void DEMO_RUNNER_RunDemos1( void );
extern void DEMO_RUNNER_StopDemos1( void );
extern bool start_sleepExtTimer(); /* we got a button action - let's stay awake now for 5 min */
extern bool isWiFiConnected();
extern void lsvApplicationIPInit( void );
extern void esp_vApplicationTickHook();
extern void esp_vApplicationIdleHook();

/* Logging Task Defines. */
#define mainLOGGING_MESSAGE_QUEUE_LENGTH    ( 32 )
#define mainLOGGING_TASK_STACK_SIZE         ( configMINIMAL_STACK_SIZE * 4 )
#define mainDEVICE_NICK_NAME                "LeakSentinel_Product"

/* Declare the firmware version structure for all to see. */
/* problem is that it conflicts with the definition in the demo dir of AFR */
/* const AppVersion32_t xAppFirmwareVersion = */
/* { */
/*     .u.x.ucMajor = APP_VERSION_MAJOR, */
/*     .u.x.ucMinor = APP_VERSION_MINOR, */
/*     .u.x.usBuild = APP_VERSION_BUILD, */
/* }; */

/* Static arrays for FreeRTOS+TCP stack initialization for Ethernet network connections
 * are use are below. If you are using an Ethernet connection on your MCU device it is
 * recommended to use the FreeRTOS+TCP stack. The default values are defined in
 * FreeRTOSConfig.h. */

static int _initializeMqtt( void )
{
    int status = EXIT_SUCCESS;
    IotMqttError_t mqttInitStatus = IOT_MQTT_SUCCESS;

    /* Initialize the MQTT library. */
    mqttInitStatus = IotMqtt_Init();

    if( mqttInitStatus == IOT_MQTT_SUCCESS )
    {
        mqttInitialized = true;
    }
    else
    {
        mqttInitialized = false;
        status = EXIT_FAILURE;
        IotMqtt_Cleanup();
    }

    return status;
}

/* -------------------------------------------------------------------------------------- */

/**
 * @brief Application runtime entry point.
 */

int app_main( void )
{
    xLoggingTaskInitialize( mainLOGGING_TASK_STACK_SIZE,
                            tskIDLE_PRIORITY + 5,
                            mainLOGGING_MESSAGE_QUEUE_LENGTH );


    const esp_partition_t * running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;

    if( esp_ota_get_state_partition( running, &ota_state ) == ESP_OK )
    {
        if( ota_state == ESP_OTA_IMG_PENDING_VERIFY )
        {
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }

    IotSemaphore_Create( &shadowSemaphore, 0, 1 );
    IotSemaphore_Create( &shadowSyncSemaphore, 0, 1 );
    IotSemaphore_Post( &shadowSyncSemaphore ); /* let's make the semaphore active (== 1) and available to grab */

    prvMiscInitialization();

    device.dev_provisioned = false;
    device.dev_installed = false;
    device.dev_tested = false;
    device.dev_status = 0;

    struct timeval now;

    gettimeofday( &now, NULL );
    /* int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000; */

/*
 *  ESP_SLEEP_WAKEUP_UNDEFINED,    //!< In case of deep sleep, reset was not caused by exit from deep sleep
 *  ESP_SLEEP_WAKEUP_ALL,          //!< Not a wakeup cause, used to disable all wakeup sources with esp_sleep_disable_wakeup_source
 *  ESP_SLEEP_WAKEUP_EXT0,         //!< Wakeup caused by external signal using RTC_IO
 *  ESP_SLEEP_WAKEUP_EXT1,         //!< Wakeup caused by external signal using RTC_CNTL
 *  ESP_SLEEP_WAKEUP_TIMER,        //!< Wakeup caused by timer
 *  ESP_SLEEP_WAKEUP_TOUCHPAD,     //!< Wakeup caused by touchpad
 *  ESP_SLEEP_WAKEUP_ULP,          //!< Wakeup caused by ULP program
 *  ESP_SLEEP_WAKEUP_GPIO,         //!< Wakeup caused by GPIO (light sleep only)
 *  ESP_SLEEP_WAKEUP_UART,         //!< Wakeup caused by UART (light sleep only)
 */
    switch( esp_sleep_get_wakeup_cause() )
    {
        case ESP_SLEEP_WAKEUP_EXT1:
           {
               uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();

               if( wakeup_pin_mask != 0 )
               {
                   int pin = __builtin_ffsll( wakeup_pin_mask ) - 1;
                   printf( "Wake up from GPIO %d\n", pin );
               }
               else
               {
                   printf( "Wake up from GPIO\n" );
               }

               break;
           }

        case ESP_SLEEP_WAKEUP_TIMER:
            /* printf("Wake up from timer. Time spent in deep sleep (likely bogus): %d sec\n", sleep_time_ms / 1000); */
            device.dev_status &= ( ~STATUS_TIMER_WAKE );
            device.dev_status |= ( STATUS_TIMER_WAKE );
            break;

        case ESP_SLEEP_WAKEUP_EXT0:
        case ESP_SLEEP_WAKEUP_TOUCHPAD:
        case ESP_SLEEP_WAKEUP_ULP:
        case ESP_SLEEP_WAKEUP_GPIO:
        case ESP_SLEEP_WAKEUP_UART:
            printf( "Unexpected wakeup source (%d)\n", esp_sleep_get_wakeup_cause() );
            device.dev_status &= ( ~STATUS_UNEXPECTED_WAKE );
            device.dev_status |= ( STATUS_UNEXPECTED_WAKE );
            break;

        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf( "Undefined wakeup cause (%d)\n", esp_sleep_get_wakeup_cause() );
            device.dev_status &= ( ~STATUS_UNKNOWN_WAKE );
            device.dev_status |= ( STATUS_UNKNOWN_WAKE );
    }

    esp_log_level_set( "*", ESP_LOG_NONE );

    esp_log_level_set( "SHADOW", ESP_LOG_DEBUG );
    /* esp_log_level_set( "GPIO", ESP_LOG_DEBUG ); */
    /* esp_log_level_set("mbedtls", ESP_LOG_DEBUG); */
    esp_log_level_set( "DEVICE", ESP_LOG_DEBUG );
    /* esp_log_level_set( "OTA", ESP_LOG_DEBUG ); */
    esp_log_level_set( "MAIN", ESP_LOG_DEBUG );
    /* esp_log_level_set("WIFI", ESP_LOG_DEBUG); */
    /* esp_log_level_set("wifi", ESP_LOG_DEBUG); */
    /* esp_log_level_set("i2c", ESP_LOG_DEBUG); */
    esp_log_level_set( "DEMORUNNER", ESP_LOG_DEBUG );
    esp_log_level_set( "ota_pal", ESP_LOG_DEBUG );
    esp_log_level_set( "boot", ESP_LOG_DEBUG );

    #if LOAD_STATE_FROM_NVS
        initLSData.shadowVersionMajor = SHADOW_MAJOR_VER; /* let's load the current compiled version of shadow */
        initLSData.shadowVersionMinor = SHADOW_MINOR_VER; /* let's load the current compiled version of shadow */

        esp_err_t deltaerr = get_state_from_nvs( &nvmState );

        if( ( deltaerr != ESP_OK ) || localStateIsBad( nvmState ) )
        {
            ESP_LOGE( TAG, "app_main: Failed to retrieve good shadow from NVS" );
            recordLocalStateAndSave( initLSData );
        }
 /*       
        else if( valveTypeIsUnknown( nvmState ) )
        {
            // we have a good state, Houston if we woke up from reset that is.
            ESP_LOGI( TAG, "app_main: retrieved good shadow with unknown valve type from NVS, copying..." );
            memcpy( &initLSData, &nvmState, sizeof( shadowLSStruct_t ) );
            LoadNVMVars( true, initLSData );
            start_sleepExtTimer(); // we got an unknown valve type - let's stay awake now for 5 min 
        }
*/
        else
        {
            /* we have a good state, Houston if we woke up from reset that is. */
            ESP_LOGI( TAG, "app_main: retrieved good shadow from NVS, copying..." );
            memcpy( &initLSData, &nvmState, sizeof( shadowLSStruct_t ) );
            LoadNVMVars( true, initLSData );
        }
    #endif /* LOAD_STATE_FROM_NVS */

    /* let's move it here to capture the wifi state more accurately */
    wifi_pwr_state();

    /* now let's stay awake if we were woken up by button push */
    if( false /* button_stay_awake */)
    {
        device.dev_status &= ( ~STATUS_BUTTON_WAKE );
        device.dev_status |= ( STATUS_BUTTON_WAKE );
        start_sleepExtTimer(); /* we got a button action - let's stay awake now for 5 min */
    }

    if( !current_wifi_state.provisioned )
    {
        start_sleepExtTimer(); /* we aren't provisioned - let's stay awake now for 5 min */
    }

    #if !USE_NETWORK
        while( 1 )
        {
            vTaskDelay( 500 / portTICK_PERIOD_MS );
        }
    #endif /* !USE_NETWORK */

    if( SYSTEM_Init() == pdPASS )
    {
        #if BLE_ENABLED
            /* Initialize BLE. */
            ESP_ERROR_CHECK( esp_bt_controller_mem_release( ESP_BT_MODE_CLASSIC_BT ) );

            if( prvBLEStackInit() != ESP_OK )
            {
                configPRINTF( ( "Failed to initialize the bluetooth stack\n" ) );

                while( 1 )
                {
                }
            }
        #else
            ESP_ERROR_CHECK( esp_bt_controller_mem_release( ESP_BT_MODE_CLASSIC_BT ) );
            ESP_ERROR_CHECK( esp_bt_controller_mem_release( ESP_BT_MODE_BLE ) );
        #endif /* if BLE_ENABLED */

        wifi_pwr_state();

        /* Run all demos. */
        ESP_LOGI( TAG, "app_main: About to run DEMO_RUNNER_RunDemos1" );
        DEMO_RUNNER_RunDemos1();
        current_wifi_state.initialized = true;
        /* IotSemaphore_Post(&buttonLockoutSemaphore); */
        /* let's make the semaphore active (== 1) and available to grab */
        /* we have to wait till this is initialized so we run this here */

        int status = _initializeMqtt();
        ESP_LOGI( TAG, "app_main: _initializeMqtt returned(%d)", status );

        pNetworkInterface = AwsIotNetworkManager_GetNetworkInterface( ourConnectedNetwork );
        pConnectionParams = AwsIotNetworkManager_GetConnectionParams( ourConnectedNetwork );
        pCredentials = AwsIotNetworkManager_GetCredentials( ourConnectedNetwork );
    }

    return 0;
}

/*-----------------------------------------------------------*/
static void prvMiscInitialization( void )
{
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();

    if( ( ret == ESP_ERR_NVS_NO_FREE_PAGES ) || ( ret == ESP_ERR_NVS_NEW_VERSION_FOUND ) )
    {
        ESP_LOGE( TAG, "prvMiscInitialization: NVS error" );
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK( ret );

    #if BLE_ENABLED
        NumericComparisonInit();
        spp_uart_init();
    #endif

    /* Create tasks that are not dependent on the WiFi being initialized. */
    /* xLoggingTaskInitialize( mainLOGGING_TASK_STACK_SIZE, */
    /*                         tskIDLE_PRIORITY + 5, */
    /*                         mainLOGGING_MESSAGE_QUEUE_LENGTH ); */

    if( get_thing_name() == ESP_OK )
    {
        ESP_LOGI( TAG, "thing name: %s", my_thing_name );
    }
    else
    {
        ESP_LOGE( TAG, "can't get thing name" );
    }

    ESP_LOGI( TAG, "Code version => (%d.%d.%d)", APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_BUILD );
#if AFR_ESP_LWIP
    configPRINTF( ("Initializing lwIP TCP stack\r\n") );
    tcpip_adapter_init();
#else
    configPRINTF( ("Initializing FreeRTOS TCP stack\r\n") );
    vApplicationIPInit();
#endif
}

/*-----------------------------------------------------------*/

#if BLE_ENABLED

    #if CONFIG_NIMBLE_ENABLED == 1
        esp_err_t prvBLEStackInit( void )
        {
            return ESP_OK;
        }


        esp_err_t xBLEStackTeardown( void )
        {
            BTStatus_t xRet = IotBle_Off(); /* */
            const BTInterface_t * xBTinter = BTGetBluetoothInterface();

            /* IotNetworkInterface_t * xBTiface = AwsIotNetworkManager_GetNetworkInterface( AWSIOT_NETWORK_TYPE_BLE ); */
            /* xBTinter->pxDisable(); */

            xBTinter->pxBtManagerCleanup();

            xRet = esp_bt_controller_mem_release( ESP_BT_MODE_BLE );
            return xRet;
        }

    #else /* if CONFIG_NIMBLE_ENABLED == 1 */

        static esp_err_t prvBLEStackInit( void )
        {
            return ESP_OK;
        }

        esp_err_t xBLEStackTeardown( void )
        {
            esp_err_t xRet = ESP_OK;

            if( esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED )
            {
                xRet = esp_bluedroid_disable();
            }

            if( xRet == ESP_OK )
            {
                xRet = esp_bluedroid_deinit();
            }

            if( xRet == ESP_OK )
            {
                if( esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED )
                {
                    xRet = esp_bt_controller_disable();
                }
            }

            if( xRet == ESP_OK )
            {
                xRet = esp_bt_controller_deinit();
            }

            if( xRet == ESP_OK )
            {
                xRet = esp_bt_controller_mem_release( ESP_BT_MODE_BLE );
            }

            if( xRet == ESP_OK )
            {
                xRet = esp_bt_controller_mem_release( ESP_BT_MODE_BTDM );
            }

            return xRet;
        }
    #endif /* if CONFIG_NIMBLE_ENABLED == 1 */
#endif /* if BLE_ENABLED */

/*-----------------------------------------------------------*/

#if BLE_ENABLED
    static void spp_uart_init( void )
    {
        uart_config_t uart_config =
        {
            .baud_rate           = 115200,
            .data_bits           = UART_DATA_8_BITS,
            .parity              = UART_PARITY_DISABLE,
            .stop_bits           = UART_STOP_BITS_1,
            .flow_ctrl           = UART_HW_FLOWCTRL_RTS,
            .rx_flow_ctrl_thresh = 122,
        };

        /* Set UART parameters */
        uart_param_config( UART_NUM_0, &uart_config );
        /*Set UART pins */
        uart_set_pin( UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE );
        /*Install UART driver, and get the queue. */
        uart_driver_install( UART_NUM_0, 4096, 8192, 10, &spp_uart_queue, 0 );
    }

/*-----------------------------------------------------------*/

    BaseType_t getUserMessage( INPUTMessage_t * pxINPUTmessage,
                               TickType_t xAuthTimeout )
    {
        uart_event_t xEvent;
        BaseType_t xReturnMessage = pdFALSE;

        if( xQueueReceive( spp_uart_queue, ( void * ) &xEvent, ( portTickType ) xAuthTimeout ) )
        {
            switch( xEvent.type )
            {
                /*Event of UART receiving data */
                case UART_DATA:

                    if( xEvent.size )
                    {
                        pxINPUTmessage->pcData = ( uint8_t * ) malloc( sizeof( uint8_t ) * xEvent.size );

                        if( pxINPUTmessage->pcData != NULL )
                        {
                            memset( pxINPUTmessage->pcData, 0x0, xEvent.size );
                            uart_read_bytes( UART_NUM_0, ( uint8_t * ) pxINPUTmessage->pcData, xEvent.size, portMAX_DELAY );
                            xReturnMessage = pdTRUE;
                        }
                        else
                        {
                            configPRINTF( ( "Malloc failed in main.c\n" ) );
                        }
                    }

                    break;

                default:
                    break;
            }
        }

        return xReturnMessage;
    }
#endif /* if BLE_ENABLED */

/*-----------------------------------------------------------*/

void IRAM_ATTR vApplicationTickHook()
{
    esp_vApplicationTickHook();
}

/*-----------------------------------------------------------*/
void vApplicationIdleHook()
{
    esp_vApplicationIdleHook();
}

/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook( void )
{
}

#if !AFR_ESP_LWIP
/*-----------------------------------------------------------*/
void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent )
{
    uint32_t ulIPAddress, ulNetMask, ulGatewayAddress, ulDNSServerAddress;
    system_event_t evt;

    if( eNetworkEvent == eNetworkUp )
    {
        /* Print out the network configuration, which may have come from a DHCP
         * server. */
        FreeRTOS_GetAddressConfiguration(
            &ulIPAddress,
            &ulNetMask,
            &ulGatewayAddress,
            &ulDNSServerAddress );

        evt.event_id = SYSTEM_EVENT_STA_GOT_IP;
        evt.event_info.got_ip.ip_changed = true;
        evt.event_info.got_ip.ip_info.ip.addr = ulIPAddress;
        evt.event_info.got_ip.ip_info.netmask.addr = ulNetMask;
        evt.event_info.got_ip.ip_info.gw.addr = ulGatewayAddress;
        esp_event_send( &evt );
    }
}
#endif /* !AFR_ESP_LWIP */

esp_err_t get_thing_name()
{
    uint32_t ulSize = 0;
    char * pucBuffer = NULL;
    nvs_handle xNvsHandle = NULL;
    esp_err_t xRet = nvs_flash_init_partition( "storage" );

    if( xRet != ESP_OK )
    {
        ESP_LOGE( TAG, "get_thing_name: error 0" );
    }

    xRet = nvs_open_from_partition( "storage", "info", NVS_READONLY, &xNvsHandle );

    if( xRet != ESP_OK )
    {
        ESP_LOGE( TAG, "get_thing_name: error 1" );
    }

    xRet = nvs_get_str( xNvsHandle, "thing_name", NULL, &ulSize );

    if( xRet != ESP_OK )
    {
        ESP_LOGE( TAG, "get_thing_name: error 2" );
    }

    if( xRet == ESP_OK )
    {
        pucBuffer = calloc( 1, ulSize );

        if( pucBuffer != NULL )
        {
            xRet = nvs_get_str( xNvsHandle, "thing_name", pucBuffer, &ulSize );
        }
        else
        {
            xRet = ESP_FAIL;
        }
    }

    if( xRet != ESP_OK )
    {
        ESP_LOGE( TAG, "get_thing_name: error 3" );
    }

    if( xRet == ESP_OK )
    {
        strncpy( my_thing_name, pucBuffer, strlen( pucBuffer ) );
        strncpy( clientcredentialIOT_THING_NAME, pucBuffer, strlen( pucBuffer ) );
        free( pucBuffer );
    }

    nvs_close( xNvsHandle );
    return xRet;
}

bool isMQTTConnecting()
{
    if( isMQTTAttempted )
    {
        ESP_LOGI( TAG, "isMQTTConnecting: still connecting" );
        return true;
    }
    else
    {
        ESP_LOGI( TAG, "isMQTTConnecting: not trying to connect anymore" );
        return false;
    }
}

bool isMQTTConnected()
{
    if( myMQTTConnection != IOT_MQTT_CONNECTION_INITIALIZER )
    {
        ESP_LOGI( TAG, "isMQTTConnected: still connected" );
        return true;
    }
    else
    {
        ESP_LOGI( TAG, "isMQTTConnected: not connected" );
        return false;
    }
}

bool isShadowConnected()
{
    if( isShadowConnectedVar )
    {
        ESP_LOGI( TAG, "isShadowConnected: still connected" );
        return true;
    }
    else
    {
        ESP_LOGI( TAG, "isShadowConnected: not connected" );
        return false;
    }
}

bool isOTAConnected()
{
    if( isOTAConnectedVar )
    {
        ESP_LOGI( TAG, "isOTAConnected: still connected" );
        return true;
    }
    else
    {
        ESP_LOGI( TAG, "isOTAConnected: not connected" );
        return false;
    }
}

static int _establishMqttConnection( const char * pIdentifier,
                                     void * pNetworkServerInfo,
                                     void * pNetworkCredentialInfo,
                                     const IotNetworkInterface_t * pNetworkInterface,
                                     IotMqttConnection_t * pMqttConnection,
                                     uint16_t keepAlive )
{
    int status = EXIT_SUCCESS;
    IotMqttError_t connectStatus = IOT_MQTT_STATUS_PENDING;

    isMQTTAttempted = true;

    if( pIdentifier == NULL )
    {
        ESP_LOGE( TAG, "_establishMqttConnection: Thing Name must be provided." );
        status = EXIT_FAILURE;
    }

    if( status == EXIT_SUCCESS )
    {
        /* Set the members of the network info not set by the initializer. This
         * struct provided information on the transport layer to the MQTT connection. */
        networkInfo.createNetworkConnection = true;
        networkInfo.u.setup.pNetworkServerInfo = pNetworkServerInfo;
        networkInfo.u.setup.pNetworkCredentialInfo = pNetworkCredentialInfo;
        networkInfo.pNetworkInterface = pNetworkInterface;

        #if ( IOT_MQTT_ENABLE_SERIALIZER_OVERRIDES == 1 ) && defined( IOT_DEMO_MQTT_SERIALIZER )
            networkInfo.pMqttSerializer = IOT_DEMO_MQTT_SERIALIZER;
        #endif

        /* Set the members of the connection info not set by the initializer. */
        connectInfo.awsIotMqttMode = true;
        connectInfo.cleanSession = true;
        connectInfo.keepAliveSeconds = keepAlive;

        /* AWS IoT recommends the use of the Thing Name as the MQTT client ID. */
        connectInfo.pClientIdentifier = pIdentifier;
        connectInfo.clientIdentifierLength = ( uint16_t ) strlen( pIdentifier );

        ESP_LOGI( TAG, "_establishMqttConnection: Thing connection named %.*s (length %hu).",
                  connectInfo.clientIdentifierLength,
                  connectInfo.pClientIdentifier,
                  connectInfo.clientIdentifierLength );

        /* Establish the MQTT connection. */
        uint16_t retryCount = 0;

        do
        {
            ESP_LOGI( TAG, "_establishMqttConnection: Connecting IotMqtt_Connect on retry(%d)", retryCount );
            /* directTasksStatus(); */

            connectStatus = IotMqtt_Connect( &networkInfo,
                                             &connectInfo,
                                             AWS_TIMEOUT_MS,
                                             pMqttConnection );

            if( connectStatus != IOT_MQTT_SUCCESS )
            {
                ESP_LOGE( TAG, "_establishMqttConnection: MQTT CONNECT returned error %s.",
                          IotMqtt_strerror( connectStatus ) );
                // IotClock_SleepMs( 1000 );
                vTaskDelay( pdMS_TO_TICKS( 1000 ) );
                status = EXIT_FAILURE;
            }
            else
            {
                status = IOT_MQTT_SUCCESS;
            }
        } while ( connectStatus != IOT_MQTT_SUCCESS && ++retryCount < MAX_RETRY_CONNECTION );
    }

    isMQTTAttempted = false; /* we have exited trying so we don't have to track attempts anymore. */

    ESP_LOGI( TAG, "_establishMqttConnection: MQTT CONNECT returned(%d)", connectStatus );
    return status;
}

/* Stop OTA agent and task */
void disconnectOTA()
{
    ESP_LOGI( TAG, "disconnectOTA:  starting" );
    ota_update_state( false );
    OTA_AgentShutdown( 1000 );
    ESP_LOGI( TAG, "disconnectOTA: ran OTA_AgentShutdown" );

    if( OTAHandle != NULL )
    {
        ESP_LOGI( TAG, "disconnectOTA: deleted OTAHandle" );
        vTaskDelete( OTAHandle );
        OTAHandle = NULL;
    }
}

/* Manage to close all shadow connections cleanly */
static void shadow_disconnectCallback( void * pCallbackContext,
                                       IotMqttCallbackParam_t * pCallbackParam )
{
    ESP_LOGI( TAG, "shadow_disconnectCallback: isShadowConnectedVar = false" );
    isShadowConnectedVar = false;

    /**
     * Deinit shadow on disconnect from mqtt, so we could make clean start when wifi connection is restored
     */
    ESP_LOGI( TAG, "shadow_disconnectCallback: about to run AwsIotShadow_Cleanup()" );
    AwsIotShadow_Cleanup();

    /**
     * Deinit OTA task on disconnect from mqtt, so we could make clean start when wifi connection is restored
     */
    ESP_LOGI( TAG, "ota_job_disconnectCallback: is myMQTTConnection == IOT_MQTT_CONNECTION_INITIALIZER? (%d)", ( bool ) ( myMQTTConnection == IOT_MQTT_CONNECTION_INITIALIZER ) );
    isOTAConnectedVar = false;
    disconnectOTA();
}

/**
 * Disconnect mqtt connection, usually before motor running
 * IOT_MQTT_FLAG_CLEANUP_ONLY == 1
 */
void disconnectMQTT()
{
    /* AwsIotShadow_Cleanup(); // cleanup shadow is called in shadow_disconnectCallback ??? */
    ESP_LOGI( TAG, "disconnectMQTT: starting" );

    if( isShadowConnected() )
    {
        isOTAConnectedVar = false;
        isShadowConnectedVar = false;
        connect_retry = MAX_MQTT_RETRY_COUNT;
        disconnectOTA();
        AwsIotShadow_Cleanup();
        IotMqtt_Disconnect( myMQTTConnection, IOT_MQTT_FLAG_CLEANUP_ONLY );
        IotMqtt_Cleanup();
        myMQTTConnection = IOT_MQTT_CONNECTION_INITIALIZER;
        ESP_LOGI( TAG, "disconnectMQTT: done" );
        /* TODO to add vTaskDelete for shadow task, maybe in shadow_disconnectCallback ??? */
    }
}

/**
 * Re connecting mqtt, shadow and OTA after motor stop running
 */
void reconnectMQTT()
{
    int timeoutInit = 30;

    ESP_LOGI( TAG, "reconnectMQTT: running" );

    /* while (my_valve_pending && timeoutInit-- > 0) */
    /* { */
    /*     ESP_LOGI(TAG, "reconnectMQTT: waiting 15s(%d) for !my_valve_pending"); */
    /*     vTaskDelay(pdMS_TO_TICKS(15000)); */
    /* } */
    while( !mqttInitialized && ( timeoutInit-- > 0 ) )
    {
        ESP_LOGI( TAG, "reconnectMQTT: waiting 1s(%d) for mqttInitialized", timeoutInit );
        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }

    if( !connectMQTTAttempt && mqttInitialized )
    {
        ESP_LOGI( TAG, "reconnectMQTT: running _connectMqttShadow" );
        _connectMqttShadow();
    }
    else
    {
        ESP_LOGW( TAG, "reconnectMQTT: already trying _connectMqttShadow or !mqttInitialized, skipping" );
    }
}

bool needsTimeUpdate()
{
    ESP_LOGI( TAG, "needsTimeUpdate: checking time_updated(%d)", current_wifi_state.time_updated );

    if( current_wifi_state.time_updated )
    {
        return false;
    }

    return true;
}

static bool didInitOTACheck = false;

bool checkOTATime()
{
    struct timeval now;
    static uint64_t check_time;

    /* have we advanced beyond the last checkin time? */
    gettimeofday( &now, NULL );

    if( now.tv_sec > tv.tv_sec )
    {
        check_time = now.tv_sec;
    }
    else
    {
        check_time = tv.tv_sec;
    }

    if( check_time > ( lastTimeOTAChecked + ( OTA_HR_CHECK * 60 * 60 ) ) ) /* every OTA_HR_CHECK hours */
    {
        ESP_LOGI( TAG, "checkOTATime:(true) as check_time(%lld) > lastTimeOTAChecked(%ld)+delta(%d)", check_time, lastTimeOTAChecked, ( OTA_HR_CHECK * 60 * 60 ) );
        return( true );
    }
    else
    {
        ESP_LOGI( TAG, "checkOTATime:(false) as check_time(%lld) > lastTimeOTAChecked(%ld)+delta(%d)", check_time, lastTimeOTAChecked, ( OTA_HR_CHECK * 60 * 60 ) );
        return( false );
    }
}

bool needsOTACheck()
{
    /* is this our first time being called? -- */
    /* let's not return true as this would be taken care by the other check in the device state */
    if( !didInitOTACheck )
    {
        didInitOTACheck = true;
        return false;
    }

    ESP_LOGI( TAG, "needsOTACheck: OTA status isOTAConnected(%d), check_for_ota(%d), ota_job_in_progress(%d)", isOTAConnected(), check_for_ota, ota_job_in_progress );

    if( ota_job_in_progress || connectMQTTAttempt )
    {
        return false;
    }

    check_for_ota = checkOTATime();
    return check_for_ota;
}

static void _connectMqttShadow()
{
    int status = EXIT_SUCCESS;
    char shadowThingName[ 20 ];

    if( !isWiFiConnected() )
    {
        ESP_LOGI( TAG, "_connectMqttShadow: wait for wifi connection by calling DEMO_RUNNER_RunDemos1" );
        DEMO_RUNNER_RunDemos1(); /* reconnect wifi? */
        return;                  /* the wifi_connected callback will do the right thing */
    }

    connectMQTTAttempt = true;

    while( !isShadowConnected() && connect_retry-- > 0 )
    {
        if( myMQTTConnection != IOT_MQTT_CONNECTION_INITIALIZER )
        {
            ESP_LOGI( TAG, "_connectMqttShadow: running IotMqtt_Disconnect" );
            IotMqtt_Disconnect( myMQTTConnection, 0 );
        }

        networkInfo.disconnectCallback.function = shadow_disconnectCallback;
        myMQTTConnection = IOT_MQTT_CONNECTION_INITIALIZER;

        sprintf( shadowThingName, "%s", my_thing_name );
        ESP_LOGI( TAG, "_connectMqttShadow: running _establishMqttConnection" );
        status = _establishMqttConnection( shadowThingName,
                                           pConnectionParams,
                                           pCredentials,
                                           pNetworkInterface,
                                           &myMQTTConnection,
                                           KEEP_ALIVE_SECONDS );
        ESP_LOGI( TAG, "_connectMqttShadow: after _establishMqttConnection status(%d) try(%d)", status, connect_retry );

        if( status == EXIT_SUCCESS )
        {
            my_mqtt_error_count = 0;
            ESP_LOGI( TAG, "_connectMqttShadow: running setupShadow" );
            status = setupShadow( myMQTTConnection );
        }
        else
        {
            ESP_LOGI( TAG, "_connectMqttShadow: _establishMqttConnection failed" );
            isShadowConnectedVar = false;
            myMQTTConnection = IOT_MQTT_CONNECTION_INITIALIZER;
            my_mqtt_error_count++;
        }

        if( status == EXIT_SUCCESS )
        {
            ESP_LOGI( TAG, "_connectMqttShadow: isShadowConnectedVar = true" );
            isShadowConnectedVar = true;

            if( needsTimeUpdate() )
            {
                xTaskNotify( xDeviceState, TIME_UPDATE, eSetBits );
            }
        }
        else
        {
            ESP_LOGI( TAG, "_connectMqttShadow: isShadowConnectedVar = false" );
            isShadowConnectedVar = false;
        }
    }

    connectMQTTAttempt = false;
}

/**
 * wifi connected callback
 */
void wifi_connectedCallback( bool awsIotMqttMode,
                             const char * pIdentifier,
                             void * pNetworkServerInfo,
                             void * pNetworkCredentialInfo,
                             const IotNetworkInterface_t * pNetworkInterface )
{
    int timeoutInit = 6;

    ESP_LOGW( TAG, "wifi_connectedCallback: Connected callback" );

    if( !deinitBLE )
    {
        xBLEStackTeardown(); /* when we are connected to wifi first time we can deinit BLE and release memory */
        deinitBLE = true;
    }

    setWiFiStatus( true );
    ESP_LOGI( TAG, "wifi_connectedCallback: calling reconnectMQTT()" );

/*
    while( my_valve_pending && timeoutInit-- > 0 )
    {
        ESP_LOGI( TAG, "reconnectMQTT: waiting 15s(%d) for !my_valve_pending", timeoutInit );
        vTaskDelay( pdMS_TO_TICKS( 15000 ) );
    }
*/
    reconnectMQTT();
    current_wifi_state.disconnected_time = xTaskGetTickCount() - current_wifi_state.disconnected_time;
}

/**
 * wifi disconnected callback
 */
void wifi_disconnectedCallback( const IotNetworkInterface_t * pNetworkInteface )
{
    ESP_LOGE( TAG, "wifi_disconnectedCallback: Disconnected callback" );
    /* tasksStatus(); */
    setWiFiStatus( false );
    /* let's try to disconnect mqtt now that WiFi is also gone */
    disconnectMQTT();
    wifi_cleanup_in_progress = true;
    DEMO_RUNNER_StopDemos1(); /* how to reconnect wifi? */
    wifi_cleanup_in_progress = false;

    current_wifi_state.disconnected_time = xTaskGetTickCount();
    /* let's try to start again after some time? */
    // IotClock_SleepMs( 10000 );
    vTaskDelay( pdMS_TO_TICKS( 10000 ) );

    /* either try to restart network stack or go to sleep and wake up again? */
    ESP_LOGW( TAG, "wifi_disconnectedCallback: About to run DEMO_RUNNER_RunDemos1 again" );
    DEMO_RUNNER_RunDemos1();
    /* SAFE_RESTART */
    /* xTaskNotify(xDeviceState, SAFE_RESTART, eSetBits); */
}

/********************************************************/

/**
 * helper function to control tasks usStackHighWaterMark
 */
void directTasksStatus()
{
    UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();
    TaskStatus_t * pxTaskStatusArray;
    uint32_t pulTotalRunTime = 0;
    uint32_t free_heap_size = 0, min_free_heap_size = 0;

    free_heap_size = esp_get_free_heap_size();
    min_free_heap_size = esp_get_minimum_free_heap_size();
    ESP_LOGI( TAG, "directTasksStatus: free heap size = %d \t  min_free_heap_size = %d \n", free_heap_size, min_free_heap_size );

    pxTaskStatusArray = ( TaskStatus_t * ) malloc( sizeof( TaskStatus_t ) * uxArraySize );
    uxTaskGetSystemState( &pxTaskStatusArray[ 0 ], uxArraySize, &pulTotalRunTime );

    for( int i = 0; i < uxArraySize; i++ )
    {
        ESP_LOGI( TAG, "[%2d] %*s => stack remaining: %*d", i, 20, pxTaskStatusArray[ i ].pcTaskName, 4, pxTaskStatusArray[ i ].usStackHighWaterMark );
    }

    heap_caps_print_heap_info( MALLOC_CAP_DEFAULT );

    if( !heap_caps_check_integrity_all( true ) )
    {
        heap_caps_dump_all();
        /* heap_caps_print_heap_info(MALLOC_CAP_DEFAULT); */
    }

    ESP_LOGI( TAG, "directTasksStatus: stack remaining: %d", uxTaskGetStackHighWaterMark( NULL ) );
    free( pxTaskStatusArray );
}

void set_timestamp( long timestamp )
{
    struct timeval now;

    ESP_LOGI( TAG, "set_timestamp: setting timestamp(%ld)", timestamp );
    tv.tv_sec = timestamp;
    int ret = settimeofday( &tv, NULL );

    ESP_LOGI( TAG, "set_timestamp: settimeofday returned(%d)", ret );
    /* let's let the time increment a little bit */
    vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    gettimeofday( &now, NULL );

    long checktime = now.tv_sec;

    ESP_LOGI( TAG, "set_timestamp: time now (%ld)", checktime );
    current_wifi_state.time_updated = true;
}

void updateLastTimeOTAChecked( long time )
{
    if( time > 0 )
    {
        ESP_LOGI( TAG, "updateLastTimeOTAChecked: assigning time(%ld) to lastTimeOTAChecked", time );
        lastTimeOTAChecked = time;
    }
    else
    {
        ESP_LOGW( TAG, "updateLastTimeOTAChecked: zero timestamp found -- assigning tv.tv_sec(%ld) to lastTimeOTAChecked", tv.tv_sec );
        lastTimeOTAChecked = tv.tv_sec;
    }
}

void getOTAUpdate()
{
    int status = EXIT_SUCCESS;
    struct timeval xtv;
    struct timezone xtz;

    /* we need to connect shadow first, because we are reusing shadow connection */
    ESP_LOGI( TAG, "getOTAUpdate: starting w/isShadowConnected(%d)", isShadowConnected() );
    check_for_ota = checkOTATime();

    if( check_for_ota && isShadowConnected() && !ota_job_in_progress )
    {
        ESP_LOGI( TAG, "getOTAUpdate: running vStartOTAUpdate" );
        status = vStartOTAUpdate( networkInfo, myMQTTConnection );

        if( status == EXIT_SUCCESS )
        {
            gettimeofday( &xtv, &xtz );
            updateLastTimeOTAChecked( xtv.tv_sec );
            ESP_LOGI( TAG, "getOTAUpdate: isOTAConnectedVar = true, updateLastTimeOTAChecked(%ld)", xtv.tv_sec );
            isOTAConnectedVar = true;
        }
        else
        {
            ESP_LOGI( TAG, "getOTAUpdate: isOTAConnectedVar = false" );
            isOTAConnectedVar = false;
        }

        check_for_ota = false;
    }
    else
    {
        ESP_LOGW( TAG, "getOTAUpdate: check_for_ota(%d), isShadowConnected(%d) ota_job_in_progress(%d)", check_for_ota, isShadowConnected(), ota_job_in_progress );
    }
}
