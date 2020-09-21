/*
 * Amazon FreeRTOS V201906.00 Major
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/**
 * @file aws_ota_update_demo.c
 * @brief A simple OTA update example.
 *
 * This example initializes the OTA agent to enable OTA updates via the
 * MQTT broker. It simply connects to the MQTT broker with the users
 * credentials and spins in an indefinite loop to allow MQTT messages to be
 * forwarded to the OTA agent for possible processing. The OTA agent does all
 * of the real work; checking to see if the message topic is one destined for
 * the OTA agent. If not, it is simply ignored.
 */
/* The config header is always included first. */
#include "iot_config.h"

/* MQTT include. */
#include "iot_mqtt.h"
/* Standard includes. */
#include <stdio.h>
#include <string.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "esp_log.h"

/* Demo network handling */
#include "aws_iot_demo_network.h"

/* Required to get the broker address and port. */
#include "aws_clientcredential.h"

/* Amazon FreeRTOS OTA agent includes. */
#include "aws_iot_ota_agent.h"

#include "iot_network_manager_private.h"

/* Required for demo task stack and priority */
#include "aws_demo_config.h"

#include "aws_application_version.h"

#include "ls_debug.h"
#include "ls_typedefs.h"
#include "ls_board_setup.h"
#include "ls_gpio.h"
#include "ls_shadow.h"
#include "ls_device_state.h"
#include "ls_globals.h"

#define TAG    "OTA"

xTaskHandle OTAHandle; /* ota task handle used in main.c to delete task on wifi disconnect */

static void App_OTACompleteCallback( OTA_JobEvent_t eEvent );

static BaseType_t prxCreateNetworkConnection( void );
static IotNetworkError_t prvNetworkDisconnectCallback( void * pvContext );
static void prvNetworkStateChangeCallback( uint32_t ulNetworkType,
                                           AwsIotNetworkState_t xNetworkState,
                                           void * pvContext );
/*-----------------------------------------------------------*/

#define otaDemoCONN_TIMEOUT_MS           ( 2000UL )

#define otaDemoCONN_RETRY_INTERVAL_MS    ( 5000 )

#define otaDemoCONN_RETRY_LIMIT          ( 5 )

#define otaDemoKEEPALIVE_SECONDS         ( 1200 )

#define ls_ONE_SECOND_DELAY_IN_TICKS     pdMS_TO_TICKS( 1000UL )
#define ls_FIVE_SECOND_DELAY_IN_TICKS    pdMS_TO_TICKS( 5000UL )

#define otaNETWORK_TYPES                 ( AWSIOT_NETWORK_TYPE_WIFI )

/**
 * @brief Structure which holds the context for an MQTT connection within Demo.
 */
static MqttConnectionContext_t xConnection =
{
    .pvNetworkConnection = NULL,
    .ulNetworkType       = AWSIOT_NETWORK_TYPE_NONE,
    .xNetworkInfo        = IOT_MQTT_NETWORK_INFO_INITIALIZER,
    .xMqttConnection     = IOT_MQTT_CONNECTION_INITIALIZER,
    .xDisconnectCallback = prvNetworkDisconnectCallback
};

/**
 * @brief Network manager subscription callback.
 */

static IotNetworkManagerSubscription_t xSubscriptionHandle = IOT_NETWORK_MANAGER_SUBSCRIPTION_INITIALIZER;

/**
 * @brief Flag used to unset, during disconnection of currently connected network. This will
 * trigger a reconnection from the main MQTT task.
 */
static BaseType_t xNetworkConnected = pdFALSE;

static void prvNetworkStateChangeCallback( uint32_t ulNetworkType,
                                           AwsIotNetworkState_t xNetworkState,
                                           void * pvContext )
{
    ( void ) pvContext;

    if( xNetworkState == eNetworkStateDisabled )
    {
        /* If the publish task is already connected to this network, set connected network flag to none,
         * to trigger a reconnect.
         */
        if( ( xNetworkConnected == pdTRUE ) && ( xConnection.ulNetworkType == ulNetworkType ) )
        {
            xNetworkConnected = pdFALSE;
        }
    }
}

static IotNetworkError_t prvNetworkDisconnectCallback( void * pvContext )
{
    ( void ) pvContext;
    xNetworkConnected = pdFALSE;

    return IOT_NETWORK_SUCCESS;
}

static BaseType_t prxCreateNetworkConnection( void )
{
    BaseType_t xRet = pdFALSE;
    uint32_t ulRetriesLeft = otaDemoCONN_RETRY_LIMIT;
    uint32_t ulRetryIntervalMS = otaDemoCONN_RETRY_INTERVAL_MS;

    do
    {
        /* No networks are available, block for a physical network connection. */
        if( ( AwsIotNetworkManager_GetConnectedNetworks() & AWSIOT_NETWORK_TYPE_WIFI ) == AWSIOT_NETWORK_TYPE_NONE )
        {
            ESP_LOGI( TAG, "prxCreateNetworkConnection: Waiting for a WiFi connection(%d)", ulRetriesLeft );
        }

        if( isMQTTConnected() )
        {
            ESP_LOGI( TAG, "prxCreateNetworkConnection: Got a MQTT connection" );
            xRet = pdTRUE;
            break;
        }
        else
        {
            /* Connection failed. Retry for a configured number of retries. */
            if( ulRetriesLeft > 0 )
            {
                ESP_LOGI( TAG, "prxCreateNetworkConnection: Network Connection failed, retry delay %u ms, retries left %u", ulRetryIntervalMS, ulRetriesLeft );
                vTaskDelay( pdMS_TO_TICKS( ulRetryIntervalMS ) );
            }
        }
    } while ( ulRetriesLeft-- > 0 );

    ESP_LOGI( TAG, "prxCreateNetworkConnection: returning %d", xRet );
    return xRet;
}

static const char * pcStateStr[ eOTA_AgentState_All ] =
{
    "Init",
    "Ready",
    "RequestingJob",
    "WaitingForJob",
    "CreatingFile",
    "RequestingFileBlock",
    "WaitingForFileBlock",
    "ClosingFile",
    "ShuttingDown",
    "Stopped"
};

void otaCheckAgentStatus()
{
    OTA_State_t eState;

    eState = OTA_GetAgentState();
    ESP_LOGI( TAG, "otaCheckAgentStatus:  OTA_GetAgentState(%s)", pcStateStr[ eState ] );
}


void vRunOTAUpdateTask( void * param )
{
    static bool inOTANow = false;
    OTA_State_t eState;
    OTA_ConnectionContext_t xOTAConnectionCtx = { 0 };
    const IotNetworkInterface_t * pNetworkInterface = AwsIotNetworkManager_GetNetworkInterface( AWSIOT_NETWORK_TYPE_WIFI );
    void * pCredentials = AwsIotNetworkManager_GetCredentials( AWSIOT_NETWORK_TYPE_WIFI );
    uint16_t packets;

    ESP_LOGI( TAG, "vRunOTAUpdateTask: OTA version %u.%u.%u",
              xAppFirmwareVersion.u.x.ucMajor,
              xAppFirmwareVersion.u.x.ucMinor,
              xAppFirmwareVersion.u.x.usBuild );

    for( ; ; )
    {
        xNetworkConnected = prxCreateNetworkConnection();

        if( xNetworkConnected )
        {
            xOTAConnectionCtx.pvControlClient = xConnection.xMqttConnection;
            xOTAConnectionCtx.pxNetworkInterface = ( void * ) pNetworkInterface;
            xOTAConnectionCtx.pvNetworkCredentials = pCredentials;

            if( ( eState = OTA_GetAgentState() ) == eOTA_AgentState_Stopped )
            {
                ESP_LOGI( TAG, "vRunOTAUpdateTask: Running OTA_AgentInit" );
                eState = OTA_AgentInit( ( void * ) ( &xOTAConnectionCtx ), ( const uint8_t * ) ( my_thing_name ), App_OTACompleteCallback, ( TickType_t ) ~0 );
                vTaskDelay( ls_ONE_SECOND_DELAY_IN_TICKS );
            }
            else
            {
                ESP_LOGI( TAG, "vRunOTAUpdateTask:  OTA_GetAgentState(%s)", pcStateStr[ eState ] );
                vTaskDelay( ls_ONE_SECOND_DELAY_IN_TICKS );
            }

            ota_update_state( false );

            while( ( eState = OTA_GetAgentState() ) != eOTA_AgentState_Stopped )
            {
                packets = ( uint16_t ) OTA_GetPacketsReceived();
                ESP_LOGI( TAG, "vRunOTAUpdateTask:  OTA_GetAgentState(%s) packets(%d)", pcStateStr[ eState ], packets );

                /* Wait forever for OTA traffic but allow other tasks to run and output statistics only once per second. */
                if( ( ( ( packets + 1 ) % 4 ) == 0 ) && !inOTANow )
                {
                    ESP_LOGI( TAG, "vRunOTAUpdateTask: OTA_GetAgentState == active OTA w/packet(%d)", packets );
                    inOTANow = true;
                    ota_update_state( true );
                }

                if( inOTANow )
                {
                    ESP_LOGI( TAG, "vRunOTAUpdateTask: State: %s  Received: %u   Queued: %u   Processed: %u   Dropped: %u", pcStateStr[ eState ],
                              packets, OTA_GetPacketsQueued(), OTA_GetPacketsProcessed(), OTA_GetPacketsDropped() );
                }

                vTaskDelay( ls_FIVE_SECOND_DELAY_IN_TICKS );
            }

            /* finished while() means eState == eOTA_AgentState_Stopped */
            ESP_LOGI( TAG, "vRunOTAUpdateTask: while escape: OTA_GetAgentState(%s)", pcStateStr[ eState ] );
            ota_update_state( false );
            /* After failure to connect or a disconnect, wait an arbitrary one second before retry. */
            vTaskDelay( ls_FIVE_SECOND_DELAY_IN_TICKS );
        }
        else
        {
            ESP_LOGI( TAG, "vRunOTAUpdateTask: Failed to establish WiFi network via prxCreateNetworkConnection()." );
            ota_update_state( false );
        }
    }

    vTaskDelete( NULL ); /* delete task I guess */
}

/* OTA states */
/* eOTA_AgentState_Init = 0, */
/* eOTA_AgentState_Ready, */
/* eOTA_AgentState_RequestingJob, */
/* eOTA_AgentState_WaitingForJob, */
/* eOTA_AgentState_CreatingFile, */
/* eOTA_AgentState_RequestingFileBlock, */
/* eOTA_AgentState_WaitingForFileBlock, */
/* eOTA_AgentState_ClosingFile, */
/* eOTA_AgentState_ShuttingDown, */
/* eOTA_AgentState_Stopped, */
/* eOTA_AgentState_All */

/* The OTA agent has completed the update job or determined that we're in
 * self test mode. If it was accepted, we want to activate the new image.
 * This typically means we should reset the device to run the new firmware.
 * If now is not a good time to reset the device, it may be activated later
 * by your user code. If the update was rejected, just return without doing
 * anything and we'll wait for another job. If it reported that we should
 * start test mode, normally we would perform some kind of system checks to
 * make sure our new firmware does the basic things we think it should do
 * but we'll just go ahead and set the image as accepted for demo purposes.
 * The accept function varies depending on your platform. Refer to the OTA
 * PAL implementation for your platform in aws_ota_pal.c to see what it
 * does for you.
 */

static void App_OTACompleteCallback( OTA_JobEvent_t eEvent )
{
    OTA_Err_t xErr = kOTA_Err_Uninitialized;
    OTA_ImageState_t newImageState;

    /* OTA job is completed. so delete the MQTT and network connection. */
    if( eEvent == eOTA_JobEvent_Activate )
    {
        /* TODO make sure valve is closed before */
        ESP_LOGI( TAG, "App_OTACompleteCallback: Received eOTA_JobEvent_Activate callback from OTA Agent." );
        newImageState = OTA_GetImageState();
        ESP_LOGW( TAG, "New image state => (%d), 0=unk,1=test,2=accept,3=reject,4=abort", newImageState );
        xErr = OTA_SetImageState( eOTA_ImageState_Accepted );
        newImageState = OTA_GetImageState();
        ESP_LOGW( TAG, "Now image state => (%d)", newImageState );
        OTA_ActivateNewImage();
        /* this does not really matter as we will have rebooted by now :) */
        IotMqtt_Disconnect( xConnection.xMqttConnection, 0 );
    }
    else if( eEvent == eOTA_JobEvent_Fail )
    {
        ESP_LOGI( TAG, "App_OTACompleteCallback: Received eOTA_JobEvent_Fail callback from OTA Agent." );
        /* Nothing special to do. The OTA agent handles it. */
        OTA_AgentShutdown( 1000 );
        ota_update_state( false );
    }
    else if( eEvent == eOTA_JobEvent_StartTest )
    {
        /* This demo just accepts the image since it was a good OTA update and networking
         * and services are all working (or we wouldn't have made it this far). If this
         * were some custom device that wants to test other things before calling it OK,
         * this would be the place to kick off those tests before calling OTA_SetImageState()
         * with the final result of either accepted or rejected. */
        ESP_LOGI( TAG, "App_OTACompleteCallback: Received eOTA_JobEvent_StartTest callback from OTA Agent." );
        xErr = OTA_SetImageState( eOTA_ImageState_Accepted );

        if( xErr != kOTA_Err_None )
        {
            ESP_LOGE( TAG, "App_OTACompleteCallback: Error! Failed to set image state as accepted." );
        }
    }
}

/*-----------------------------------------------------------*/

int vStartOTAUpdate( IotMqttNetworkInfo_t networkInfo,
                     IotMqttConnection_t mqttConnection )
{
    int xRet = EXIT_SUCCESS;
    BaseType_t ret;

    ESP_LOGI( TAG, "vStartOTAUpdate: starting" );
    xConnection.xNetworkInfo = networkInfo;
    xConnection.xMqttConnection = mqttConnection;

    /**
     * Create a Network Manager Subscription for all the network types supported.
     */
    if( xSubscriptionHandle == IOT_NETWORK_MANAGER_SUBSCRIPTION_INITIALIZER )
    {
        if( AwsIotNetworkManager_SubscribeForStateChange( otaNETWORK_TYPES,
                                                          prvNetworkStateChangeCallback,
                                                          NULL,
                                                          &xSubscriptionHandle ) != pdTRUE )
        {
            ESP_LOGE( TAG, "vStartOTAUpdate: Failed to create Network Manager subscription." );
            xRet = EXIT_FAILURE;
        }
    }

    if( xRet == EXIT_SUCCESS )
    {
        if( OTAHandle != NULL )
        {
            vTaskDelete( OTAHandle ); /* delete task I guess */
            OTAHandle = NULL;
        }

        ret = xTaskCreate( vRunOTAUpdateTask, "otaTask", 4 * 1024, NULL, PRIORITY_OTA, &OTAHandle );

        if( ( OTAHandle == NULL ) || ( ret == pdFALSE ) )
        {
            xRet = EXIT_FAILURE;
        }
    }

    if( xRet == EXIT_FAILURE )
    {
        if( xSubscriptionHandle != IOT_NETWORK_MANAGER_SUBSCRIPTION_INITIALIZER )
        {
            AwsIotNetworkManager_RemoveSubscription( xSubscriptionHandle );
        }
    }

    return xRet;
}
