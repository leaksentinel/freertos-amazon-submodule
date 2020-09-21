/*
 * Copyright (C) 2020 LeakSentinel, Inc. or its affiliates.  All Rights Reserved.
 *
 */

#include "FreeRTOSConfig.h"
#include "iot_demo_logging.h"
#include "iot_ble_config.h"
#include "iot_ble.h"
#include "task.h"
#include "semphr.h"
#include "platform/iot_network.h"
#include "esp_log.h"
#include <string.h>

#define TAG    "CUSTOM_BLE"

extern char my_thing_name[ 64 ];
char BLE_NAME[ 25 ];

#define gattDemoSVC_UUID                                                                               \
    {                                                                                                  \
        0x00, 0xFF, 0x69, 0xD6, 0xC6, 0xBF, 0x14, 0x90, 0x25, 0x41, 0xE7, 0x49, 0xE3, 0xD9, 0xF2, 0xC6 \
    }
#define gattDemoCHAR_UUID_MASK    0x69, 0xD6, 0xC6, 0xBF, 0x14, 0x90, 0x25, 0x41, 0xE7, 0x49, 0xE3, 0xD9, 0xF2, 0xC6
#define gattDemoCHAR_COUNTER_UUID          \
    {                                      \
        0x01, 0xFF, gattDemoCHAR_UUID_MASK \
    }

#define xServiceUUID_TYPE             \
    {                                 \
        .uu.uu128 = gattDemoSVC_UUID, \
        .ucType = eBTuuidType128      \
    }
#define xCharCounterUUID_TYPE                  \
    {                                          \
        .uu.uu128 = gattDemoCHAR_COUNTER_UUID, \
        .ucType = eBTuuidType128               \
    }

static uint16_t usHandlesBuffer[ 2 ];

static void vReadName( IotBleAttributeEvent_t * pEventParam )
{
    IotBleReadEventParams_t * pxReadParam;
    IotBleAttributeData_t xAttrData = { 0 };
    IotBleEventResponse_t xResp;

    xResp.pAttrData = &xAttrData;
    xResp.rspErrorStatus = eBTRspErrorNone;
    xResp.eventStatus = eBTStatusFail;
    xResp.attrDataOffset = 0;

    if( pEventParam->xEventType == eBLERead )
    {
        pxReadParam = pEventParam->pParamRead;
        xResp.pAttrData->handle = pxReadParam->attrHandle;
        xResp.pAttrData->pData = ( uint8_t * ) my_thing_name;
        xResp.pAttrData->size = strlen( my_thing_name );
        xResp.attrDataOffset = 0;
        xResp.eventStatus = eBTStatusSuccess;
        IotBle_SendResponse( &xResp, pxReadParam->connId, pxReadParam->transId );
    }
}

static const BTAttribute_t pxAttributeTable[] =
{
    { .xServiceUUID   = xServiceUUID_TYPE },
    { .xAttributeType = eBTDbCharacteristic,
        .xCharacteristic =
        {
            .xUuid        = xCharCounterUUID_TYPE,
            .xPermissions = ( IOT_BLE_CHAR_READ_PERM ),
            .xProperties  = ( eBTPropRead )
        } }
};

static const BTService_t xGattDemoService =
{
    .xNumberOfAttributes = 2,
    .ucInstId            = 0,
    .xType               = eBTServiceTypePrimary,
    .pusHandlesBuffer    = usHandlesBuffer,
    .pxBLEAttributes     = ( BTAttribute_t * ) pxAttributeTable
};

static const IotBleAttributeEventCallback_t pxCallBackArray[ 2 ] =
{
    NULL,
    vReadName
};

void IotBle_AddCustomServicesCb( void )
{
    ESP_LOGI( TAG, "IotBle_AddCustomServicesCb" );
    BaseType_t xRet = pdFALSE;
    BTStatus_t xStatus;
    IotBleEventsCallbacks_t xCallback;

    /* Select the handle buffer. */
    xStatus = IotBle_CreateService( ( BTService_t * ) &xGattDemoService, ( IotBleAttributeEventCallback_t * ) pxCallBackArray );

    if( xStatus == eBTStatusSuccess )
    {
        xRet = pdTRUE;
    }

    if( xRet == pdTRUE )
    {
        xCallback.pConnectionCb = NULL;

        if( IotBle_RegisterEventCb( eBLEConnection, xCallback ) != eBTStatusSuccess )
        {
            xRet = pdFAIL;
        }
    }
}

static const BTUuid_t _advUUID =
{
    .uu.uu128 = IOT_BLE_ADVERTISING_UUID,
    .ucType   = eBTuuidType128
};

static IotBleAdvertisementParams_t _advParams =
{
    .includeTxPower    = false,
    .setScanRsp        = false,
    .appearance        = IOT_BLE_ADVERTISING_APPEARANCE,
    .minInterval       = 0,
    .maxInterval       = 0,
    .serviceDataLen    = 0,
    .pServiceData      = NULL,
    .manufacturerLen   = 0,
    .pManufacturerData = NULL,
    .pUUID1            = ( BTUuid_t * ) &_advUUID,
    .pUUID2            = NULL
};

static IotBleAdvertisementParams_t _scanRespParams =
{
    .includeTxPower    = false,
    .name              = { BTGattAdvNameComplete,25 },
    .setScanRsp        = true,
    .appearance        = NULL,
    .minInterval       = 0,
    .maxInterval       = 0,
    .serviceDataLen    = 0,
    .pServiceData      = NULL,
    .manufacturerLen   = 0,
    .pManufacturerData = NULL,
    .pUUID1            = NULL,
    .pUUID2            = NULL
};

void IotBle_SetCustomAdvCb( IotBleAdvertisementParams_t * pAdvParams,
                            IotBleAdvertisementParams_t * pScanParams )
{
    ESP_LOGI( TAG, "IotBle_SetCustomAdvCb custom advertising callback" );
    strcpy( BLE_NAME, "LeakSentinel device" );
    BTProperty_t property =
    {
        .xType = eBTpropertyBdname,
        .xLen  = strlen( BLE_NAME ) - 1,
        .pvVal = ( void * ) BLE_NAME
    };
    const BTInterface_t * interface = BTGetBluetoothInterface();
    interface->pxSetDeviceProperty( &property );
    *pScanParams = _scanRespParams;
    *pAdvParams = _advParams;
}
