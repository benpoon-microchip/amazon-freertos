/*******************************************************************************
*  Network Interface file
*
*  Summary:
*   Network Interface file for FreeRTOS-Plus-TCP stack
*
*  Description:
*   - Interfaces PIC32 to the FreeRTOS TCP/IP stack
*******************************************************************************/

/*******************************************************************************
*  File Name:  pic32_NetworkInterface.c
*  Copyright 2017 Microchip Technology Incorporated and its subsidiaries.
*
*  FreeRTOS Wi-Fi for Curiosity PIC32MZEF V1.0.4
*  Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy of
*  this software and associated documentation files (the "Software"), to deal in
*  the Software without restriction, including without limitation the rights to
*  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
*  of the Software, and to permit persons to whom the Software is furnished to do
*  so, subject to the following conditions:
*  The above copyright notice and this permission notice shall be included in all
*  copies or substantial portions of the Software.
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*  SOFTWARE
*******************************************************************************/
#ifdef PIC32_USE_RIO2_WIFI
#include <sys/kmem.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "NetworkInterface.h"
#include "NetworkBufferManagement.h"

#include "iot_wifi.h"
#include "iot_secure_sockets.h"

//#include "peripheral/eth/plib_eth.h"

#include "definitions.h"
#include "system_config.h"
#include "system/console/sys_console.h"
#include "system/debug/sys_debug.h"
#include "system/command/sys_command.h"

//#include "driver/ethmac/drv_ethmac.h"
//#include "driver/miim/drv_miim.h"

#include "tcpip/tcpip.h"
#include "tcpip/src/tcpip_private.h"
#include "tcpip/src/link_list.h"
//#include "wilc1000_task.h"
//#include "wdrv_wilc1000_main.h"
//#include "wdrv_wilc1000_common.h"
//#include "wdrv_wilc1000_scan_helper.h"
//#include "driver/flash/drv_flash.h"

#include "wdrv_pic32mzw_client_api.h"

/* initialization done timeout. */
#define WIFI_MAC_INIT_TIMEOUT                  ( pdMS_TO_TICKS( 5000 ) )
#define WIFI_MAC_CONNECT_TIMEOUT               ( pdMS_TO_TICKS( 10000 ) )
#define WIFI_MAC_DISCONNECT_TIMEOUT_SECONDS    10

/*Ping block for a free buffer timeout (ms) */
#define WIFI_PING_WAIT_FOR_BUFF_TIMEOUT        ( 10000 / portTICK_PERIOD_MS )

/*Ping packet size (bytes) */
#define WIFI_PING_PKT_SIZE                     256
#define WIFI_MAX_PSK_LEN                       65

/*!< Maximum size for the WPA PSK including the NULL termination.
 */

#define WDRV_DBG_INFORM_MESSAGE( X )     vLoggingPrintf X
//#define WDRV_DBG_ERROR_MESSAGE printf
#define WDRV_DBG_ERROR_MESSAGE( X )          vLoggingPrintf X

static DRV_HANDLE wdrvHandle;
TaskHandle_t waiting_task;

/*ping reply queue*/
QueueHandle_t xPingReplyQueue;

bool g_is_wifi_connected = false;



typedef enum
{
    WDRV_MAC_EVENT_INIT_NONE = 0x000,       /* no event/invalid */

    WDRV_MAC_EVENT_INIT_DONE = 0x001,       /* initialization done event */
    WDRV_MAC_EVENT_TIMEOUT = 0x002,         /* periodic timeout event */
    WDRV_MAC_EVENT_IF_PENDING = 0x004,      /* an interface event signal: RX, TX, errors. etc. */
    WDRV_MAC_EVENT_CONNECT_DONE = 0x005,    /* Connection done event */
    WDRV_MAC_EVENT_CONNECT_TIMEOUT = 0x006, /* Connection timeout event */
} WDRV_MAC_EVENT_TYPE;

typedef enum
{
    WIFI_SCAN_STATE_INIT = 0,
    WIFI_SCAN_STATE_SCANNING,
    WIFI_SCAN_STATE_SCAN_GET_RESULTS,
    WIFI_SCAN_STATE_SCAN_DONE,
    WIFI_SCAN_STATE_ERROR
} WIFI_SCAN_STATE;

typedef enum 
{
    OPEN = 0,
    WPA2,
    WPAWPA2MIXED,
    WPA3,
    WPA2WPA3MIXED,
    WEP,
    NONE
} WIFI_AUTH_MODE;

typedef struct wifiConfiguration
{
    WDRV_PIC32MZW_AUTH_CONTEXT authCtx;
    WDRV_PIC32MZW_BSS_CONTEXT bssCtx;
} wifiConfig;

///static wifiConfig g_wifiConfig;
static WIFIDeviceMode_t g_wifiMode;
#if 0
bool isLinkUp();

WDRV_HOOKS WILC1000_hooks;
extern void WDRV_WILC1000_Default_Hook_Initialize( WDRV_HOOKS * ehooks );
extern uint32_t SOCKETS_GetHostByName( const char * pcHostName );
void xNetworkFrameReceived( uint32_t len,
                            uint8_t const * const frame );


/*Flash driver handle*/
#define WDRV_FLASH_HANDLE    0

/*Netork parameters section base address in Curiosity flash; it's*/
/*the page just before the latest one which is used by pkcs11 module*/
#define WIFI_NETWORK_PARAM_SECTION_START_ADDRESS    ( 0x9d200000 - 0x8000 )

/*Netork parameters section size in Curiosity flash*/
#define WIFI_NETWORK_PARAM_SECTION_SIZE             ( DRV_FLASH_PAGE_SIZE )


/*Network parameters entry size; should be the same as */
#define WIFI_NETWORK_PARAM_ENTRY_SIZE    sizeof( WIFINetworkProfile_t )

/*Max number of networks to be saved*/
#define WIFI_MAX_NETWORK_PROFILES        64

/* Network parameters section in curiosity flash
 *
 * Point p_network_param to the network parameter storage area by direct address since
 * the XC32 compiler will create a single byte initializer at that address if assigning
 * it to a const section. That, in turn, will cause the binary image to be too large,
 * about 2MB in size, since the address is at the end of program memory. */
const uint8_t * p_network_param = ( const uint8_t * ) WIFI_NETWORK_PARAM_SECTION_START_ADDRESS;

/*Bitmap to mark busy and free entries in network parameters section*/
uint64_t g_network_param_bitmap = 0;

/*buffer used for read and write network parameters to flash*/
uint8_t g_network_param_copy[ WIFI_NETWORK_PARAM_SECTION_SIZE ] = { 0 };

/*ping reply queue*/
QueueHandle_t xPingReplyQueue;



typedef struct
{
    uint8_t ucSSIDLength;                               /*SSID length*/
    char pcSSID[ WDRV_MAX_SSID_LENGTH ];                /*SSID of the WiFi network*/
    uint8_t ucPasswordLength;                           /**<SSID length. */
    char pcPassword[ WDRV_MAX_WPA_PASS_PHRASE_LENGTH ]; /*Password needed to connect*/
    WIFISecurity_t xSecurity;                           /*WiFi Security. @see WIFISecurity_t*/
    int8_t cChannel;                                    /*Channel number*/
    uint8_t __PAD24__[ 5 ];                             /*Padding to make struct quad word aligned*/
} WIFI_NetworkParams_t;
/************************************* Section: worker code ************************************************** */
/* */

void WIFI_SecuritySet( WIFISecurity_t securityMode,
                       const char * pcKey,
                       uint8_t u8KeyLength )
{
    uint8_t au8PSK[ WIFI_MAX_PSK_LEN ];

    if( securityMode != eWiFiSecurityOpen )
    {
        memcpy( au8PSK, pcKey, u8KeyLength );
    }

    switch( securityMode )
    {
        case eWiFiSecurityOpen:
            WDRV_EXT_CmdSecNoneSet();
            break;

        case eWiFiSecurityWEP:

            if( ( u8KeyLength == 5 ) || ( u8KeyLength == 13 ) )
            {
                int i;
                uint8_t buf[ 26 ] = { 0 };
                uint8_t c_value;

                for( i = 0; i < u8KeyLength * 2; i++ )
                {
                    c_value = ( i % 2 == 0 ) ? ( au8PSK[ i / 2 ] >> 4 ) : ( au8PSK[ i / 2 ] & 0x0F );
                    buf[ i ] = ( c_value > 9 ) ? ( 'A' + c_value - 0x0A ) : ( '0' + c_value - 0x00 );
                }

                u8KeyLength *= 2;
                memcpy( au8PSK, buf, u8KeyLength );
                au8PSK[ u8KeyLength ] = 0x00;
            }

            WDRV_EXT_CmdSecWEPSet( au8PSK, u8KeyLength );
            break;

        case eWiFiSecurityWPA:
        case eWiFiSecurityWPA2:
            WDRV_EXT_CmdSecWPASet( au8PSK, u8KeyLength );
            break;

        default:
            WDRV_ASSERT( pdFALSE, "Undefined security mode" );
            break;
    }
}


/**
 * @brief Sets Wi-Fi mode.
 *
 * @param[in] xDeviceMode - Mode of the device Station / Access Point /P2P.
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise.
 */
WIFIReturnCode_t WIFI_SetMode( WIFIDeviceMode_t xDeviceMode )
{
    WIFIReturnCode_t ret = eWiFiNotSupported;

    switch( xDeviceMode )
    {
        case eWiFiModeStation:
            WDRV_EXT_CmdNetModeBSSSet();
            gp_wdrv_cfg->networkType = WDRV_NETWORK_TYPE_INFRASTRUCTURE;
            ret = eWiFiSuccess;
            break;

        case eWiFiModeAP:
            WDRV_EXT_CmdNetModeAPSet();
            gp_wdrv_cfg->networkType = WDRV_NETWORK_TYPE_SOFT_AP;
            ret = eWiFiSuccess;
            break;

        case eWiFiModeP2P:

            /* WDRV_ASSERT drops execution into a while(1) loop. It is better to exit this
             * function. */
            /* WDRV_ASSERT( pdFALSE, "P2P mode will be supported in a future drop\n" ); */
            configPRINTF( ( "P2P mode will be supported in a future drop\r\n" ) );
            ret = eWiFiNotSupported;
            break;

        default:

            /* WDRV_ASSERT drops execution into a while(1) loop. It is better to exit this
             * function. */
            /* WDRV_ASSERT( pdFALSE, "Undefined network type" ); */
            configPRINTF( ( "Undefined network type\r\n" ) );
            ret = eWiFiNotSupported;
            break;
    }

    return ret;
}


/**
 * @brief Gets Wi-Fi mode.
 *
 * @param[in] pxDeviceMode - return mode Station / Access Point /P2P
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise..
 */
WIFIReturnCode_t WIFI_GetMode( WIFIDeviceMode_t * pxDeviceMode )
{
    configASSERT( pxDeviceMode != NULL );

    *pxDeviceMode = WDRV_EXT_CmdNetModeGet();

    return eWiFiSuccess;
}






static void AWS_MCHP_InitDoneCB( void )
{
    xTaskNotify( waiting_task, WDRV_MAC_EVENT_INIT_DONE, eSetBits );
}
#endif

/*-----------------------------------------------------------*/

static void AWS_MCHP_ConnectEvent( bool connected)
{
    if( ( connected ) ) /*hseth && (client[0] == 0)) */
    {
        xTaskNotify( waiting_task, WDRV_MAC_EVENT_CONNECT_DONE, eSetBits );
    }
}


/*-----------------------------------------------------------*/

static void WIFI_ConnectCallback(DRV_HANDLE handle, WDRV_PIC32MZW_ASSOC_HANDLE assocHandle, WDRV_PIC32MZW_CONN_STATE currentState)
{
    
    switch (currentState)
    {
    case WDRV_PIC32MZW_CONN_STATE_DISCONNECTED:
        g_is_wifi_connected = false;
        printf("APP: WiFi Disconnected\r\n");
        break;
    case WDRV_PIC32MZW_CONN_STATE_CONNECTED:
        g_is_wifi_connected = true;
        printf("APP: WiFi Connected\r\n");
        AWS_MCHP_ConnectEvent(true);
        break;
    case WDRV_PIC32MZW_CONN_STATE_FAILED:
        g_is_wifi_connected = false;
        printf("APP: WiFi Connection Failed\r\n");
        break;
    case WDRV_PIC32MZW_CONN_STATE_CONNECTING:
        g_is_wifi_connected = false;
        printf("APP: WiFi Connecting\r\n");
        break;
    }
    //drvAssocHandle = assocHandle;
}


/**
 * @brief Initializes the Wi-Fi module.
 *
 * This function must be called exactly once before any other
 * Wi-Fi functions (including socket functions) can be used.
 *
 * @return eWiFiSuccess if everything succeeds, eWiFiFailure otherwise.
 */
WIFIReturnCode_t WIFI_On( void )
{
    EventBits_t evBits;
    //WDRV_DBG_INFORM_MESSAGE(("\r\n[%s] In\r\n", __func__));
    
    if (SYS_STATUS_READY != WDRV_PIC32MZW_Status(sysObj.drvWifiPIC32MZW1))
    {
        //WDRV_DBG_INFORM_MESSAGE(("\r\n[%s] is not SYS_STATUS_READY...\r\n", __func__));
        return eWiFiFailure;
    }
    
    wdrvHandle = WDRV_PIC32MZW_Open(0, 0);
    if (DRV_HANDLE_INVALID == wdrvHandle)
    {
        return eWiFiFailure;
    }
    //WDRV_DBG_INFORM_MESSAGE(("\r\nWiFi Driver Open success...\r\n"));
    if (xPingReplyQueue == NULL)
        xPingReplyQueue = xQueueCreate( 10, sizeof( uint16_t ) );
#if 0
    /* Make sure the Wi-Fi is off before turning on. */
    WDRV_EXT_Deinitialize();

    waiting_task = xTaskGetCurrentTaskHandle();

    WDRV_WILC1000_Default_Hook_Initialize( &WILC1000_hooks );
    WILC1000_hooks.frame_received = xNetworkFrameReceived;
    WILC1000_hooks.wdrvIndications.connectEvent = AWS_MCHP_ConnectEvent;
    WILC1000_hooks.wdrvIndications.initCompleted = AWS_MCHP_InitDoneCB;
    WILC1000_hooks.isEthMode = true;
    WILC1000_hooks.RFReady = NULL;
    WILC1000_hooks.ScanDone = NULL;
    WILC1000_hooks.get_rx_buf = NULL;
    WILC1000_hooks.WPSDone = NULL;

    /*Initialize WiFi Driver Radio and Firmware .*/
    WDRV_EXT_Initialize( &WILC1000_hooks, true );

    /* Wait for Wi-Fi initialization to complete. */
    xTaskNotifyWait( WDRV_MAC_EVENT_INIT_DONE, WDRV_MAC_EVENT_INIT_DONE, &evBits, WIFI_MAC_INIT_TIMEOUT );

    if( ( evBits & WDRV_MAC_EVENT_INIT_DONE ) == 0 )
    {
        /* Timed out. */
        return eWiFiTimeout;
    }

    /*Check if init was successful. */
    if( isWdrvExtReady() != pdTRUE )
    {
        /* Init failed. */
        return eWiFiFailure;
    }
#endif
    return eWiFiSuccess;
}


/**
 * @brief Turns off the Wi-Fi module.
 *
 * This function turns off Wi-Fi module.
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise.
 */
WIFIReturnCode_t WIFI_Off( void )
{
    //WDRV_EXT_Deinitialize();
   WDRV_PIC32MZW_Close(wdrvHandle);

   
    return eWiFiSuccess;
}

/**
 * @brief Resets the WiFi Module.
 *
 * @param[in] None.
 *
 * @return eWiFiSuccess if disconnected, failure code otherwise.
 */
WIFIReturnCode_t WIFI_Reset( void )
{
    WIFIReturnCode_t ret = eWiFiSuccess;


    ret = WIFI_Off();

    if( ret != eWiFiSuccess )
    {
        WDRV_DBG_ERROR_MESSAGE( ( "Failed to turn off WiFi module\n" ) );

        return ret;
    }

    ret = WIFI_On();

    if( ret != eWiFiSuccess )
    {
        WDRV_DBG_ERROR_MESSAGE( ( "Failed to turn on WiFi module\n" ) );

        return ret;
    }

    return ret;
}



WIFIReturnCode_t WIFI_SetMode( WIFIDeviceMode_t xDeviceMode )
{
    WIFIReturnCode_t ret = eWiFiNotSupported;

    switch( xDeviceMode )
    {
        case eWiFiModeStation:
            g_wifiMode = xDeviceMode;
            //WDRV_EXT_CmdNetModeBSSSet();
            //gp_wdrv_cfg->networkType = WDRV_NETWORK_TYPE_INFRASTRUCTURE;
            ret = eWiFiSuccess;
            break;

        case eWiFiModeAP:
            g_wifiMode = xDeviceMode;
            //WDRV_EXT_CmdNetModeAPSet();
            //gp_wdrv_cfg->networkType = WDRV_NETWORK_TYPE_SOFT_AP;
            ret = eWiFiSuccess;
            break;

        case eWiFiModeP2P:

            /* WDRV_ASSERT drops execution into a while(1) loop. It is better to exit this
             * function. */
            /* WDRV_ASSERT( pdFALSE, "P2P mode will be supported in a future drop\n" ); */
            configPRINTF( ( "P2P mode will be supported in a future drop\r\n" ) );
            ret = eWiFiNotSupported;
            break;

        default:

            /* WDRV_ASSERT drops execution into a while(1) loop. It is better to exit this
             * function. */
            /* WDRV_ASSERT( pdFALSE, "Undefined network type" ); */
            configPRINTF( ( "Undefined network type\r\n" ) );
            ret = eWiFiNotSupported;
            break;
    }
      return ret;
}

/**
 * @brief Set power management mode
 *
 * @param[in] usPMModeType - Low power mode type.
 *
 * @param[in] pvOptionValue - A buffer containing the value of the option to set
 *                            depends on the mode type
 *                            ex - beacon interval in sec
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise.
 */
WIFIReturnCode_t WIFI_SetPMMode( WIFIPMMode_t xPMModeType,
                                 const void * pvOptionValue )
{
    return eWiFiNotSupported;
}

/**
 * @brief Configure SoftAP
 *
 * @param[in] pxNetworkParams - Network params to configure and start soft AP.
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise.
 */
WIFIReturnCode_t WIFI_ConfigureAP( const WIFINetworkParams_t * const pxNetworkParams )
{
    
}
/**
 * @brief Connects to Access Point.
 *
 * @param[in] pxNetworkParams Configuration to join AP.
 *
 * @return eWiFiSuccess if connection is successful, failure code otherwise.
 */
WIFIReturnCode_t WIFI_ConnectAP( const WIFINetworkParams_t * const pxNetworkParams )
{
    bool ret = true;
    EventBits_t evBits;
    wifiConfig g_wifiConfig;
    
    memset(&g_wifiConfig, 0, sizeof(wifiConfig));
    //WDRV_DBG_INFORM_MESSAGE(("password = %s\r\n", pxNetworkParams->pcPassword));
    //WDRV_DBG_INFORM_MESSAGE(("password len = %d\r\n", pxNetworkParams->ucPasswordLength));
    //WDRV_DBG_INFORM_MESSAGE(("ssid len = %d\r\n", pxNetworkParams->ucSSIDLength));        
            
    if ( (pxNetworkParams ==NULL)  || (pxNetworkParams->pcSSID == NULL) )
    {
        WDRV_DBG_ERROR_MESSAGE( ( "\r\n[%s] Fail, pxNetworkParams or pxNetworkParams->pcSSID is null\r\n", __func__ ) );
        return eWiFiFailure;
    }

    if( pxNetworkParams->xSecurity != eWiFiSecurityOpen )
    {
        if ( pxNetworkParams->pcPassword == NULL )
        {
            WDRV_DBG_ERROR_MESSAGE( ( "\r\n[%s] Fail, pxNetworkParams->pcPassword is null\r\n", __func__ ) );
            return eWiFiFailure;
        }
    }
    /* Set mode to station mode. */
    WIFI_SetMode( eWiFiModeStation );
    

    //if (WDRV_PIC32MZW_STATUS_OK != WDRV_PIC32MZW_RegDomainSet(appData.wdrvHandle, "GEN", SYS_WIFI_SetRegDomainCallback))
    //{
    //    printf("APP: Country Code set fail \r\n");
    //}    
    if (WDRV_PIC32MZW_STATUS_OK != WDRV_PIC32MZW_BSSCtxSetSSID(&g_wifiConfig.bssCtx, pxNetworkParams->pcSSID, pxNetworkParams->ucSSIDLength))
    {
        WDRV_DBG_ERROR_MESSAGE(("APP: SSID set fail \r\n"));
        return eWiFiFailure;
    }

    if (WDRV_PIC32MZW_STATUS_OK != WDRV_PIC32MZW_BSSCtxSetChannel(&g_wifiConfig.bssCtx, 0))
    {
        WDRV_DBG_ERROR_MESSAGE(("APP: channel set fail %d \r\n", 0));
        return eWiFiFailure;
    }
    
    //memset(&g_wifiConfig.authCtx, 0, sizeof(WDRV_PIC32MZW_AUTH_CONTEXT));
    switch (pxNetworkParams->xSecurity)
    {
    case eWiFiSecurityOpen:
    {
        if (WDRV_PIC32MZW_STATUS_OK != WDRV_PIC32MZW_AuthCtxSetOpen(&g_wifiConfig.authCtx))
        {
            WDRV_DBG_ERROR_MESSAGE(("APP: Unable to set Authentication\r\n"));
            return eWiFiFailure;
        }
        break;
    }
    case eWiFiSecurityWPA2:
    {
        if (WDRV_PIC32MZW_STATUS_OK != WDRV_PIC32MZW_AuthCtxSetPersonal(&g_wifiConfig.authCtx, pxNetworkParams->pcPassword, strlen(pxNetworkParams->pcPassword), WDRV_PIC32MZW_AUTH_TYPE_WPA2_PERSONAL))
        {
            WDRV_DBG_ERROR_MESSAGE(("APP: Unable to set authentication to WPA2\r\n"));
            return eWiFiFailure;
        }
        break;
    }
    case eWiFiSecurityWPA:
    {
        if (WDRV_PIC32MZW_STATUS_OK != WDRV_PIC32MZW_AuthCtxSetPersonal(&g_wifiConfig.authCtx, pxNetworkParams->pcPassword, strlen(pxNetworkParams->pcPassword), WDRV_PIC32MZW_AUTH_TYPE_WPAWPA2_PERSONAL))
        {
            WDRV_DBG_ERROR_MESSAGE(("APP: Unable to set authentication to WPAWPA2 MIXED\r\n"));
            return eWiFiFailure;
        }
        break;
    }

    case eWiFiSecurityWEP:
    {
        if (WDRV_PIC32MZW_STATUS_OK != WDRV_PIC32MZW_AuthCtxSetWEP(&g_wifiConfig.authCtx, 0, pxNetworkParams->pcPassword, pxNetworkParams->ucPasswordLength))
        {
            WDRV_DBG_ERROR_MESSAGE(("APP: Unable to set authentication to WEP \r\n"));
            return eWiFiFailure;
        }
        if (WDRV_PIC32MZW_STATUS_OK != WDRV_PIC32MZW_AuthCtxSharedKey(&g_wifiConfig.authCtx, true))
        {
            WDRV_DBG_ERROR_MESSAGE(("APP: Unable to Enable shared key authentication \r\n"));
            return eWiFiFailure;
        }
        break;
    }
    default:
    {
        return eWiFiNotSupported;
    }
    }
    
    waiting_task = xTaskGetCurrentTaskHandle();
    
    //WDRV_PIC32MZW_BSSDisconnect(wdrvHandle);
    
    if (WDRV_PIC32MZW_STATUS_OK != WDRV_PIC32MZW_BSSConnect(wdrvHandle, &g_wifiConfig.bssCtx, &g_wifiConfig.authCtx, WIFI_ConnectCallback))
    {
        WDRV_DBG_ERROR_MESSAGE(("fail call api\r\n"));
    }
    
      /* Wait for Wi-Fi connection to complete. */
    xTaskNotifyWait( WDRV_MAC_EVENT_CONNECT_DONE, WDRV_MAC_EVENT_CONNECT_DONE, &evBits, WIFI_MAC_CONNECT_TIMEOUT );

    if( ( evBits & WDRV_MAC_EVENT_CONNECT_DONE ) == 0 )
    {
        /* Timed out. */
        WDRV_DBG_ERROR_MESSAGE( ( "Connection timeout\r\n" ) );

        return eWiFiTimeout;
    }
    
    WDRV_DBG_INFORM_MESSAGE( ( "Connection success\r\n" ) );
     return eWiFiSuccess;
  


#if 0
    EventBits_t evBits;
    uint8_t ucMACAddress[ ipMAC_ADDRESS_LENGTH_BYTES ];

    configASSERT( pxNetworkParams != NULL );
    configASSERT( pxNetworkParams->pcSSID != NULL );

    if( pxNetworkParams->xSecurity != eWiFiSecurityOpen )
    {
        configASSERT( pxNetworkParams->pcPassword != NULL );
    }

    /* Set mode to station mode. */
    WIFI_SetMode( eWiFiModeStation );


    waiting_task = xTaskGetCurrentTaskHandle();


    /* Set the AP channel. */

    /* Setting the AP channel is optional to connect to an AP. The router automatically
     * sets a channel for the connection. */
    /* WDRV_EXT_CmdChannelSet( pxNetworkParams->cChannel ); */

    /*Set the AP SSID. */
    if( pxNetworkParams->ucSSIDLength > wificonfigMAX_SSID_LEN )
    {
        return eWiFiFailure;
    }

    WDRV_EXT_CmdSSIDSet( ( uint8_t * ) pxNetworkParams->pcSSID, pxNetworkParams->ucSSIDLength );

    /*Set the AP security. */
    if( pxNetworkParams->ucPasswordLength > wificonfigMAX_PASSPHRASE_LEN )
    {
        return eWiFiFailure;
    }

    WIFI_SecuritySet( pxNetworkParams->xSecurity, pxNetworkParams->pcPassword, pxNetworkParams->ucPasswordLength );

    /*Set driver state. */
    WDRV_IsDisconnectRequestedSet( pdFALSE );
    WDRV_ConnectionStateSet( WDRV_CONNECTION_STATE_IN_PROGRESS );

    WDRV_DBG_INFORM_MESSAGE( ( "Start Wi-Fi Connection to SSID %.*s...\r\n", pxNetworkParams->ucSSIDLength, pxNetworkParams->pcSSID ) );

    /* Read MAC address from the chip and set it in FreeRTOS network stack. */
    WDRV_EXT_CmdMacAddressGet( ucMACAddress );
    FreeRTOS_UpdateMACAddress( ucMACAddress );

    if( WDRV_EXT_CmdConnect() != WDRV_SUCCESS )
    {
        /* Connection failed miserably. */
        return eWiFiFailure;
    }

    /* Wait for Wi-Fi connection to complete. */
    xTaskNotifyWait( WDRV_MAC_EVENT_CONNECT_DONE, WDRV_MAC_EVENT_CONNECT_DONE, &evBits, WIFI_MAC_CONNECT_TIMEOUT );

    if( ( evBits & WDRV_MAC_EVENT_CONNECT_DONE ) == 0 )
    {
        /* Timed out. */
        WDRV_DBG_INFORM_MESSAGE( ( "Connection timeout\r\n" ) );

        return eWiFiTimeout;
    }

    if( isLinkUp() )
    {
        return eWiFiSuccess;
    }
    else
    {
        return eWiFiFailure;
    }
#endif
    return eWiFiSuccess;
}

/**
 * @brief Disconnects from Access Point.
 *
 * @param[in] None.
 *
 * @return eWiFiSuccess if disconnected, failure code otherwise.
 */
WIFIReturnCode_t WIFI_Disconnect( void )
{
    WIFIReturnCode_t xReturnStatus = eWiFiFailure;
    
    if (!g_is_wifi_connected)
    {
        xReturnStatus = eWiFiSuccess;
    }
    else
    {
        if ( WDRV_PIC32MZW_STATUS_OK == WDRV_PIC32MZW_BSSDisconnect(wdrvHandle) )
        {
            xReturnStatus = eWiFiSuccess;
        }
        else
        {
            xReturnStatus = eWiFiFailure;
        }
    }
        
    return xReturnStatus;
}


/**
 * @brief Retrieves host IP address from URL using DNS
 *
 * @param[in] pcHost - Host URL.
 * @param[in] pucIPAddr - IP Address buffer.
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise.
 *
 */
WIFIReturnCode_t WIFI_GetHostIP( char * pcHost,
                                 uint8_t * pucIPAddr )
{
    WIFIReturnCode_t xStatus = eWiFiFailure;

    if (pucIPAddr == NULL || pcHost == NULL)
    {
        WDRV_DBG_ERROR_MESSAGE( ( "\r\n[%s] Fail, input arg  is null\r\n", __func__ ) );
        return eWiFiFailure;
    }
    

    *( ( uint32_t * ) pucIPAddr ) = SOCKETS_GetHostByName( pcHost );

    if( *( ( uint32_t * ) pucIPAddr ) != 0 )
    {
        xStatus = eWiFiSuccess;
    }

    return xStatus;
}


/**
 * @brief Retrieves the Wi-Fi interface's IP address.
 *
 * @param[in] pucIPAddr - IP Address buffer.
 * unsigned long ulIPAddress = 0;
 * WIFI_GetIP(&ulIPAddress);
 *
 * @return eWiFiSuccess if disconnected, eWiFiFailure otherwise.
 */
WIFIReturnCode_t WIFI_GetIP( uint8_t * pucIPAddr )
{
    if (pucIPAddr == NULL)
    {
        WDRV_DBG_ERROR_MESSAGE( ( "\r\n[%s] Fail, pucIPAddr is null\r\n", __func__ ) );
        return eWiFiFailure;
    }

    *( ( uint32_t * ) pucIPAddr ) = FreeRTOS_GetIPAddress();

    return eWiFiSuccess;
}

/**
 * @brief Retrieves the Wi-Fi interface's MAC address.
 *
 * @param[out] MAC Address buffer.
 *
 * @return eWiFiSuccess if disconnected, eWiFiFailure otherwise.
 */
WIFIReturnCode_t WIFI_GetMAC( uint8_t * pucMac )
{
    WIFIReturnCode_t xReturnStatus = eWiFiFailure;

    if (pucMac == NULL)
    {
        WDRV_DBG_ERROR_MESSAGE( ( "\r\n[%s] Fail, pucMac is null\r\n", __func__ ) );
        return eWiFiFailure;
    }
    
    if ( WDRV_PIC32MZW_STATUS_OK == WDRV_PIC32MZW_InfoDeviceMACAddressGet(wdrvHandle, pucMac) )
    {
        xReturnStatus = eWiFiSuccess;
    }

    return xReturnStatus;
}

/**
 * @brief Gets Wi-Fi mode.
 *
 * @param[in] pxDeviceMode - return mode Station / Access Point /P2P
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise..
 */
WIFIReturnCode_t WIFI_GetMode( WIFIDeviceMode_t * pxDeviceMode )
{
    if (pxDeviceMode == NULL)
    {
        WDRV_DBG_ERROR_MESSAGE( ( "\r\n[%s] Fail, pxDeviceMode is null\r\n", __func__ ) );
        return eWiFiFailure;
    }
   
    *pxDeviceMode = g_wifiMode;

    return eWiFiSuccess;
}

/**
 * @brief Get power management mode
 *
 * @param[out] xPMModeType - pointer to get current power mode set.
 *
 * @param[out] pvOptionValue - optional value
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise.
 */
WIFIReturnCode_t WIFI_GetPMMode( WIFIPMMode_t * pxPMModeType,
                                 void * pvOptionValue )
{
    return eWiFiNotSupported;
}

/*-----------------------------------------------------------*/

void vApplicationPingReplyHook( ePingReplyStatus_t eStatus,
                                uint16_t usIdentifier )
{
    /*handle ping reply. */
    switch( eStatus )
    {
        case eSuccess:
            /* A valid ping reply has been received. */
            xQueueSend( xPingReplyQueue, &usIdentifier, pdMS_TO_TICKS( 10 ) );
            break;

        case eInvalidChecksum:
        case eInvalidData:
            /* A reply was received but it was not valid. */
            break;
    }
}

/**
 * @brief Ping an IP address in the network.
 *
 * @param[in] IP Address to ping.
 * @param[in] Number of times to ping
 * @param[in] Interval in mili seconds for ping operation
 *
 * @return eWiFiSuccess if disconnected, failure code otherwise.
 */
WIFIReturnCode_t WIFI_Ping( uint8_t * pucIPAddr,
                            uint16_t usCount,
                            uint32_t ulIntervalMS )
{
    uint32_t ulIPAddress = 0;
    uint16_t usPingSeqNum = pdFAIL;
    uint16_t usPingReplySeqNum = pdFAIL;
    uint32_t ulIndex = 0;

    if( ( NULL == pucIPAddr ) || ( 0 == usCount ) )
    {
        return eWiFiFailure;
    }

    //waiting_task = xTaskGetCurrentTaskHandle();

    /*If link is not up, return failure */
    if( g_is_wifi_connected )
    {

        ulIPAddress = FreeRTOS_inet_addr( ( const char * ) pucIPAddr );

        for( ulIndex = 0; ulIndex < usCount; ulIndex++ )
        {
            usPingSeqNum = FreeRTOS_SendPingRequest( ulIPAddress, WIFI_PING_PKT_SIZE, pdMS_TO_TICKS( ulIntervalMS ) );

            WDRV_DBG_ERROR_MESSAGE( ( "\r\nSending Ping request %d\r\n", usPingSeqNum ) );

            if( usPingSeqNum == pdFAIL )
            {
                /* ping failed */
                WDRV_DBG_ERROR_MESSAGE( ( "\r\nSending Ping request failed\r\n" ) );

                return eWiFiFailure;
            }
            else
            {   
                /* The ping was sent.  Wait for a reply.  */
                if( xQueueReceive( xPingReplyQueue,
                                   &usPingReplySeqNum,
                                   pdMS_TO_TICKS( ulIntervalMS ) ) == pdPASS )
                {
                    /* A ping reply was received.  Was it a reply to the ping just sent? */
                    if( usPingSeqNum == usPingReplySeqNum )
                    {
                        /* This was a reply to the request just sent. */
                        WDRV_DBG_INFORM_MESSAGE( ( "Ping Reply [%d] out of [%d] received\r\n", ulIndex, usCount ) );
                    }
                }
            }
        }
        /* all ping were sent*/
        return eWiFiSuccess;
    }
    WDRV_DBG_ERROR_MESSAGE( ( "\r\nLink is not yet ready; cannot send ping request\r\n" ) );

    return eWiFiFailure;
}

/**
 * @brief Check if the Wi-Fi is connected.
 *
 * @return pdTRUE if the link is up, pdFalse otherwise.
 */
BaseType_t WIFI_IsConnected( void )
{
    BaseType_t xIsConnected = pdFALSE;

    if( g_is_wifi_connected )
    {
        xIsConnected = pdTRUE;
    }

    return xIsConnected;
}

WIFIReturnCode_t WIFI_RegisterNetworkStateChangeEventCallback( IotNetworkStateChangeEventCallback_t xCallback  )
{
    /** Needs to implement dispatching network state change events **/
    return eWiFiNotSupported;
}


/**
 * @brief Perform WiF Scan
 *
 * @param[in] pxBuffer - Buffer for scan results.
 * @param[in] uxNumNetworks - Number of networks in scan result.
 *
 * @return eWiFiSuccess if disconnected, eWiFiFailure otherwise.
 */
WIFIReturnCode_t WIFI_Scan( WIFIScanResult_t * pxBuffer,
                            uint8_t ucNumNetworks )
{
    WDRV_PIC32MZW_STATUS status;
    WIFI_SCAN_STATE state;
    int found_num = 0;

    if (pxBuffer == NULL)
    {
        WDRV_DBG_ERROR_MESSAGE( ( "\r\n[%s] Fail, pxBuffer is null\r\n", __func__ ) );
        return eWiFiFailure;
    }
    
    if (DRV_HANDLE_INVALID == wdrvHandle)
    {
        configPRINTF(("[%s] Driver not yet open ...\r\n", __func__));
        return eWiFiFailure ;
    }
    
    

    state = WIFI_SCAN_STATE_INIT;
    while (1)
    {
        switch (state)
        {
            case WIFI_SCAN_STATE_INIT:
            {
                if (WDRV_PIC32MZW_STATUS_OK == WDRV_PIC32MZW_BSSFindFirst(wdrvHandle, WDRV_PIC32MZW_CID_ANY, true, NULL))
                {
                    state = WIFI_SCAN_STATE_SCANNING;
                }
            }
            case WIFI_SCAN_STATE_SCANNING:
            {
                /* Wait for BSS find operation to complete, then report the number
                 of results found. */
                if (false == WDRV_PIC32MZW_BSSFindInProgress(wdrvHandle))
                {

                    configPRINTF(("Scan complete, %d AP(s) found\r\n", WDRV_PIC32MZW_BSSFindGetNumBSSResults(wdrvHandle)));
                    state = WIFI_SCAN_STATE_SCAN_GET_RESULTS;
                }
                break;
            }
            case WIFI_SCAN_STATE_SCAN_GET_RESULTS:
            {
                WDRV_PIC32MZW_BSS_INFO BSSInfo;

                /* Request the current BSS find results. */
                if (WDRV_PIC32MZW_STATUS_OK == WDRV_PIC32MZW_BSSFindGetInfo(wdrvHandle, &BSSInfo))
                {
                    configPRINTF(("AP found: RSSI: %d %s\r\n", BSSInfo.rssi, BSSInfo.ctx.ssid.name));

                    if (found_num >= ucNumNetworks)
                    {
                        return eWiFiSuccess;
                    }
                    memset(&pxBuffer[found_num], 0, sizeof(WIFIScanResult_t));
                    memcpy(pxBuffer[found_num].cSSID,BSSInfo.ctx.ssid.name,BSSInfo.ctx.ssid.length);
                    memcpy(pxBuffer[found_num].ucBSSID,BSSInfo.ctx.bssid.addr,wificonfigMAX_BSSID_LEN);
                    pxBuffer[found_num].cChannel = BSSInfo.ctx.channel;
                    pxBuffer[found_num].cRSSI = BSSInfo.rssi;                

                    if (BSSInfo.authTypeRecommended == WDRV_PIC32MZW_AUTH_TYPE_OPEN)      
                        pxBuffer[found_num].xSecurity = eWiFiSecurityOpen;
                    else if (BSSInfo.authTypeRecommended == WDRV_PIC32MZW_AUTH_TYPE_WEP)      
                        pxBuffer[found_num].xSecurity = eWiFiSecurityWEP;
                    else if (BSSInfo.authTypeRecommended == WDRV_PIC32MZW_AUTH_TYPE_WPAWPA2_PERSONAL)      
                        pxBuffer[found_num].xSecurity = eWiFiSecurityWPA2 ;
                    else if (BSSInfo.authTypeRecommended == WDRV_PIC32MZW_AUTH_TYPE_WPA2_PERSONAL)      
                        pxBuffer[found_num].xSecurity = eWiFiSecurityWPA2 ;
                    else 
                        pxBuffer[found_num].xSecurity = eWiFiSecurityNotSupported ;


                    found_num++;
                    /* Request the next set of BSS find results. */
                    status = WDRV_PIC32MZW_BSSFindNext(wdrvHandle, NULL);

                    if (WDRV_PIC32MZW_STATUS_BSS_FIND_END == status)
                    {   
                        state = WIFI_SCAN_STATE_SCAN_DONE;  
                        return eWiFiSuccess;
                    }
                    else if ((WDRV_PIC32MZW_STATUS_NOT_OPEN == status) || (WDRV_PIC32MZW_STATUS_INVALID_ARG == status))
                    {
                        /* An error occurred requesting results. */
                        state = WIFI_SCAN_STATE_ERROR;
                    }
                }
                break;
            }
            case WIFI_SCAN_STATE_ERROR:
            {                    
                return eWiFiFailure;
                break;
            }
            default:
            {
                break;
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    return eWiFiFailure;
}

/**
 * @brief Wi-Fi Add Network profile.
 *
 * Adds Wi-Fi network to network list in non-volatile memory
 *
 * @param[in] pxNetworkProfile - network profile parameters
 * @param[out] pusIndex - network profile index
 *
 * @return Profile stored index on success, or negative error code on failure.
 */
WIFIReturnCode_t WIFI_NetworkAdd( const WIFINetworkProfile_t * const pxNetworkProfile,
                                  uint16_t * pusIndex )
{
    #ifdef ENABLE_NETWORK_STORAGE
        uint8_t * pSrcBuf = pxNetworkProfile;
        uint8_t * pDestBuf;
        uint16_t u8Idx = 0;
        uint16_t u8NumOfWrites = 0;
        uint8_t u8NetworkIdx = 0;

        /*Check input parameters*/
        if( ( pxNetworkProfile == NULL ) || ( pusIndex == NULL ) )
        {
            WDRV_DBG_ERROR_MESSAGE( ( "\r\nSave network to flash failed\r\n" ) );

            return eWiFiFailure;
        }

        /*Get network index*/
        u8NetworkIdx = network_param_get_free_index();

        if( u8NetworkIdx == WIFI_MAX_NETWORK_PROFILES )
        {
            WDRV_DBG_ERROR_MESSAGE( ( "\r\nNo more space for new network %d\r\n", WIFI_MAX_NETWORK_PROFILES ) );

            return eWiFiFailure;
        }

        /*Get copy from network parameters flash section*/
        memcpy( g_network_param_copy, p_network_param, WIFI_NETWORK_PARAM_SECTION_SIZE );

        /*Erase network parameters flash section*/
        taskENTER_CRITICAL();
        DRV_FLASH_ErasePage( WDRV_FLASH_HANDLE, ( uint32_t ) p_network_param );

        while( !DRV_FLASH_IsBusy( WDRV_FLASH_HANDLE ) )
        {
        }

        taskEXIT_CRITICAL();

        /*Prepare write buffer*/
        pDestBuf = g_network_param_copy + ( u8NetworkIdx * WIFI_NETWORK_PARAM_ENTRY_SIZE );
        memcpy( pDestBuf, pSrcBuf, WIFI_NETWORK_PARAM_ENTRY_SIZE );

        /*Write network parameters entire section (page)*/
        pDestBuf = ( uint8_t * ) p_network_param;
        pSrcBuf = g_network_param_copy;
        u8NumOfWrites = WIFI_NETWORK_PARAM_SECTION_SIZE / 16;
        WDRV_DBG_INFORM_MESSAGE( ( "\r\nWriting network parameters - index %d\r\n", u8NetworkIdx ) );

        for( u8Idx = 0; u8Idx < u8NumOfWrites; u8Idx++ )
        {
            taskENTER_CRITICAL();
            DRV_FLASH_WriteQuadWord( WDRV_FLASH_HANDLE, ( uint32_t ) pDestBuf, ( uint32_t * ) pSrcBuf );

            while( DRV_FLASH_IsBusy( WDRV_FLASH_HANDLE ) )
            {
            }

            taskEXIT_CRITICAL();
            pDestBuf += 16;
            pSrcBuf += 16;
        }

        /*verify write operation*/
        pDestBuf = ( uint8_t * ) p_network_param;
        pSrcBuf = g_network_param_copy;

        if( memcmp( pDestBuf, pSrcBuf, WIFI_NETWORK_PARAM_SECTION_SIZE ) == 0 )
        {
            WDRV_DBG_INFORM_MESSAGE( ( "\r\nWrite network parameters done\r\n" ) );

            /*Network write success; update network parameters bitmap*/
            network_param_set_busy_index( u8NetworkIdx );

            *pusIndex = ( int16_t ) u8NetworkIdx;

            return eWiFiSuccess;
        }

        WDRV_DBG_ERROR_MESSAGE( ( "\r\nWrite network parameters failed\r\n" ) );

        return eWiFiFailure;
    #else /* ifdef ENABLE_NETWORK_STORAGE */
        return eWiFiNotSupported;
    #endif /* ifdef ENABLE_NETWORK_STORAGE */
}

/**
 * @brief Wi-Fi Get Network profile .
 *
 * Gets Wi-Fi network parameters at given index from network list in Non volatile memory
 *
 * @param[out] pxNetworkProfile - pointer to return network profile parameters
 * @param[in] usIndex - Index of the network profile,
 *                       must be between 0 to wificonfigMAX_NETWORK_PROFILES
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise.
 */
WIFIReturnCode_t WIFI_NetworkGet( WIFINetworkProfile_t * pxNetworkProfile,
                                  uint16_t usIndex )
{
    #ifdef ENABLE_NETWORK_STORAGE
        /*Check input parameters*/
        if( ( usIndex >= WIFI_MAX_NETWORK_PROFILES ) ||
            ( pxNetworkProfile == NULL ) ||
            ( network_param_get_busy_index( usIndex ) == NULL ) )
        {
            WDRV_DBG_ERROR_MESSAGE( ( "\r\nGet network from flash failed\r\n" ) );

            return eWiFiFailure;
        }

        /*Check if address is in range*/
        uint8_t * pSrcBuf = ( uint8_t * ) p_network_param + ( usIndex * WIFI_NETWORK_PARAM_ENTRY_SIZE );

        if( pSrcBuf >
            ( p_network_param + WIFI_NETWORK_PARAM_SECTION_SIZE - WIFI_NETWORK_PARAM_ENTRY_SIZE ) )
        {
            WDRV_DBG_ERROR_MESSAGE( ( "\r\nAddress is out of network parameters section range\r\n" ) );

            return eWiFiFailure;
        }

        /*Copy network parameters buffer*/
        memcpy( ( uint8_t * ) pxNetworkProfile, pSrcBuf, WIFI_NETWORK_PARAM_ENTRY_SIZE );

        return eWiFiSuccess;
    #else /* ifdef ENABLE_NETWORK_STORAGE */
        return eWiFiNotSupported;
    #endif /* ifdef ENABLE_NETWORK_STORAGE */
}

/**
 * @brief Wi-Fi Delete Network profile .
 *
 * Deletes Wi-Fi network from network list at given index in Non volatile memory
 *
 * @param[in] usIndex - Index of the network profile, must be between 0 to wificonfigMAX_NETWORK_PROFILES
 *                      wificonfigMAX_NETWORK_PROFILES as index will delete all network profiles
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise.
 */
WIFIReturnCode_t WIFI_NetworkDelete( uint16_t usIndex )
{
    #ifdef ENABLE_NETWORK_STORAGE
        /*Check input parameters*/
        if( usIndex > WIFI_MAX_NETWORK_PROFILES )
        {
            WDRV_DBG_ERROR_MESSAGE( ( "\r\nOut of range index\r\n" ) );

            return eWiFiFailure;
        }

        /*Update network parameters bitmap*/
        if( usIndex == WIFI_MAX_NETWORK_PROFILES )
        {
            network_param_set_bitmap( 0 );
        }
        else
        {
            network_param_set_free_index( usIndex );
        }

        return eWiFiSuccess;
    #else /* ifdef ENABLE_NETWORK_STORAGE */
        return eWiFiNotSupported;
    #endif /* ifdef ENABLE_NETWORK_STORAGE */
}
/*-----------------------------------------------------------*/
#if 0


/**
 * @brief Resets the WiFi Module.
 *
 * @param[in] None.
 *
 * @return eWiFiSuccess if disconnected, failure code otherwise.
 */
WIFIReturnCode_t WIFI_Reset( void )
{
    WIFIReturnCode_t ret = eWiFiSuccess;


    ret = WIFI_Off();

    if( ret != eWiFiSuccess )
    {
        WDRV_DBG_ERROR_MESSAGE( ( "Failed to turn off WiFi module\n" ) );

        return ret;
    }

    ret = WIFI_On();

    if( ret != eWiFiSuccess )
    {
        WDRV_DBG_ERROR_MESSAGE( ( "Failed to turn on WiFi module\n" ) );

        return ret;
    }

    return ret;
}

/**
 * @brief Retrieves host IP address from URL using DNS
 *
 * @param[in] pcHost - Host URL.
 * @param[in] pucIPAddr - IP Address buffer.
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise.
 *
 */
WIFIReturnCode_t WIFI_GetHostIP( char * pcHost,
                                 uint8_t * pucIPAddr )
{
    WIFIReturnCode_t xStatus = eWiFiFailure;

    configASSERT( pcHost != NULL );
    configASSERT( pucIPAddr != NULL );

    *( ( uint32_t * ) pucIPAddr ) = SOCKETS_GetHostByName( pcHost );

    if( *( ( uint32_t * ) pucIPAddr ) != 0 )
    {
        xStatus = eWiFiSuccess;
    }

    return xStatus;
}


/**
 * @brief Retrieves the Wi-Fi interface's IP address.
 *
 * @param[in] pucIPAddr - IP Address buffer.
 * unsigned long ulIPAddress = 0;
 * WIFI_GetIP(&ulIPAddress);
 *
 * @return eWiFiSuccess if disconnected, eWiFiFailure otherwise.
 */
WIFIReturnCode_t WIFI_GetIP( uint8_t * pucIPAddr )
{
    configASSERT( pucIPAddr != NULL );

    *( ( uint32_t * ) pucIPAddr ) = FreeRTOS_GetIPAddress();

    return eWiFiSuccess;
}


/**
 * @brief Configure SoftAP
 *
 * @param[in] pxNetworkParams - Network params to configure and start soft AP.
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise.
 */
WIFIReturnCode_t WIFI_ConfigureAP( const WIFINetworkParams_t * const pxNetworkParams )
{
    configASSERT( pxNetworkParams != NULL );
    configASSERT( pxNetworkParams->pcSSID != NULL );

    if( pxNetworkParams->xSecurity != eWiFiSecurityOpen )
    {
        configASSERT( pxNetworkParams->pcPassword != NULL );
    }

    /* Set mode to station mode. */
    WIFI_SetMode( eWiFiModeAP );

    /* Set the AP channel. */
    WDRV_EXT_CmdChannelSet( pxNetworkParams->cChannel );

    /*Set the AP SSID. */
    if( pxNetworkParams->ucSSIDLength > wificonfigMAX_SSID_LEN )
    {
        return eWiFiFailure;
    }

    WDRV_EXT_CmdSSIDSet( ( uint8_t * ) pxNetworkParams->pcSSID, pxNetworkParams->ucSSIDLength );

    /*Set the AP security. */
    if( pxNetworkParams->ucPasswordLength > wificonfigMAX_PASSPHRASE_LEN )
    {
        return eWiFiFailure;
    }

    WIFI_SecuritySet( pxNetworkParams->xSecurity, pxNetworkParams->pcPassword, pxNetworkParams->ucPasswordLength );

    /*Set driver state. */
    WDRV_IsDisconnectRequestedSet( pdFALSE );
    WDRV_ConnectionStateSet( WDRV_CONNECTION_STATE_IN_PROGRESS );

    return eWiFiSuccess;
}

/**
 * @brief Stop SoftAP operation.
 *
 * @param[in] None
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise.
 */
WIFIReturnCode_t WIFI_StartAP( void )
{
    EventBits_t evBits;

    if( WDRV_EXT_CmdConnect() != WDRV_SUCCESS )
    {
        /* Connection failed miserably. */
        return eWiFiFailure;
    }

    /* Wait for Wi-Fi connection to complete. */
    xTaskNotifyWait( WDRV_MAC_EVENT_CONNECT_DONE, WDRV_MAC_EVENT_CONNECT_DONE, &evBits, WIFI_MAC_CONNECT_TIMEOUT );

    if( ( evBits & WDRV_MAC_EVENT_CONNECT_DONE ) == 0 )
    {
        /* Timed out. */
        return eWiFiTimeout;
    }

    if( isLinkUp() )
    {
        return eWiFiSuccess;
    }
    else
    {
        return eWiFiFailure;
    }
}

/**
 * @brief Stop SoftAP operation.
 *
 * @param[in] None
 *
 * @return eWiFiSuccess if disconnected, failure code otherwise.
 */
WIFIReturnCode_t WIFI_StopAP( void )
{
    return WIFI_Disconnect();
}

/**
 * @brief Set power management mode
 *
 * @param[in] usPMModeType - Low power mode type.
 *
 * @param[in] pvOptionValue - A buffer containing the value of the option to set
 *                            depends on the mode type
 *                            ex - beacon interval in sec
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise.
 */
WIFIReturnCode_t WIFI_SetPMMode( WIFIPMMode_t xPMModeType,
                                 const void * pvOptionValue )
{
    return eWiFiNotSupported;
}

/**
 * @brief Get power management mode
 *
 * @param[out] xPMModeType - pointer to get current power mode set.
 *
 * @param[out] pvOptionValue - optional value
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise.
 */
WIFIReturnCode_t WIFI_GetPMMode( WIFIPMMode_t * pxPMModeType,
                                 void * pvOptionValue )
{
    return eWiFiNotSupported;
}

/**
 * @brief Retrieves the Wi-Fi interface's MAC address.
 *
 * @param[out] MAC Address buffer.
 *
 * @return eWiFiSuccess if disconnected, eWiFiFailure otherwise.
 */
WIFIReturnCode_t WIFI_GetMAC( uint8_t * pucMac )
{
    WIFIReturnCode_t xReturnStatus = eWiFiFailure;

    configASSERT( pucMac != NULL );

    if( WDRV_EXT_CmdMacAddressGet( pucMac ) == WDRV_SUCCESS )
    {
        xReturnStatus = eWiFiSuccess;
    }

    return xReturnStatus;
}


/**
 * @brief Perform WiF Scan
 *
 * @param[in] pxBuffer - Buffer for scan results.
 * @param[in] uxNumNetworks - Number of networks in scan result.
 *
 * @return eWiFiSuccess if disconnected, eWiFiFailure otherwise.
 */
WIFIReturnCode_t WIFI_Scan( WIFIScanResult_t * pxBuffer,
                            uint8_t ucNumNetworks )
{
    uint32_t ret;
    uint8_t idx = 0;
    uint16_t num_scan_results = 0;
    WDRV_SCAN_RESULT scanResult;

    /*Check input parameters*/
    if( ( pxBuffer == NULL ) || ( ucNumNetworks == 0 ) )
    {
        WDRV_DBG_ERROR_MESSAGE( ( "\r\nInvalid scan parameters\r\n" ) );

        return eWiFiFailure;
    }

    /*Check if scan already in progress*/
    if( WDRV_EXT_ScanIsInProgress() )
    {
        WDRV_DBG_ERROR_MESSAGE( ( "\r\nScan already in progress\r\n" ) );

        return eWiFiFailure;
    }

    /*Set scan in progress flag*/
    WDRV_EXT_ScanInProgressSet();

    /*Trigger scan start command and wait for scan done*/
    ret = WDRV_EXT_CmdScanStart();

    if( ret != WDRV_SUCCESS )
    {
        WDRV_DBG_ERROR_MESSAGE( ( "\r\nFailed to start scan\r\n" ) );

        return eWiFiFailure;
    }

    /*Get number of scan results*/
    WDRV_EXT_CmdScanGet( &num_scan_results );

    if( num_scan_results == 0 )
    {
        WDRV_DBG_ERROR_MESSAGE( ( "\r\nNo scan result!\r\n" ) );

        return eWiFiFailure;
    }

    do
    {
        /*Get scan result*/
        WDRV_EXT_ScanResultGet( idx, &scanResult );

        /*Fill in scan result buffer*/
        m2m_memcpy( ( uint8_t * ) pxBuffer[ idx ].cSSID, scanResult.ssid, scanResult.ssidLen );

        if( scanResult.ssidLen < wificonfigMAX_SSID_LEN )
        {
            pxBuffer[ idx ].cSSID[ scanResult.ssidLen ] = '\0';
        }

        m2m_memcpy( pxBuffer[ idx ].ucBSSID, scanResult.bssid, wificonfigMAX_BSSID_LEN );
        pxBuffer[ idx ].xSecurity = scanResult.apConfig;
        pxBuffer[ idx ].cRSSI = scanResult.rssi;
        pxBuffer[ idx ].cChannel = ( int8_t ) scanResult.channel;
        pxBuffer[ idx ].ucHidden = 0;
    } while( ++idx < ucNumNetworks );

    WDRV_EXT_ScanDoneSet();

    return eWiFiSuccess;
}


/**
 * @brief Check if the Wi-Fi is connected.
 *
 * @return pdTRUE if the link is up, pdFalse otherwise.
 */
BaseType_t WIFI_IsConnected( void )
{
    BaseType_t xIsConnected = pdFALSE;

    if( isLinkUp() )
    {
        xIsConnected = pdTRUE;
    }

    return xIsConnected;
}

/*-----------------------------------------------------------*/

void vApplicationPingReplyHook( ePingReplyStatus_t eStatus,
                                uint16_t usIdentifier )
{
    /*handle ping reply. */
    switch( eStatus )
    {
        case eSuccess:
            /* A valid ping reply has been received. */
            xQueueSend( xPingReplyQueue, &usIdentifier, pdMS_TO_TICKS( 10 ) );
            break;

        case eInvalidChecksum:
        case eInvalidData:
            /* A reply was received but it was not valid. */
            break;
    }
}

/**
 * @brief Ping an IP address in the network.
 *
 * @param[in] IP Address to ping.
 * @param[in] Number of times to ping
 * @param[in] Interval in mili seconds for ping operation
 *
 * @return eWiFiSuccess if disconnected, failure code otherwise.
 */
WIFIReturnCode_t WIFI_Ping( uint8_t * pucIPAddr,
                            uint16_t usCount,
                            uint32_t ulIntervalMS )
{
    uint32_t ulIPAddress = 0;
    uint16_t usPingSeqNum = pdFAIL;
    uint16_t usPingReplySeqNum = pdFAIL;
    uint32_t ulIndex = 0;

    if( ( NULL == pucIPAddr ) || ( 0 == usCount ) )
    {
        return eWiFiFailure;
    }

    waiting_task = xTaskGetCurrentTaskHandle();

    /*If link is not up, return failure */
    if( isLinkUp() )
    {
        ulIPAddress = FreeRTOS_inet_addr( ( const char * ) pucIPAddr );

        for( ulIndex = 0; ulIndex < usCount; ulIndex++ )
        {
            usPingSeqNum = FreeRTOS_SendPingRequest( ulIPAddress, WIFI_PING_PKT_SIZE, pdMS_TO_TICKS( ulIntervalMS ) );
            WDRV_DBG_ERROR_MESSAGE( ( "\r\nSending Ping request %d\r\n", usPingSeqNum ) );

            if( usPingSeqNum == pdFAIL )
            {
                /* ping failed */
                WDRV_DBG_ERROR_MESSAGE( ( "\r\nSending Ping request failed\r\n" ) );

                return eWiFiFailure;
            }
            else
            {
                /* The ping was sent.  Wait for a reply.  */
                if( xQueueReceive( xPingReplyQueue,
                                   &usPingReplySeqNum,
                                   pdMS_TO_TICKS( ulIntervalMS ) ) == pdPASS )
                {
                    /* A ping reply was received.  Was it a reply to the ping just sent? */
                    if( usPingSeqNum == usPingReplySeqNum )
                    {
                        /* This was a reply to the request just sent. */
                        WDRV_DBG_INFORM_MESSAGE( ( "Ping Reply [%d] out of [%d] received\r\n", ulIndex, usCount ) );
                    }
                }
            }
        }

        /* all ping were sent*/
        return eWiFiSuccess;
    }

    WDRV_DBG_ERROR_MESSAGE( ( "\r\nLink is not yet ready; cannot send ping request\r\n" ) );

    return eWiFiFailure;
}


/*Get network parameters flash section bitmap*/
uint64_t network_param_get_bitmap()
{
    return g_network_param_bitmap;
}

/*Set network parameters flash section bitmap*/
void network_param_set_bitmap( uint64_t val )
{
    g_network_param_bitmap = val;
}

/*Get network parameters flash section nearest free entry*/
uint8_t network_param_get_free_index()
{
    uint8_t idx;
    uint64_t bitmap = g_network_param_bitmap;

    for( idx = 0; idx < WIFI_MAX_NETWORK_PROFILES; idx++ )
    {
        if( ( bitmap & ( uint64_t ) 0x1 ) == 0 )
        {
            break;
        }

        bitmap = bitmap >> 1;
    }

    return idx;
}

/*Set network parameters flash section entry as free*/
void network_param_set_free_index( uint8_t u8Idx )
{
    if( u8Idx < WIFI_MAX_NETWORK_PROFILES )
    {
        g_network_param_bitmap &= ~( ( uint64_t ) 0x1 << u8Idx );
    }
}

/*Set network parameters flash section entry as busy*/
void network_param_set_busy_index( uint8_t u8Idx )
{
    if( u8Idx < WIFI_MAX_NETWORK_PROFILES )
    {
        g_network_param_bitmap |= ( uint64_t ) 0x1 << u8Idx;
    }
}

/*Get network parameters flash section entry as busy*/
uint8_t network_param_get_busy_index( uint8_t u8Idx )
{
    if( ( g_network_param_bitmap & ( ( uint64_t ) 0x1 << u8Idx ) ) == 0x1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief Wi-Fi Add Network profile.
 *
 * Adds Wi-Fi network to network list in non-volatile memory
 *
 * @param[in] pxNetworkProfile - network profile parameters
 * @param[out] pusIndex - network profile index
 *
 * @return Profile stored index on success, or negative error code on failure.
 */
WIFIReturnCode_t WIFI_NetworkAdd( const WIFINetworkProfile_t * const pxNetworkProfile,
                                  uint16_t * pusIndex )
{
    #ifdef ENABLE_NETWORK_STORAGE
        uint8_t * pSrcBuf = pxNetworkProfile;
        uint8_t * pDestBuf;
        uint16_t u8Idx = 0;
        uint16_t u8NumOfWrites = 0;
        uint8_t u8NetworkIdx = 0;

        /*Check input parameters*/
        if( ( pxNetworkProfile == NULL ) || ( pusIndex == NULL ) )
        {
            WDRV_DBG_ERROR_MESSAGE( ( "\r\nSave network to flash failed\r\n" ) );

            return eWiFiFailure;
        }

        /*Get network index*/
        u8NetworkIdx = network_param_get_free_index();

        if( u8NetworkIdx == WIFI_MAX_NETWORK_PROFILES )
        {
            WDRV_DBG_ERROR_MESSAGE( ( "\r\nNo more space for new network %d\r\n", WIFI_MAX_NETWORK_PROFILES ) );

            return eWiFiFailure;
        }

        /*Get copy from network parameters flash section*/
        memcpy( g_network_param_copy, p_network_param, WIFI_NETWORK_PARAM_SECTION_SIZE );

        /*Erase network parameters flash section*/
        taskENTER_CRITICAL();
        DRV_FLASH_ErasePage( WDRV_FLASH_HANDLE, ( uint32_t ) p_network_param );

        while( !DRV_FLASH_IsBusy( WDRV_FLASH_HANDLE ) )
        {
        }

        taskEXIT_CRITICAL();

        /*Prepare write buffer*/
        pDestBuf = g_network_param_copy + ( u8NetworkIdx * WIFI_NETWORK_PARAM_ENTRY_SIZE );
        memcpy( pDestBuf, pSrcBuf, WIFI_NETWORK_PARAM_ENTRY_SIZE );

        /*Write network parameters entire section (page)*/
        pDestBuf = ( uint8_t * ) p_network_param;
        pSrcBuf = g_network_param_copy;
        u8NumOfWrites = WIFI_NETWORK_PARAM_SECTION_SIZE / 16;
        WDRV_DBG_INFORM_MESSAGE( ( "\r\nWriting network parameters - index %d\r\n", u8NetworkIdx ) );

        for( u8Idx = 0; u8Idx < u8NumOfWrites; u8Idx++ )
        {
            taskENTER_CRITICAL();
            DRV_FLASH_WriteQuadWord( WDRV_FLASH_HANDLE, ( uint32_t ) pDestBuf, ( uint32_t * ) pSrcBuf );

            while( DRV_FLASH_IsBusy( WDRV_FLASH_HANDLE ) )
            {
            }

            taskEXIT_CRITICAL();
            pDestBuf += 16;
            pSrcBuf += 16;
        }

        /*verify write operation*/
        pDestBuf = ( uint8_t * ) p_network_param;
        pSrcBuf = g_network_param_copy;

        if( memcmp( pDestBuf, pSrcBuf, WIFI_NETWORK_PARAM_SECTION_SIZE ) == 0 )
        {
            WDRV_DBG_INFORM_MESSAGE( ( "\r\nWrite network parameters done\r\n" ) );

            /*Network write success; update network parameters bitmap*/
            network_param_set_busy_index( u8NetworkIdx );

            *pusIndex = ( int16_t ) u8NetworkIdx;

            return eWiFiSuccess;
        }

        WDRV_DBG_ERROR_MESSAGE( ( "\r\nWrite network parameters failed\r\n" ) );

        return eWiFiFailure;
    #else /* ifdef ENABLE_NETWORK_STORAGE */
        return eWiFiNotSupported;
    #endif /* ifdef ENABLE_NETWORK_STORAGE */
}

/**
 * @brief Wi-Fi Get Network profile .
 *
 * Gets Wi-Fi network parameters at given index from network list in Non volatile memory
 *
 * @param[out] pxNetworkProfile - pointer to return network profile parameters
 * @param[in] usIndex - Index of the network profile,
 *                       must be between 0 to wificonfigMAX_NETWORK_PROFILES
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise.
 */
WIFIReturnCode_t WIFI_NetworkGet( WIFINetworkProfile_t * pxNetworkProfile,
                                  uint16_t usIndex )
{
    #ifdef ENABLE_NETWORK_STORAGE
        /*Check input parameters*/
        if( ( usIndex >= WIFI_MAX_NETWORK_PROFILES ) ||
            ( pxNetworkProfile == NULL ) ||
            ( network_param_get_busy_index( usIndex ) == NULL ) )
        {
            WDRV_DBG_ERROR_MESSAGE( ( "\r\nGet network from flash failed\r\n" ) );

            return eWiFiFailure;
        }

        /*Check if address is in range*/
        uint8_t * pSrcBuf = ( uint8_t * ) p_network_param + ( usIndex * WIFI_NETWORK_PARAM_ENTRY_SIZE );

        if( pSrcBuf >
            ( p_network_param + WIFI_NETWORK_PARAM_SECTION_SIZE - WIFI_NETWORK_PARAM_ENTRY_SIZE ) )
        {
            WDRV_DBG_ERROR_MESSAGE( ( "\r\nAddress is out of network parameters section range\r\n" ) );

            return eWiFiFailure;
        }

        /*Copy network parameters buffer*/
        memcpy( ( uint8_t * ) pxNetworkProfile, pSrcBuf, WIFI_NETWORK_PARAM_ENTRY_SIZE );

        return eWiFiSuccess;
    #else /* ifdef ENABLE_NETWORK_STORAGE */
        return eWiFiNotSupported;
    #endif /* ifdef ENABLE_NETWORK_STORAGE */
}

/**
 * @brief Wi-Fi Delete Network profile .
 *
 * Deletes Wi-Fi network from network list at given index in Non volatile memory
 *
 * @param[in] usIndex - Index of the network profile, must be between 0 to wificonfigMAX_NETWORK_PROFILES
 *                      wificonfigMAX_NETWORK_PROFILES as index will delete all network profiles
 *
 * @return eWiFiSuccess if everything succeeds, failure code otherwise.
 */
WIFIReturnCode_t WIFI_NetworkDelete( uint16_t usIndex )
{
    #ifdef ENABLE_NETWORK_STORAGE
        /*Check input parameters*/
        if( usIndex > WIFI_MAX_NETWORK_PROFILES )
        {
            WDRV_DBG_ERROR_MESSAGE( ( "\r\nOut of range index\r\n" ) );

            return eWiFiFailure;
        }

        /*Update network parameters bitmap*/
        if( usIndex == WIFI_MAX_NETWORK_PROFILES )
        {
            network_param_set_bitmap( 0 );
        }
        else
        {
            network_param_set_free_index( usIndex );
        }

        return eWiFiSuccess;
    #else /* ifdef ENABLE_NETWORK_STORAGE */
        return eWiFiNotSupported;
    #endif /* ifdef ENABLE_NETWORK_STORAGE */
}

WIFIReturnCode_t WIFI_RegisterNetworkStateChangeEventCallback( IotNetworkStateChangeEventCallback_t xCallback  )
{
    /** Needs to implement dispatching network state change events **/
    return eWiFiNotSupported;
}

#endif
#endif /*PIC32_USE_ETHERNET*/
