/*
 * Broadcom Proprietary and Confidential. Copyright 2016 Broadcom
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

/** @file
 * This file contains common APP and MQTT functionalities which can be used across Arrow Connect (ACN) applications
 */

#include "./acn_common.h"

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t acn_app_init( void )
{
    wiced_result_t ret = WICED_SUCCESS;

    /* Disable roaming to other access points */
    wiced_wifi_set_roam_trigger( -99 ); /* -99dBm ie. extremely low signal level */

    WPRINT_APP_INFO((" Please wait, connecting to network...\n"));

    /* Configure the device */
    ret = acn_configure_device();
    if ( ret != WICED_ALREADY_INITIALIZED )
    {
        WPRINT_APP_INFO(("Restarting the device...\n"));
        wiced_framework_reboot();
        return ret;
    }

    /* Bringup the network interface */
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    return WICED_SUCCESS;
}
