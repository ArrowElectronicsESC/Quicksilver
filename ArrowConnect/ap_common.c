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
 * This file contains common APP and MQTT functionalities which can be used across AWS IOT applications
 */

#include "./ap_common.h"

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t aws_app_init( void )
{
    wiced_result_t ret = WICED_SUCCESS;

    /* Disable roaming to other access points */
    wiced_wifi_set_roam_trigger( -99 ); /* -99dBm ie. extremely low signal level */

    WPRINT_APP_INFO((" Please wait, connecting to network...\n"));
    WPRINT_APP_INFO(("(To return to SSID console screen, hold USER switch for 5 seconds during RESET to clear DCT configuration)\n"));
    wiced_rtos_delay_milliseconds( 1000 );

    for ( int i = 0; i < 25; i++ )
    {
        wiced_gpio_output_high( WICED_LED1 );
        wiced_rtos_delay_milliseconds( 100 );
        wiced_gpio_output_low( WICED_LED1 );
        wiced_rtos_delay_milliseconds( 100 );

        if ( !wiced_gpio_input_get( WICED_BUTTON1 ) )
        {
            wiced_rtos_delay_milliseconds( 5000 );

            if ( !wiced_gpio_input_get( WICED_BUTTON1 ) )
            {
                aws_config_dct_t aws_dct =
                {
                    .is_configured = WICED_FALSE,
                    .device_name = DEFAULT_DEVICE_NAME
                };

                wiced_gpio_output_high( WICED_LED2 );
                WPRINT_APP_INFO(( "DCT clearing start\n" ));
                wiced_dct_write( &aws_dct, DCT_APP_SECTION, 0, sizeof( aws_config_dct_t ) );
                wiced_rtos_delay_milliseconds( 1000 );
                wiced_gpio_output_low( WICED_LED2 );
                WPRINT_APP_INFO(( "DCT clearing end\n" ));

                break;
            }
        }
    }

    /* Configure the device */
    ret = aws_configure_device();
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
