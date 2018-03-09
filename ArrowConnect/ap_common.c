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

static aws_app_info_t *aws_app_info;
static wiced_ip_address_t broker_address;


/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t aws_app_init( aws_app_info_t *app_info )
{
    wiced_result_t ret = WICED_SUCCESS;
    aws_config_dct_t *aws_app_dct = NULL;

    aws_app_info = app_info;

    wiced_init( );

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
                    .thing_name = AWS_DEFAULT_THING_NAME
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

    wiced_hostname_lookup( AWS_IOT_HOST_NAME, &broker_address, 10000, WICED_STA_INTERFACE );

    WPRINT_APP_INFO(("[MQTT] Connecting to broker %u.%u.%u.%u ...\n\n", (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 24),
                    (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 16),
                    (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 8),
                    (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 0)));

    wiced_rtos_init_semaphore( &aws_app_info->msg_semaphore );
    wiced_rtos_init_semaphore( &aws_app_info->wake_semaphore );

    /* ------------- Read thing-name from DCT ------------- */
    ret = wiced_dct_read_lock( (void**) &aws_app_dct, WICED_FALSE, DCT_APP_SECTION, 0, sizeof( aws_config_dct_t ) );
    if ( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Unable to lock DCT to read certificate\n"));
        return ret;
    }

    strncpy(app_info->thing_name, aws_app_dct->thing_name, sizeof(app_info->thing_name)-1);
    snprintf(app_info->shadow_state_topic, sizeof(app_info->shadow_state_topic), THING_STATE_TOPIC_STR_BUILDER, app_info->thing_name);
    snprintf(app_info->shadow_delta_topic, sizeof(app_info->shadow_delta_topic), THING_DELTA_TOPIC_STR_BUILDER, app_info->thing_name);

    printf("Thing Name: %s\n", aws_app_dct->thing_name);
    printf("Shadow State Topic: %s\n", app_info->shadow_state_topic);
    printf("Shadow Delta Topic: %s\n", app_info->shadow_delta_topic);

    /* Finished accessing the AWS APP DCT */
    ret = wiced_dct_read_unlock( aws_app_dct, WICED_FALSE );
    if ( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "DCT Read Unlock Failed. Error = [%d]\n", ret ));
        return ret;
    }
    /* ---------------------------------------------------- */

    return ret;
}
