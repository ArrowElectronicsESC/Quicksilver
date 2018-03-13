/* Copyright (c) 2017 Arrow Electronics, Inc.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Apache License 2.0
 * which accompanies this distribution, and is available at
 * http://apache.org/licenses/LICENSE-2.0
 * Contributors: Arrow Electronics, Inc.
 */

#include "wiced.h"
#include "ap_config.h"
#include "arrow/storage.h"
#include <arrow/utf8.h>
#include <debug.h>
#include <sys/type.h>

/* ------------- Read thing-name from DCT ------------- */
//    ret = wiced_dct_read_lock( (void**) &aws_app_dct, WICED_FALSE, DCT_APP_SECTION, 0, sizeof( aws_config_dct_t ) );
//    if ( ret != WICED_SUCCESS )
//    {
//        WPRINT_APP_INFO(("Unable to lock DCT to read certificate\n"));
//        return ret;
//    }
//
//    strncpy(app_info->thing_name, aws_app_dct->thing_name, sizeof(app_info->thing_name)-1);
//    snprintf(app_info->shadow_state_topic, sizeof(app_info->shadow_state_topic), THING_STATE_TOPIC_STR_BUILDER, app_info->thing_name);
//    snprintf(app_info->shadow_delta_topic, sizeof(app_info->shadow_delta_topic), THING_DELTA_TOPIC_STR_BUILDER, app_info->thing_name);
//
//    printf("Thing Name: %s\n", aws_app_dct->thing_name);
//    printf("Shadow State Topic: %s\n", app_info->shadow_state_topic);
//    printf("Shadow Delta Topic: %s\n", app_info->shadow_delta_topic);
//
//    /* Finished accessing the AWS APP DCT */
//    ret = wiced_dct_read_unlock( aws_app_dct, WICED_FALSE );
//    if ( ret != WICED_SUCCESS )
//    {
//        WPRINT_APP_INFO(( "DCT Read Unlock Failed. Error = [%d]\n", ret ));
//        return ret;
//    }
    /* ---------------------------------------------------- */

int restore_gateway_info(arrow_gateway_t *gateway)
{
    wiced_result_t           result;
    aws_config_dct_t*        aws_dct_ptr;

    /* Read the Application DCT to get the Gateway HID */
    result = wiced_dct_read_lock( (void**) &aws_dct_ptr, WICED_FALSE, DCT_APP_SECTION, 0, sizeof( aws_config_dct_t ) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Unable to lock DCT to read certificate\n"));
        return WICED_ERROR;
    }
    result = wiced_dct_read_unlock( aws_dct_ptr, WICED_FALSE );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "DCT Read Unlock Failed. Error = [%d]\n", result ));
        return WICED_ERROR;
    }

    /* Safety check the Gateway HID */
    if(utf8check(aws_dct_ptr->gateway_hid) && strlen(aws_dct_ptr->gateway_hid) > 0)
    {
//        P_COPY(gateway->hid, p_const(aws_dct_ptr->gateway_hid) ); // FIXME weak pointer
        strncpy(gateway->hid.value, aws_dct_ptr->gateway_hid, 64);
        WPRINT_APP_INFO(("Existing Gateway, Load HID: %s\n", gateway->hid.value));
        return 0;
    }
    else
    {
        // New Gateway
        WPRINT_APP_INFO(("New Gateway\n"));
        return -1;
    }
}

void save_gateway_info(const arrow_gateway_t *gateway)
{
    wiced_result_t           result;
    aws_config_dct_t*        aws_dct_ptr;

    result = wiced_dct_read_lock( (void**) &aws_dct_ptr, WICED_TRUE, DCT_APP_SECTION, 0, sizeof( aws_config_dct_t ) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Unable to lock DCT to read certificate\n"));
    }

    strcpy(aws_dct_ptr->gateway_hid, P_VALUE(gateway->hid));
    WPRINT_APP_INFO(("Storing New Gateway HID: %s\n", aws_dct_ptr->gateway_hid));

    wiced_dct_write( (const void*)aws_dct_ptr, DCT_APP_SECTION, 0, sizeof(aws_config_dct_t) );

    /* Finished accessing the AWS APP DCT */
    result = wiced_dct_read_unlock( aws_dct_ptr, WICED_TRUE );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "DCT Read Unlock Failed. Error = [%d]\n", result ));
    }

    return;
}

int restore_device_info(arrow_device_t *device)
{
    wiced_result_t           result;
    aws_config_dct_t*        aws_dct_ptr;

    /* Read the Application DCT to get the Device HID */
    result = wiced_dct_read_lock( (void**) &aws_dct_ptr, WICED_FALSE, DCT_APP_SECTION, 0, sizeof( aws_config_dct_t ) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Unable to lock DCT to read certificate\n"));
        return WICED_ERROR;
    }
    result = wiced_dct_read_unlock( aws_dct_ptr, WICED_FALSE );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "DCT Read Unlock Failed. Error = [%d]\n", result ));
        return WICED_ERROR;
    }

    /* Safety check the Device HID */
    if(utf8check(aws_dct_ptr->device_hid) && strlen(aws_dct_ptr->device_hid) > 0)
    {
        P_COPY(device->hid, p_const(aws_dct_ptr->device_hid) ); // FIXME weak pointer
        WPRINT_APP_INFO(("Existing Device, Load HID: %s\n", device->hid.value));
        return 0;
    }
    else
    {
        // New Gateway
        WPRINT_APP_INFO(("New Device\n"));
        return -1;
    }
}

void save_device_info(arrow_device_t *device)
{
    wiced_result_t           result;
    aws_config_dct_t*        aws_dct_ptr;

    result = wiced_dct_read_lock( (void**) &aws_dct_ptr, WICED_TRUE, DCT_APP_SECTION, 0, sizeof( aws_config_dct_t ) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Unable to lock DCT to read certificate\n"));
    }

    strcpy(aws_dct_ptr->device_hid, P_VALUE(device->hid));
    WPRINT_APP_INFO(("Storing New Device HID: %s\n", aws_dct_ptr->device_hid));

    wiced_dct_write( (const void*)aws_dct_ptr, DCT_APP_SECTION, 0, sizeof(aws_config_dct_t) );

    /* Finished accessing the AWS APP DCT */
    result = wiced_dct_read_unlock( aws_dct_ptr, WICED_TRUE );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "DCT Read Unlock Failed. Error = [%d]\n", result ));
    }

    return;
}

void save_wifi_setting(const char *ssid, const char *pass, int sec) {
//  if (FLASH_update((uint32_t)mem.ssid, ssid, strlen(ssid)+1) < 0) {
//    DBG("Failed updating the wifi configuration in FLASH");
//  }
//  if (FLASH_update((uint32_t)mem.ssid, ssid, strlen(pass)+1) < 0) {
//    DBG("Failed updating the wifi configuration in FLASH");
//  }
//  if (FLASH_update((uint32_t)&mem.sec, &sec, sizeof(sec)) < 0) {
//    DBG("Failed updating the wifi configuration in FLASH");
//  }
}

int restore_wifi_setting(char *ssid, char *pass, int *sec) {
#if defined(DEFAULT_WIFI_SSID) \
  && defined(DEFAULT_WIFI_PASS) \
  && defined(DEFAULT_WIFI_SEC)
  strcpy(ssid, DEFAULT_WIFI_SSID);
  strcpy(pass, DEFAULT_WIFI_PASS);
  *sec = DEFAULT_WIFI_SEC;
#else
//  if ( mem.magic != FLASH_MAGIC_NUMBER ) {
//    FLASH_unlock_erase((uint32_t)&mem, sizeof(mem));
//    return -1;
//  }
//  if ( !utf8check(mem.ssid) || !strlen(mem.ssid) ) {
//    return -1;
//  }
//  strcpy(ssid, mem.ssid);
//  DBG("--- flash load %s", mem.ssid);
//  if ( !utf8check(mem.pass) || !strlen(mem.ssid) ) {
//    return -1;
//  }
//  strcpy(pass, mem.pass);
//  DBG("--- flash load %s", mem.pass);
//  *sec = mem.sec;
#endif
  return 0;
}
