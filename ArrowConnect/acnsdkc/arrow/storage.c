/* Copyright (c) 2017 Arrow Electronics, Inc.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Apache License 2.0
 * which accompanies this distribution, and is available at
 * http://apache.org/licenses/LICENSE-2.0
 * Contributors: Arrow Electronics, Inc.
 */

#include "wiced.h"
#include "acn_config.h"
#include "arrow/storage.h"
#include <arrow/utf8.h>
#include <debug.h>
#include <sys/type.h>

int restore_gateway_info(arrow_gateway_t *gateway)
{
    wiced_result_t           result;
    acn_config_dct_t*        acn_dct_ptr;

    /* Read the Application DCT to get the Gateway HID */
    result = wiced_dct_read_lock( (void**) &acn_dct_ptr, WICED_FALSE, DCT_APP_SECTION, 0, sizeof( acn_config_dct_t ) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Unable to lock DCT to read certificate\n"));
        return WICED_ERROR;
    }

    P_COPY(gateway->hid, p_stack(acn_dct_ptr->gateway_hid));

    result = wiced_dct_read_unlock( acn_dct_ptr, WICED_FALSE );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "DCT Read Unlock Failed. Error = [%d]\n", result ));
        return WICED_ERROR;
    }

    /* Safety check the Gateway HID */
    if(utf8check(acn_dct_ptr->gateway_hid) && strlen(acn_dct_ptr->gateway_hid) > 0)
    {
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
    acn_config_dct_t*        acn_dct_ptr;

    result = wiced_dct_read_lock( (void**) &acn_dct_ptr, WICED_TRUE, DCT_APP_SECTION, 0, sizeof( acn_config_dct_t ) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Unable to lock DCT to read certificate\n"));
    }

    strcpy(acn_dct_ptr->gateway_hid, P_VALUE(gateway->hid));
    WPRINT_APP_INFO(("Storing New Gateway HID: %s\n", acn_dct_ptr->gateway_hid));

    wiced_dct_write( (const void*)acn_dct_ptr, DCT_APP_SECTION, 0, sizeof(acn_config_dct_t) );

    /* Finished accessing the AWS APP DCT */
    result = wiced_dct_read_unlock( acn_dct_ptr, WICED_TRUE );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "DCT Read Unlock Failed. Error = [%d]\n", result ));
    }

    return;
}

int restore_device_info(arrow_device_t *device)
{
    wiced_result_t           result;
    acn_config_dct_t*        acn_dct_ptr;

    /* Read the Application DCT to get the Device HID */
    result = wiced_dct_read_lock( (void**) &acn_dct_ptr, WICED_FALSE, DCT_APP_SECTION, 0, sizeof( acn_config_dct_t ) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Unable to lock DCT to read certificate\n"));
        return WICED_ERROR;
    }

    P_COPY(device->hid, p_stack(acn_dct_ptr->device_hid));
#if defined(__IBM__)
    {
        P_COPY(device->eid, p_stack(acn_dct_ptr->device_eid));
    }
#endif

    result = wiced_dct_read_unlock( acn_dct_ptr, WICED_FALSE );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "DCT Read Unlock Failed. Error = [%d]\n", result ));
        return WICED_ERROR;
    }

    /* Safety check the Device HID */
    if(utf8check(acn_dct_ptr->device_hid) && (strlen(acn_dct_ptr->device_hid) > 0))
    {
        WPRINT_APP_INFO(("Existing Device, Load HID: %s\n", device->hid.value));
        return 0;
    }
#if defined (__IBM__)
    /* Safety check the Device HID */
    if(utf8check(acn_dct_ptr->device_hid) && (strlen(acn_dct_ptr->device_hid) > 0))
    {
        WPRINT_APP_INFO(("Existing IBM Device, Load EID: %s\n", device->hid.value));
        return 0;
    }
#endif

    // New Gateway
    WPRINT_APP_INFO(("New Device\n"));
    return -1;
}

void save_device_info(arrow_device_t *device)
{
    wiced_result_t           result;
    acn_config_dct_t*        acn_dct_ptr;

    result = wiced_dct_read_lock( (void**) &acn_dct_ptr, WICED_TRUE, DCT_APP_SECTION, 0, sizeof( acn_config_dct_t ) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Unable to lock DCT to read certificate\n"));
    }

    strcpy(acn_dct_ptr->device_hid, P_VALUE(device->hid));
    WPRINT_APP_INFO(("Storing New Device HID: %s\n", acn_dct_ptr->device_hid));

#if defined(__IBM__)
    strcpy(acn_dct_ptr->device_eid, P_VALUE(device->eid));
    WPRINT_APP_INFO(("Storing New IBM Device EID: %s\n", acn_dct_ptr->device_eid));
#endif

    wiced_dct_write( (const void*)acn_dct_ptr, DCT_APP_SECTION, 0, sizeof(acn_config_dct_t) );

    /* Finished accessing the AWS APP DCT */
    result = wiced_dct_read_unlock( acn_dct_ptr, WICED_TRUE );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "DCT Read Unlock Failed. Error = [%d]\n", result ));
    }

    return;
}
