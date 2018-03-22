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
 *
 */

#include "./ap_config.h"
#include "wiced_framework.h"
#include "ota2_ArrowConnect_dct.h"


/******************************************************
 *               Variable Definitions
 ******************************************************/
DEFINE_APP_DCT(ota2_dct_t)
{
    .is_configured = WICED_FALSE,
    .device_name = DEFAULT_DEVICE_NAME,
    .reboot_count       = 0,
    .ota2_major_version = APP_VERSION_FOR_OTA2_MAJOR,
    .ota2_minor_version = APP_VERSION_FOR_OTA2_MINOR,
};
