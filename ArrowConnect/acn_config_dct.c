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

#include "./acn_config.h"
#include "wiced_framework.h"


/******************************************************
 *               Variable Definitions
 ******************************************************/
DEFINE_APP_DCT(acn_config_dct_t)
{
    .is_configured = WICED_FALSE,
    .device_name = DEFAULT_DEVICE_NAME
};
