/*
 * Broadcom Proprietary and Confidential. Copyright 2016 Broadcom
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

#include "wiced.h"
#include "arrow/gateway.h"
#include "arrow/device.h"

#pragma once

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                      Macros
 ******************************************************/
#define DEFAULT_DEVICE_NAME   DEVICE_NAME

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
//typedef struct aws_config_dct_s
//{
//    wiced_bool_t  is_wifi_configured;
//    wiced_bool_t  is_gateway_configured;
//    wiced_bool_t  is_device_configured;
//    char          device_name[32];
//    arrow_gateway_t* gateway_info;
//    arrow_device_t* device_info;
//} aws_config_dct_t;

typedef struct aws_config_dct_s
{
    wiced_bool_t  is_configured;
    char          device_name[32];
    char          gateway_hid[64];
    char          device_hid[64];
} aws_config_dct_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
wiced_result_t aws_configure_device(void);


#ifdef __cplusplus
} /* extern "C" */
#endif
