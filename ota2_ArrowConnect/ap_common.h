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
#include "wiced.h"
#include "resources.h"
#include "gpio_button.h"

#pragma once

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                      Macros
 ******************************************************/

#define AWS_IOT_HOST_NAME                   "amk6m51qrxr2u.iot.us-east-1.amazonaws.com"
#define MQTT_REQUEST_TIMEOUT                (5000)
#define MQTT_DELAY_IN_MILLISECONDS          (1000)
#define MQTT_MAX_RESOURCE_SIZE              (0x7fffffff)
#define THING_STATE_TOPIC_STR_BUILDER       "$aws/things/%s/shadow/update"
#define THING_DELTA_TOPIC_STR_BUILDER       "$aws/things/%s/shadow/update/delta"

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

typedef struct aws_app_info_s
{
    wiced_semaphore_t         msg_semaphore;
    wiced_semaphore_t         wake_semaphore;
    char                      thing_name[32];
    char                      shadow_state_topic[64];
    char                      shadow_delta_topic[64];
    char                      mqtt_client_id[64];
} aws_app_info_t;

/******************************************************
 *               Function Declarations
 ******************************************************/
wiced_result_t aws_app_init( void );

#ifdef __cplusplus
} /* extern "C" */
#endif
