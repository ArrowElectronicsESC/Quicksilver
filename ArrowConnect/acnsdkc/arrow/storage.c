/* Copyright (c) 2017 Arrow Electronics, Inc.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Apache License 2.0
 * which accompanies this distribution, and is available at
 * http://apache.org/licenses/LICENSE-2.0
 * Contributors: Arrow Electronics, Inc.
 */

#include "arrow/storage.h"
#include <arrow/utf8.h>
#include <debug.h>
#include <sys/type.h>

//flash_mem_t mem __attribute__((section("UNINIT_FIXED_LOC")));

int restore_gateway_info(arrow_gateway_t *gateway) {
//  if ( mem.magic != (int) FLASH_MAGIC_NUMBER ) {
//    FLASH_unlock_erase((uint32_t)&mem, sizeof(mem));
//    return -1;
//  }
//  if ( utf8check(mem.gateway_hid) && strlen(mem.gateway_hid) ) {
    //property_copy(&gateway->hid, p_const(mem.gateway_hid));
//    DBG("--- flash load %s", mem.gateway_hid);
//    return 0;
//  }

    //gateway->hid.value = strdup("f40b47c27e980a60e173042f87ae03e1a9be3b7a");
    gateway->hid.value = strdup("38057c7e0af2e0357f6ff30f98d270684137d8c6");
    //sprintf(&gateway->hid.value, "f40b47c27e980a60e173042f87ae03e1a9be3b7a");
    gateway->hid.flags = is_dynamic;
    DBG("gateway hid: value - %s flag: %d\n",gateway->hid.value, gateway->hid.flags);
    //TODO(bman): Actually make this work good
  return 0;
}

void save_gateway_info(const arrow_gateway_t *gateway) {
//  if ( gateway && P_SIZE(gateway->hid) < 64 ) {
//    uint32_t magic = FLASH_MAGIC_NUMBER;
//    if (FLASH_update((uint32_t)&mem.magic, &magic, sizeof(magic)) < 0) {
//      DBG("Failed updating the wifi configuration in FLASH");
//    }
//    if (FLASH_update((uint32_t)mem.gateway_hid, P_VALUE(gateway->hid), P_SIZE(gateway->hid)+1) < 0) {
//      DBG("Failed updating the wifi configuration in FLASH");
//    }
//  }
}

int restore_device_info(arrow_device_t *device) {
//  if ( mem.magic != (int) FLASH_MAGIC_NUMBER ) {
//    FLASH_unlock_erase((uint32_t)&mem, sizeof(mem));
//    return -1;
//  }
//  if ( !utf8check(mem.device_hid) || !strlen(mem.device_hid) ) {
//    return -1;
//  }
//  property_copy(&device->hid, p_const(mem.device_hid));
//  DBG("--- flash load %s", mem.device_hid);
//#if defined(__IBM__)
//  if ( !utf8check(mem.device_eid) || !strlen(mem.device_eid) ) {
//    return -1;
//  }
//  property_copy(&device->eid, p_const(mem.device_eid));
//#endif
  return -1;
}

void save_device_info(arrow_device_t *device) {
//  if ( device ) {
//    if ( P_SIZE(device->hid) < 64 ) {
//      if (FLASH_update((uint32_t)mem.device_hid, P_VALUE(device->hid), P_SIZE(device->hid)+1) < 0) {
//        DBG("Failed updating the wifi configuration in FLASH");
//      }
//    }
//#if defined(__IBM__)
//    if ( P_SIZE(device->eid) < 64 ) {
//      if (FLASH_update((uint32_t)mem.device_eid, P_VALUE(device->eid), P_SIZE(device->eid)+1) < 0) {
//        DBG("Failed updating the wifi configuration in FLASH");
//      }
//    }
//#endif
//  }
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
