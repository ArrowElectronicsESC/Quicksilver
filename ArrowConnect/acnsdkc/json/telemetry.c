/* Copyright (c) 2017 Arrow Electronics, Inc.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Apache License 2.0
 * which accompanies this distribution, and is available at
 * http://apache.org/licenses/LICENSE-2.0
 * Contributors: Arrow Electronics, Inc.
 */

#include "json/telemetry.h"
#include <config.h>
#include <json/json.h>
#include "quicksilver.h"

char *telemetry_serialize(arrow_device_t *device, void *d) {
    quicksilver_data *data = (quicksilver_data *)d;
  JsonNode *_node = json_mkobject();
#ifdef __IBM__
  JsonNode *_data = json_mkobject();
  json_append_member(_data, TELEMETRY_TEMPERATURE, json_mknumber(data->temperature));
  json_append_member(_data, TELEMETRY_HUMIDITY, json_mknumber(data->humidity));
  json_append_member(_data, TELEMETRY_ACCELEROMETER_X, json_mknumber(data->accelerometer.x));
  json_append_member(_data, TELEMETRY_ACCELEROMETER_Y, json_mknumber(data->accelerometer.y));
  json_append_member(_data, TELEMETRY_ACCELEROMETER_Z, json_mknumber(data->accelerometer.z));
  json_append_member(_node, "d", _data);
#else
  json_append_member(_node, TELEMETRY_DEVICE_HID, json_mkstring(P_VALUE(device->hid)));
  json_append_member(_node, TELEMETRY_TEMPERATURE, json_mknumber(data->temperature));
  json_append_member(_node, TELEMETRY_HUMIDITY, json_mknumber(data->humidity));
  json_append_member(_node, TELEMETRY_ACCELEROMETER_X, json_mknumber(data->accelerometer.x));
  json_append_member(_node, TELEMETRY_ACCELEROMETER_Y, json_mknumber(data->accelerometer.y));
  json_append_member(_node, TELEMETRY_ACCELEROMETER_Z, json_mknumber(data->accelerometer.z));
  json_append_member(_node, "i|ledBrightness", json_mknumber(data->led.brightness));
  json_append_member(_node, "i|ledRed", json_mknumber(data->led.red));
  json_append_member(_node, "i|ledGreen", json_mknumber(data->led.green));
  json_append_member(_node, "i|ledBlue", json_mknumber(data->led.blue));
#endif
  char *tmp = json_encode(_node);
  json_delete(_node);
  return tmp;
}
