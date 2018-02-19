/* Copyright (c) 2017 Arrow Electronics, Inc.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Apache License 2.0
 * which accompanies this distribution, and is available at
 * http://apache.org/licenses/LICENSE-2.0
 * Contributors: Arrow Electronics, Inc.
 */

#include "arrow/state.h"
#include <arrow/utf8.h>
#include <debug.h>
#include <sys/type.h>

int state_handler(char *str) {
    DBG("Override State Handler:%s", str);
    return 0;
}
