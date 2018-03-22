#ifndef ACN_SDK_C_PRIVATE_H_
#define ACN_SDK_C_PRIVATE_H_

//#define __IBM__                     /* Use IBM Watson for Data analysis*/

/* Application */                   /* TODO: Update these keys to the "Staging Area" keys for Quicksilver */
#define DEFAULT_API_KEY             "3dedf2dac00d395ffd4c67e4394f24bc932218aeaecae87a6fb80ec0de86a835"
#define DEFAULT_SECRET_KEY          "9Jzuizwk6f64PkOn8gzV11wL14yb+f8iaol3s1v4IJo="

/* gateway */
#define GATEWAY_UID_PREFIX          "QS"
#define GATEWAY_NAME                "QuicksilverGateway"
#define GATEWAY_OS                  "WICED_RTOS"

/* gateway firmware */
#define GATEWAY_SOFTWARE_NAME       "QsEval"
#define GATEWAY_SOFTWARE_VERSION    "1.0.0"

/* device */
#define DEVICE_NAME                 "QuicksilverDevice"
#define DEVICE_TYPE                 "QuicksilverEvalKit"
#define DEVICE_UID_PREFIX           "Qs-device"
#define DEVICE_UID_SUFFIX           "device"

#define TELEMETRY_DELAY             100

#endif
