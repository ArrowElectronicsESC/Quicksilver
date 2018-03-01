#ifndef ACN_SDK_C_PRIVATE_H_
#define ACN_SDK_C_PRIVATE_H_

#define __IBM__                     /* Use IBM Watson for Data analysis*/

/* Application */                   /* TODO: Update these keys to the "Staging Area" keys for Quicksilver */
#define DEFAULT_API_KEY             "96462710f86616894ba92b8f8f046bd3259429ecea0dc65ae5be917cae2a6388"
#define DEFAULT_SECRET_KEY          "/l5jun1vAJ7KQFGP2d6HsuMfX0NP5KrCjQ9+oYaZpkM="

/* gateway */
#define GATEWAY_UID_PREFIX          "QS"
#define GATEWAY_NAME                "QuicksilverGateway"
#define GATEWAY_OS                  "WICED_RTOS"

/* gateway firmware */
#define GATEWAY_SOFTWARE_NAME       "QsEval"
#define GATEWAY_SOFTWARE_VERSION    "1.0"

/* device */
#define DEVICE_NAME                 "QuicksilverDevice"
#define DEVICE_TYPE                 "QuicksilverEvalKit"
#define DEVICE_UID_PREFIX           "Qs-device"
#define DEVICE_UID_SUFFIX           "device"

#endif
