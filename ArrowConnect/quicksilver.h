#include <sys/type.h>
#include "Drivers/Sensors/LIS2DH12/lis2dh12.h"
#include "Drivers/Sensors/HTS221/hts221.h"
#include "Drivers/LED/APA102/apa102.h"

struct axis {
  int32_t x;
  int32_t y;
  int32_t z;
};

typedef struct {
  float temperature;
  float humidity;
  struct axis accelerometer;
} quicksilver_data;
