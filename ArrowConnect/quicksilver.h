#include <sys/type.h>

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
