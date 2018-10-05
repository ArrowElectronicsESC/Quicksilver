#include <sys/type.h>
#include "wiced.h"
#include "Drivers/Sensors/LIS2DH12/lis2dh12.h"
#include "Drivers/Sensors/HTS221/hts221.h"
#include "Drivers/LED/APA102/apa102.h"

typedef struct axis {
  int32_t x;
  int32_t y;
  int32_t z;
}axis_t;

typedef struct color_s{
    uint8_t brightness;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
}color_t;

typedef struct {
  float temperature;
  float humidity;
  axis_t accelerometer;
  color_t led;
} quicksilver_data;

wiced_result_t quicksilver_init(void);
wiced_result_t i2c_init(void);
wiced_result_t gpio_init(void);
wiced_result_t temperature_init( void );
wiced_result_t accelerometer_init( void );
wiced_result_t rgb_init( void );
int accelerometer_get(axis_t * axis_data);
int humidity_get(float * humidity);
int temperature_get(float * temperature);
int rgb_color_get(uint8_t * brightness, uint8_t * red, uint8_t * green, uint8_t * blue);
int rgb_color_set(uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue);
int rgb_brightness_set(uint8_t val);
int rgb_red_set(uint8_t val);
int rgb_green_set(uint8_t val);
int rgb_blue_set(uint8_t val);

