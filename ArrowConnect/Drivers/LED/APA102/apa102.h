#ifndef __APA102_DRIVER__H
#define __APA102_DRIVER__H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "debug.h"

/** @addtogroup apa102
 * @{
 */
  typedef struct apa102_color_s
  {
      uint8_t brightness;
      uint8_t red;
      uint8_t green;
      uint8_t blue;
  } apa102_color_t;

/** @defgroup lis2dh12_interface
  * @{
  */

typedef int32_t (*apa102_pin_set)(void *, uint32_t);
typedef int32_t (*apa102_pin_clear) (void *, uint32_t);
typedef int32_t (*apa102_delay_ms) (void *, uint32_t);

typedef struct {
  /** Component mandatory fields **/
    uint32_t clk_in_pin;
    uint32_t data_in_pin;
    apa102_pin_set  pin_set;
    apa102_pin_clear   pin_clear;
    apa102_delay_ms delay_ms;
  /** Customizable optional pointer **/
  void *handle;
} apa102_ctx_t;

/**
  * @}
  */

int32_t apa102_init(apa102_ctx_t *ctx);

void apa102_color_get(apa102_color_t *pColor);

int32_t apa102_led_off(apa102_ctx_t *ctx);

int32_t apa102_led_color_set(apa102_ctx_t *ctx, apa102_color_t color);

int32_t apa102_led_rainbow_sequence(apa102_ctx_t *ctx);

int32_t apa102_led_ramp_sequence(apa102_ctx_t *ctx);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__APA102_DRIVER__H */
