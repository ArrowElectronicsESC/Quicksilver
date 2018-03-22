#include "apa102.h"

static apa102_color_t led_color = {
        .brightness = 0,
        .red = 0,
        .green = 0,
        .blue = 0,
};

/**
  * @brief  Initialize the AP102 LED.
  *
  * @param  apa102_ctx_t *ctx: Interface definition
  *
  */
int32_t apa102_init(apa102_ctx_t *ctx)
{
    ctx->pin_set(ctx, ctx->data_in_pin);
    ctx->pin_set(ctx, ctx->clk_in_pin);

    return 0;
}

/**
  * @brief  Get the APA102 Color.
  *
  * @param  apa102_color_t *color: Pointer to an APA102 Color Structure
  *
  */
void apa102_color_get(apa102_color_t *pColor)
{
    pColor->brightness = (led_color.brightness & 0x1F);
    pColor->red = led_color.red;
    pColor->green = led_color.green;
    pColor->blue = led_color.blue;
}

/**
  * @brief  Turn LED Off.
  *
  * @param  apa102_ctx_t *ctx: Interface definition
  *
  */
int32_t apa102_led_off(apa102_ctx_t *ctx)
{
    led_color.brightness = 0;
    led_color.red = 0;
    led_color.green = 0;
    led_color.blue = 0;

    apa102_led_color_set(ctx, led_color);

    return 0;
}

void spiBitbang(apa102_ctx_t *ctx, uint8_t * data, uint16_t length)
{
    for(int i = 0; i < length; i++)
    {
        for ( int j = 0; j < 8; j++)
        {
            ctx->pin_clear(ctx, ctx->clk_in_pin);

            if ( data[i] & 0x80 )
            {
                ctx->pin_set(ctx, ctx->data_in_pin);
            }
            else
            {
                ctx->pin_clear(ctx, ctx->data_in_pin);
            }

            ctx->pin_set(ctx, ctx->clk_in_pin);

            data[i] = (data[i] << 1); // left shift by 1
        }
    }
}

/**
  * @brief  Set the color of the LED.
  *
  * @param  apa102_ctx_t *ctx: Interface definition
  * @param  apa102_color_t color: LED color
  *
  */
int32_t apa102_led_color_set(apa102_ctx_t *ctx, apa102_color_t color)
{
    uint8_t data_array[12];

    led_color.brightness = color.brightness < 0x1F ? (color.brightness | 0xE0): 0xFF;
    led_color.red = color.red;
    led_color.green = color.green;
    led_color.blue = color.blue;

    data_array[0] = 0x00;
    data_array[1] = 0x00;
    data_array[2] = 0x00;
    data_array[3] = 0x00;
    data_array[4] = led_color.brightness;
    data_array[5] = led_color.blue;
    data_array[6] = led_color.green;
    data_array[7] = led_color.red;
    data_array[8] = 0xFF;
    data_array[9] = 0xFF;
    data_array[10] = 0xFF;
    data_array[11] = 0xFF;

    spiBitbang(ctx, data_array, sizeof(data_array));

    return 0;
}

/**
  * @brief  Set the brightness of the LED.
  *
  * @param  apa102_ctx_t *ctx: Interface definition
  * @param  uint8_t value: LED brightness value
  *
  * @note   This function only changes the brightness of the LED
  *         and does not change the color values. Acceptable values 0 - 31,
  *         values above this limit will be forced to 31.
  */
int32_t apa102_led_brightness_set(apa102_ctx_t *ctx, uint8_t value)
{
    uint8_t data_array[12];

    led_color.brightness = value < 0x1F ? (value | 0xE0): 0xFF;

    data_array[0] = 0x00;
    data_array[1] = 0x00;
    data_array[2] = 0x00;
    data_array[3] = 0x00;
    data_array[4] = led_color.brightness;
    data_array[5] = led_color.blue;
    data_array[6] = led_color.green;
    data_array[7] = led_color.red;
    data_array[8] = 0xFF;
    data_array[9] = 0xFF;
    data_array[10] = 0xFF;
    data_array[11] = 0xFF;

    spiBitbang(ctx, data_array, sizeof(data_array));

    return 0;
}

/**
  * @brief  Set the Red value of the LED.
  *
  * @param  apa102_ctx_t *ctx: Interface definition
  * @param  uint8_t value: LED red value
  *
  * @note   This function only changes the red value of the LED
  *         and does not change the other color values or brightness.
  */
int32_t apa102_led_red_set(apa102_ctx_t *ctx, uint8_t value)
{
    uint8_t data_array[12];

    led_color.red = value;

    data_array[0] = 0x00;
    data_array[1] = 0x00;
    data_array[2] = 0x00;
    data_array[3] = 0x00;
    data_array[4] = led_color.brightness;
    data_array[5] = led_color.blue;
    data_array[6] = led_color.green;
    data_array[7] = led_color.red;
    data_array[8] = 0xFF;
    data_array[9] = 0xFF;
    data_array[10] = 0xFF;
    data_array[11] = 0xFF;

    spiBitbang(ctx, data_array, sizeof(data_array));

    return 0;
}

/**
  * @brief  Set the Green value of the LED.
  *
  * @param  apa102_ctx_t *ctx: Interface definition
  * @param  uint8_t value: LED green value
  *
  * @note   This function only changes the green value of the LED
  *         and does not change the other color values or brightness.
  */
int32_t apa102_led_green_set(apa102_ctx_t *ctx, uint8_t value)
{
    uint8_t data_array[12];

    led_color.green = value;

    data_array[0] = 0x00;
    data_array[1] = 0x00;
    data_array[2] = 0x00;
    data_array[3] = 0x00;
    data_array[4] = led_color.brightness;
    data_array[5] = led_color.blue;
    data_array[6] = led_color.green;
    data_array[7] = led_color.red;
    data_array[8] = 0xFF;
    data_array[9] = 0xFF;
    data_array[10] = 0xFF;
    data_array[11] = 0xFF;

    spiBitbang(ctx, data_array, sizeof(data_array));

    return 0;
}

/**
  * @brief  Set the Blue value of the LED.
  *
  * @param  apa102_ctx_t *ctx: Interface definition
  * @param  uint8_t value: LED blue value
  *
  * @note   This function only changes the blue value of the LED
  *         and does not change the other color values or brightness.
  */
int32_t apa102_led_blue_set(apa102_ctx_t *ctx, uint8_t value)
{
    uint8_t data_array[12];

    led_color.blue = value;

    data_array[0] = 0x00;
    data_array[1] = 0x00;
    data_array[2] = 0x00;
    data_array[3] = 0x00;
    data_array[4] = led_color.brightness;
    data_array[5] = led_color.blue;
    data_array[6] = led_color.green;
    data_array[7] = led_color.red;
    data_array[8] = 0xFF;
    data_array[9] = 0xFF;
    data_array[10] = 0xFF;
    data_array[11] = 0xFF;

    spiBitbang(ctx, data_array, sizeof(data_array));

    return 0;
}

/**
  * @brief  Set the color of the LED.
  *
  * @param  apa102_ctx_t *ctx: Interface definition
  * @param  apa102_color_t * sequence: LED color sequence
  * @param  uint32_t * size: Size of the array containing the color sequences
  *
  */
int32_t apa102_led_show_sequence(apa102_ctx_t *ctx, apa102_color_t * sequence, uint32_t size)
{
    for ( int i = 0; i < size; i++ )
    {
        led_color.brightness = (0xE0 | sequence[i].brightness);
        led_color.red = sequence[i].red;
        led_color.green = sequence[i].green;
        led_color.blue = sequence[i].blue;

        apa102_led_color_set(ctx, led_color);
        ctx->delay_ms(ctx, 500);
    }

    return 0;
}

/**
  * @brief  Display the LED Rainbow sequence.
  *
  * @param  apa102_ctx_t *ctx: Interface definition
  *
  */
int32_t apa102_led_rainbow_sequence(apa102_ctx_t *ctx)
{
    apa102_color_t Rainbow[8] = {
            {1,255,0,0},
            {1,255,110,0},
            {1,255,255,0},
            {1,0,255,0},
            {1,0,0,255},
            {1,0,255,255},
            {1,255,0,255},
            {1,255,255,255}
        };

    apa102_led_show_sequence(ctx, Rainbow, 8);

    return 0;
}

/**
  * @brief  @brief  Display the LED Ramp sequence.
  *
  * @param  apa102_ctx_t *ctx: Interface definition
  *
  */
int32_t apa102_led_ramp_sequence(apa102_ctx_t *ctx)
{
    apa102_color_t color =
    {
        .brightness = 0x01,
        .red = 0,
        .green = 0,
        .blue = 0,
    };

    for(int i = 0; i < 255; i += 1)
    {
        color.red = i;

        apa102_led_color_set(ctx, color);
        ctx->delay_ms(ctx, 3);
    }
    for(int i = 0; i < 255; i += 1)
    {
        color.green = i;

        apa102_led_color_set(ctx, color);
        ctx->delay_ms(ctx, 3);
    }
    for(int i = 0; i < 255; i += 1)
    {
        color.blue = i;

        apa102_led_color_set(ctx, color);
        ctx->delay_ms(ctx, 3);
    }

    apa102_color_t blink[7] = {
                {0,0,0,0},
                {1,255,255,255},
                {0,0,0,0},
                {1,255,255,255},
                {0,0,0,0},
                {1,255,255,255},
                {0,0,0,0},
            };

    apa102_led_show_sequence(ctx, blink, 7);

    return 0;
}

