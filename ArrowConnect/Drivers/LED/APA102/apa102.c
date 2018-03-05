#include "apa102.h"

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
  * @brief  Turn LED Off.
  *
  * @param  apa102_ctx_t *ctx: Interface definition
  *
  */
int32_t apa102_led_off(apa102_ctx_t *ctx)
{
    apa102_color_t color = {
            .red = 0,
            .green = 0,
            .blue = 0,
    };

    apa102_led_color_set(ctx, color);

    return 0;
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
    unsigned int mask = 1 <<31;
    unsigned int data_array[3] = {};
    data_array[0] = 0;
    data_array[1] = 0;
    data_array[2] = 0xFFFFFFFF;

    data_array[1] = 0;
    data_array[1] = 0b111 << 29;
    data_array[1] |= 0b00010 << 24;
    data_array[1] |= color.blue << 16;
    data_array[1] |= color.green << 8;
    data_array[1] |= color.red;

    for( int i = 0; i< 3; i++ )
    {
        mask = 1<<31;

        for ( int j = 0; j<32; j++)
        {
            ctx->pin_clear(ctx, ctx->clk_in_pin);

            if ( data_array[i] & mask )
            {
                ctx->pin_set(ctx, ctx->data_in_pin);
            }
            else
            {
                ctx->pin_clear(ctx, ctx->data_in_pin);
            }

            ctx->pin_set(ctx, ctx->clk_in_pin);

            mask >>= 1; // right shift by 1
        }
    }

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
        apa102_led_color_set(ctx, sequence[i]);
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
            {255,0,0},
            {255,110,0},
            {255,255,0},
            {0,255,0},
            {0,0,255},
            {0,255,255},
            {255,0,255},
            {255,255,255}
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
                {0,0,0},
                {255,255,255},
                {0,0,0},
                {255,255,255},
                {0,0,0},
                {255,255,255},
                {0,0,0},
            };

    apa102_led_show_sequence(ctx, blink, 7);

    return 0;
}

