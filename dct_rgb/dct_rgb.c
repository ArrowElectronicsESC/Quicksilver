#include "wiced.h"
#include "command_console.h"
#include "rgb_dct.h"
#include "platform.h"
#include "../ArrowConnect/Drivers/LED/APA102/apa102.h"

static apa102_ctx_t rgb_ctx;

#define RGB_CONSOLE_COMMAND_MAX_LENGTH     (85)
#define RGB_CONSOLE_COMMAND_HISTORY_LENGTH (10)

static char rgb_command_buffer[RGB_CONSOLE_COMMAND_MAX_LENGTH];
static char rgb_command_history_buffer[RGB_CONSOLE_COMMAND_MAX_LENGTH * RGB_CONSOLE_COMMAND_HISTORY_LENGTH];

int rgb_console_command(int argc, char *argv[]);
void save_color_to_dct(apa102_color_t C);

const command_t rgb_command_table[] =
{
    { (char*) "rgb_set", rgb_console_command, 0, NULL, NULL, (char *)"<red> <green> <blue> <brightness>", (char *)"Set RGB Color" },
    CMD_TABLE_END
};

const char* rgb_console_delimiter_string = " ";

int rgb_console_command(int argc, char *argv[])
{
    apa102_color_t C = {0};
    wiced_result_t Status = WICED_SUCCESS;

    if (strcmp("rgb_set", argv[0]) != 0)
    {
        WPRINT_APP_INFO( ( "\r\nUNRECOGNIZED COMMAND\r\n\r\n") );
        Status = WICED_ERROR;
    }

    if(Status == WICED_SUCCESS)
    {
        if( argc != 5)
        {
            WPRINT_APP_INFO(("Please provide 3 values\n\r"));
            Status = WICED_ERROR;
        }
    }

    if(Status == WICED_SUCCESS)
    {
        C.red = atoi(argv[1]);
        C.green = atoi(argv[2]);
        C.blue = atoi(argv[3]);
        C.brightness = atoi(argv[4]);

        if(C.red > 255 || C.green > 255 || C.blue > 255)
        {
            WPRINT_APP_INFO(("RGB Values cannot exceed 255\n\r"));
            Status = WICED_ERROR;
        }
    }

    if(Status == WICED_SUCCESS)
    {
        apa102_led_color_set(&rgb_ctx, C);
        save_color_to_dct(C);
    }

    return Status;
}

void save_color_to_dct(apa102_color_t C)
{
    rgb_dct_t *dct_app = NULL;

    wiced_dct_read_lock( (void**) &dct_app, WICED_TRUE, DCT_APP_SECTION, 0, sizeof( *dct_app ) );
    dct_app->red = C.red;
    dct_app->green = C.green;
    dct_app->blue = C.blue;
    dct_app->brightness = C.brightness;
    wiced_dct_write( (const void*) dct_app, DCT_APP_SECTION, 0, sizeof(rgb_dct_t) );
    wiced_dct_read_unlock( dct_app, WICED_TRUE );
}

int32_t rgb_pin_set(void * dev_ctx, uint32_t pin)
{
    wiced_gpio_output_high(pin);

    return 0;
}

int32_t rgb_pin_clear(void * dev_ctx, uint32_t pin)
{
    wiced_gpio_output_low(pin);

    return 0;
}

int32_t rgb_delay_ms(void * dev_ctx, uint32_t milliseconds)
{
    wiced_rtos_delay_milliseconds(milliseconds);

    return 0;
}

wiced_result_t rgb_init( void )
{
    wiced_gpio_init( WICED_RGB_CLOCK, OUTPUT_PUSH_PULL );
    wiced_gpio_init( WICED_RGB_DATA, OUTPUT_PUSH_PULL );

    rgb_ctx.clk_in_pin = WICED_RGB_CLOCK;
    rgb_ctx.data_in_pin = WICED_RGB_DATA;
    rgb_ctx.pin_set = rgb_pin_set;
    rgb_ctx.pin_clear = rgb_pin_clear;
    rgb_ctx.delay_ms = rgb_delay_ms;

    apa102_init(&rgb_ctx);

    return WICED_SUCCESS;
}




static wiced_result_t get_last_rgb_color(apa102_color_t *C)
{
    rgb_dct_t *dct_app = NULL;
    wiced_result_t Status = WICED_SUCCESS;

    Status = wiced_dct_read_lock( (void**) &dct_app, WICED_FALSE, DCT_APP_SECTION, 0, sizeof(dct_app) );

    if ( Status == WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "\r\n---------------------------------------------------------------\r\n") );

        WPRINT_APP_INFO( ( "Saved RGB Values:\r\n") );
        WPRINT_APP_INFO( ( "  red        : %u \r\n", (unsigned int)(((rgb_dct_t*)dct_app)->red) ) );
        WPRINT_APP_INFO( ( "  green      : %u \r\n", (unsigned int)(((rgb_dct_t*)dct_app)->green) ) );
        WPRINT_APP_INFO( ( "  blue       : %u \r\n", (unsigned int)(((rgb_dct_t*)dct_app)->blue) ) );
        WPRINT_APP_INFO( ( "  brightness : %u \r\n", (unsigned int)(((rgb_dct_t*)dct_app)->brightness) ) );
        WPRINT_APP_INFO( ( "\r\n---------------------------------------------------------------\r\n") );

        C->red = dct_app->red;
        C->green = dct_app->green;
        C->blue = dct_app->blue;
        C->brightness = dct_app->brightness;

        /* Here ptr_is_writable should be same as what we passed during wiced_dct_read_lock() */
        wiced_dct_read_unlock( dct_app, WICED_FALSE );
    }

    return Status;
}

void application_start()
{
    wiced_result_t Status = WICED_SUCCESS;
    apa102_color_t C = {0};

    wiced_init();
    rgb_init();

    get_last_rgb_color(&C);
    apa102_led_color_set(&rgb_ctx, C);

    Status = command_console_init(STDIO_UART, sizeof(rgb_command_buffer), rgb_command_buffer, RGB_CONSOLE_COMMAND_HISTORY_LENGTH, rgb_command_history_buffer, " ");

    console_add_cmd_table(rgb_command_table);

    while( 1 )
    {

    }

}
