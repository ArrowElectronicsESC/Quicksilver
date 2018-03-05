#include "wiced.h"
#include "math.h"

#define RGB_CLOCK WICED_GPIO_10
#define RGB_DATA WICED_GPIO_8

typedef struct color
{
    unsigned char Red;
    unsigned char Green;
    unsigned char Blue;
} color;

void show_color( color c)
{
    unsigned int mask = 1 <<31;
    unsigned int data_array[3] = {};
    data_array[0] = 0;
    data_array[1] = 0;
    data_array[2] = 0xFFFFFFFF;

    data_array[1] = 0;
    data_array[1] = 0b111 << 29;
    data_array[1] |= 0b00010 << 24;
    data_array[1] |= c.Blue << 16;
    data_array[1] |= c.Green << 8;
    data_array[1] |= c.Red;

    for( int i = 0; i< 3; i++ )
    {
        mask = 1<<31;

        for ( int j = 0; j<32; j++)
        {
            wiced_gpio_output_low( RGB_CLOCK );

            if ( data_array[i] & mask )
            {
                wiced_gpio_output_high( RGB_DATA );
            }
            else
            {
                wiced_gpio_output_low( RGB_DATA );
            }

            wiced_gpio_output_high( RGB_CLOCK );

            mask >>= 1; // right shift by 1
        }
    }
}

void application_start()
{
    wiced_init();

    color Rainbow[8] = {
            {255,0,0},
            {255,110,0},
            {255,255,0},
            {0,255,0},
            {0,0,255},
            {0,255,255},
            {255,0,255},
            {255,255,255}
    };

    wiced_gpio_init( RGB_CLOCK, OUTPUT_PUSH_PULL );
    wiced_gpio_init( RGB_DATA, OUTPUT_PUSH_PULL );

    double Angle = 0;
    color C;

    while( 1 )
    {
        C.Red = 255 * ((sin(Angle) + 1)/2.0);
        C.Green = 255 * ((sin(Angle + (3.14/2.0)) + 1)/2.0);
        C.Blue = 255 * ((sin(Angle+(3.14/3.0)) + 1)/2.0);

        show_color(C);

        Angle += 0.0001;
    }
}
