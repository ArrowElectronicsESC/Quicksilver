#include "quicksilver.h"

#define NUM_I2C_MESSAGE_RETRIES   (3)

/******************************************************
 *                      Macros
 ******************************************************/
#define DELAY_MS(x)             wiced_rtos_delay_milliseconds(x)

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef struct {
  float x0;
  float y0;
  float x1;
  float y1;
} lin_t;


/******************************************************
 *                   Enumerations
 ******************************************************/
enum LED_COLOR_VALUES
{
    LED_COLOR_OFF = 0x00,
    LED_COLOR_MIN = 0x01,
    LED_COLOR_MAX = 0xFF
};

enum LED_BRIGHTNESS_VALUES
{
    LED_BRIGHTNESS_OFF = 0x00,
    LED_BRIGHTNESS_MIN = 0x01,
    LED_BRIGHTNESS_MAX = 0x1F
};


/******************************************************
 *               Variable Definitions
 ******************************************************/
static lis2dh12_ctx_t accel_ctx;
static hts221_ctx_t hts_ctx;
static apa102_ctx_t rgb_ctx;
static axis3bit16_t data_raw_acceleration;
static float acceleration_mg[3];
static axis1bit16_t data_raw_humidity;
static axis1bit16_t data_raw_temperature;
static float humidity_perc;
static float temperature_degC;
static uint8_t whoamI;
static axis1bit16_t coeff;
static lin_t lin_hum;
static lin_t lin_temp;


/******************************************************
 *               CTX Interface Function Definitions
 ******************************************************/

/* Function for performing an I2C write */
int32_t i2c_write(void * handle, uint8_t reg, uint8_t* buffer, uint16_t length)
{
    uint8_t write_buffer[length + 1];
    write_buffer[0] = (reg | 0x80);
    memcpy(&write_buffer[1], buffer, length);

    wiced_i2c_write( handle, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, write_buffer, sizeof(write_buffer) );

    return 0;
}

/* Function for performing an I2C read */
int32_t i2c_read(void * handle, uint8_t reg, uint8_t* buffer, uint16_t length)
{
    uint8_t tx_buffer [1] = {(reg | 0x80)};

    wiced_i2c_write( handle, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, tx_buffer, 1 );
    wiced_i2c_read( handle, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, buffer, length );

    return 0;
}

/* Function for setting a pin in the RGB driver */
int32_t rgb_pin_set(void * dev_ctx, uint32_t pin)
{
    wiced_gpio_output_high(pin);

    return 0;
}

/* Function for clearing a pin in the RGB driver */
int32_t rgb_pin_clear(void * dev_ctx, uint32_t pin)
{
    wiced_gpio_output_low(pin);

    return 0;
}

/* Function for performing an RTOS delay in the RGB driver */
int32_t rgb_delay_ms(void * dev_ctx, uint32_t milliseconds)
{
    DELAY_MS(milliseconds);

    return 0;
}

/******************************************************
 *               Helper Function Definitions
 ******************************************************/

/* Function used to apply coefficient */
float linear_interpolation(lin_t *lin, int16_t x)
{
 return ((lin->y1 - lin->y0) * x +  ((lin->x1 * lin->y0) - (lin->x0 * lin->y1)))
        / (lin->x1 - lin->x0);
}

static wiced_i2c_device_t i2c_device_temperature =
{
        .port = WICED_I2C_2,  //I2C_1
        .address = HTS221_I2C_ADDRESS,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .speed_mode = I2C_STANDARD_SPEED_MODE,
};

static wiced_i2c_device_t i2c_device_accelerometer =
{
        .port = WICED_I2C_2,  //I2C_1
        .address = LIS2DH12_I2C_ADD_H,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .speed_mode = I2C_STANDARD_SPEED_MODE,
};

/******************************************************
 *               Public Function Definitions
 ******************************************************/
/* Function for initializing the Quicksilver board and on-board hardware */
wiced_result_t quicksilver_init(void)
{
    /* Initialize I2C*/
    if(i2c_init() != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* Initialize GPIO*/
    if(gpio_init() != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* Initialize the RGB */
    if(rgb_init() != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* probe for temperature device */
    if(temperature_init() != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* probe for accelerometer device */
    if(accelerometer_init() != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* Initialize the AP for receiving Wi-Fi credentials if needed */
    if(acn_app_init() != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/* Function for probing sensors on the I2C bus. */
wiced_result_t i2c_sensor_probe(void)
{
    /* Probe I2C bus for accelerometer */
    if( wiced_i2c_probe_device( &i2c_device_accelerometer, NUM_I2C_MESSAGE_RETRIES ) != WICED_TRUE )
    {
        return WICED_ERROR;
    }

    /* Probe I2C bus for temperature sensor */
    if( wiced_i2c_probe_device( &i2c_device_temperature, NUM_I2C_MESSAGE_RETRIES ) != WICED_TRUE )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/* Function for initializing the I2C bus */
wiced_result_t i2c_init(void)
{
    /* Initialize I2C for the accelerometer */
    if ( wiced_i2c_init( &i2c_device_accelerometer ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    /* Initialize I2C for the temperature sensor */
    if ( wiced_i2c_init( &i2c_device_temperature ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    /* Probe the I2C bus for the sensors */
    if ( i2c_sensor_probe() != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/* Initialize GPIO */
wiced_result_t gpio_init(void)
{
    if(wiced_gpio_init( WICED_RGB_CLOCK, OUTPUT_PUSH_PULL ) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    if(wiced_gpio_init( WICED_RGB_DATA, OUTPUT_PUSH_PULL ) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/* Function for initializing the temperature sensor */
wiced_result_t temperature_init( void )
{
    /* Initialize mems driver interface */
    hts_ctx.write_reg = i2c_write;
    hts_ctx.read_reg = i2c_read;
    hts_ctx.handle = &i2c_device_temperature;

    /* Read the device ID to verify communications to the sensor */
    hts221_device_id_get(&hts_ctx, &whoamI);
    if( whoamI != HTS221_ID )
    {
        WPRINT_APP_INFO(("Failed to read WHOAMI from temperature device; addr 0x%x\r\n", i2c_device_temperature.address));
        return WICED_ERROR;
    }

    /* Read humidity calibration coefficient */
    hts221_hum_adc_point_0_get(&hts_ctx, coeff.u8bit);
    lin_hum.x0 = (float)coeff.i16bit;
    hts221_hum_rh_point_0_get(&hts_ctx, coeff.u8bit);
    lin_hum.y0 = (float)coeff.u8bit[0];
    hts221_hum_adc_point_1_get(&hts_ctx, coeff.u8bit);
    lin_hum.x1 = (float)coeff.i16bit;
    hts221_hum_rh_point_1_get(&hts_ctx, coeff.u8bit);
    lin_hum.y1 = (float)coeff.u8bit[0];

    /* Read temperature calibration coefficient */
    hts221_temp_adc_point_0_get(&hts_ctx, coeff.u8bit);
    lin_temp.x0 = (float)coeff.i16bit;
    hts221_temp_deg_point_0_get(&hts_ctx, coeff.u8bit);
    lin_temp.y0 = (float)coeff.u8bit[0];
    hts221_temp_adc_point_1_get(&hts_ctx, coeff.u8bit);
    lin_temp.x1 = (float)coeff.i16bit;
    hts221_temp_deg_point_1_get(&hts_ctx, coeff.u8bit);
    lin_temp.y1 = (float)coeff.u8bit[0];

    /* Enable Block Data Update */
    hts221_block_data_update_set(&hts_ctx, PROPERTY_ENABLE);

    /* Set Output Data Rate */
    hts221_data_rate_set(&hts_ctx, HTS221_ODR_12Hz5);

    /* Device power on */
    hts221_power_on_set(&hts_ctx, PROPERTY_ENABLE);

    return WICED_SUCCESS;
}

/* Function to getting temperature and humidity data from the sensor */
int humidity_get(float * humidity)
{
    hts221_reg_t reg;
    hts221_status_get(&hts_ctx, &reg.status_reg);

    /* Get Humidity data if new data available */
    if (reg.status_reg.h_da)
    {
      /* Read humidity data */
        memset(data_raw_humidity.u8bit, 0x00, sizeof(int16_t));
        hts221_humidity_raw_get(&hts_ctx, data_raw_humidity.u8bit);
        humidity_perc = linear_interpolation(&lin_hum, data_raw_humidity.i16bit);
        if (humidity_perc < 0) humidity_perc = 0;
        if (humidity_perc > 100) humidity_perc = 100;
        *humidity = humidity_perc;
    }

    return 0;
}

/* Function to getting temperature and humidity data from the sensor */
int temperature_get(float * temperature)
{
    hts221_reg_t reg;
    hts221_status_get(&hts_ctx, &reg.status_reg);

    /* Get temperature data if new data available */
    if (reg.status_reg.t_da)
    {
        /* Read temperature data */
        memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
        hts221_temperature_raw_get(&hts_ctx, data_raw_temperature.u8bit);
        temperature_degC = linear_interpolation(&lin_temp, data_raw_temperature.i16bit);
        *temperature = temperature_degC;
    }

    return 0;
}

/* Function for initializing the accelerometer */
wiced_result_t accelerometer_init( void )
{
   /* Initialize mems driver interface */
    accel_ctx.write_reg = i2c_write;
    accel_ctx.read_reg = i2c_read;
    accel_ctx.handle = &i2c_device_accelerometer;

    /* Read the device ID to verify communications to the sensor */
    lis2dh12_device_id_get(&accel_ctx, &whoamI);
    if(whoamI != LIS2DH12_ID)
    {
        WPRINT_APP_INFO(("Failed to read WHOAMI from accelerometer device; addr 0x%x\r\n", i2c_device_accelerometer.address));
    }

    /* Enable Block Data Update */
    lis2dh12_block_data_update_set(&accel_ctx, PROPERTY_ENABLE);

    /* Set the Data Rate */
    lis2dh12_data_rate_set(&accel_ctx, LIS2DH12_ODR_400Hz);

    /* Set the scale */
    lis2dh12_full_scale_set(&accel_ctx, LIS2DH12_4g);

    /* Set-up temperature measurements */
    lis2dh12_temperature_meas_set(&accel_ctx, LIS2DH12_TEMP_ENABLE);

    /* Set normal mode */
    lis2dh12_operating_mode_set(&accel_ctx ,LIS2DH12_HR_12bit);

    return WICED_SUCCESS;
}

/* Function for getting accelerometer data from the sensor */
int accelerometer_get(axis_t * axis_data)
{

    lis2dh12_reg_t reg;
    lis2dh12_status_get(&accel_ctx, &reg.status_reg);

    /* Get new accelerometer data if new data available */
    if( reg.status_reg.zyxda )
    {

        memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
        lis2dh12_acceleration_raw_get(&accel_ctx, data_raw_acceleration.u8bit);
        acceleration_mg[0] = LIS2DH12_FROM_FS_4g_HR_TO_mg( data_raw_acceleration.i16bit[0] );
        acceleration_mg[1] = LIS2DH12_FROM_FS_4g_HR_TO_mg( data_raw_acceleration.i16bit[1] );
        acceleration_mg[2] = LIS2DH12_FROM_FS_4g_HR_TO_mg( data_raw_acceleration.i16bit[2] );

        axis_data->x = acceleration_mg[0];
        axis_data->y = acceleration_mg[1];
        axis_data->z = acceleration_mg[2];
    }

    return 0;
}

/* Initializes RGB LED driver */
wiced_result_t rgb_init( void )
{
    rgb_ctx.clk_in_pin = WICED_RGB_CLOCK;
    rgb_ctx.data_in_pin = WICED_RGB_DATA;
    rgb_ctx.pin_set = rgb_pin_set;
    rgb_ctx.pin_clear = rgb_pin_clear;
    rgb_ctx.delay_ms = rgb_delay_ms;

    apa102_init(&rgb_ctx);

    // Set the LED red during initialization
    apa102_led_brightness_set(&rgb_ctx, LED_BRIGHTNESS_MIN);
    apa102_led_red_set(&rgb_ctx, LED_COLOR_MAX);

    return WICED_SUCCESS;
}

/* Function for getting the RGB LED color data. */
int rgb_color_get(uint8_t * brightness, uint8_t * red, uint8_t * green, uint8_t * blue)
{
    static apa102_color_t color;
    apa102_color_get(&color);

    *brightness = color.brightness;
    *red = color.red;
    *green = color.green;
    *blue = color.blue;

    return WICED_SUCCESS;
}

/* Function for setting the RGB LED color data. */
int rgb_color_set(uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue)
{
    apa102_color_t color = {
            .brightness = brightness,
            .red = red,
            .green = green,
            .blue = blue
    };

    apa102_led_set(&rgb_ctx, color);

    return WICED_SUCCESS;
}

/* Function for setting the RGB LED Brightness Value. */
int rgb_brightness_set(uint8_t val)
{
    return apa102_led_brightness_set(&rgb_ctx, val);
}

/* Function for setting the RGB LED Red Value. */
int rgb_red_set(uint8_t val)
{
    return apa102_led_red_set(&rgb_ctx, val);
}

/* Function for setting the RGB LED Green Value. */
int rgb_green_set(uint8_t val)
{
    return apa102_led_green_set(&rgb_ctx, val);
}

/* Function for setting the RGB LED Blue Value. */
int rgb_blue_set(uint8_t val)
{
    return apa102_led_blue_set(&rgb_ctx, val);
}
