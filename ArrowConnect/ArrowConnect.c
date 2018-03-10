#include "wiced.h"
#include "ArrowConnect.h"
#include "quicksilver.h"
#include <debug.h>

/******************************************************
 *                    Constants
 ******************************************************/
#define RGB_CLOCK WICED_GPIO_10
#define RGB_DATA WICED_GPIO_8
#define BUFFER_LENGTH     (2048)
#define MAX_LINE_LENGTH  (128)
#define MAX_HISTORY_LENGTH (20)
#define MAX_NUM_COMMAND_TABLE  (8)
#define NUM_I2C_MESSAGE_RETRIES   (3)

/******************************************************
 *                      Macros
 ******************************************************/
#define PING_TIMEOUT_MS          2000
#define DELAY_MS(x)             wiced_rtos_delay_milliseconds(x)
#define VERIFY_SUCCESS(x)       if(x != WICED_SUCCESS) {wiced_framework_reboot();}

/******************************************************
 *                   Enumerations
 ******************************************************/

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
 *                    Structures
 ******************************************************/
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

static const wiced_ip_setting_t ap_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS( 255,255,255,  0 ) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
};

static aws_app_info_t  app_info = {
    .mqtt_client_id = 0,
};

/******************************************************
 *               Static Function Declarations
 ******************************************************/
wiced_result_t temperature_init( void );
wiced_result_t accelerometer_init( void );
wiced_result_t adc_init( void );
wiced_result_t rgb_init( void );
wiced_result_t probe_sensors(void);

/******************************************************
 *               Variable Definitions
 ******************************************************/
static quicksilver_data telemetryData;
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
int32_t i2c_write_accel(void * dev_ctx, uint8_t reg, uint8_t* buffer, uint16_t length)
{
    uint8_t write_buffer[length + 1];
    write_buffer[0] = (reg | 0x80);
    memcpy(&write_buffer[1], buffer, length);

    wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, write_buffer, sizeof(write_buffer) );

    return 0;
}

int32_t i2c_read_accel(void * dev_ctx, uint8_t reg, uint8_t* buffer, uint16_t length)
{
    uint8_t tx_buffer [1] = {(reg | 0x80)};

    wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, tx_buffer, 1 );
    wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, buffer, length );

    return 0;
}

int32_t i2c_write_hts(void * dev_ctx, uint8_t reg, uint8_t* buffer, uint16_t length)
{
    uint8_t write_buffer[length + 1];
    write_buffer[0] = (reg | 0x80);
    memcpy(&write_buffer[1], buffer, length);

    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, write_buffer, sizeof(write_buffer) );

    return 0;
}

int32_t i2c_read_hts(void * dev_ctx, uint8_t reg, uint8_t* buffer, uint16_t length)
{
    uint8_t tx_buffer [1] = {(reg | 0x80)};

    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, tx_buffer, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, buffer, length );

    return 0;
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
    DELAY_MS(milliseconds);

    return 0;
}

/******************************************************
 *               Helper Function Definitions
 ******************************************************/
/*
 *  Function used to apply coefficient
 */
float linear_interpolation(lin_t *lin, int16_t x)
{
 return ((lin->y1 - lin->y0) * x +  ((lin->x1 * lin->y0) - (lin->x0 * lin->y1)))
        / (lin->x1 - lin->x0);
}

/******************************************************
 *               Application Function Definitions
 ******************************************************/
int state_handler(char *str)
{
    int Status = 0;
    char json_str_buffer[64] = {0};
    DBG("State Handler Payload: %s",str);

    JsonNode *main_ = json_decode(str);
    JsonNode *deviceStateNode = NULL;

    DBG("State Handler!!!\r\n");

    if(main_)
    {
        json_foreach(deviceStateNode, main_) {
            if(strcmp(deviceStateNode->key, "rgbValues") == 0)
            {
                int Red = -1;
                int Green = -1;
                int Blue = -1;
                JsonNode *OriginalValue = NULL;
                OriginalValue = json_find_member(deviceStateNode, "value");

                JsonNode *ValueObject = json_mkobject();
                if(OriginalValue)
                {
                    sprintf(json_str_buffer, "{\"value\":%s}",OriginalValue->string_);
                    DBG("Original value: %s",OriginalValue->string_);
                    DBG("Re-formed value: %s", json_str_buffer);
                    ValueObject = json_decode(json_str_buffer);
                    if(ValueObject)
                    {
                        JsonNode *ArrayKey = json_find_member(ValueObject, "value");
                        JsonNode *ArrayElement = NULL;
                        int ListIndex = 0;
                        json_foreach(ArrayElement, ArrayKey){
                            switch(ListIndex)
                            {
                            case 0:
                                Red = ArrayElement->number_;
                                break;
                            case 1:
                                Green = ArrayElement->number_;
                                break;
                            case 2:
                                Blue = ArrayElement->number_;
                                break;
                            default:
                                DBG("Should not get here!!!");
                                break;
                            }

                            ListIndex++;
                        }

                        DBG("Got colors: %d %d %d", Red, Green, Blue);
                        apa102_color_t C = {
                                .red =Red,
                                .green = Green,
                                .blue =Blue
                        };
                        apa102_led_color_set(&rgb_ctx, C);

                        //add_state("rgbValues","[123,255,45]");
                        //arrow_post_state_update(current_device());
                    }
                    else
                    {
                        DBG("Failed to parse new value object");
                    }
                }
                else
                {
                    DBG("Failed to find member: \"value\"");
                }
            }
            else
            {
                DBG("Found node: %s", deviceStateNode->key);
            }
        }
    }

  return Status;
}

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

wiced_result_t i2c_init(void)
{
    /* Initialize I2C */
    if ( wiced_i2c_init( &i2c_device_accelerometer ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    if ( wiced_i2c_init( &i2c_device_temperature ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return i2c_sensor_probe();
}

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

/*
 * Initializes I2C, probes for temperature device
 */
wiced_result_t temperature_init( void )
{
    /* Initialize mems driver interface */
    hts_ctx.write_reg = i2c_write_hts;
    hts_ctx.read_reg = i2c_read_hts;
    hts_ctx.handle = &i2c_device_temperature;

    hts221_device_id_get(&hts_ctx, &whoamI);
    if( whoamI != HTS221_ID )
    {
        DBG("Failed to read WHOAMI from temperature device; addr 0x%x\n", i2c_device_temperature.address);
        return WICED_ERROR;
    }

    /*
     *  Read humidity calibration coefficient
     */
    hts221_hum_adc_point_0_get(&hts_ctx, coeff.u8bit);
    lin_hum.x0 = (float)coeff.i16bit;
    hts221_hum_rh_point_0_get(&hts_ctx, coeff.u8bit);
    lin_hum.y0 = (float)coeff.u8bit[0];
    hts221_hum_adc_point_1_get(&hts_ctx, coeff.u8bit);
    lin_hum.x1 = (float)coeff.i16bit;
    hts221_hum_rh_point_1_get(&hts_ctx, coeff.u8bit);
    lin_hum.y1 = (float)coeff.u8bit[0];

    /*
     *  Read temperature calibration coefficient
     */
    hts221_temp_adc_point_0_get(&hts_ctx, coeff.u8bit);
    lin_temp.x0 = (float)coeff.i16bit;
    hts221_temp_deg_point_0_get(&hts_ctx, coeff.u8bit);
    lin_temp.y0 = (float)coeff.u8bit[0];
    hts221_temp_adc_point_1_get(&hts_ctx, coeff.u8bit);
    lin_temp.x1 = (float)coeff.i16bit;
    hts221_temp_deg_point_1_get(&hts_ctx, coeff.u8bit);
    lin_temp.y1 = (float)coeff.u8bit[0];

    /* Power-up the device */
    /*
     *  Enable Block Data Update
     */
    hts221_block_data_update_set(&hts_ctx, PROPERTY_ENABLE);

    /*
     * Set Output Data Rate
     */
    hts221_data_rate_set(&hts_ctx, HTS221_ODR_12Hz5);

    /*
     * Device power on
     */
    hts221_power_on_set(&hts_ctx, PROPERTY_ENABLE);

    return WICED_SUCCESS;
}

/*
 * Holder function to get HTS221 temperature
 */
int temperature_get(int argc, char *argv[])
{
    hts221_reg_t reg;
    hts221_status_get(&hts_ctx, &reg.status_reg);

    if (reg.status_reg.h_da)
    {
      /* Read humidity data */
        memset(data_raw_humidity.u8bit, 0x00, sizeof(int16_t));
        hts221_humidity_raw_get(&hts_ctx, data_raw_humidity.u8bit);
        humidity_perc = linear_interpolation(&lin_hum, data_raw_humidity.i16bit);
        if (humidity_perc < 0) humidity_perc = 0;
        if (humidity_perc > 100) humidity_perc = 100;
        telemetryData.humidity = humidity_perc;
    }
    if (reg.status_reg.t_da)
    {
        /* Read temperature data */
        memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
        hts221_temperature_raw_get(&hts_ctx, data_raw_temperature.u8bit);
        temperature_degC = linear_interpolation(&lin_temp, data_raw_temperature.i16bit);
        telemetryData.temperature = temperature_degC;
    }

    return 0;
}

/*
 * Initializes I2C, probes for accelerometer device
 */
wiced_result_t accelerometer_init( void )
{
   /*
    *  Initialize mems driver interface
    */
    accel_ctx.write_reg = i2c_write_accel;
    accel_ctx.read_reg = i2c_read_accel;
    accel_ctx.handle = &i2c_device_accelerometer;

    lis2dh12_device_id_get(&accel_ctx, &whoamI);
    if(whoamI != LIS2DH12_ID)
    {
        DBG("Failed to read WHOAMI from accelerometer device; addr 0x%x\n", i2c_device_accelerometer.address);
    }

    /* Power-up the device */
    lis2dh12_block_data_update_set(&accel_ctx, PROPERTY_ENABLE);
    lis2dh12_data_rate_set(&accel_ctx, LIS2DH12_ODR_400Hz);
    lis2dh12_full_scale_set(&accel_ctx, LIS2DH12_4g);
    lis2dh12_temperature_meas_set(&accel_ctx, LIS2DH12_TEMP_ENABLE);

    /* Set normal mode */
    lis2dh12_operating_mode_set(&accel_ctx ,LIS2DH12_HR_12bit);

    return WICED_SUCCESS;
}

/*
 * Holder function to get LIS2DH12 accelerometer
 */
int accelerometer_get(int argc, char *argv[])
{

    lis2dh12_reg_t reg;
    lis2dh12_status_get(&accel_ctx, &reg.status_reg);

    if( reg.status_reg.zyxda )
    {

        memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
        lis2dh12_acceleration_raw_get(&accel_ctx, data_raw_acceleration.u8bit);
        acceleration_mg[0] = LIS2DH12_FROM_FS_4g_HR_TO_mg( data_raw_acceleration.i16bit[0] );
        acceleration_mg[1] = LIS2DH12_FROM_FS_4g_HR_TO_mg( data_raw_acceleration.i16bit[1] );
        acceleration_mg[2] = LIS2DH12_FROM_FS_4g_HR_TO_mg( data_raw_acceleration.i16bit[2] );

        telemetryData.accelerometer.x = acceleration_mg[0];
        telemetryData.accelerometer.y = acceleration_mg[1];
        telemetryData.accelerometer.z = acceleration_mg[2];
    }

    return 0;
}

/*
 * Initializes RGB LED
 */
wiced_result_t rgb_init( void )
{
    rgb_ctx.clk_in_pin = WICED_RGB_CLOCK;
    rgb_ctx.data_in_pin = WICED_RGB_DATA;
    rgb_ctx.pin_set = rgb_pin_set;
    rgb_ctx.pin_clear = rgb_pin_clear;
    rgb_ctx.delay_ms = rgb_delay_ms;

    apa102_init(&rgb_ctx);

    apa102_led_ramp_sequence(&rgb_ctx);

    return WICED_SUCCESS;
}

int update_sensor_data(void * data)
{
    accelerometer_get(0, NULL);
    temperature_get(0, NULL);

    return 0;
}

int rgb_handler(const char *data)
{
    apa102_color_t color = {0};

    DBG("---------------------------------------------");
    DBG("rgb_handler: %s", data);
    DBG("---------------------------------------------");
    JsonNode * rgb_color = json_decode(data);
    if(rgb_color)
    {
        JsonNode *red_val = json_find_member(rgb_color, "red");
        if(red_val && red_val->tag == JSON_NUMBER)
        {
            color.red = (int)red_val->number_;
        }
        JsonNode *green_val = json_find_member(rgb_color, "green");
        if(green_val && green_val->tag == JSON_NUMBER)
        {
            color.green = (int)green_val->number_;
        }
        JsonNode *blue_val = json_find_member(rgb_color, "blue");
        if(blue_val && blue_val->tag == JSON_NUMBER)
        {
            color.blue = (int)blue_val->number_;
        }
        json_delete(rgb_color);

        apa102_led_color_set(&rgb_ctx, color);
    }
    else
    {
        DBG("json parse failed");
    }

    return 0;
}

wiced_result_t quicksilver_init(void)
{
    /* Initialize I2C*/
    if(i2c_init() != WICED_SUCCESS)
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

    if(aws_app_init(&app_info) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

wiced_result_t arrow_cloud_init(void)
{
    /* Register the Quicksilver board as both a gateway and device and establish HTTP connection */
    if(arrow_initialize_routine() != ROUTINE_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* Connect to the MQTT Service */
    if(arrow_mqtt_connect_routine() != ROUTINE_SUCCESS)
    {
        return WICED_ERROR;
    }

    // Add command handlers
    add_cmd_handler("rgb", rgb_handler);

    return WICED_SUCCESS;
}

void application_start( )
{
    /* Initialize the WICED platform */
    VERIFY_SUCCESS(wiced_init());

    VERIFY_SUCCESS(quicksilver_init());

    VERIFY_SUCCESS(arrow_cloud_init());

    while(1)
    {
        /* Send the latest data to Arrow Connect */
        arrow_mqtt_send_telemetry_routine(update_sensor_data, &telemetryData);
    }


}
