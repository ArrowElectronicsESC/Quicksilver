#include "wiced.h"
#include "ArrowConnect.h"
#include "quicksilver.h"
#include <debug.h>
#include "wiced_framework.h"
#include "wiced_ota_server.h"

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
#define PING_TIMEOUT_MS          2000

/******************************************************
 *                      Macros
 ******************************************************/
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

typedef struct {
    char * cmd;
    int (*handler)(const char *data);
}command_handler_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/
wiced_result_t temperature_init( void );
wiced_result_t accelerometer_init( void );
wiced_result_t rgb_init( void );
int cmd_handler_setLED(const char *data);
int cmd_handler_updateLED(const char *data);
int cmd_handler_update(const char *data);

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

static command_handler_t arrowCommandHandlers[] = {
    // Command       // Handler
    { "setLED",      &cmd_handler_setLED },
    { "updateLED",   &cmd_handler_updateLED },
    { "update",      &cmd_handler_update },
};

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

static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255, 255, 255, 0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
};

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

wiced_result_t parseCommand_RGB(JsonNode * node)
{
    apa102_color_t commandColor;
    char json_str_buffer[100] = {0};

    WPRINT_APP_INFO(("RGB Command Received\r\n"));

    // jsonValue will contain a string of RGB values. Ex. "[10, 255, 255, 0]"
    JsonNode * jsonValue = json_find_member(node, "value");
    // valueObject will ultimately contain an array of RGB values
    JsonNode * valueObject = json_mkobject();

    if(jsonValue)
    {
        // Rebuild a JSON string so it can be easily parsed
        sprintf(json_str_buffer, "{\"value\":%s}",jsonValue->string_);
        // Parse the new JSON string
        valueObject = json_decode(json_str_buffer);
        if(valueObject)
        {
            JsonNode *arrayKey = json_find_member(valueObject, "value");
            JsonNode *arrayElement;
            uint8_t arrayIndex = 0;
            json_foreach(arrayElement, arrayKey)
            {
                switch(arrayIndex)
                {
                    case 0:
                        commandColor.brightness = arrayElement->number_ < 0x1F ? arrayElement->number_: 0x1F;
                        break;
                    case 1:
                        commandColor.red = arrayElement->number_;
                        break;
                    case 2:
                        commandColor.green = arrayElement->number_;
                        break;
                    case 3:
                        commandColor.blue = arrayElement->number_;
                        break;
                    default:
                        WPRINT_APP_INFO(("Number of RGB command values greater than expected\r\n"));
                        json_delete(valueObject);
                        json_delete(jsonValue);
                        return WICED_ERROR;
                        break;
                }

                arrayIndex++;
            }

            WPRINT_APP_INFO(("RGB Value: Brightness-%d, R-%d, G-%d, B-%d\r\n",
                    commandColor.brightness, commandColor.red, commandColor.blue, commandColor.green));
            apa102_led_color_set(&rgb_ctx, commandColor);
        }
        else
        {
            WPRINT_APP_INFO(("Failed to parse new value object\r\n"));
            json_delete(valueObject);
            json_delete(jsonValue);
            return WICED_ERROR;
        }
    }
    else
    {
        WPRINT_APP_INFO(("Failed to find member: \"value\"\r\n"));
        json_delete(valueObject);
        json_delete(jsonValue);
        return WICED_ERROR;
    }

    json_delete(valueObject);
    json_delete(jsonValue);

    WPRINT_APP_INFO(("RGB Command Handling Complete\r\n"));

    return WICED_SUCCESS;
}

/******************************************************
 *               Application Function Definitions
 ******************************************************/
int state_handler(char *str)
{
    int Status = 0;


    JsonNode *main_ = json_decode(str);
    JsonNode *deviceStateNode = NULL;

    if(main_)
    {
        json_foreach(deviceStateNode, main_)
        {
            if(strcmp(deviceStateNode->key, "rgbValues") == 0)
            {
                parseCommand_RGB(deviceStateNode);
            }
            else
            {
                WPRINT_APP_INFO(("Found node: %s\r\n", deviceStateNode->key));
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

    if ( i2c_sensor_probe() != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
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
        WPRINT_APP_INFO(("Failed to read WHOAMI from temperature device; addr 0x%x\r\n", i2c_device_temperature.address));
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
        WPRINT_APP_INFO(("Failed to read WHOAMI from accelerometer device; addr 0x%x\r\n", i2c_device_accelerometer.address));
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

int rgb_color_get(int argc, char *argv[])
{
    apa102_color_get(&telemetryData.led);

    return WICED_SUCCESS;
}

int update_sensor_data(void * data)
{
    accelerometer_get(0, NULL);
    temperature_get(0, NULL);
    rgb_color_get(0, NULL);

    return WICED_SUCCESS;
}

int cmd_handler_update(const char *data)
{
    WPRINT_APP_INFO(("Update Command Handler\r\n"));

    wiced_network_down(WICED_STA_INTERFACE);
    wiced_network_up( WICED_AP_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &device_init_ip_settings );
    wiced_ota_server_start( WICED_AP_INTERFACE );
    while ( 1 )
    {
        wiced_rtos_delay_milliseconds( 100 );
    }

    return WICED_SUCCESS;
}

int cmd_handler_setLED(const char *data)
{
    static apa102_color_t color = {0};

    WPRINT_APP_INFO(("---------------------------------------------\r\n"));
    WPRINT_APP_INFO(("setLED cmd handler: %s\r\n", data));
    WPRINT_APP_INFO(("---------------------------------------------\r\n"));
    JsonNode * rgb_color = json_decode(data);
    if(rgb_color)
    {
        JsonNode *brightness = json_find_member(rgb_color, "brightness");
        if(brightness && brightness->tag == JSON_NUMBER)
        {
            color.brightness = (int)brightness->number_;
        }
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
        WPRINT_APP_INFO(("json parse failed\r\n"));
    }

    return 0;
}

int cmd_handler_updateLED(const char *data)
{
    JsonNode * rgb_color = json_decode(data);
    if(rgb_color)
    {
        JsonNode *brightness = json_find_member(rgb_color, "brightness");
        if(brightness && brightness->tag == JSON_NUMBER)
        {
            apa102_led_brightness_set(&rgb_ctx, (int)brightness->number_);
        }
        JsonNode *red_val = json_find_member(rgb_color, "red");
        if(red_val && red_val->tag == JSON_NUMBER)
        {
            apa102_led_red_set(&rgb_ctx, (int)red_val->number_);
        }
        JsonNode *green_val = json_find_member(rgb_color, "green");
        if(green_val && green_val->tag == JSON_NUMBER)
        {
            apa102_led_green_set(&rgb_ctx, (int)green_val->number_);
        }
        JsonNode *blue_val = json_find_member(rgb_color, "blue");
        if(blue_val && blue_val->tag == JSON_NUMBER)
        {
            apa102_led_blue_set(&rgb_ctx, (int)blue_val->number_);
        }

        json_delete(rgb_color);
    }
    else
    {
        WPRINT_APP_INFO(("json parse failed\r\n"));
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

    if(aws_app_init() != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

int gateway_software_update_cb(const char *url)
{
    WPRINT_APP_INFO(("Gateway Software Update Callback\r\n"));

    return 0;
}

int arrow_release_download_payload(const char *payload, int size, int flag)
{
    static uint32_t offset;

    if ( flag == FW_FIRST )
    {
        WPRINT_APP_INFO(("Release Download Started\r\n"));
        offset = 0;
    }

//    wiced_ota2_image_write_data(payload, offset, size);

    offset += size;

    return 0;
}

int arrow_release_download_init(void)
{
    WPRINT_APP_INFO(("Arrow Release Download Init Callback!\r\n"));
    return 0;
}

int arrow_release_download_complete(int flag)
{
    if(flag == FW_SUCCESS)
    {
//        WPRINT_APP_INFO(("Release Download Completed Successfully\r\n"));
//
//        platform_dct_ota2_config_t  dct_ota2_config;
//
//        /* set status to extract on next boot */
//        wiced_ota2_image_update_staged_status(WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT);
//
//        /* set boot_type so bootloader will extract on next boot */
//        wiced_dct_read_with_copy( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) );
//        dct_ota2_config.boot_type = OTA2_BOOT_EXTRACT_UPDATE;
//        wiced_dct_write( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) );
//
//        /* run the ota2_extractor on next boot */
//        wiced_waf_app_set_boot( DCT_OTA_APP_INDEX, PLATFORM_DEFAULT_LOAD );
    }
    else
    {
        WPRINT_APP_INFO(("Release Download Failed, OTA SDK MD5SUM Checksum is NOT correct\r\n"));
    }

    return 0;
}

wiced_result_t arrow_cloud_init(void)
{
    /* Register the Quicksilver board as both a gateway and device and establish HTTP connection */
    if(arrow_initialize_routine() != ROUTINE_SUCCESS)
    {
        return WICED_ERROR;
    }

    arrow_mqtt_events_init();

    // Add command handlers
    for(int i = 0; i < sizeof(arrowCommandHandlers)/sizeof(command_handler_t); i++)
    {
        arrow_command_handler_add(arrowCommandHandlers[i].cmd, arrowCommandHandlers[i].handler);
    }

#if !defined(NO_SOFTWARE_UPDATE)
    arrow_gateway_software_update_set_cb(gateway_software_update_cb);
#endif
    arrow_software_release_dowload_set_cb(arrow_release_download_init, arrow_release_download_payload, arrow_release_download_complete);

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
        arrow_mqtt_connect_routine();
        int ret = arrow_mqtt_send_telemetry_routine(update_sensor_data, &telemetryData);
        switch ( ret ) {
        case ROUTINE_RECEIVE_EVENT:
            arrow_mqtt_disconnect_routine();
            arrow_mqtt_event_proc();
            break;
        default:
            break;
        }
    }
}
