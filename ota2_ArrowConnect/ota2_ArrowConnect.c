#include "wiced.h"
#include "ota2_ArrowConnect.h"
#include "quicksilver.h"
#include <debug.h>
#include "ota2_ArrowConnect_config.h"
#include "data/chunk.h"

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
#define URI_LEN sizeof(ARROW_API_SOFTWARE_RELEASE_ENDPOINT) + 200

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
static ota2_data_t* init_player(void);
wiced_result_t ota2_test_get_update(ota2_data_t* player);

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

/* template for HTTP GET */
char ota2_get_request_template[] =
{
    "GET %s HTTP/1.1\r\n"
    "Host: %s%s \r\n"
    "\r\n"
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
ota2_data_t *g_player;
static wiced_bool_t update_in_progress = WICED_FALSE;

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

//    apa102_led_ramp_sequence(&rgb_ctx);

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

//    JsonNode * update_url = json_decode(data);
//    if(update_url)
//    {
//        JsonNode * url = json_find_member(update_url, "url");
//        if(update_url && update_url->tag == JSON_STRING)
//        {
//            WPRINT_APP_INFO(("Update URL: %s\r\n", url));
//            memset(g_player->uri_to_stream, 0, sizeof(g_player->uri_to_stream));
//            strlcpy(g_player->uri_to_stream, url, (sizeof(g_player->uri_to_stream) - 1) );
//            json_delete(update_url);
//
//            wiced_result_t result = ota2_test_get_update(g_player);
//            if (result != WICED_SUCCESS)
//            {
//                    WPRINT_APP_INFO(("ota2_test_get_update() failed! %d \r\n", result));
//            }
//            else
//            {
//                WPRINT_APP_INFO(("ota2_test_get_update() done.\r\n"));
//            }
//        }
//    }

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
    if ( flag == FW_FIRST )
    {
        WPRINT_APP_INFO(("Release Download Started\r\n"));
    }

    return 0;
}

int arrow_release_download_init(const char * token, const char * hid)
{
    WPRINT_APP_INFO(("Arrow Release Download Init Callback!\r\n"));
    WPRINT_APP_INFO(("Download Token: %s\r\n", token));
    WPRINT_APP_INFO(("Download HID: %s\r\n", hid));

    CREATE_CHUNK(uri, URI_LEN);
    int n = snprintf(uri, URI_LEN, ARROW_API_SOFTWARE_RELEASE_ENDPOINT "/%s/%s/file", hid, token);
    uri[n] = 0x0;

    WPRINT_APP_INFO(("Update URL: %s\r\n", uri));
    memset(g_player->uri_to_stream, 0, sizeof(g_player->uri_to_stream));
    strlcpy(g_player->uri_to_stream, uri, (sizeof(g_player->uri_to_stream) - 1) );

    update_in_progress = WICED_TRUE;

    wiced_result_t result = ota2_test_get_update(g_player);
    if (result != WICED_SUCCESS)
    {
            WPRINT_APP_INFO(("ota2_test_get_update() failed! %d \r\n", result));
    }
    else
    {
        WPRINT_APP_INFO(("ota2_test_get_update() done.\r\n"));
    }

    FREE_CHUNK(uri);

    return 0;
}

int arrow_release_download_complete(int flag)
{
    if(flag == FW_SUCCESS)
    {
        WPRINT_APP_INFO(("Release Download Completed Successfully\r\n"));
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

static wiced_result_t ota2_init(void)
{
    ota2_data_t*   player;

    if ((player = init_player()) == NULL)
    {
        return WICED_ERROR;
    }
    g_player = player;

    return WICED_SUCCESS;
}

void application_start( )
{
    /* Initialize the WICED platform */
    VERIFY_SUCCESS(wiced_init());

    VERIFY_SUCCESS(quicksilver_init());

    VERIFY_SUCCESS(arrow_cloud_init());

    ota2_init();

    while(1)
    {
        if(!update_in_progress)
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
        else
        {
            DELAY_MS(500);
        }
    }
}

wiced_result_t my_ota2_callback(void* session_id, wiced_ota2_service_status_t status, uint32_t value, void* opaque )
{
    ota2_data_t*    player = (ota2_data_t*)opaque;

    UNUSED_PARAMETER(session_id);
    UNUSED_PARAMETER(player);


    switch( status )
    {
    case OTA2_SERVICE_STARTED:      /* Background service has started
                                         * return - None                                                             */
        WPRINT_APP_INFO(("----------------------------- OTA2 Service Called : SERVE STARTED -----------------------------\r\n"));
        break;
    case OTA2_SERVICE_AP_CONNECT_ERROR:
        WPRINT_APP_INFO(("----------------------------- OTA2 Service Called : AP CONNECT_ERROR -----------------------------\r\n"));
        WPRINT_APP_INFO(("        return SUCESS (not used by service). This is informational \r\n"));
        break;

    case OTA2_SERVICE_SERVER_CONNECT_ERROR:
        WPRINT_APP_INFO(("----------------------------- OTA2 Service Called : SERVER_CONNECT_ERROR -----------------------------\r\n"));
        WPRINT_APP_INFO(("        return SUCESS (not used by service). This is informational \r\n"));
        break;

    case OTA2_SERVICE_AP_CONNECTED:
        WPRINT_APP_INFO(("----------------------------- OTA2 Service Called : AP_CONNECTED -----------------------------\r\n"));
        WPRINT_APP_INFO(("        return SUCESS (not used by service). This is informational \r\n"));
        break;

    case OTA2_SERVICE_SERVER_CONNECTED:
        WPRINT_APP_INFO(("----------------------------- OTA2 Service Called : SERVER_CONNECTED -----------------------------\r\n"));
        WPRINT_APP_INFO(("        return SUCESS (not used by service). This is informational \r\n"));
        break;


    case OTA2_SERVICE_CHECK_FOR_UPDATE: /* Time to check for updates.
                                         * return - WICED_SUCCESS = Service will check for update availability
                                         *        - WICED_ERROR   = Application will check for update availability   */
        WPRINT_APP_INFO(("----------------------------- OTA2 Service Called : CHECK_FOR_UPDATE -----------------------------\r\n"));
        WPRINT_APP_INFO(("        return SUCCESS, let Service do the checking.\r\n"));
        return WICED_SUCCESS;

    case OTA2_SERVICE_UPDATE_AVAILABLE: /* Service has contacted server, update is available
                                         * return - WICED_SUCCESS = Application indicating that it wants the
                                         *                           OTA Service to perform the download
                                         *        - WICED_ERROR   = Application indicating that it will perform
                                         *                           the download, the OTA Service will do nothing.  */
    {
        /* the OTA2 header for the update is pointed to by the value argument and is only valid for this function call */
        wiced_ota2_image_header_t* ota2_header;

        ota2_header = (wiced_ota2_image_header_t*)value;

        WPRINT_APP_INFO(("----------------------------- OTA2 Service Called : UPDATE_AVAILABLE -----------------------------\r\n"));
        /* the OTA2 header for the update is pointed to by the value argument and is only valid for this function call */

        /*
         * In an actual application, the application would look at the headers information and decide if the
         * file on the update server is a newer version that the currently running application.
         *
         * If the application wants the update to continue, it would return WICED_SUCCESS here
         * If not, return WICED_ERROR
         *
         */
        WPRINT_APP_INFO(("Current Version %d.%d\r\n", player->dct_app->ota2_major_version,
                                                                      player->dct_app->ota2_minor_version));
        WPRINT_APP_INFO(("   OTA2 Version %d.%d\r\n", ota2_header->major_version, ota2_header->minor_version));

#if defined(CHECK_OTA2_UPDATE_VERSION)
        if ((player->dct_app->ota2_major_version > ota2_header->major_version) ||
            ((player->dct_app->ota2_major_version == ota2_header->major_version) &&
             (player->dct_app->ota2_minor_version >= ota2_header->minor_version)) )
        {
            WPRINT_APP_INFO(("OTA2 Update Version Fail - return ERROR, do not update!\r\n"));
            return WICED_ERROR;
        }
#endif
        WPRINT_APP_INFO(("        return SUCCESS, let Service perform the download.\r\n"));
        return WICED_SUCCESS;
    }

    case OTA2_SERVICE_DOWNLOAD_STATUS:  /* Download status - value has % complete (0-100)
                                         *   NOTE: This will only occur when Service is performing download
                                         * return - WICED_SUCCESS = Service will continue download
                                         *        - WICED_ERROR   = Service will STOP download and service will
                                         *                          issue OTA2_SERVICE_TIME_TO_UPDATE_ERROR           */
        WPRINT_APP_INFO(("my_ota2_callback() OTA2_SERVICE_DOWNLOAD_STATUS %ld %%!\r\n", value));
        return WICED_SUCCESS;

    case OTA2_SERVICE_PERFORM_UPDATE:   /* Download is complete
                                         * return - WICED_SUCCESS = Service will inform Bootloader to extract
                                         *                          and update on next power cycle
                                         *        - WICED_ERROR   = Service will inform Bootloader that download
                                         *                          is complete - Bootloader will NOT extract        */
        WPRINT_APP_INFO(("----------------------------- OTA2 Service Called : PERFORM_UPDATE -----------------------------\r\n"));
        WPRINT_APP_INFO(("        return SUCCESS, let Service extract update on next reboot.\r\n"));
        return WICED_SUCCESS;

    case OTA2_SERVICE_UPDATE_ERROR:     /* There was an error in transmission
                                         * This will only occur if Error during Service performing data transfer
                                         * upon return - if retry_interval is set, Service will use retry_interval
                                         *               else, Service will retry on next check_interval
                                         */
        WPRINT_APP_INFO(("----------------------------- OTA2 Service Called : UPDATE_ERROR -----------------------------\r\n"));
        WPRINT_APP_INFO(("        return SUCCESS, Service will retry as parameters defined in init.\r\n"));
        return WICED_SUCCESS;

    case OTA2_SERVICE_UPDATE_ENDED:     /* All update actions for this check are complete.
                                         * This callback is to allow the application to take any actions when
                                         * the service is done checking / downloading an update
                                         * (succesful, unavailable, or error)
                                         * return - None                                                             */
        WPRINT_APP_INFO(("----------------------------- OTA2 Service Called : UPDATE_ENDED -----------------------------\r\n"));
        WPRINT_APP_INFO(("        return SUCESS (not used by service). This is informational \r\n"));
        update_in_progress = WICED_FALSE;
        if ((value == WICED_SUCCESS) && (player->dct_app->ota2_reboot_after_download != 0))
        {
            WPRINT_APP_INFO(("        REBOOTING !!!\r\n"));
            wiced_framework_reboot();
        }

        break;

    case OTA2_SERVICE_STOPPED:
        WPRINT_APP_INFO(("----------------------------- OTA2 Service Called : SERVICE ENDED -----------------------------\r\n"));
        WPRINT_APP_INFO(("        return SUCESS (not used by service). This is informational \r\n"));
        break;

    default:
        WPRINT_APP_INFO(("my_ota2_callback() UNKNOWN STATUS %d!\r\n", status));
        break;
    }

    return WICED_SUCCESS;
}

wiced_result_t ota2_test_get_update(ota2_data_t* player)
{
    wiced_result_t result = WICED_ERROR;

    /* get the image from the server & save in staging area */

    wiced_ota2_service_uri_split(player->uri_to_stream, player->ota2_host_name, sizeof(player->ota2_host_name),
                                     player->ota2_file_path, sizeof(player->ota2_file_path), &player->ota2_bg_params.port);

    player->ota2_bg_params.host_name                = player->ota2_host_name;
    player->ota2_bg_params.file_path                = player->ota2_file_path;

    player->ota2_bg_params.auto_update              = 0;
    player->ota2_bg_params.initial_check_interval   = 5;            /* initial check in 5 seconds */
    player->ota2_bg_params.check_interval           = 10 * 60;      /* 10 minutes - use SECONDS_IN_24_HOURS for 1 day */
    player->ota2_bg_params.retry_check_interval     = SECONDS_PER_MINUTE;   /* minimum retry is 1 minute */
    player->ota2_bg_params.max_retries              = 3;       /* maximum retries per update attempt         */
    player->ota2_bg_params.default_ap_info          = player->dct_wifi->stored_ap_list;
    player->ota2_bg_params.ota2_interface           = player->dct_network->interface;
#ifdef WICED_USE_ETHERNET_INTERFACE
    if (player->ota2_bg_params.ota2_interface == WICED_ETHERNET_INTERFACE)
    {
        if ( wiced_network_is_up( WICED_ETHERNET_INTERFACE) == WICED_FALSE )
        {
            /* Currently not connected to Ethernet, use WiFI */
            player->ota2_bg_params.ota2_interface = WICED_STA_INTERFACE;
        }
    }
#endif
    if (player->ota2_bg_params.ota2_interface != WICED_ETHERNET_INTERFACE)
    {
        player->ota2_bg_params.ota2_ap_info             = NULL;
        player->ota2_bg_params.ota2_ap_list             = &player->dct_wifi->stored_ap_list[0]; /* use the DCT AP list */
        player->ota2_bg_params.ota2_ap_list_count       = CONFIG_AP_LIST_SIZE;
    }

    player->deinit_ota2_bg = WICED_TRUE;
    if (player->ota2_bg_service == NULL)
    {
        player->ota2_bg_service = wiced_ota2_service_init(&player->ota2_bg_params, player);
        WPRINT_APP_INFO(("ota2_test_get_update() wiced_ota2_service_init() bg_service:%p \r\n", player->ota2_bg_service));
    }
    else
    {
        /* bg service already started - this is OK, just don't deinit at the end of this function */
        player->deinit_ota2_bg = WICED_FALSE;
    }

    if (player->ota2_bg_service != NULL)
    {
        /* add a callback */
        result = wiced_ota2_service_register_callback(player->ota2_bg_service, my_ota2_callback);
        if (result != WICED_SUCCESS)
        {
            WPRINT_APP_INFO(("ota2_test_get_update register callback failed! %d \r\n", result));
            wiced_ota2_service_deinit(player->ota2_bg_service);
            player->ota2_bg_service = NULL;
        }

        if (player->ota2_bg_service != NULL)
        {
            WPRINT_APP_INFO(("Download the OTA Image file - get it NOW!\r\n"));
            /* NOTE: THis is a blocking call! */
            result = wiced_ota2_service_check_for_updates(player->ota2_bg_service);
            if (result != WICED_SUCCESS)
            {
                WPRINT_APP_INFO(("ota2_test_get_update wiced_ota2_service_check_for_updates() failed! %d \r\n", result));
            }
        }
    }
    return result;
}

static ota2_data_t* init_player(void)
{
    ota2_data_t*            player = NULL;
    wiced_result_t          result;
    uint32_t                tag;
    ota2_boot_type_t        boot_type;

   /*
     * Allocate the main data structure.
     */
    player = calloc_named("ota2_test", 1, sizeof(ota2_data_t));
    if (player == NULL)
    {
        WPRINT_APP_INFO(("Unable to allocate player structure\r\n"));
        return NULL;
    }

    /* read in our configurations */
    ota2_test_config_init(player);

    /* determine if this is a first boot, factory reset, or after an update boot */
    boot_type = wiced_ota2_get_boot_type();
    switch( boot_type )
    {
        case OTA2_BOOT_FAILSAFE_FACTORY_RESET:
        case OTA2_BOOT_FAILSAFE_UPDATE:
        default:
            /* We should never get here! */
            WPRINT_APP_INFO(("Unexpected boot_type %d!\r\n", boot_type));
            /* FALL THROUGH */
        case OTA2_BOOT_NEVER_RUN_BEFORE:
            WPRINT_APP_INFO(("First BOOT EVER\r\n"));
            /* Set the reboot type back to normal so we don't think we updated next reboot */
            wiced_dct_ota2_save_copy( OTA2_BOOT_NORMAL );
            break;
        case OTA2_BOOT_NORMAL:
            WPRINT_APP_INFO(("Normal reboot - count:%ld.\r\n", player->dct_app->reboot_count));
            break;
        case OTA2_BOOT_EXTRACT_FACTORY_RESET:   /* pre-OTA2 failsafe ota2_bootloader designation for OTA2_BOOT_FACTORY_RESET */
        case OTA2_BOOT_FACTORY_RESET:
            WPRINT_APP_INFO(("Factory Reset Occurred!\r\n"));
#if RESTORE_DCT_APP_SETTINGS
            over_the_air_2_app_restore_settings_after_update(player, boot_type);
#endif
            /* Set the reboot type back to normal so we don't think we updated next reboot */
            wiced_dct_ota2_save_copy( OTA2_BOOT_NORMAL );
            break;
        case OTA2_BOOT_EXTRACT_UPDATE:   /* pre-OTA2 failsafe ota2_bootloader designation for OTA2_BOOT_UPDATE */
        case OTA2_BOOT_SOFTAP_UPDATE:    /* pre-OTA2 failsafe ota2_bootloader designation for a SOFTAP update */
        case OTA2_BOOT_UPDATE:
            WPRINT_APP_INFO(("Update Occurred!\r\n"));
#if RESTORE_DCT_APP_SETTINGS
            over_the_air_2_app_restore_settings_after_update(player, boot_type);
#endif
            /* Set the reboot type back to normal so we don't think we updated next reboot */
            wiced_dct_ota2_save_copy( OTA2_BOOT_NORMAL );
            break;
        case OTA2_BOOT_LAST_KNOWN_GOOD:
            WPRINT_APP_INFO(("Last Known Good used!\r\n"));
            break;
    }

    /* Get RTC Clock time and set it here */
    {
        wiced_time_t time = 0;
        wiced_time_set_time( &time );
    }

    return player;
}


wiced_result_t over_the_air_2_app_restore_settings_after_update(ota2_data_t* player, ota2_boot_type_t boot_type)
{
    uint16_t major = 0, minor = 0;
    platform_dct_network_config_t   dct_network = { 0 };
    platform_dct_wifi_config_t      dct_wifi = { 0 };
    ota2_dct_t                      dct_app = { 0 };

    /* read in our configurations from the DCT copy */
    /* network */
    if (wiced_dct_ota2_read_saved_copy( &dct_network, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t)) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("over_the_air_2_app_restore_settings_after_update() failed reading Network Config!\r\n"));
        return WICED_ERROR;
    }

    /* wifi */
    if (wiced_dct_ota2_read_saved_copy( &dct_wifi, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t)) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("over_the_air_2_app_restore_settings_after_update() failed reading WiFi Config!\r\n"));
        return WICED_ERROR;
    }

    /* App */
    if (wiced_dct_ota2_read_saved_copy( &dct_app, DCT_APP_SECTION, 0, sizeof(ota2_dct_t)) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("over_the_air_2_app_restore_settings_after_update() failed reading App Config!\r\n"));
        return WICED_ERROR;
    }

    memcpy(player->dct_network, &dct_network, sizeof(platform_dct_network_config_t));
    memcpy(player->dct_wifi, &dct_wifi, sizeof(platform_dct_wifi_config_t));
    memcpy(player->dct_app, &dct_app, sizeof(ota2_dct_t));

    /* update version number based on boot type */
    switch (boot_type)
    {
        default:
            break;
        case OTA2_BOOT_EXTRACT_FACTORY_RESET:   /* pre-OTA2 failsafe ota2_bootloader designation for OTA2_BOOT_FACTORY_RESET */
        case OTA2_BOOT_FACTORY_RESET:
            if (wiced_ota2_image_get_version( WICED_OTA2_IMAGE_TYPE_FACTORY_RESET_APP, &major, &minor) == WICED_SUCCESS)
            {
                player->dct_app->ota2_major_version = major;
                player->dct_app->ota2_minor_version = minor;
            }
            break;
        case OTA2_BOOT_SOFTAP_UPDATE:
        case OTA2_BOOT_EXTRACT_UPDATE:   /* pre-OTA2 failsafe ota2_bootloader designation for OTA2_BOOT_UPDATE */
        case OTA2_BOOT_UPDATE:
            if (wiced_ota2_image_get_version( WICED_OTA2_IMAGE_TYPE_STAGED, &major, &minor) == WICED_SUCCESS)
            {
                player->dct_app->ota2_major_version = major;
                player->dct_app->ota2_minor_version = minor;
            }
            break;
    }

    /* now, save them all! */
    if (ota2_save_config(player) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("over_the_air_2_app_restore_settings_after_update() failed Saving Config!\r\n"));
        return WICED_ERROR;
    }

    WPRINT_APP_INFO(("Restored saved Configuration!\r\n"));
    return WICED_SUCCESS;
}

