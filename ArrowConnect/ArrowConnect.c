
#include <stdlib.h>
#include "wiced.h"
#include "wiced_tls.h"
#include "command_console.h"
#include "device_onboarding.h"
#include "ArrowConnect.h"

#include "quicksilver.h"

#include "arrow/api/gateway/info.h"
#include "arrow/api/gateway/gateway.h"


//#include <bsd/socket.h>
//#include <time/time.h>
//#include <ntp/ntp.h>
#include <arrow/routine.h>
#include <arrow/utf8.h>
//#include <sys/mac.h>
//#include <ssl/ssl.h>
#include <debug.h>

#include <math.h>
#include "resources.h"
#include "wiced_management.h"
#include "command_console_ping.h"

#include "Drivers/Sensors/LIS2DH12/lis2dh12.h"
#include "Drivers/Sensors/HTS221/hts221.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define PING_TIMEOUT_MS          2000
#define DELAY_MS(x)             wiced_rtos_delay_milliseconds(x)

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
static const wiced_i2c_device_t i2c_device_temperature =
{
        .port = WICED_I2C_2,  //I2C_1
        .address = HTS221_I2C_ADDRESS,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .speed_mode = I2C_STANDARD_SPEED_MODE,
};

static const wiced_i2c_device_t i2c_device_accelerometer =
{
        .port = WICED_I2C_2,  //I2C_1
        .address = LIS2DH12_I2C_ADD_H,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .speed_mode = I2C_STANDARD_SPEED_MODE,
};

static color ledColor = {
        .Red = 0,
        .Green = 0,
        .Blue = 0,
};

static const wiced_ip_setting_t ap_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS( 255,255,255,  0 ) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
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
static wiced_semaphore_t app_semaphore;
static uint32_t onboarding_status;

/******************************************************
 *               Function Definitions
 ******************************************************/

/* When WiFi onboarding service is started, application will wait for
 * this callback(onboarding either failed or succeed), application then may
 * decide the next step. For example - if onboarding is success, application
 * may call the wifi_onboarding_stop and trigger a reboot.
 * On onboarding failure, application may retry certain number of times by starting the
 * onboarding service again(after stopping it first)
 */
static void app_wifi_onboarding_callback( uint32_t result )
{
    WPRINT_APP_INFO( ( "[App] WiFi Onboarding callback..." ) );
    if( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("Onboarding Failed\r\n" ) );
    }
    else
    {
        WPRINT_APP_INFO( ("Onboarding successfull\r\n" ) );
    }

    onboarding_status = result;

    wiced_rtos_set_semaphore(&app_semaphore);
}

int32_t i2c_write_accel(void * dev_ctx, uint8_t reg, uint8_t* buffer, uint16_t length)
{
    uint8_t write_buffer[length + 1];
    write_buffer[0] = reg;
    memcpy(&write_buffer[1], buffer, length);

    wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, write_buffer, sizeof(write_buffer) );

    return 0;
}

int32_t i2c_read_accel(void * dev_ctx, uint8_t reg, uint8_t* buffer, uint16_t length)
{
    uint8_t tx_buffer [1] = {reg};

    wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, tx_buffer, 1 );
    wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, buffer, length );

    return 0;
}

int32_t i2c_write_hts(void * dev_ctx, uint8_t reg, uint8_t* buffer, uint16_t length)
{
    uint8_t write_buffer[length + 1];
    write_buffer[0] = reg;
    memcpy(&write_buffer[1], buffer, length);

    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, write_buffer, sizeof(write_buffer) );

    return 0;
}

int32_t i2c_read_hts(void * dev_ctx, uint8_t reg, uint8_t* buffer, uint16_t length)
{
    uint8_t tx_buffer [1] = {reg};

    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, tx_buffer, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, buffer, length );

    return 0;
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

    return probe_sensors();
}

wiced_result_t probe_sensors(void)
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

/*
 *  Function used to apply coefficient
 */
float linear_interpolation(lin_t *lin, int16_t x)
{
 return ((lin->y1 - lin->y0) * x +  ((lin->x1 * lin->y0) - (lin->x0 * lin->y1)))
        / (lin->x1 - lin->x0);
}

/*
 * Holder function to get HTS221 temperature
 */
int temperature_get(int argc, char *argv[]){

    hts221_reg_t reg;
    hts221_status_get(&hts_ctx, &reg.status_reg);

    if (reg.status_reg.h_da)
    {
      /* Read humidity data */
      memset(data_raw_humidity.u8bit, 0x00, sizeof(int16_t));
      hts221_read_reg(&hts_ctx, HTS221_HUMIDITY_OUT_L, data_raw_humidity.u8bit, 2);
      data_raw_humidity.i16bit =(data_raw_humidity.u8bit[1]<<8 | data_raw_humidity.u8bit[0]);
      humidity_perc = linear_interpolation(&lin_hum, data_raw_humidity.i16bit);
      if (humidity_perc < 0) humidity_perc = 0;
      if (humidity_perc > 100) humidity_perc = 100;
    }
    if (reg.status_reg.t_da)
    {
      /* Read temperature data */
      memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
      hts221_read_reg(&hts_ctx, HTS221_TEMP_OUT_L, data_raw_temperature.u8bit, 2);
      data_raw_temperature.i16bit =(data_raw_temperature.u8bit[1]<<8 | data_raw_temperature.u8bit[0]);
      temperature_degC = linear_interpolation(&lin_temp, data_raw_temperature.i16bit);
    }

    telemetryData.temperature = temperature_degC;
    telemetryData.humidity = humidity_perc;

    return 0;
}

/*
 * Holder function to get LIS2DH12 accelerometer
 */
int accelerometer_get(int argc, char *argv[]){

    lis2dh12_reg_t reg;
    lis2dh12_status_get(&accel_ctx, &reg.status_reg);

    if( reg.status_reg.zyxda ) {

        lis2dh12_read_reg(&accel_ctx, LIS2DH12_OUT_X_L, data_raw_acceleration.u8bit, 6);

        data_raw_acceleration.i16bit[0] =(data_raw_acceleration.u8bit[1]<<8 | data_raw_acceleration.u8bit[0]);
        data_raw_acceleration.i16bit[1] =(data_raw_acceleration.u8bit[3]<<8 | data_raw_acceleration.u8bit[2]);
        data_raw_acceleration.i16bit[2] =(data_raw_acceleration.u8bit[5]<<8 | data_raw_acceleration.u8bit[4]);

        acceleration_mg[0] = LIS2DH12_FROM_FS_4g_HR_TO_mg( data_raw_acceleration.i16bit[0] );
        acceleration_mg[1] = LIS2DH12_FROM_FS_4g_HR_TO_mg( data_raw_acceleration.i16bit[1] );
        acceleration_mg[2] = LIS2DH12_FROM_FS_4g_HR_TO_mg( data_raw_acceleration.i16bit[2] );

        telemetryData.accelerometer.x = acceleration_mg[0];
        telemetryData.accelerometer.y = acceleration_mg[1];
        telemetryData.accelerometer.z = acceleration_mg[2];

        DBG("Accel X: %d", telemetryData.accelerometer.x);

    }

    return 0;
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
            wiced_gpio_output_low( WICED_RGB_CLOCK );

            if ( data_array[i] & mask )
            {
                wiced_gpio_output_high( WICED_RGB_DATA );
            }
            else
            {
                wiced_gpio_output_low( WICED_RGB_DATA );
            }

            wiced_gpio_output_high( WICED_RGB_CLOCK );

            mask >>= 1; // right shift by 1
        }
    }
}

/*
 * Initializes RGB LED
 */
wiced_result_t rgb_init( void )
{
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

    wiced_gpio_init( WICED_RGB_CLOCK, OUTPUT_PUSH_PULL );
    wiced_gpio_init( WICED_RGB_DATA, OUTPUT_PUSH_PULL );

    wiced_gpio_output_high( WICED_RGB_CLOCK );
    wiced_gpio_output_high( WICED_RGB_DATA );

    for ( int i = 0; i< 8; i++ )
    {
        show_color(Rainbow[i]);
        wiced_rtos_delay_milliseconds(500);
    }

    wiced_gpio_output_high( WICED_RGB_CLOCK );
    wiced_gpio_output_high( WICED_RGB_DATA );

    show_color(ledColor);

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
    color RGBColor = {0};

    DBG("---------------------------------------------");
    DBG("rgb_handler: %s", data);
    DBG("---------------------------------------------");
    JsonNode *main = json_decode(data);
    if(main)
    {
        JsonNode *red_val = json_find_member(main, "red");
        if(red_val && red_val->tag == JSON_NUMBER)
        {
            RGBColor.Red = (int)red_val->number_;
        }
        JsonNode *green_val = json_find_member(main, "green");
        if(green_val && green_val->tag == JSON_NUMBER)
        {
            RGBColor.Green = (int)green_val->number_;
        }
        JsonNode *blue_val = json_find_member(main, "blue");
        if(blue_val && blue_val->tag == JSON_NUMBER)
        {
            RGBColor.Blue = (int)blue_val->number_;
        }
        json_delete(main);

        show_color(RGBColor);
    }
    else
    {
        DBG("json parse failed");
    }

    return 0;
}

wiced_result_t quicksilver_ap_init(void)
{
    wiced_bool_t* device_configured;
    wiced_result_t result;

    result = wiced_dct_read_lock( (void**) &device_configured, WICED_FALSE, DCT_WIFI_CONFIG_SECTION, OFFSETOF(platform_dct_wifi_config_t, device_configured), sizeof(wiced_bool_t) );

    if( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("[App] Error fetching platform DCT section\n") );
        wiced_dct_read_unlock(device_configured, WICED_FALSE);
        return result;
    }

    if( *device_configured != WICED_TRUE )
    {
        wiced_rtos_init_semaphore(&app_semaphore);

        WPRINT_APP_INFO( ("[App] Starting Wifi Onboarding service...\n") );

        result = wiced_wifi_device_onboarding_start(&ap_ip_settings, app_wifi_onboarding_callback);

        if( result != WICED_SUCCESS )
        {
            WPRINT_APP_INFO( ("[App] Error Starting the on-boarding of device\n") );
            wiced_dct_read_unlock(device_configured, WICED_FALSE);
            return result;
        }

        WPRINT_APP_INFO( ("[App] Waiting for Onboarding callback...\n") );

        result = wiced_rtos_get_semaphore(&app_semaphore, WICED_NEVER_TIMEOUT);
        if( result != WICED_SUCCESS )
        {
            WPRINT_APP_INFO( ( "[App] Sempahore get error or timeout\r\n" ) );
            wiced_rtos_deinit_semaphore(&app_semaphore);
            return result;
        }

        WPRINT_APP_INFO( ("[App] Stopping Wifi Onboarding service...\n") );
        result = wiced_wifi_device_onboarding_stop();

        if( result != WICED_SUCCESS )
        {
            WPRINT_APP_INFO( ("[App] Error Stopping the on-boarding service\n") );
            wiced_rtos_deinit_semaphore(&app_semaphore);
            return result;
        }

        wiced_framework_reboot();
    }

    WPRINT_APP_INFO( ( "[App] Normal Application start\n" ) );

    result = wiced_network_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "[App] Failed to join network \n") );
        wiced_dct_read_unlock(device_configured, WICED_FALSE);
        return result;
    }

    WPRINT_APP_INFO( ( "[App] Started STA successfully with saved configuration\n" ) );
    wiced_dct_read_unlock(device_configured, WICED_FALSE);
    return WICED_SUCCESS;
}

void application_start( )
{
    wiced_result_t result;

    /* Initialize the WICED platform */
    result = wiced_init();
    if(result != WICED_SUCCESS)
    {
        DBG("Error Initializing WICED");
    }

    /* Initialize I2C*/
    result = i2c_init();
    if(result != WICED_SUCCESS)
    {
        DBG("Error Initializing I2C");
    }

    /* Initialize the RGB */
    result = rgb_init();
    if(result != WICED_SUCCESS)
    {
        DBG("Error Initializing RGB");
    }

    /* probe for temperature device */
    result = temperature_init();
    if(result != WICED_SUCCESS)
    {
        DBG("Error Initializing Temp/Humidity");
    }

    /* probe for accelerometer device */
    accelerometer_init();
    if(result != WICED_SUCCESS)
    {
        DBG("Error Initializing Accelerometer");
    }

    result = quicksilver_ap_init();
    if(result != WICED_SUCCESS)
    {
        DBG("Error Initializing AP Server");
        while(1);
    }

    /* Register the Quicksilver board as both a gateway and device and establish HTTP connection */
    arrow_initialize_routine();

    /* Connect to the MQTT Service */
    arrow_mqtt_connect_routine();

    // Add command handlers
    add_cmd_handler("rgb", rgb_handler);

    while(1)
    {
        /* Send the latest data to Arrow Connect */
        arrow_mqtt_send_telemetry_routine(update_sensor_data, &telemetryData);
    }


}
