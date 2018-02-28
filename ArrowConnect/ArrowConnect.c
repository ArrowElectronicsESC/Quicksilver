
#include <stdlib.h>
#include "wiced.h"
#include "wiced_tls.h"
#include "command_console.h"
#include "ArrowConnect.h"

#include "quicksilver.h"

#include "arrow/api/gateway/info.h"
#include "arrow/api/gateway/gateway.h"


#include <bsd/socket.h>
#include <time/time.h>
#include <ntp/ntp.h>
#include <arrow/routine.h>
#include <arrow/utf8.h>
#include <sys/mac.h>
#include <ssl/ssl.h>
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

/******************************************************
 *                    Constants
 ******************************************************/

#define RGB_CLOCK WICED_GPIO_10
#define RGB_DATA WICED_GPIO_8

#define BUFFER_LENGTH     (2048)
#define MAX_LINE_LENGTH  (128)
#define MAX_HISTORY_LENGTH (20)
#define MAX_NUM_COMMAND_TABLE  (8)

#define HTS221_SLAVE_ADDR        (0x5F)
#define HTS221_WOAMI_REG         (0x0F | 0x80)
#define HTS221_CTRL_REG1         (0x20 | 0x80)
#define HTS221_TEMP_OUT_L        (0x2A | 0x80)
#define HTS221_TEMP_OUT_H        (0x2B | 0x80)
#define HTS221_T0_DEGC_X8        (0x32 | 0x80)
#define HTS221_T1_DEGC_X8        (0x33 | 0x80)
#define HTS221_T1_T0_MSB         (0x35 | 0x80)
#define HTS221_T0_OUT_L          (0x3C | 0x80)
#define HTS221_T0_OUT_H          (0x3D | 0x80)
#define HTS221_T1_OUT_L          (0x3E | 0x80)
#define HTS221_T1_OUT_H          (0x3F | 0x80)

#define HTS221_CTRL1_PD          (0x80)
#define HTS221_CTRL1_BDU         (0x02)

#define LIS2DH12_SLAVE_ADDR      (0x19)
#define LIS2DH12_WOAMI_REG       (0x0F | 0x80)
#define LIS2DH12_CTRL_REG0       (0x1E | 0x80)
#define LIS2DH12_CTRL_REG1       (0x20 | 0x80)
#define LIS2DH12_CTRL_REG2       (0x21 | 0x80)
#define LIS2DH12_CTRL_REG3       (0x22 | 0x80)
#define LIS2DH12_CTRL_REG4       (0x23 | 0x80)
#define LIS2DH12_CTRL_REG5       (0x24 | 0x80)
#define LIS2DH12_CTRL_REG6       (0x25 | 0x80)
#define LIS2DH12_STATUS_REG      (0x27 | 0x80)
#define LIS2DH12_OUT_X_L         (0x28 | 0x80)
#define LIS2DH12_OUT_X_H         (0x29 | 0x80)
#define LIS2DH12_OUT_Y_L         (0x2A | 0x80)
#define LIS2DH12_OUT_Y_H         (0x2B | 0x80)
#define LIS2DH12_OUT_Z_L         (0x2C | 0x80)
#define LIS2DH12_OUT_Z_H         (0x2D | 0x80)

#define LIS2DH12_CTRL1_ODR_400    (0x07<<4) // 400 Hz
#define LIS2DH12_CTRL1_ODR_25     (0x03<<4) // 25 Hz
#define LIS2DH12_CTRL1_ODR_10     (0x02<<4) // 10 Hz
#define LIS2DH12_CTRL1_ODR_1      (0x01<<4) // 1 Hz
#define LIS2DH12_CTRL1_ZEN        (0x01<<2)
#define LIS2DH12_CTRL1_YEN        (0x01<<1)
#define LIS2DH12_CTRL1_XEN        (0x01)

#define LIS2DH12_CTRL4_BDU        (0x80)

#define LIS2DH12_STAT_ZYXDA       (0x01<<3)

#define NUM_I2C_MESSAGE_RETRIES   (3)

#define SPI_CLOCK_SPEED_HZ        ( 500000 )
#define SPI_BIT_WIDTH             ( 8 )
#define SPI_MODE                  ( SPI_CLOCK_FALLING_EDGE | SPI_CLOCK_IDLE_LOW | SPI_MSB_FIRST | SPI_CS_ACTIVE_LOW )

#define MCP3208_START             (0x01<<10)
#define MCP3208_SE                (0x01<<9)
#define MCP3208_DIFF              (0x00<<9)
#define MCP3208_CH0               (0x00<<6)
#define MCP3208_CH1               (0x01<<6)
#define MCP3208_CH2               (0x02<<6)
#define MCP3208_CH3               (0x03<<6)
#define MCP3208_CH4               (0x04<<6)
#define MCP3208_CH5               (0x05<<6)
#define MCP3208_CH6               (0x06<<6)
#define MCP3208_CH7               (0x07<<6)

#define RESET_PIN WICED_GPIO_14

int wifi_connect(int argc, char *argv[]);
int do_ntp_time(int argc, char *argv[]);
int arrow_connect_test(int argc, char *argv[]);
int find_gateway_by_os(int argc, char *argv[]);
int send_telemetry(int argc, char *argv[]);

#define DIAGNOSTICS_COMMANDS \
{ "wifi",  wifi_connect,  NULL, NULL, NULL, "", "does a wifi" }, \
{ "ntp_set_time_cycle", do_ntp_time, NULL, NULL, NULL, "", "does a ntp"},\
{ "arrow_connect", arrow_connect_test, NULL, NULL, NULL, "", "does a connect"},\
{ "gateway", find_gateway_by_os, NULL, NULL, NULL, "", "does a gateway"},\
{ "telemetry", send_telemetry, 1, NULL, NULL, "", "does a telemetry"},

#if 0
static char line_buffer[MAX_LINE_LENGTH];
static char history_buffer_storage[MAX_LINE_LENGTH * MAX_HISTORY_LENGTH];

static const command_t init_commands[] = {
        DIAGNOSTICS_COMMANDS
        CMD_TABLE_END
};
#endif
/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

//typedef struct color
//{
//    unsigned char Red;
//    unsigned char Green;
//    unsigned char Blue;
//} color;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static wiced_result_t send_ping              ( int interface );
static wiced_result_t print_wifi_config_dct ( void );
wiced_result_t scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result );
wiced_result_t temperature_init( void );
wiced_result_t accelerometer_init( void );
wiced_result_t adc_init( void );
wiced_result_t rgb_init( void );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static char line_buffer[MAX_LINE_LENGTH];
static char history_buffer_storage[MAX_LINE_LENGTH * MAX_HISTORY_LENGTH];

static wiced_ip_address_t  ping_target_ip;
static wiced_usb_user_config_t usb_host_config;

static quicksilver_data telemetryData;

static wiced_i2c_device_t i2c_device_temperature =
{
        .port = WICED_I2C_2,  //I2C_1
        .address = HTS221_SLAVE_ADDR,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .speed_mode = I2C_STANDARD_SPEED_MODE,
};

static wiced_i2c_device_t i2c_device_accelerometer =
{
        .port = WICED_I2C_2,  //I2C_1
        .address = LIS2DH12_SLAVE_ADDR,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .speed_mode = I2C_STANDARD_SPEED_MODE,
};

static const wiced_spi_device_t spi0_device =
{
        .port        = WICED_SPI_1,
        .chip_select = WICED_GPIO_22,
        .speed       = SPI_CLOCK_SPEED_HZ,
        .mode        = SPI_MODE,
        .bits        = SPI_BIT_WIDTH
};

static const wiced_spi_device_t spi1_device =
{
        .port        = WICED_SPI_2,
        .chip_select = WICED_GPIO_NONE,
        .speed       = SPI_CLOCK_SPEED_HZ,
        .mode        = SPI_MODE,
        .bits        = SPI_BIT_WIDTH
};

typedef struct {
  float x0;
  float y0;
  float x1;
  float y1;
} lin_t;

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
static color ledColor = {
        .Red = 0,
        .Green = 0,
        .Blue = 0,
};

#define DIAGNOSTICS_COMMANDS \
{ "config",  wifi_config,  1, NULL, NULL, "<STA name and pass key>", "adds AP settings to DCT" }, \
{ "print_config",  print_wifi,  0, NULL, NULL,"",  "prints current wifi configuration" }, \
{ "scan",  scan_wifi,  0, NULL, NULL,"",  "scans for broadcasting wifi access points" }, \
{ "ping",  ping,  1, NULL, NULL,"<ip address>",  "pings wifi configuration" }, \
{ "temp",  temperature_get,  0, NULL, NULL,"",  "get HTS221 temperature" }, \
{ "accel",  accelerometer_get,  0, NULL, NULL,"",  "get LIS2DH12 accelerometer" }, \


/******************************************************
 *               Function Definitions
 ******************************************************/

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
    uint8_t wbuf[8];
    uint8_t rbuf[8];
    int16_t T0, T1, T2, T3, raw;
    uint8_t val[4];
    int32_t temperature;
    float tempC, tempF;

    hts221_reg_t reg;
    hts221_status_get(&hts_ctx, &reg.status_reg);

    if (reg.status_reg.h_da)
    {
      /* Read humidity data */
      memset(data_raw_humidity.u8bit, 0x00, sizeof(int16_t));
//      hts221_humidity_raw_get(&hts_ctx, data_raw_humidity.u8bit);
      hts221_read_reg(&hts_ctx, HTS221_HUMIDITY_OUT_L, data_raw_humidity.u8bit, 2);
      data_raw_humidity.i16bit =(data_raw_humidity.u8bit[1]<<8 | data_raw_humidity.u8bit[0]);
      humidity_perc = linear_interpolation(&lin_hum, data_raw_humidity.i16bit);
      if (humidity_perc < 0) humidity_perc = 0;
      if (humidity_perc > 100) humidity_perc = 100;
//      DBG("Humidity [%%]:%3.2f\r\n", humidity_perc);
    }
    if (reg.status_reg.t_da)
    {
      /* Read temperature data */
      memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
//      hts221_temperature_raw_get(&hts_ctx, data_raw_temperature.u8bit);
      hts221_read_reg(&hts_ctx, HTS221_TEMP_OUT_L, data_raw_temperature.u8bit, 2);
      data_raw_temperature.i16bit =(data_raw_temperature.u8bit[1]<<8 | data_raw_temperature.u8bit[0]);
      temperature_degC = linear_interpolation(&lin_temp, data_raw_temperature.i16bit);
//      DBG("Temperature [degC]:%6.2f\r\n", temperature_degC );
    }

    telemetryData.temperature = temperature_degC;
    telemetryData.humidity = humidity_perc;

    return 0;
}

/*
 * Holder function to get LIS2DH12 accelerometer
 */
int accelerometer_get(int argc, char *argv[]){
    uint8_t data_ready;
    float xdataf, ydataf, zdataf;

    lis2dh12_reg_t reg;
    lis2dh12_status_get(&accel_ctx, &reg.status_reg);

    if( reg.status_reg.zyxda ) {

//        i2c_read(&accel_ctx, LIS2DH12_OUT_X_L, val, 6);
//        lis2dh12_acceleration_raw_get(&accel_ctx, data_raw_acceleration.u8bit);
        lis2dh12_read_reg(&accel_ctx, LIS2DH12_OUT_X_L, data_raw_acceleration.u8bit, 6);

        data_raw_acceleration.i16bit[0] =(data_raw_acceleration.u8bit[1]<<8 | data_raw_acceleration.u8bit[0]);
        data_raw_acceleration.i16bit[1] =(data_raw_acceleration.u8bit[3]<<8 | data_raw_acceleration.u8bit[2]);
        data_raw_acceleration.i16bit[2] =(data_raw_acceleration.u8bit[5]<<8 | data_raw_acceleration.u8bit[4]);

        acceleration_mg[0] = LIS2DH12_FROM_FS_4g_HR_TO_mg( data_raw_acceleration.i16bit[0] );
        acceleration_mg[1] = LIS2DH12_FROM_FS_4g_HR_TO_mg( data_raw_acceleration.i16bit[1] );
        acceleration_mg[2] = LIS2DH12_FROM_FS_4g_HR_TO_mg( data_raw_acceleration.i16bit[2] );

//        DBG("1 - Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
//                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);

        // Convert to g and round
        xdataf = (acceleration_mg[0] * 0.001);
        ydataf = (acceleration_mg[1] * 0.001);
        zdataf = (acceleration_mg[2] * 0.001);

//        xdataf = roundf(xdata * 0.001);
//        ydataf = roundf(ydata * 0.001);
//        zdataf = roundf(zdata * 0.001);

//        DBG("2 - Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
//                xdataf, ydataf, zdataf);

        telemetryData.accelerometer.x = acceleration_mg[0];
        telemetryData.accelerometer.y = acceleration_mg[1];
        telemetryData.accelerometer.z = acceleration_mg[2];

    } else {
//        WPRINT_APP_INFO(("No new XYZ data\n"));
        return 0;
    }

    return 0;
}

/*
 * Initializes I2C, probes for temperature device
 */
wiced_result_t temperature_init( void )
{
    uint8_t wbuf[8];
    uint8_t rbuf[8];

    /* Initialize I2C */
    if ( wiced_i2c_init( &i2c_device_temperature ) != WICED_SUCCESS )
    {
//        WPRINT_APP_INFO( ( "I2C Initialization Failed\n" ) );
        return WICED_ERROR;
    }

    /*
    *  Initialize mems driver interface
    */
    hts_ctx.write_reg = i2c_write_hts;
    hts_ctx.read_reg = i2c_read_hts;
    hts_ctx.handle = &i2c_device_temperature;

    /* Probe I2C bus for temperature sensor */
    if( wiced_i2c_probe_device( &i2c_device_temperature, NUM_I2C_MESSAGE_RETRIES ) != WICED_TRUE )
    {
//        WPRINT_APP_INFO( ( "Failed to connect to temperature device; addr 0x%x\n", i2c_device_temperature.address ) );
        return WICED_ERROR;
    }

    wbuf[0] = HTS221_WOAMI_REG;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );

    hts221_device_id_get(&hts_ctx, &whoamI);
    if( whoamI != HTS221_ID )
    {
        DBG("Failed to read WHOAMI from temperature device; addr 0x%x\n", i2c_device_temperature.address);
        return WICED_ERROR;
    }
    DBG("HTS221 device (0x%x) at address 0x%x\n", rbuf[0], i2c_device_temperature.address);

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
    hts221_data_rate_set(&hts_ctx, HTS221_ODR_1Hz);

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
    uint8_t wbuf[8];
    uint8_t rbuf[8];

    /* Initialize I2C */
    if ( wiced_i2c_init( &i2c_device_accelerometer ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    /*
    *  Initialize mems driver interface
    */
    accel_ctx.write_reg = i2c_write_accel;
    accel_ctx.read_reg = i2c_read_accel;
    accel_ctx.handle = &i2c_device_accelerometer;

    /* Probe I2C bus for accelerometer */
    if( wiced_i2c_probe_device( &i2c_device_accelerometer, NUM_I2C_MESSAGE_RETRIES ) != WICED_TRUE )
    {
        return WICED_ERROR;
    }

    lis2dh12_device_id_get(&accel_ctx, &whoamI);
    if(whoamI != LIS2DH12_ID)
    {
        DBG("Failed to read WHOAMI from accelerometer device; addr 0x%x\n", i2c_device_accelerometer.address);
    }
    DBG("LIS2DH12 device (0x%x) at address 0x%x\n", rbuf[0], i2c_device_accelerometer.address);

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
 * Initializes SPI for ADC
 */
wiced_result_t adc_init( void )
{
    wiced_result_t result;
    wiced_spi_message_segment_t spi_segment;
    uint16_t wbuf[16];
    uint16_t rbuf[16];
    uint16_t code;
    float vin;

    memset(wbuf, 0, 32);
    memset(rbuf, 0, 32);

    /* Initialize SPI1 */
    if ( wiced_spi_init( &spi1_device ) != WICED_SUCCESS )
    {
//        WPRINT_APP_INFO( ( "SPI1 Initialization Failed\n" ) );
        return WICED_ERROR;
    }

    spi_segment.tx_buffer = (void*)wbuf;
    spi_segment.rx_buffer = (void*)rbuf;
    spi_segment.length = 4;

    wbuf[0] = MCP3208_START | MCP3208_SE | MCP3208_CH0; /* single-ended, CH0) */
    result = wiced_spi_transfer( &spi1_device, &spi_segment, 1 );
    if (result != WICED_SUCCESS)
    {
//        WPRINT_APP_INFO( ( "SPI1 Transfer Failed\n" ) );
        return WICED_ERROR;
    }

    code = (rbuf[0]<<8 | rbuf[1]) & 0xfff;
    vin = (code * 3.30) / 4096;
//    WPRINT_APP_INFO( ( "MCP3208 CH0 %.1fV\n", vin ) );

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

int do_ntp_time(int argc, char *argv[])
{
    int status = ntp_set_time_cycle();
    DBG("ntp status: %d \n", status);
    return status;
}

int wifi_connect(int argc, char *argv[]) {
    wiced_result_t r = wiced_network_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);

    return r;
}

int get_telemetry_data(void *data)
{
    ((quicksilver_data*)data)->humidity = 6;
    ((quicksilver_data*)data)->temperature = 7;

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

void application_start( )
{
    wiced_result_t result;

    /* Initialize the WICED platform */
    wiced_init();

    /* Initialize the RGB */
    rgb_init();

    /* Connect to Wi-Fi */
    wifi_connect(0, NULL);

    /* Register the Quicksilver board as both a gateway and device and establish HTTP connection */
    arrow_initialize_routine();

    /* probe for temperature device */
    temperature_init();

    /* probe for accelerometer device */
    accelerometer_init();

    /* initialize SPI for ADC */
    adc_init();

    /* Connect to the MQTT Service */
    arrow_mqtt_connect_routine();

    quicksilver_data data = {0};

    // Add command handlers
    add_cmd_handler("rgb", rgb_handler);

    while(1)
    {
        /* Send the latest data to Arrow Connect */
        arrow_mqtt_send_telemetry_routine(update_sensor_data, &telemetryData);
    }


}
