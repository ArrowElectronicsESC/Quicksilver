
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

    // Temperature Calibration values
    // Read 1 byte of data from address 0x32(50)
    wbuf[0] = HTS221_T0_DEGC_X8;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    T0 = rbuf[0];

    // Read 1 byte of data from address 0x33(51)
    wbuf[0] = HTS221_T1_DEGC_X8;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    T1 = rbuf[0];

    // Read 1 byte of data from address 0x35(53)
    wbuf[0] = HTS221_T1_T0_MSB;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    raw = rbuf[0];

    // Convert the temperature Calibration values to 10-bits
    T0 = ((raw & 0x03) * 256) + T0;
    T1 = ((raw & 0x0C) * 64) + T1;

    // Read 1 byte of data from address 0x3C(60)
    wbuf[0] = HTS221_T0_OUT_L;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[0] = rbuf[0];

    // Read 1 byte of data from address 0x3D(61)
    wbuf[0] = HTS221_T0_OUT_H;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[1] = rbuf[0];

    T2 = ((val[1] & 0xFF) * 256) + (val[0] & 0xFF);

    // Read 1 byte of data from address 0x3E(62)
    wbuf[0] = HTS221_T1_OUT_L;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[0] = rbuf[0];

    // Read 1 byte of data from address 0x3F(63)
    wbuf[0] = HTS221_T1_OUT_H;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[1] = rbuf[0];

    T3 = ((val[1] & 0xFF) * 256) + (val[0] & 0xFF);

    // Read 2 bytes of data; temperature msb and lsb
    wbuf[0] = HTS221_TEMP_OUT_L;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[0] = rbuf[0];

    wbuf[0] = HTS221_TEMP_OUT_H;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[1] = rbuf[0];

    temperature = ((val[1] & 0xFF) * 256) + (val[0] & 0xFF);
    if(temperature > 32767)
    {
        temperature -= 65536;
    }

    tempC = ((T1 - T0) / 8.0) * (temperature - T2) / (T3 - T2) + (T0 / 8.0);
    tempF = (tempC * 1.8 ) + 32;

//    WPRINT_APP_INFO( ( "HTS221 temperature %.1f°C, %.1f°F\n", tempC, tempF ) );

    telemetryData.temperature = tempC;

    return 0;
}

/*
 * Converts raw accelerometer data to mg.
*/
void convert_accel_data(int16_t* x, int16_t* y, int16_t* z) {
    uint16_t lx, ly, lz;

    lx = *x;
    ly= *y;
    lz= *z;

    *x = (int32_t)lx*1000/(1024*16); // transform data to millig, for 2g scale axis*1000/(1024*16),
    *y = (int32_t)ly*1000/(1024*16); // for 4g scale axis*1000/(1024*8),
    *z = (int32_t)lz*1000/(1024*16); // for 8g scale axis*1000/(1024*4)

    return;
}

/*
 * Holder function to get LIS2DH12 accelerometer
 */
int accelerometer_get(int argc, char *argv[]){
    uint8_t wbuf[8];
    uint8_t rbuf[8];
    uint8_t val[8];
    int16_t xdata, ydata, zdata;
    float xdataf, ydataf, zdataf;

    // Read status register from address 0x27
    wbuf[0] = LIS2DH12_STATUS_REG;
    wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[0] = rbuf[0];

    if( val[0] & LIS2DH12_STAT_ZYXDA ) {

        // Read 1 byte of data from address 0x29
        wbuf[0] = LIS2DH12_OUT_X_H;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[1] = rbuf[0];

        // Read 1 byte of data from address 0x28
        wbuf[0] = LIS2DH12_OUT_X_L;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[0] = rbuf[0];

        // Read 1 byte of data from address 0x2B
        wbuf[0] = LIS2DH12_OUT_Y_H;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[3] = rbuf[0];

        // Read 1 byte of data from address 0x2A
        wbuf[0] = LIS2DH12_OUT_Y_L;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[2] = rbuf[0];

        // Read 1 byte of data from address 0x2D
        wbuf[0] = LIS2DH12_OUT_Z_H;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[5] = rbuf[0];

        // Read 1 byte of data from address 0x2C
        wbuf[0] = LIS2DH12_OUT_Z_L;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[4] = rbuf[0];

        xdata =(val[1]<<8 | val[0]);
        ydata =(val[3]<<8 | val[2]);
        zdata =(val[5]<<8 | val[4]);

        // Convert to mg
        convert_accel_data(&xdata, &ydata, &zdata);

        // Check for negative
        if(xdata&0x800) {
            xdata = (((~xdata&0x7ff)+1) * -1);
        }
        if(ydata&0x800) {
            ydata = (((~ydata&0x7ff)+1) * -1);
        }
        if(zdata&0x800) {
            zdata = (((~zdata&0x7ff)+1) * -1);
        }

        // Convert to g and round
        xdataf = roundf(xdata * 0.001);
        ydataf = roundf(ydata * 0.001);
        zdataf = roundf(zdata * 0.001);

//        WPRINT_APP_INFO(("x: %.fg\t", xdataf));
//        WPRINT_APP_INFO(("y: %.fg\t", ydataf));
//        WPRINT_APP_INFO(("z: %.fg\n", zdataf));

        telemetryData.accelerometer.x = xdataf;
        telemetryData.accelerometer.y = ydataf;
        telemetryData.accelerometer.z = zdataf;

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

    /* Probe I2C bus for temperature sensor */
    if( wiced_i2c_probe_device( &i2c_device_temperature, NUM_I2C_MESSAGE_RETRIES ) != WICED_TRUE )
    {
//        WPRINT_APP_INFO( ( "Failed to connect to temperature device; addr 0x%x\n", i2c_device_temperature.address ) );
        return WICED_ERROR;
    }

    wbuf[0] = HTS221_WOAMI_REG;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    if( rbuf[0] != 0xbc )
    {
//        WPRINT_APP_INFO( ( "Failed to read WHOAMI from temperature device; addr 0x%x\n", i2c_device_temperature.address ) );
        return WICED_ERROR;
    }
//    WPRINT_APP_INFO( ( "HTS221 device (0x%x) at address 0x%x\n", rbuf[0], i2c_device_temperature.address ) );

    /* Power-up the device */
    wbuf[0] = HTS221_CTRL_REG1;
    wbuf[1] = (HTS221_CTRL1_PD | HTS221_CTRL1_BDU);
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 2 );

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
//        WPRINT_APP_INFO( ( "I2C Initialization Failed\n" ) );
        return WICED_ERROR;
    }

    /* Probe I2C bus for accelerometer */
    if( wiced_i2c_probe_device( &i2c_device_accelerometer, NUM_I2C_MESSAGE_RETRIES ) != WICED_TRUE )
    {
//        WPRINT_APP_INFO( ( "Failed to connect to accelerometer device; addr 0x%x\n", i2c_device_accelerometer.address ) );
        return WICED_ERROR;
    }

    wbuf[0] = LIS2DH12_WOAMI_REG;
    wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    if( rbuf[0] != 0x33 )
    {
//        WPRINT_APP_INFO( ( "Failed to read WHOAMI from accelerometer device; addr 0x%x\n", i2c_device_accelerometer.address ) );
        return WICED_ERROR;
    }
//    WPRINT_APP_INFO( ( "LIS2DH12 device (0x%x) at address 0x%x\n", rbuf[0], i2c_device_accelerometer.address ) );

    /* Power-up the device */
    wbuf[0] = LIS2DH12_CTRL_REG1;
    wbuf[1] = 0;
    wbuf[1] = (LIS2DH12_CTRL1_ODR_400 | LIS2DH12_CTRL1_ZEN | LIS2DH12_CTRL1_YEN | LIS2DH12_CTRL1_XEN);
    wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 2 );

    /* Set normal mode */
    wbuf[0] = LIS2DH12_CTRL_REG4;
    wbuf[1] = LIS2DH12_CTRL4_BDU;
    wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 2 );

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

    return WICED_SUCCESS;
}

wiced_result_t update_sensor_data(void * data)
{
    accelerometer_get(0, NULL);
    temperature_get(0, NULL);

    return WICED_SUCCESS;
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
#if 1

    char tmp[170];
    tmp[0] = 1;
    result = tmp[0];
#endif

    //add_cmd_handler("testhandler", testhandler);
    add_cmd_handler("rgb", rgb_handler);
    //add_state("rgbValues","[123,255,45]");
    //arrow_post_state_request(current_device());
    //arrow_post_state_update(current_device());
//    arrow_mqtt_send_telemetry_routine(get_telemetry_data, &data);

//    arrow_close();
    while(1)
    {
        DBG("SLEEP...");
        /* Send the latest data to Arrow Connect */
        arrow_mqtt_send_telemetry_routine(update_sensor_data, &telemetryData);
//        arrow_send_telemetry_routine(&telemetryData);

        /* Refresh sensor data */
//        update_sensor_data(NULL);

        /* Wait for 100ms */
        wiced_rtos_delay_milliseconds(100);
    }


}
