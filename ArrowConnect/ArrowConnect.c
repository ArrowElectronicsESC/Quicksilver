
#include <stdlib.h>
#include "wiced.h"
#include "wiced_tls.h"
#include "command_console.h"

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

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define RGB_CLOCK WICED_GPIO_10
#define RGB_DATA WICED_GPIO_8

#define BUFFER_LENGTH     (2048)
#define MAX_LINE_LENGTH  (128)
#define MAX_HISTORY_LENGTH (20)
#define MAX_NUM_COMMAND_TABLE  (8)

typedef struct color
{
    unsigned char Red;
    unsigned char Green;
    unsigned char Blue;
} color;


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

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/
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


#define RESET_PIN WICED_GPIO_14

int arrow_connect_test(int argc, char *argv[])
{
    wiced_network_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);
    ntp_set_time_cycle();
    //DBG("Gonna do the connect\n\n");
    arrow_initialize_routine();
    //send_telemetry(0, NULL);

    return 0;
}
#if 1
int send_telemetry(int argc, char *argv[])
{
    quicksilver_data data = {};
    data.temperature = 50;
    arrow_send_telemetry_routine(&data);
    /*if(argc == 2)
    {
        DBG("Sending:  %s",argv[1]);
    }
    else
    {
        DBG("Sending: {\"f|data1\": 5}");
        arrow_send_telemetry_routine("{\"f|data1\": 5}");
    }*/
    return 0;
}
#endif

int find_gateway_by_os(int argc, char *argv[])
{
    DBG("ready to do a test");
    //TODO(bman): Delet this

    gateway_info_t *list = NULL;
    DBG("Got here: %s Line: %d\n",__FUNCTION__,__LINE__);
    int r = arrow_gateway_find_by(&list, 2, find_by(osNames, "Windows95"), find_by(f_size, "100"));
    DBG("Got here: %s Line: %d\n",__FUNCTION__,__LINE__);
    if ( r == 0 ) {
      gateway_info_t *tmp;
      int gateway_counter = 0;
      for_each_node_hard(tmp, list, gateway_info_t) {
          DBG("[%d]\n\thid: %s\n\tname: %s", gateway_counter++, P_VALUE(tmp->hid), P_VALUE(tmp->name));
          DBG("\tcreatedBy: %s",P_VALUE(tmp->created.by));
          //DBG("\tlastModifiedBy: %s\n",P_VALUE(tmp->lastModifiedBy));
          DBG("\tuid: %s",P_VALUE(tmp->uid));
          DBG("\tname: %s",P_VALUE(tmp->name));
          DBG("\ttype: %s",P_VALUE(tmp->type));
          DBG("\tdeviceType: %s",P_VALUE(tmp->deviceType));
          DBG("\tosName: %s",P_VALUE(tmp->osName));
          DBG("\tsoftwareName: %s",P_VALUE(tmp->softwareName));
          DBG("\tsoftwareVersion: %s\n",P_VALUE(tmp->softwareVersion));

          gateway_info_free(tmp);
          free(tmp);
      }
    }
    DBG("done with the test");

    return 0;
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

    wiced_init();

    wiced_gpio_init( RGB_CLOCK, OUTPUT_PUSH_PULL );
    wiced_gpio_init( RGB_DATA, OUTPUT_PUSH_PULL );


    //WPRINT_APP_INFO( ( "\r\nType help to know more about commands ...\r\n" ) );
    //command_console_init( STDIO_UART, MAX_LINE_LENGTH, line_buffer, MAX_HISTORY_LENGTH, history_buffer_storage, " " );
    //console_add_cmd_table( init_commands );
    //arrow_connect_test(0,NULL);
    wifi_connect(0, NULL);
    find_gateway_by_os(0,NULL);
    arrow_connect_test(0,NULL);

    arrow_mqtt_connect_routine();

    quicksilver_data data = {0};
#if 1
    DBG("THIS IS A TEST");
    DBG("THIS IS A TEST please work oh please");
    DBG("THIS IS A TEST");
    DBG("THIS IS A TEST please work oh please");
    DBG("THIS IS A TEST please work oh please");
    DBG("THIS IS A TEST please work oh please");
    DBG("THIS IS A TEST please work oh please");
    DBG("THIS IS A TEST please work oh please");
    DBG("THIS IS A TEST please work oh please");
    char tmp[170];
    tmp[0] = 1;
    result = tmp[0];
#endif
#if 1
    add_cmd_handler("ServerToGateway_DeviceCommand", rgb_handler);
    //add_cmd_handler("rgb", rgb_handler);

    arrow_mqtt_send_telemetry_routine(get_telemetry_data, &data);
#endif
    arrow_close();
    while(1)
    {
        DBG("sleeping...");
        wiced_rtos_delay_milliseconds(1000);
    }
}
