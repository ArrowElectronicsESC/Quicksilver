
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

#define BUFFER_LENGTH     (2048)
#define MAX_LINE_LENGTH  (128)
#define MAX_HISTORY_LENGTH (20)
#define MAX_NUM_COMMAND_TABLE  (8)




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

static char line_buffer[MAX_LINE_LENGTH];
static char history_buffer_storage[MAX_LINE_LENGTH * MAX_HISTORY_LENGTH];

static const command_t init_commands[] = {
        DIAGNOSTICS_COMMANDS
        CMD_TABLE_END
};
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
    wiced_result_t r = wiced_network_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);
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
    int r = arrow_gateway_find_by(&list, 2, find_by(osNames, "Windows10"), find_by(f_size, "100"));
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

void application_start( )
{
    wiced_result_t result;

    wiced_init();
    //WPRINT_APP_INFO( ( "\r\nType help to know more about commands ...\r\n" ) );
    //command_console_init( STDIO_UART, MAX_LINE_LENGTH, line_buffer, MAX_HISTORY_LENGTH, history_buffer_storage, " " );
    //console_add_cmd_table( init_commands );
    //arrow_connect_test(0,NULL);
    wifi_connect(0, NULL);
    arrow_initialize_routine();
//    find_gateway_by_os(0,NULL);
//    arrow_connect_test(0,NULL);

    arrow_mqtt_connect_routine();

    while(1)
    {
        DBG("sleeping...");
        wiced_rtos_delay_milliseconds(1000);
    }
}
