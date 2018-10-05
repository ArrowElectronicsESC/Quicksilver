#include "wiced.h"
#include "Sensirion.h"
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
    char * cmd;
    int (*handler)(const char *data);
}command_handler_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/
int cmd_handler_setLED(const char *data);
int cmd_handler_updateLED(const char *data);
int cmd_handler_update(const char *data);

/******************************************************
 *                    Structures
 ******************************************************/
static command_handler_t arrowCommandHandlers[] = {
    // Command       // Handler
    { "setLED",      &cmd_handler_setLED },
    { "updateLED",   &cmd_handler_updateLED },
    { "update",      &cmd_handler_update },
};

static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255, 255, 255, 0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
};

/******************************************************
 *               Variable Definitions
 ******************************************************/
static quicksilver_data telemetryData;

/******************************************************
 *               Helper Function Definitions
 ******************************************************/

/* Function for parsing RGB commands */
wiced_result_t parseCommand_RGB(JsonNode * node)
{
    static color_t color;

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
                        color.brightness = arrayElement->number_ < 0x1F ? arrayElement->number_: 0x1F;
                        break;
                    case 1:
                        color.red = arrayElement->number_;
                        break;
                    case 2:
                        color.green = arrayElement->number_;
                        break;
                    case 3:
                        color.blue = arrayElement->number_;
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
                    color.brightness, color.red, color.green, color.blue));
            rgb_color_set(color.brightness, color.red, color.green, color.blue);
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

/* Function for processing device state updates.
 * NOTE: This function overrides a default weak implementation */
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



/* Function for updating all Quicksilver sensor data. */
int update_sensor_data(void * data)
{
    int32_t temperature, humidity;

    accelerometer_get(&telemetryData.accelerometer);
    humidity_get(&telemetryData.humidity);
    temperature_get(&telemetryData.temperature);
    rgb_color_get(&telemetryData.led.brightness, &telemetryData.led.red, &telemetryData.led.green, &telemetryData.led.blue);
    ess_measure_iaq(&telemetryData.tvoc_ppb, &telemetryData.co2_eq_ppm);
    ess_measure_rht(&temperature, &humidity);
    telemetryData.shtc1_temperature = temperature / 1000.0f;
    telemetryData.shtc1_humidity = humidity / 1000.0f;

    /* Control the ESS LED based on the IAQ reading */
    if (telemetryData.co2_eq_ppm > 3000) {
        ess_set_leds_ryg(1, 0, 0);
    } else if (telemetryData.co2_eq_ppm > 1000) {
        ess_set_leds_ryg(0, 1, 0);
    } else {
        ess_set_leds_ryg(0, 0, 1);
    }

    return WICED_SUCCESS;
}

/* Command handler for receiving the OTA update command */
int cmd_handler_update(const char *data)
{
    WPRINT_APP_INFO(("Update Command Handler\r\n"));

    /* Take down the Wi-Fi interface and bring up the OTA server. */
    wiced_network_down(WICED_STA_INTERFACE);
    wiced_network_up( WICED_AP_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &device_init_ip_settings );

    /* Start the OTA server.  */
    wiced_ota_server_start( WICED_AP_INTERFACE );
    while ( 1 )
    {
        wiced_rtos_delay_milliseconds( 100 );
    }

    return WICED_SUCCESS;
}

/* Command handler for receiving set LED commands */
int cmd_handler_setLED(const char *data)
{
    static color_t color;

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
        rgb_color_set(color.brightness, color.red, color.green, color.blue);
    }
    else
    {
        WPRINT_APP_INFO(("json parse failed\r\n"));
    }

    return 0;
}

/* Command handler for receiving update LED commands */
int cmd_handler_updateLED(const char *data)
{
    JsonNode * rgb_color = json_decode(data);
    if(rgb_color)
    {
        JsonNode *brightness = json_find_member(rgb_color, "brightness");
        if(brightness && brightness->tag == JSON_NUMBER)
        {
            rgb_brightness_set((int)brightness->number_);
        }
        JsonNode *red_val = json_find_member(rgb_color, "red");
        if(red_val && red_val->tag == JSON_NUMBER)
        {
            rgb_red_set((int)red_val->number_);
        }
        JsonNode *green_val = json_find_member(rgb_color, "green");
        if(green_val && green_val->tag == JSON_NUMBER)
        {
            rgb_green_set((int)green_val->number_);
        }
        JsonNode *blue_val = json_find_member(rgb_color, "blue");
        if(blue_val && blue_val->tag == JSON_NUMBER)
        {
            rgb_blue_set((int)blue_val->number_);
        }

        json_delete(rgb_color);
    }
    else
    {
        WPRINT_APP_INFO(("json parse failed\r\n"));
    }

    return 0;
}

/* Callback function for gateway software updates */
int gateway_software_update_cb(const char *url)
{
    WPRINT_APP_INFO(("Gateway Software Update Callback\r\n"));

    return 0;
}

/* Callback function for software release download initialization */
int arrow_release_download_init(void)
{
    WPRINT_APP_INFO(("Arrow Release Download Init Callback!\r\n"));
    return 0;
}

/* Callback function for software release download payload */
int arrow_release_download_payload(const char *payload, int size, int flag)
{
    if ( flag == FW_FIRST )
    {
        WPRINT_APP_INFO(("Release Download Started\r\n"));
    }

    return 0;
}

/* Callback function for software release download complete */
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

/* Function for initializing Arrow Connect cloud interface. */
wiced_result_t arrow_cloud_init(void)
{
    /* Register the Quicksilver board as both a gateway and device and establish HTTP connection */
    if(arrow_initialize_routine() != ROUTINE_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* Initialize MQTT events */
    arrow_mqtt_events_init();

    /* Add command handlers */
    for(int i = 0; i < sizeof(arrowCommandHandlers)/sizeof(command_handler_t); i++)
    {
        arrow_command_handler_add(arrowCommandHandlers[i].cmd, arrowCommandHandlers[i].handler);
    }

    /* Setup OTA firmware updates from Arrow Connect.
     * TODO Implement full functionality, using command handlers for now. */
#if !defined(NO_SOFTWARE_UPDATE)
    arrow_gateway_software_update_set_cb(gateway_software_update_cb);
#endif
    arrow_software_release_dowload_set_cb(arrow_release_download_init, arrow_release_download_payload, arrow_release_download_complete);

    // Initialization complete, set the LED green.
    rgb_color_set(1, 0, 255, 0);

    return WICED_SUCCESS;
}

void application_start( )
{
    /* Initialize the WICED platform */
    VERIFY_SUCCESS(wiced_init());

    /* Initialize the Quicksilver board */
    VERIFY_SUCCESS(quicksilver_init());


    /* Initialize Sensirion SGP30 and SHCT1 on ESS */
    VERIFY_SUCCESS(ess_init(&ESS_DEVICE_CONFIG_QUICKSILVER));

    /* Initialize the Arrow Connect interface */
    VERIFY_SUCCESS(arrow_cloud_init());

    while(1)
    {
        /* Continuously send data */
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
