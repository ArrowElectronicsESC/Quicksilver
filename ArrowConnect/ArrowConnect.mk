NAME := QuickSilver_ArrowConnect

SDK_ROOT = ../../../libraries/acn-sdk-c
PROJ_DIR := .

WIFI_CONFIG_DCT_H := wifi_config_dct.h
APPLICATION_DCT := ap_config_dct.c

#$(NAME)_DEFINES    += DEBUG
#$(NAME)_DEFINES    += DEBUG_WOLFSSL
#$(NAME)_DEFINES    += HTTP_DEBUG
$(NAME)_DEFINES    += USER_BYTE_CONVERTER
#$(NAME)_DEFINES    += _POSIX_THREADS
$(NAME)_DEFINES    += ARCH_SSL

GLOBAL_INCLUDES := . \
                    $(SDK_ROOT) \
                    $(SDK_ROOT)/include \
                    $(SDK_ROOT)/sys \
                    $(PROJ_DIR)/acnsdkc \
                    $(SDK_ROOT)/src/wolfSSL \

$(NAME)_SOURCES	:= $(PROJ_DIR)/ArrowConnect.c \
                   $(PROJ_DIR)/ap_config.c \
                   $(PROJ_DIR)/ap_common.c \
                   $(PROJ_DIR)/acnsdkc/bsd/socket.c \
                   $(PROJ_DIR)/acnsdkc/time/time.c \
                   $(PROJ_DIR)/acnsdkc/debug.c \
                   $(PROJ_DIR)/acnsdkc/sys/mac.c \
                   $(PROJ_DIR)/acnsdkc/arrow/storage.c \
                   $(PROJ_DIR)/acnsdkc/json/telemetry.c \
                   $(PROJ_DIR)/Drivers/Sensors/LIS2DH12/lis2dh12.c \
                   $(PROJ_DIR)/Drivers/Sensors/HTS221/hts221.c \
                   $(PROJ_DIR)/Drivers/LED/APA102/apa102.c \
                   $(SDK_ROOT)/src/ntp/client.c \
                   $(SDK_ROOT)/src/ntp/ntp.c \
                   $(SDK_ROOT)/src/time/watchdog_weak.c \
                   $(SDK_ROOT)/src/time/time.c \
                   $(SDK_ROOT)/src/bsd/inet.c \
                   $(SDK_ROOT)/src/arrow/mqtt.c \
                   $(SDK_ROOT)/src/arrow/routine.c \
                   $(SDK_ROOT)/src/arrow/device.c \
                   $(SDK_ROOT)/src/arrow/gateway.c \
                   $(SDK_ROOT)/src/arrow/sign.c \
                   $(SDK_ROOT)/src/arrow/api/device/device.c \
                   $(SDK_ROOT)/src/arrow/api/device/info.c \
                   $(SDK_ROOT)/src/arrow/api/gateway/gateway.c \
                   $(SDK_ROOT)/src/arrow/api/json/parse.c \
                   $(SDK_ROOT)/src/arrow/utf8.c \
                   $(SDK_ROOT)/src/http/client.c \
                   $(SDK_ROOT)/src/http/request.c \
                   $(SDK_ROOT)/src/http/response.c \
                   $(SDK_ROOT)/src/http/routine.c \
                   $(SDK_ROOT)/src/arrow/events.c \
                   $(SDK_ROOT)/src/arrow/state.c \
                   $(SDK_ROOT)/src/arrow/software_update.c \
                   $(SDK_ROOT)/src/arrow/software_release.c \
                   $(SDK_ROOT)/src/arrow/gateway_payload_sign.c \
                   $(SDK_ROOT)/src/arrow/device_command.c \
                   $(SDK_ROOT)/src/arrow/telemetry_api.c \
                   $(SDK_ROOT)/src/arrow/api/gateway/info.c \
                   $(SDK_ROOT)/src/arrow/api/device/info.c \
                   $(SDK_ROOT)/src/json/json.c \
                   $(SDK_ROOT)/src/debug.c \
                   $(SDK_ROOT)/src/data/property.c \
                   $(SDK_ROOT)/src/data/propmap.c \
                   $(SDK_ROOT)/src/data/linkedlist.c \
                   $(SDK_ROOT)/src/data/ringbuffer.c \
                   $(SDK_ROOT)/src/data/find_by.c \
                   $(SDK_ROOT)/src/ssl/crypt.c \
                   $(SDK_ROOT)/src/ssl/md5sum.c \
                   $(SDK_ROOT)/src/ssl/ssl.c \
                   $(SDK_ROOT)/src/mqtt/client/src/network.c \
                   $(SDK_ROOT)/src/mqtt/client/src/MQTTClient.c \
                   $(SDK_ROOT)/src/mqtt/client/src/timer.c \
                   $(SDK_ROOT)/src/mqtt/packet/src/MQTTPacket.c \
                   $(SDK_ROOT)/src/mqtt/packet/src/MQTTDeserializePublish.c \
                   $(SDK_ROOT)/src/mqtt/packet/src/MQTTSerializePublish.c \
                   $(SDK_ROOT)/src/mqtt/packet/src/MQTTConnectClient.c \
                   $(SDK_ROOT)/src/mqtt/packet/src/MQTTSubscribeClient.c \
                   $(SDK_ROOT)/src/sys/reboot_weak.c \
                   $(SDK_ROOT)/src/arrow/mqtt/acn.c   \
                   $(SDK_ROOT)/src/arrow/mqtt/azure.c   \
                   $(SDK_ROOT)/src/arrow/mqtt/ibm.c \
                   $(SDK_ROOT)/src/wolfSSL/src/crl.c \
                   $(SDK_ROOT)/src/wolfSSL/src/internal.c \
                   $(SDK_ROOT)/src/wolfSSL/src/io.c \
                   $(SDK_ROOT)/src/wolfSSL/src/keys.c \
                   $(SDK_ROOT)/src/wolfSSL/src/ocsp.c \
                   $(SDK_ROOT)/src/wolfSSL/src/sniffer.c \
                   $(SDK_ROOT)/src/wolfSSL/src/ssl.c \
                   $(SDK_ROOT)/src/wolfSSL/src/tls.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/aes.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/asm.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/asn.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/ecc.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/coding.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/compress.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/error.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/fe_low_mem.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/fe_operations.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/ge_low_mem.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/ge_operations.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/hash.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/hmac.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/integer.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/tfm.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/md5.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/memory.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/misc.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/random.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/sha.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/rsa.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/sha256.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/signature.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/wc_encrypt.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/wc_port.c \
                   $(SDK_ROOT)/src/wolfSSL/wolfcrypt/src/logging.c \

$(NAME)_RESOURCES += images/cypresslogo.png \
                     images/cypresslogo_line.png \
                     images/favicon.ico \
                     images/scan_icon.png \
                     images/wps_icon.png \
                     images/64_0bars.png \
                     images/64_1bars.png \
                     images/64_2bars.png \
                     images/64_3bars.png \
                     images/64_4bars.png \
                     images/64_5bars.png \
                     images/tick.png \
                     images/cross.png \
                     images/lock.png \
                     images/progress.gif \
                     scripts/general_ajax_script.js \
                     scripts/wpad.dat \
                     apps/aws_iot/aws_config.html \
                     config/scan_page_outer.html \
                     styles/buttons.css \
                     styles/border_radius.htc

$(NAME)_COMPONENTS := utilities/command_console \
					  utilities/command_console/ping \
					  daemons/device_onboarding \
					  daemons/HTTP_server \
                      daemons/DNS_redirect \
                      protocols/DNS
                      
#OTA_APPLICATION       := snip.ota2_extract-$(PLATFORM)
#OTA_APP               := build/$(OTA_APPLICATION)/binary/$(OTA_APPLICATION).stripped.elf
