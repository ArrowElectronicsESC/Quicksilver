
SDK_PATH = ../../../libraries/acn-sdk-c
SDK_IMPL = $(current_dir)/acnsdkc
#include $(SDK_PATH)/Makefile.wolf
include libraries/acn-sdk-c/Makefile.wolf

NAME := QuickSilver_ArrowConnect

$(NAME)_INCLUDES   := . \
                      daemons/device_onboarding/

$(NAME)_SOURCES    := ArrowConnect.c

#$(NAME)_SOURCES    += acnsdkc/debug.c
$(NAME)_SOURCES    += acnsdkc/bsd/socket.c
$(NAME)_SOURCES    += acnsdkc/time/time.c
$(NAME)_SOURCES    += acnsdkc/debug.c
$(NAME)_SOURCES    += acnsdkc/sys/mac.c
$(NAME)_SOURCES    += acnsdkc/arrow/storage.c
$(NAME)_SOURCES    += acnsdkc/arrow/state.c
$(NAME)_SOURCES    += acnsdkc/json/telemetry.c

$(NAME)_SOURCES    += Drivers/Sensors/LIS2DH12/lis2dh12.c
$(NAME)_SOURCES    += Drivers/Sensors/HTS221/hts221.c

#$(NAME)_SOURCES    += $(patsubst $(SDK_PATH)%,../acn-sdk-c/%,$(WOLF_SRC))



#___________________________________________
$(NAME)_SOURCES    += $(SDK_PATH)/src/ntp/client.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/ntp/ntp.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/time/watchdog_weak.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/time/time.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/bsd/inet.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/mqtt.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/routine.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/device.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/gateway.c
#$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/request.c //no existy
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/sign.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/api/device/device.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/api/device/info.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/api/gateway/gateway.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/api/json/parse.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/utf8.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/http/client.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/http/request.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/http/response.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/http/routine.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/events.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/state.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/software_update.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/software_release.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/gateway_payload_sign.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/device_command.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/telemetry_api.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/api/gateway/info.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/api/device/info.c
#$(NAME)_SOURCES    += $(SDK_PATH)/src/arrow/api/json/telemetry.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/json/json.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/debug.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/data/property.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/data/propmap.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/data/linkedlist.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/data/ringbuffer.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/data/find_by.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/ssl/crypt.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/ssl/md5sum.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/ssl/ssl.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/mqtt/client/src/network.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/mqtt/client/src/MQTTClient.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/mqtt/client/src/timer.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/mqtt/packet/src/MQTTPacket.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/mqtt/packet/src/MQTTDeserializePublish.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/mqtt/packet/src/MQTTSerializePublish.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/mqtt/packet/src/MQTTConnectClient.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/mqtt/packet/src/MQTTSubscribeClient.c
$(NAME)_SOURCES    += $(SDK_PATH)/src/sys/reboot_weak.c
#______________________________________________________
$(NAME)_SOURCES    += $(patsubst $(SDK_PATH)%,../acn-sdk-c/%,$(WOLF_SRC))
# This may not be working in a Windows environment
$(NAME)_SOURCES    += $(patsubst $(SDK_PATH)%,$(SDK_PATH)/%,$(WOLF_SRC))

WOLF_SRC += $(SDK_PATH)/src/wolfSSL/src/crl.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/src/internal.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/src/io.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/src/keys.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/src/ocsp.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/src/sniffer.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/src/ssl.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/src/tls.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/aes.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/asm.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/asn.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/ecc.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/coding.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/compress.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/error.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/fe_low_mem.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/fe_operations.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/ge_low_mem.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/ge_operations.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/hash.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/hmac.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/integer.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/tfm.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/md5.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/memory.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/misc.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/random.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/sha.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/rsa.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/sha256.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/signature.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/wc_encrypt.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/wc_port.c
WOLF_SRC += $(SDK_PATH)/src/wolfSSL/wolfcrypt/src/logging.c

$(NAME)_SOURCES += $(WOLF_SRC)

$(info $($(NAME)_SOURCES))

$(NAME)_INCLUDES += .
$(NAME)_INCLUDES += $(SDK_PATH)
$(NAME)_INCLUDES += $(SDK_PATH)/include
$(NAME)_INCLUDES += $(SDK_PATH)/src/wolfSSL
$(NAME)_INCLUDES += $(SDK_PATH)/src/wolfSSL/wolfssl
$(NAME)_INCLUDES += ./acnsdkc

$(info $($(NAME)_INCLUDES))

$(NAME)_DEFINES    += DEBUG
#$(NAME)_DEFINES    += DEBUG_WOLFSSL
$(NAME)_DEFINES    += HTTP_DEBUG
$(NAME)_DEFINES    += USER_BYTE_CONVERTER
#$(NAME)_DEFINES    += _POSIX_THREADS
$(NAME)_DEFINES    += ARCH_SSL

#WIFI_CONFIG_DCT_H  := wifi_config_dct.h

$(NAME)_COMPONENTS := utilities/command_console \
					  utilities/command_console/ping \
					  daemons/device_onboarding