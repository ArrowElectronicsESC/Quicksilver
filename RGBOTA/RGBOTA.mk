NAME :=App_RGB

$(NAME)_SOURCES := RGBOTA.c

OTA_APPLICATION	:= snip.ota2_extract-$(PLATFORM)
OTA_APP    := build/$(OTA_APPLICATION)/binary/$(OTA_APPLICATION).stripped.elf
