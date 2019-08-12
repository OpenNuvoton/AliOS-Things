NAME := device_sal_esp8266_aos

$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION := 1.0.0
$(NAME)_SUMMARY := sal hal implementation for esp8266
GLOBAL_DEFINES += DEV_SAL_ESP8266_AOS

$(NAME)_COMPONENTS += yloop

$(NAME)_SOURCES += esp8266.c
$(NAME)_SOURCES += esp8266_at.c
$(NAME)_SOURCES += ringbuffer.c
$(NAME)_SOURCES += wifi_port.c
GLOBAL_INCLUDES += ./
