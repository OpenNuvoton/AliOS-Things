NAME := numicro_aliyun

$(NAME)_MBINS_TYPE := app
$(NAME)_VERSION := 1.0.0
$(NAME)_SUMMARY := mqtt examples

$(NAME)_SOURCES := app_entry.c mqtt_example.c numicro_m487_led_button.c
$(NAME)_COMPONENTS := linkkit_sdk_c netmgr cjson cli

GLOBAL_INCLUDES += ./

