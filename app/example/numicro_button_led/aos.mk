NAME := numicro_button_led
$(NAME)_COMPONENTS := yloop

GLOBAL_DEFINES += AOS_NO_WIFI

ifeq ($(CONFIG_SYSINFO_DEVICE_NAME),board_numaker-iot-m487)
	$(NAME)_SOURCES := numicro_m487.c
endif
