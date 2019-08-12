NAME := numicro_accelerometer_sensor
$(NAME)_COMPONENTS := yloop

GLOBAL_DEFINES += AOS_NO_WIFI

ifeq ($(CONFIG_SYSINFO_DEVICE_NAME),board_numaker-iot-m487)
	$(NAME)_SOURCES := numicro_m487.c BMX055/BMA2x2_driver/bma2x2.c BMX055/BMG160_driver/bmg160.c BMX055/BMM050_driver/bmm050.c
	$(NAME)_INCLUDES += BMX055/BMA2x2_driver BMX055/BMG160_driver BMX055/BMM050_driver
endif
