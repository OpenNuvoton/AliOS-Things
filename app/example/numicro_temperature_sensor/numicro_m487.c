#include <stdint.h>
#include "aos_hal.h"
#include "board.h"
#include "max31875_c.h"
#include "max31875.h"

#define DEF_I2C_BUS_NO_MAX31875	1  // I2C bus No in mikroBUS interface

i2c_dev_t			g_sI2cDev[] =
{
    {DEF_I2C_BUS_NO_MAX31875,  {8, 100000, I2C_MODE_MASTER, 0}, NULL},// For max31875 I2C slave definition
};

I2C	sI2C_drv= {0};

static int numicro_i2c_read(int address, const char* reg, int reg_length, char *data, int length)
{
    if ( hal_i2c_master_send(&g_sI2cDev[0], (address<<1), reg, reg_length, 0) < 0 )
        return -1;
    return hal_i2c_master_recv(&g_sI2cDev[0], (address<<1), data, length, 0);
}

static int numicro_i2c_write(int address, const char *data, int length)
{
    return hal_i2c_master_send ( &g_sI2cDev[0], (address<<1), data, length, 0);
}

static void max31875_getvalues(void)
{
    float f_temperature;
    f_temperature = max31875_read_reg_as_temperature(MAX31875_REG_TEMPERATURE, &sI2C_drv);
    printf("Temperature is %3.4f Celsius, %3.4f Fahrenheit\r\n",
           f_temperature,
           max31875_celsius_to_fahrenheit(f_temperature));
}

static int testcase_init()
{
    int ret = -1;
    if ( (ret = hal_i2c_init(&g_sI2cDev[0])) != 0 )
        printf("Initilize I2C bus failure ...!\n");
    else if ( ( ret = max31875_init(MAX31875_I2C_SLAVE_ADR_R0)) < 0 )
        printf("Initilize MAX31875 Sensor failure ...!\n");
    else
    {
        sI2C_drv.read = numicro_i2c_read;
        sI2C_drv.write = numicro_i2c_write;
        /* Configure temperature sensor for 8 times per second */
        ret = max31875_write_cfg( MAX31875_CFG_CONV_RATE_8 | MAX31875_CFG_RESOLUTION_12BIT, &sI2C_drv);	
    }
		
		return ret;
}

static void testcase_run(void)
{
    max31875_getvalues();
}

static void app_delayed_action(void *arg)
{
		// Get temperature now.
    testcase_run();
		// trigger new alarm after 1 second
    aos_post_delayed_action(1000, app_delayed_action, NULL);
}

int application_start(int argc, char *argv[])
{
    printf("NuMicro Get temperature value from sensor started.\n");

    if ( !testcase_init() )	
			aos_post_delayed_action(1000, app_delayed_action, NULL);
		else
			printf("Initilize temperature sensor failure\n");

    aos_loop_run();
	
    return 0;
}
