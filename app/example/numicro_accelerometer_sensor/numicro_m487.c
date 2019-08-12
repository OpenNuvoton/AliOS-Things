#include <stdint.h>
#include "aos_hal.h"
#include "board.h"
#include "bma2x2.h"
#include "bmm050.h"
#include "bmg160.h"

typedef struct {

    /* Structure used for Magnetometer */
    struct bmm050_t mag;

    /* Structure used for read the mag xyz data*/
    struct bmm050_mag_data_s16_t mag_data;

    /* Structure used for Accelerometer */
    struct bma2x2_t accel;

    /* Structure used to read accel xyz and temperature data*/
    struct bma2x2_accel_data_temp accel_xyzt;

    /* Structure used for Gyroscope */
    struct bmg160_t gyro;

    /* structure used to read gyro xyz and interrupt status*/
    struct bmg160_data_t gyro_xyzi;

} bmx055_t;

bmx055_t bmx055;

#define DEF_I2C_BUS_NO_BMX055	2  // I2C bus No

i2c_dev_t g_sI2cDev[] =
{
    {DEF_I2C_BUS_NO_BMX055,  {16, 100000, I2C_MODE_MASTER, 0}, NULL},// For max31875 I2C slave definition
};

static int8_t numicro_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len)
{
    uint8_t Tx_Data[1];
    Tx_Data[0] = (uint8_t)(reg_addr & 0x00FF);
    if ( hal_i2c_master_send(&g_sI2cDev[0], (dev_id<<1), &Tx_Data[0], sizeof(Tx_Data), 0) < 0 )
        return -1;
    return hal_i2c_master_recv(&g_sI2cDev[0], (dev_id<<1), reg_data, len, 0);
}

static uint8_t tmp_buf[32];
static int8_t numicro_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len)
{
    tmp_buf[0] = reg_addr;
    if (len > 0)
        memcpy(&tmp_buf[1], reg_data, len );
    return hal_i2c_master_send ( &g_sI2cDev[0], (dev_id<<1), &tmp_buf[0], len+1, 0);
}

void numicro_delay_ms(uint32_t period)
{
    aos_msleep(period);
}

static int8_t bmx055_init_mag(struct bmm050_t* mag)
{
    int8_t com_rslt = 0;

    mag->dev_addr = BMM050_I2C_ADDRESS;
    mag->bus_write = numicro_i2c_write;
    mag->bus_read = numicro_i2c_read;
    mag->delay_msec = numicro_delay_ms;

    com_rslt = bmm050_init(mag);
    com_rslt += bmm050_set_functional_state(BMM050_NORMAL_MODE);
    com_rslt += bmm050_set_data_rate(BMM050_DATA_RATE_30HZ);

    return com_rslt;
}

static int8_t bmx055_init_accel(struct bma2x2_t* accel)
{
    int8_t com_rslt = 0;
    uint8_t bandwidth = 0x08;	// bandwidth of 7.81Hz

    accel->dev_addr = BMA2x2_I2C_ADDR1;
    accel->bus_write = numicro_i2c_write;
    accel->bus_read = numicro_i2c_read;
    accel->delay_msec = numicro_delay_ms;

    com_rslt = bma2x2_init(accel);
    com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
    com_rslt += bma2x2_set_bw(bandwidth);

    return com_rslt;
}

static int8_t bmx055_init_gyro(struct bmg160_t* gyro)
{
    int8_t com_rslt = 0;

    gyro->dev_addr = BMG160_I2C_ADDR1;
    gyro->bus_write = numicro_i2c_write;
    gyro->bus_read = numicro_i2c_read;
    gyro->delay_msec = numicro_delay_ms;

    com_rslt = bmg160_init(gyro);
    com_rslt += bmg160_set_power_mode(BMG160_MODE_NORMAL);
    com_rslt += bmg160_set_bw(C_BMG160_BW_230HZ_U8X);

    return com_rslt;
}

static u8 g_u8Range=0;
static int16_t twos_comp_to_int(
    int16_t i16Val,
    uint8_t u8Bits
)
{
    if ((i16Val & (1 << (u8Bits - 1))) != 0)  //if sign bit is set
        i16Val = i16Val - (1 << u8Bits);     // compute negative value
    return i16Val;                         // return positive value as is
}

static float get_g_value( int16_t i16AccelVal )
{
    //i16AccelVal = twos_comp_to_int(i16AccelVal, 12);

    if (g_u8Range == BMA2x2_RANGE_2G)
        return i16AccelVal * 0.98 / 1000;	//res: 0.98,
    else if(g_u8Range == BMA2x2_RANGE_4G)
        return i16AccelVal * 1.95 / 1000;
    else if(g_u8Range == BMA2x2_RANGE_8G)
        return i16AccelVal * 3.91 / 1000;
    else if(g_u8Range == BMA2x2_RANGE_16G)
        return i16AccelVal * 7.81 / 1000;
    else return 0.0f;
}

static int bmx055_getvalues(void)
{
    /* Read the accel XYZT data */
    bma2x2_read_accel_xyzt(&bmx055.accel_xyzt);

    printf("Accelerometer -X:%.6f -Y:%.6f -Z:%.6f\n", \
           get_g_value(bmx055.accel_xyzt.x), \
           get_g_value(bmx055.accel_xyzt.y), \
           get_g_value(bmx055.accel_xyzt.z) );

    return 0;
}

static int8_t numicro_bmx055_init(bmx055_t* dev)
{
    int8_t com_rslt = 0;

    /* Initilize Magnetometer of BMX055 */
    com_rslt += bmx055_init_mag(&dev->mag);

    /* Initilize Accelerometer of BMX055 */
    com_rslt += bmx055_init_accel(&dev->accel);

    /* Initilize Gyroscope of BMX055 */
    com_rslt += bmx055_init_gyro(&dev->gyro);

    return com_rslt;
}

static int testcase_init()
{
    int ret = 0;
    if ( (ret = hal_i2c_init(&g_sI2cDev[0])) != 0 )
        printf("Initilize I2C bus failure ...!\n");
    else if (numicro_bmx055_init(&bmx055) < 0)
        printf("Initilize BMX055 Fail ...!\n");
    else
        bma2x2_get_range(&g_u8Range);
    return ret;
}

static void testcase_run(void)
{
    bmx055_getvalues();
}

static void app_delayed_action(void *arg)
{
    // Get value from accelerometer now.
    testcase_run();
    // trigger new alarm after 100 msecond
    aos_post_delayed_action(100, app_delayed_action, NULL);
}

int application_start(int argc, char *argv[])
{
    printf("NuMicro Get value from accelerometer sensor started.\n");

    if ( !testcase_init() )
        aos_post_delayed_action(100, app_delayed_action, NULL);
    else
        printf("Initilize accelerometer sensor failure\n");

    aos_loop_run();

    return 0;
}
