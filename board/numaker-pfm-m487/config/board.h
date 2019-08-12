#ifndef __BOARD_H
#define __BOARD_H

#include "objects.h"

#define HARDWARE_REVISION   "V3.0"
#define MODEL               "NuMaker-PFM-M487"

/* Platform HAL will refer these structures are defined by board-resource. */

extern struct serial_s 	 board_uart [];
extern struct i2c_s 	 board_i2c [];
extern struct analogin_s board_analogin [];
extern struct analogout_s board_analogout [];
extern struct gpio_s 	 board_gpio [];
extern struct pwmout_s   board_pwm [];
extern struct spi_s      board_spi [];
extern struct qspi_s     board_qspi [];
extern struct sdh_s 			 board_sdh [];

extern const int i32BoardMaxUartNum;
extern const int i32BoardMaxI2CNum;
extern const int i32BoardMaxADCNum;
extern const int i32BoardMaxDACNum;
extern const int i32BoardMaxGPIONum;
extern const int i32BoardMaxPWMNum;
extern const int i32BoardMaxSPINum;
extern const int i32BoardMaxQSPINum;
extern const int i32BoardMaxSDHNum;

typedef enum {
    // Arduino UNO naming
    A0 = PB_6,
    A1 = PB_7,
    A2 = PB_8,
    A3 = PB_9,
    A4 = PB_0,
    A5 = PB_1,

    D0 = PB_2,
    D1 = PB_3,
    D2 = PC_9,
    D3 = PC_10,
    D4 = PC_11,
    D5 = PC_12,
    D6 = PE_4,
    D7 = PE_5,
    D8 = PA_5,
    D9 = PA_4,
    D10 = PA_3,
    D11 = PA_0,
    D12 = PA_1,
    D13 = PA_2,
    D14 = PG_1,
    D15 = PG_0,

    I2C_SCL = D15,
    I2C_SDA = D14,

    EPWM1_CH3 = D2,		//PC_9
    EPWM1_CH2 = D3,		//PC_10
    EPWM1_CH1 = D4,		//PC_11
    EPWM1_CH0 = D5,		//PC_12
    EPWM0_CH3 = D13,	//PA_2
    EPWM0_CH2 = D7,		//PE_5

    EPWM0_CH0 = D8,		//PA_5
    EPWM0_CH1 = D9,		//PA_4
    EPWM0_CH4 = D12,	//PA_1
    EPWM0_CH5 = D11,	//PA_0

    //		EPWM0_CH3 = D6,	//PE_4
    //		EPWM0_CH2 = D10,//PA_3

    // Note: board-specific
    // UART naming
    USBTX = PB_13,
    USBRX = PB_12,
    STDIO_UART_TX   = USBTX,
    STDIO_UART_RX   = USBRX,
    SERIAL_TX = USBTX,
    SERIAL_RX = USBRX,
    // LED naming
    LED_RED = PH_0,
    LED_YELLOW = PH_1,
    LED_GREEN = PH_2,
    LED1 = LED_RED,
    LED2 = LED_YELLOW,
    LED3 = LED_GREEN,
    LED4 = LED1,    // No real LED. Just for passing ATS.
    // Button naming
    SW2 = PG_15,
    SW3 = PF_11,

} Board_PinName;

#endif /* __BOARD */

