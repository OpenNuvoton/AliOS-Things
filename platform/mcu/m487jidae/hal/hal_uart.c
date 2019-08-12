/* Microcontroller Library
 * Copyright (c) 2015-2016 Nuvoton
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "objects.h"
#include "nu_modutil.h"
#include "nu_bitutil.h"
#include "hal_status.h"
#include <k_api.h>

#include "aos/hal/uart.h"
#include "board.h"

#include "nu_rbuf.h"

#define DEVICE_UART_SERIAL_TX_ASYNCH 1
#define DEVICE_UART_SERIAL_RX_ASYNCH 1

#if DEVICE_UART_SERIAL_TX_ASYNCH || DEVICE_UART_SERIAL_RX_ASYNCH
#include "hal_uart_event.h"
#include "hal_dma.h"
#endif

#if DEVICE_UART_SERIAL_TX_ASYNCH
void uart_dma_tx_done_cb ( uint32_t idx, uint32_t dma_event);
#endif

#if DEVICE_UART_SERIAL_RX_ASYNCH
void uart_dma_rx_done_cb ( uint32_t idx, uint32_t dma_event);
#endif


#define MAX_BUF_UART_BYTES		(4096)

struct nu_uart_var {
    uint32_t    ref_cnt;                // Reference count of the H/W module
    struct serial_s *  obj;
    uart_dev_t	* uart_dev;
    nu_rbuf_t			fifo_rbuf_rx;
    char *				fifo_buf_rx;
    uint32_t			fifo_buf_size;
    kmutex_t 			port_tx_mutex;
    kmutex_t 			port_rx_mutex;
    ksem_t				fifo_rx_sem;
    ksem_t				fifo_tx_sem;

#if DEVICE_UART_SERIAL_TX_ASYNCH
    uint8_t     	pdma_perp_tx;
    int8_t     		pdma_chanid_tx;
#endif

#if DEVICE_UART_SERIAL_RX_ASYNCH
    uint8_t     	pdma_perp_rx;
    int8_t     		pdma_chanid_rx;
#endif

};

static struct nu_uart_var uart0_var = {
    .ref_cnt            =   0,
    .obj                =   NULL,
    .fifo_buf_rx				=		NULL,
#if DEVICE_UART_SERIAL_TX_ASYNCH
    .pdma_perp_tx				= 	PDMA_UART0_TX,
    .pdma_chanid_tx			=		-1,
#endif
#if DEVICE_UART_SERIAL_RX_ASYNCH
    .pdma_perp_rx				=		PDMA_UART0_RX,
    .pdma_chanid_rx			=		-1,
#endif
};
static struct nu_uart_var uart1_var = {
    .ref_cnt            =   0,
    .obj                =   NULL,
    .fifo_buf_rx				=		NULL,
#if DEVICE_UART_SERIAL_TX_ASYNCH
    .pdma_perp_tx				= 	PDMA_UART1_TX,
    .pdma_chanid_tx			=		-1,
#endif
#if DEVICE_UART_SERIAL_RX_ASYNCH
    .pdma_perp_rx				=		PDMA_UART1_RX,
    .pdma_chanid_rx			=		-1,
#endif
};
static struct nu_uart_var uart2_var = {
    .ref_cnt            =   0,
    .obj                =   NULL,
    .fifo_buf_rx				=		NULL,
#if DEVICE_UART_SERIAL_TX_ASYNCH
    .pdma_perp_tx				= 	PDMA_UART2_TX,
    .pdma_chanid_tx			=		-1,
#endif
#if DEVICE_UART_SERIAL_RX_ASYNCH
    .pdma_perp_rx				=		PDMA_UART2_RX,
    .pdma_chanid_rx			=		-1,
#endif
};
static struct nu_uart_var uart3_var = {
    .ref_cnt            =   0,
    .obj                =   NULL,
    .fifo_buf_rx				=		NULL,
#if DEVICE_UART_SERIAL_TX_ASYNCH
    .pdma_perp_tx				= 	PDMA_UART3_TX,
    .pdma_chanid_tx			=		-1,
#endif
#if DEVICE_UART_SERIAL_RX_ASYNCH
    .pdma_perp_rx				=		PDMA_UART3_RX,
    .pdma_chanid_rx			=		-1,
#endif
};
static struct nu_uart_var uart4_var = {
    .ref_cnt            =   0,
    .obj                =   NULL,
    .fifo_buf_rx				=		NULL,
#if DEVICE_UART_SERIAL_TX_ASYNCH
    .pdma_perp_tx				= 	PDMA_UART4_TX,
    .pdma_chanid_tx			=		-1,
#endif
#if DEVICE_UART_SERIAL_RX_ASYNCH
    .pdma_perp_rx				=		PDMA_UART4_RX,
    .pdma_chanid_rx			=		-1,
#endif
};
static struct nu_uart_var uart5_var = {
    .ref_cnt            =   0,
    .obj                =   NULL,
    .fifo_buf_rx				=		NULL,
#if DEVICE_UART_SERIAL_TX_ASYNCH
    .pdma_perp_tx				= 	PDMA_UART5_TX,
    .pdma_chanid_tx			=		-1,
#endif
#if DEVICE_UART_SERIAL_RX_ASYNCH
    .pdma_perp_rx				=		PDMA_UART5_RX,
    .pdma_chanid_rx			=		-1,
#endif
};

static const struct nu_modinit_s uart_modinit_tab[] =
{
    {UART_0, UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1), UART0_RST, UART0_IRQn, &uart0_var},
    {UART_1, UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1), UART1_RST, UART1_IRQn, &uart1_var},
    {UART_2, UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1), UART2_RST, UART2_IRQn, &uart2_var},
    {UART_3, UART3_MODULE, CLK_CLKSEL3_UART3SEL_HIRC, CLK_CLKDIV4_UART3(1), UART3_RST, UART3_IRQn, &uart3_var},
    {UART_4, UART4_MODULE, CLK_CLKSEL3_UART4SEL_HIRC, CLK_CLKDIV4_UART4(1), UART4_RST, UART4_IRQn, &uart4_var},
    {UART_5, UART5_MODULE, CLK_CLKSEL3_UART5SEL_HIRC, CLK_CLKDIV4_UART5(1), UART5_RST, UART5_IRQn, &uart5_var},

    {NC, 0, 0, 0, 0, (IRQn_Type) 0, NULL}
};

static uint32_t uart_modinit_mask = 0;

static int hal_uart_lock(kmutex_t* uart_mutex, uint32_t timeout)
{
    kstat_t stat;
    if (uart_mutex)
    {
        stat = krhino_mutex_lock(uart_mutex, krhino_ms_to_ticks(timeout));
        if ( (stat == RHINO_SUCCESS) || (stat == RHINO_MUTEX_OWNER_NESTED) )
            return 0;
    }
    return -1;
}

static int hal_uart_unlock(kmutex_t* uart_mutex)
{
    kstat_t stat;
    if (uart_mutex)
    {
        stat = krhino_mutex_unlock(uart_mutex);
        if ( (stat == RHINO_SUCCESS) || (stat == RHINO_MUTEX_OWNER_NESTED) )
            return 0;
    }
    return -1;
}

static void call_hal_event_cb(uart_dev_t * puart_dev, E_HAL_UART_EVENT eHalUartEvent)
{
    if ( puart_dev && puart_dev->priv )
    {
        hal_uart_event_cbfun invoke_event_cb = puart_dev->priv;
        invoke_event_cb ( (int)puart_dev,  eHalUartEvent );
    }
}

static int push_nu_rbuf( struct nu_uart_var* psNuUartVar, uint8_t* pu8Buf, int len)
{
    int wrote_byte=0, ret;
    nu_rbuf_t* pnu_rbuf = &psNuUartVar->fifo_rbuf_rx;

    while ( wrote_byte < len )
    {
        /* Wait for Lock */
        ret = nu_rbuf_write ( pnu_rbuf, &pu8Buf[wrote_byte], (len - wrote_byte) );
        wrote_byte += ret;
    }
    return wrote_byte;
}

static int UART_GET_FIFO_LENGTH(UART_T *uart_base)
{
    return ((uart_base->FIFOSTS & UART_FIFOSTS_RXPTR_Msk) >> UART_FIFOSTS_RXPTR_Pos);
}

static void hal_uart_rxbuf_irq ( struct nu_uart_var* psNuUartVar, uint32_t u32INTSTS )
{
    uint8_t dat[UART0_FIFO_SIZE]= {0};
    int counter=0;
    int aval_bytes=0;
    uart_dev_t * puart_dev = psNuUartVar->uart_dev;
    struct serial_s *obj = psNuUartVar->obj;
    UART_T *uart_base = (UART_T *) NU_MODBASE(obj->uart);
    int bDoSignal = (u32INTSTS & UART_INTSTS_RXTOIF_Msk) >> UART_INTSTS_RXTOIF_Pos;

    while ( (aval_bytes=nu_rbuf_avail_write_space(&psNuUartVar->fifo_rbuf_rx)) >= UART0_FIFO_SIZE )
    {
        if ( bDoSignal )
            counter=0;

        if ( (!bDoSignal) && ((aval_bytes=UART_GET_FIFO_LENGTH (uart_base))==1) ) break;

        counter=0;
        while ( UART_IS_RX_READY(uart_base) )
        {
            if ( (!bDoSignal) && ((aval_bytes=UART_GET_FIFO_LENGTH (uart_base))==1) ) break;

            counter=0;
            while ( UART_IS_RX_READY(uart_base) )
            {
                if ( (!bDoSignal) && ((aval_bytes=UART_GET_FIFO_LENGTH (uart_base))==1) ) break;
                if ( counter < UART0_FIFO_SIZE )
                {
                    dat[counter]=(uint8_t)uart_base->DAT;
                    counter++;
                }
                else
                    break;
            }
        }
        if ( counter )
            push_nu_rbuf( psNuUartVar, &dat[0], counter);
        else
            break;
    }//while

    if ( (nu_rbuf_avail_read_space(&psNuUartVar->fifo_rbuf_rx) > 0) && bDoSignal )
    {
        krhino_sem_give ( &psNuUartVar->fifo_rx_sem );
        if ( puart_dev && puart_dev->priv )
            call_hal_event_cb ( puart_dev, eUART_EVENT_RECEIVED );
    }
}

static void uart_irq(struct nu_uart_var* psNuUartVar)
{
    struct serial_s *obj = psNuUartVar->obj;
    UART_T *uart_base = (UART_T *) NU_MODBASE(obj->uart);
    uint32_t u32INTSTS = uart_base->INTSTS;
    uint32_t u32FIFOSTS = uart_base->FIFOSTS;

#if DEVICE_UART_SERIAL_RX_ASYNCH
    if(u32INTSTS & UART_INTSTS_HWRLSIF_Msk)
    {
        uart_base->FIFOSTS = (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk);
        return;
    }
#endif

    if ( u32INTSTS & (UART_INTSTS_RDAINT_Msk|UART_INTSTS_RXTOINT_Msk) ) {
        // Simulate clear of the interrupt flag. Temporarily disable the interrupt here and to be recovered on next read.
        //UART_DISABLE_INT(uart_base, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
        hal_uart_rxbuf_irq ( psNuUartVar, u32INTSTS );
        //UART_ENABLE_INT ( uart_base, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk) );
    }

#if 0
    if (uart_base->INTSTS & UART_INTSTS_THREINT_Msk) {
        // Simulate clear of the interrupt flag. Temporarily disable the interrupt here and to be recovered on next write.
        UART_DISABLE_INT(uart_base, UART_INTEN_THREIEN_Msk);
        if (obj->serial.irq_handler) {
            ((uart_irq_handler) obj->serial.irq_handler)(obj->serial.irq_id, TxIrq);
        }
    }
#endif

    // FIXME: Ignore all other interrupt flags. Clear them. Otherwise, program will get stuck in interrupt.
    //uart_base->INTSTS = uart_base->INTSTS;
    //uart_base->FIFOSTS = uart_base->FIFOSTS;
    uart_base->INTSTS = u32INTSTS;
    uart_base->FIFOSTS = u32FIFOSTS;

}

void UART0_IRQHandler (void)
{
    krhino_intrpt_enter();
    uart_irq(&uart0_var);
    krhino_intrpt_exit();
}
void UART1_IRQHandler (void)
{
    krhino_intrpt_enter();
    uart_irq(&uart1_var);
    krhino_intrpt_exit();
}
void UART2_IRQHandler (void)
{
    krhino_intrpt_enter();
    uart_irq(&uart2_var);
    krhino_intrpt_exit();
}
void UART3_IRQHandler (void)
{
    krhino_intrpt_enter();
    uart_irq(&uart3_var);
    krhino_intrpt_exit();
}
void UART4_IRQHandler (void)
{
    krhino_intrpt_enter();
    uart_irq(&uart4_var);
    krhino_intrpt_exit();
}
void UART5_IRQHandler (void)
{
    krhino_intrpt_enter();
    uart_irq(&uart5_var);
    krhino_intrpt_exit();
}


/**
 * Configure UART flowcontrol
 *
 *
 * @param[in]  pUart 	the UART register base of NuMicro
 * @param[in]  hufc_t  	the flowcontrol setting.
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
static int32_t hal_uart_set_flowcontol ( UART_T* pUart, hal_uart_flow_control_t hufc_t )
{
    if (!pUart)
        goto exit_hal_uart_set_flowcontol;

    // Disable to CTS/RTS first.
    UART_DISABLE_INT (pUart, UART_INTEN_ATORTSEN_Msk | UART_INTEN_ATOCTSEN_Msk);
    switch (hufc_t)
    {
    case FLOW_CONTROL_DISABLED:
        break;

    case FLOW_CONTROL_CTS:	//CTS only
        /* Set CTS pin input is low level active */
        pUart->MODEMSTS |= UART_MODEMSTS_CTSACTLV_Msk;

        /* Set CTS auto flow control enable */
        UART_ENABLE_INT (pUart, UART_INTEN_ATOCTSEN_Msk);
        break;

    case FLOW_CONTROL_RTS:	//RTS only
        /* Set RTS pin output is low level active */
        pUart->MODEM |= UART_MODEM_RTSACTLV_Msk;
        pUart->FIFO = (pUart->FIFO &~ UART_FIFO_RTSTRGLV_Msk) | UART_FIFO_RTSTRGLV_14BYTES;

        /* Set RTS auto flow control enable */
        UART_ENABLE_INT (pUart, UART_INTEN_ATORTSEN_Msk);
        break;

    case FLOW_CONTROL_CTS_RTS:
        /* Set RTS pin output is low level active */
        pUart->MODEM |= UART_MODEM_RTSACTLV_Msk;

        /* Set CTS pin input is low level active */
        pUart->MODEMSTS |= UART_MODEMSTS_CTSACTLV_Msk;
        pUart->FIFO = (pUart->FIFO &~ UART_FIFO_RTSTRGLV_Msk) | UART_FIFO_RTSTRGLV_14BYTES;

        /* Set RTS and CTS auto flow control enable */
        UART_ENABLE_INT (pUart, UART_INTEN_ATORTSEN_Msk | UART_INTEN_ATOCTSEN_Msk);
        break;

    default:
        goto exit_hal_uart_set_flowcontol;
        break;
    }

    return HAL_OK;

exit_hal_uart_set_flowcontol:

    return HAL_ERROR;
}

/**
 * Configure UART line configuration
 *
 *
 * @param[in]  pUart 	the UART register base of NuMicro
 * @param[in]  hufc_t  	the flowcontrol setting.
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
static int32_t hal_uart_set_lineconf ( UART_T* pUart, hal_uart_data_width_t udw, hal_uart_stop_bits_t usb, hal_uart_parity_t up)
{
    uint32_t u32data_width;
    uint32_t u32parity;
    uint32_t u32stop_bits;

    if (!pUart)
        goto exit_hal_uart_set_lineconf;

    // Check data width parameters
    switch (udw)
    {
    case DATA_WIDTH_5BIT:
        u32data_width=UART_WORD_LEN_5;
        break;
    case DATA_WIDTH_6BIT:
        u32data_width=UART_WORD_LEN_6;
        break;
    case DATA_WIDTH_7BIT:
        u32data_width=UART_WORD_LEN_7;
        break;
    case DATA_WIDTH_8BIT:
        u32data_width=UART_WORD_LEN_8;
        break;

    case DATA_WIDTH_9BIT:	//Not support
    default:
        goto exit_hal_uart_set_lineconf;
        break;
    }

    // Check stop bit parameters
    switch (usb)
    {
    case STOP_BITS_1:
        u32stop_bits=UART_STOP_BIT_1;
        break;
    case STOP_BITS_2:
        u32stop_bits=UART_STOP_BIT_2;
        break;
    default:
        goto exit_hal_uart_set_lineconf;
        break;
    }

    // Check parity parameters
    switch (up)
    {
    case NO_PARITY:
        u32parity=UART_PARITY_NONE;
        break;
    case ODD_PARITY:
        u32parity=UART_PARITY_ODD;
        break;
    case EVEN_PARITY:
        u32parity=UART_PARITY_EVEN;
        break;
    default:
        goto exit_hal_uart_set_lineconf;
        break;
    }

    // Dont change baudrate setting.
    UART_SetLineConfig ( pUart, 0, u32data_width, u32parity, u32stop_bits );

    return HAL_OK;

exit_hal_uart_set_lineconf:
    return HAL_ERROR;
}

static struct serial_s* hal_get_serial_s ( uart_dev_t *uart )
{
    if ( !(uart) || (uart->port >= i32BoardMaxUartNum) )
        goto exit_hal_get_serial_s;

    // Get UART Private configuration, these setting are defined in board/xxxxx.
    return (struct serial_s*)&board_uart[uart->port];

exit_hal_get_serial_s:
    return NULL;
}

/**
 * Initialises a UART interface
 *
 *
 * @param[in]  uart  the interface which should be initialised
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_uart_init(uart_dev_t *uart)
{
    struct nu_modinit_s *modinit;
    struct serial_s* pserial_s;
    struct nu_uart_var *var;
    UARTName s_UartName;
    UART_T* pUart;

    if ( !(pserial_s=hal_get_serial_s ( uart )) )
        goto exit_hal_uart_init;

    // found UARTNAME from the two PinName.
    uint32_t uart_tx = pinmap_peripheral(pserial_s->pin_tx, PinMap_UART_TX);
    uint32_t uart_rx = pinmap_peripheral(pserial_s->pin_rx, PinMap_UART_RX);
    uint32_t uart_data = NC;

    // Get final UARTName
    uart_data = (UARTName) pinmap_merge(uart_tx, uart_rx);
    if ( uart_data == NC )
        goto exit_hal_uart_init;

    uint32_t uart_cts = NC;
    uint32_t uart_rts = NC;
    uint32_t uart_flowctl = NC;

    if ( pserial_s->pin_cts != NC )
        uart_cts = pinmap_peripheral(pserial_s->pin_cts, PinMap_UART_CTS);

    if ( pserial_s->pin_rts != NC )
        uart_rts = pinmap_peripheral(pserial_s->pin_rts, PinMap_UART_RTS);

    if ( (pserial_s->pin_cts!=NC) || (pserial_s->pin_rts!=NC) )
    {
        uart_flowctl = pinmap_merge(uart_cts, uart_rts);
        s_UartName = (UARTName)  pinmap_merge(uart_data, uart_flowctl);
    } else
        s_UartName = (UARTName) uart_data;

    if ( s_UartName == NC )
        goto exit_hal_uart_init;

    // Find entry by UARTNAME
    if ( !(modinit = get_modinit(s_UartName, uart_modinit_tab)) )
        goto exit_hal_uart_init;

    var = (struct nu_uart_var *) modinit->var;
    if (! var->ref_cnt) {

        if ( uart->priv )
        {
#if DEVICE_UART_SERIAL_TX_ASYNCH
            var->pdma_chanid_tx			=		hal_dma_channel_allocate(0);
            if ( var->pdma_chanid_tx < 0 )
                goto exit_hal_uart_init;

            hal_dma_set_handler ( var->pdma_chanid_tx,
                                  (uint32_t)uart_dma_tx_done_cb,
                                  (uint32_t)var,
                                  DMA_EVENT_TRANSFER_DONE|DMA_EVENT_TIMEOUT );
#endif

#if DEVICE_UART_SERIAL_RX_ASYNCH
            var->pdma_chanid_rx			=		hal_dma_channel_allocate(0);
            if ( var->pdma_chanid_rx < 0 )
                goto exit_hal_uart_init;

            hal_dma_set_handler ( var->pdma_chanid_rx,
                                  (uint32_t)uart_dma_rx_done_cb,
                                  (uint32_t)var,
                                  DMA_EVENT_TRANSFER_DONE|DMA_EVENT_TIMEOUT );
#endif
        }

        do {
            SYS_UnlockReg();

            // Reset this module
            SYS_ResetModule(modinit->rsetidx);

            // Select IP clock source
            CLK_SetModuleClock(modinit->clkidx, modinit->clksrc, modinit->clkdiv);

            // Enable IP clock
            CLK_EnableModuleClock(modinit->clkidx);

            pinmap_pinout(pserial_s->pin_tx, PinMap_UART_TX);
            pinmap_pinout(pserial_s->pin_rx, PinMap_UART_RX);

            if ( pserial_s->pin_rts!=NC )
                pinmap_pinout(pserial_s->pin_rts, PinMap_UART_RTS);

            if ( pserial_s->pin_cts!=NC )
                pinmap_pinout(pserial_s->pin_cts, PinMap_UART_CTS);

        } while (0);

        if ( uart->priv )
            var->fifo_buf_size = MAX_BUF_UART_BYTES;
        else
            var->fifo_buf_size = MAX_BUF_UART_BYTES/8;

        if ( var->fifo_buf_rx == NULL )
            var->fifo_buf_rx = aos_malloc(var->fifo_buf_size);

        if ( var->fifo_buf_rx == NULL ) {
            printf("fail to malloc memory size %d at %s %d \r\d", var->fifo_buf_size, __FILE__, __LINE__);
            goto exit_hal_uart_init;
        }

        if ( nu_rbuf_create(&var->fifo_rbuf_rx, var->fifo_buf_rx, var->fifo_buf_size ) != 0 )
            goto exit_hal_uart_init;

        if ( krhino_mutex_create(&var->port_tx_mutex, "uartTxMutex") != RHINO_SUCCESS )
            goto exit_hal_uart_init;

        if ( krhino_mutex_create(&var->port_rx_mutex, "uartTxMutex") != RHINO_SUCCESS )
            goto exit_hal_uart_init;

        if ( krhino_sem_create(&var->fifo_rx_sem, "sem_read", 0) != RHINO_SUCCESS ) {
            printf( "fail to create rx sem\r\d" );
            goto exit_hal_uart_init;
        }

#if DEVICE_UART_SERIAL_TX_ASYNCH
        if ( krhino_sem_create(&var->fifo_tx_sem, "sem_write", 0) != RHINO_SUCCESS ) {
            printf( "fail to create tx sem\r\d" );
            goto exit_hal_uart_init;
        }
#endif

        // Get Uart base address
        pUart = (UART_T *) NU_MODBASE(s_UartName);

        /* Set UART Baudrate */
        UART_Open(pUart, uart->config.baud_rate);

        // Set line configuration
        if ( hal_uart_set_lineconf ( pUart, uart->config.data_width, uart->config.stop_bits, uart->config.parity) < 0 )
            goto exit_hal_uart_init;

        // Set flow-control parameters
        if ( hal_uart_set_flowcontol ( pUart, uart->config.flow_control ) < 0 )
            goto exit_hal_uart_init;

        // Check mode parameters
        switch (uart->config.mode)
        {
        case MODE_TX:
        case MODE_RX:
        case MODE_TX_RX:
            break;

        default:
            goto exit_hal_uart_init;
            break;

        } //switch

        /* Link parent and children. */
        var->obj = pserial_s;
        var->uart_dev = uart;


#if DEVICE_UART_SERIAL_RX_ASYNCH
        if ( var->pdma_chanid_rx  != -1 )
        {
            //					NVIC_DisableIRQ(modinit->irq_n);
            NVIC_EnableIRQ(modinit->irq_n);
            pUart->FIFO = (pUart->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE;
            UART_ENABLE_INT(pUart, (UART_INTEN_RLSIEN_Msk)); // Enable Receive Line interrupt
            uart_dma_rx_start ( var );
        }
        else
#endif
        {
            NVIC_EnableIRQ(modinit->irq_n);
            pUart->FIFO = (pUart->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE;

            UART_SetTimeoutCnt(pUart, 40);
            /* Enable Interrupt */
            UART_ENABLE_INT(pUart, (UART_INTEN_RDAIEN_Msk|UART_INTEN_RXTOIEN_Msk));
        }
    } //if (! var->ref_cnt)

    var->ref_cnt ++;

    if (var->ref_cnt) {
        // Mark this module to be inited.
        int i = modinit - uart_modinit_tab;
        uart_modinit_mask |= 1 << i;
    }

    return HAL_OK;

exit_hal_uart_init:

    if ( pUart )
        UART_Close(pUart);

    return HAL_ERROR;
}

#if DEVICE_UART_SERIAL_TX_ASYNCH
void uart_dma_tx_done_cb ( uint32_t idx, uint32_t dma_event)
{
    struct nu_uart_var * var = (struct nu_uart_var *)idx;
    uart_dev_t * puart_dev = var->uart_dev;

    if ( dma_event&DMA_EVENT_TRANSFER_DONE )
    {
        krhino_sem_give ( &var->fifo_tx_sem );
        call_hal_event_cb ( puart_dev, eUART_EVENT_SEND_COMPLETE);
    }

    if ( dma_event&(DMA_EVENT_ABORT|DMA_EVENT_TIMEOUT) )
        while(1);

    //UART_DISABLE_INT(uart_base, UART_INTEN_TXPDMAEN_Msk);
}
#endif

int32_t platform_uart_write(struct nu_uart_var *var, uint8_t pu8TxBuf[], uint32_t u32WriteBytes, uint32_t timeout )
{
    uint32_t tx_count = 0;
    struct serial_s *pserial_s = var->obj;
    UART_T *uart_base = (UART_T *) NU_MODBASE(pserial_s->uart);
    uart_dev_t * puart_dev = var->uart_dev;

#if DEVICE_UART_SERIAL_TX_ASYNCH
    if ( (u32WriteBytes <= 16) || ( (uint32_t)pu8TxBuf % 4 != 0) || var->pdma_chanid_tx==-1  )
#endif
    {
        //if( puart_dev->priv )
        //	printf("CPU W->%d\r\n", u32WriteBytes);

        UART_DISABLE_INT(uart_base, UART_INTEN_TXPDMAEN_Msk);  // Start DMA transfer
        if ( UART_Write(uart_base, (uint8_t*)pu8TxBuf, u32WriteBytes) == u32WriteBytes )
            tx_count = u32WriteBytes;

        if ( puart_dev && puart_dev->priv )
            call_hal_event_cb ( puart_dev, eUART_EVENT_SEND_COMPLETE );

    }
#if DEVICE_UART_SERIAL_TX_ASYNCH
    else {
        uart_dev_t * puart_dev = var->uart_dev;
        uart_base = (UART_T *) NU_MODBASE(pserial_s->uart);

        //printf("DMA W->%d\r\n", u32WriteBytes);

        hal_dma_fill_description ( 	var->pdma_chanid_tx,
                                    (uint32_t)var->pdma_perp_tx,
                                    8,
                                    (uint32_t)&pu8TxBuf[0],
                                    (uint32_t)uart_base,
                                    u32WriteBytes,
                                    0 );

        //UART_DISABLE_INT(uart_base, UART_INTEN_THREIEN_Msk);
        UART_ENABLE_INT(uart_base, UART_INTEN_TXPDMAEN_Msk);  // Start DMA transfer

        if ( krhino_sem_take( &var->fifo_tx_sem, krhino_ms_to_ticks(timeout)) == 0 ) //Timeout?
        {
            tx_count = u32WriteBytes;
            UART_DISABLE_INT(uart_base, UART_INTEN_TXPDMAEN_Msk);  // Start DMA transfer
        }

        //printf("DMA->%d done\r\n", u32WriteBytes);

    }
#endif

    return tx_count;
}


/**
 * Transmit data on a UART interface
 *
 * @param[in]  uart     the UART interface
 * @param[in]  data     pointer to the start of data
 * @param[in]  size     number of bytes to transmit
 * @param[in]  timeout  timeout in milisecond, set this value to HAL_WAIT_FOREVER
 *                      if you want to wait forever
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_uart_send(uart_dev_t *uart, const void *data, uint32_t size, uint32_t timeout)
{
    struct nu_modinit_s *modinit;
    struct serial_s* pserial_s;
    struct nu_uart_var *var;
    UART_T* pUart;
    int32_t ret = -1;

    pserial_s = hal_get_serial_s ( uart );
    if ( !pserial_s )
        goto exit_hal_uart_send;

    modinit = get_modinit(pserial_s->uart, uart_modinit_tab);
    /* Valid? */
    if ( !modinit ) goto exit_hal_uart_send;

    var = modinit->var;
    /* Initialized? */
    if ( !var->ref_cnt ) goto exit_hal_uart_send;

    if (timeout == 0)
        timeout = 0xffffffff;

    pserial_s = var->obj;
    pUart = (UART_T *) NU_MODBASE(pserial_s->uart);

    if ( hal_uart_lock(&var->port_tx_mutex, timeout) != 0 )
        goto exit_hal_uart_send;

    if ( platform_uart_write(var, (uint8_t*)data, size, timeout ) == size  )
        ret = 0;

    hal_uart_unlock(&var->port_tx_mutex);

exit_hal_uart_send:

    return ret;
}

#if DEVICE_UART_SERIAL_RX_ASYNCH
#define DEF_DMA_RXBUF_NUM	 32
void uart_dma_rx_start ( struct nu_uart_var *var );

void uart_dma_rx_start ( struct nu_uart_var *var )
{
    struct serial_s *pserial_s = var->obj;
    UART_T *uart_base = (UART_T *) NU_MODBASE(pserial_s->uart);
    //uint8_t* pu8Buf = &u8DMABuffer[toggle_ctl][0];
    int trigger_len = var->fifo_buf_size/DEF_DMA_RXBUF_NUM;
    int toggle_ctl = var->fifo_rbuf_rx.chunk_idx;
    uint8_t* pu8Buf = &var->fifo_rbuf_rx.buffer[trigger_len*toggle_ctl];

    hal_dma_fill_description ( 	var->pdma_chanid_rx,
                                (uint32_t)var->pdma_perp_rx,
                                8,
                                (uint32_t)uart_base,
                                (uint32_t)pu8Buf,
                                trigger_len, //half buffer
                                5000 );
    //(1000000*40)/115200 );

    UART_ENABLE_INT(uart_base, UART_INTEN_RXPDMAEN_Msk); // Start DMA transfer
}

void uart_dma_rx_stop ( struct nu_uart_var *var )
{
    struct serial_s *pserial_s = var->obj;
    UART_T *uart_base = (UART_T *) NU_MODBASE(pserial_s->uart);
    UART_DISABLE_INT(uart_base, UART_INTEN_RXPDMAEN_Msk); // Stop DMA transfer
}

void uart_dma_rx_done_cb ( uint32_t idx, uint32_t dma_event)
{
    struct nu_uart_var * var = (struct nu_uart_var *)idx;
    uart_dev_t * puart_dev = var->uart_dev;
    struct serial_s *pserial_s = var->obj;
    UART_T *uart_base = (UART_T *) NU_MODBASE(pserial_s->uart);
    int cur = var->fifo_rbuf_rx.chunk_idx;
    int trigger_len = var->fifo_buf_size/DEF_DMA_RXBUF_NUM;
    int transferred_byte_count = hal_dma_transferred_bytecount ( var->pdma_chanid_rx );
    transferred_byte_count = (transferred_byte_count==0)? trigger_len: trigger_len-(transferred_byte_count+1);
    int i32RBufWOff = 0;

    if ( dma_event & ( DMA_EVENT_TIMEOUT | DMA_EVENT_TRANSFER_DONE ) )
    {
        if ( dma_event & DMA_EVENT_TRANSFER_DONE ) //Transfer done
        {
            uart_dma_rx_stop ( var );
            var->fifo_rbuf_rx.chunk_idx = (var->fifo_rbuf_rx.chunk_idx+1)%DEF_DMA_RXBUF_NUM;
            uart_dma_rx_start ( var ); // trigger next
        }
        else if ( (dma_event & DMA_EVENT_TIMEOUT) && !UART_GET_RX_EMPTY(uart_base) )
        {
            return;
        }

        i32RBufWOff = var->fifo_rbuf_rx.tail - (trigger_len*cur);

        if ( transferred_byte_count > i32RBufWOff )
        {
            //push_nu_rbuf( var, &u8DMABuffer[cur][0], transferred_byte_count );
            nu_rbuf_dummy_write ( &var->fifo_rbuf_rx, transferred_byte_count - i32RBufWOff  );

            krhino_sem_give ( &var->fifo_rx_sem );
            if ( puart_dev && puart_dev->priv )
                call_hal_event_cb ( puart_dev, eUART_EVENT_RECEIVED );
        }
    }

    if ( dma_event & DMA_EVENT_ABORT )
        while(1);

}


void UART_DisableRxTimeout(UART_T* uart)
{
    /* Disable time-out counter enable */
    uart->INTEN &= ~UART_INTEN_TOCNTEN_Msk;
}

#endif

int32_t platform_uart_read(struct nu_uart_var *var, uint8_t pu8RxBuf[], uint32_t u32ReadBytes, uint32_t timeout )
{
    struct serial_s *pserial_s = var->obj;
    UART_T *uart_base = (UART_T *) NU_MODBASE(pserial_s->uart);
    nu_rbuf_t *prbuf = &var->fifo_rbuf_rx;
    uint32_t rx_count = 0;
    size_t rev_size = 0;
    int32_t	ret;
    long long leave_ms = aos_now_ms()+timeout;

    //printf("[%06d]%s %x: %d %d, to=%d. in\r\n", (unsigned)aos_now_ms(), __func__, var, rx_count, u32ReadBytes, timeout);
    while (rx_count < u32ReadBytes) {
        rev_size = u32ReadBytes - rx_count;
        rx_count += nu_rbuf_read(prbuf, &pu8RxBuf[rx_count], rev_size);

        if ( (rx_count == u32ReadBytes) || (aos_now_ms() > leave_ms) ) break;

        //printf("[%06d]%x: %d %d Wait.\r\n", (unsigned)aos_now_ms(), var, rx_count, u32ReadBytes);
        if ( (ret=krhino_sem_take( &var->fifo_rx_sem, krhino_ms_to_ticks(timeout)) ) == 0 ) //Timeout?
        {
            //printf("[%06d]Wait %d.\r\n", (unsigned)aos_now_ms(), ret );
            //printf("[%06d]Got %d.\r\n", (unsigned)aos_now_ms(), ret );
            //nu_rbuf_dump ( &var->fifo_rbuf_rx );
            continue;
        }
        else
            break;
    }
    //printf("[%06d]%x: %d %d. out\r\n", (unsigned)aos_now_ms(), var, rx_count, u32ReadBytes);
    return rx_count;
}

/**
 * Receive data on a UART interface
 *
 * @param[in]   uart         the UART interface
 * @param[out]  data         pointer to the buffer which will store incoming data
 * @param[in]   expect_size  number of bytes to receive
 * @param[in]   timeout      timeout in milisecond, set this value to HAL_WAIT_FOREVER
 *                           if you want to wait forever
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_uart_recv(uart_dev_t *uart, void *data, uint32_t expect_size, uint32_t timeout)
{
    struct nu_modinit_s *modinit;
    struct serial_s* pserial_s;
    struct nu_uart_var *var;
    kstat_t stat = RHINO_SUCCESS;

    int count=0;

    pserial_s = hal_get_serial_s ( uart );
    if ( !pserial_s )
        goto exit_hal_uart_recv;

    if (timeout == 0)
        timeout = 0xffffffff;

    modinit = get_modinit(pserial_s->uart, uart_modinit_tab);
    /* Valid? */
    if ( !modinit ) goto exit_hal_uart_recv;

    var = (struct nu_uart_var *)modinit->var;
    /* Initialized? */
    if ( !var->ref_cnt ) goto exit_hal_uart_recv;

    pserial_s = var->obj;

    if ( hal_uart_lock(&var->port_rx_mutex, timeout) != 0 )
        goto exit_hal_uart_recv;

    count = platform_uart_read(var, (uint8_t*)data, expect_size, timeout);

    hal_uart_unlock(&var->port_rx_mutex);

    if ( count != expect_size  )
        goto exit_hal_uart_recv;

    return HAL_OK;

exit_hal_uart_recv:

    return HAL_ERROR;
}

/**
 * Receive data on a UART interface
 *
 * @param[in]   uart         the UART interface
 * @param[out]  data         pointer to the buffer which will store incoming data
 * @param[in]   expect_size  number of bytes to receive
 * @param[out]  recv_size    number of bytes received
 * @param[in]   timeout      timeout in milisecond, set this value to HAL_WAIT_FOREVER
 *                           if you want to wait forever
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_uart_recv_II(uart_dev_t *uart, void *data, uint32_t expect_size,
                         uint32_t *recv_size, uint32_t timeout)
{
    struct nu_modinit_s *modinit;
    struct serial_s* pserial_s;
    struct nu_uart_var *var;
    kstat_t stat = RHINO_SUCCESS;
    int count=0;

    pserial_s = hal_get_serial_s ( uart );
    if ( !pserial_s )
        goto exit_hal_uart_recv_ii;

    if (timeout == 0)
        timeout = 0xffffffff;

    modinit = get_modinit(pserial_s->uart, uart_modinit_tab);
    /* Valid? */
    if ( !modinit ) goto exit_hal_uart_recv_ii;

    var = (struct nu_uart_var *)modinit->var;
    /* Initialized? */
    if ( !var->ref_cnt ) goto exit_hal_uart_recv_ii;

    pserial_s = var->obj;

    if ( hal_uart_lock(&var->port_rx_mutex, timeout) != 0 )
        goto exit_hal_uart_recv_ii;

    *recv_size = platform_uart_read(var, (uint8_t*)data, expect_size, timeout);

    hal_uart_unlock(&var->port_rx_mutex);

    //printf("[%s](%d, %d)\r\n", __func__, expect_size, *recv_size );

    return  (*recv_size!=0)?HAL_OK:HAL_ERROR;

exit_hal_uart_recv_ii:
    *recv_size = 0;
    return HAL_ERROR;
}

/**
 * Deinitialises a UART interface
 *
 * @param[in]  uart  the interface which should be deinitialised
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_uart_finalize(uart_dev_t *uart)
{
    struct serial_s* pserial_s;
    struct nu_modinit_s *modinit;
    struct nu_uart_var *var;
    UART_T* pUart;

    pserial_s = hal_get_serial_s ( uart );
    if ( !pserial_s )
        goto exit_hal_uart_finalize;

    modinit = get_modinit(pserial_s->uart, uart_modinit_tab);
    /* Valid? */
    if ( !modinit ) goto exit_hal_uart_finalize;

    var = (struct nu_uart_var *)modinit->var;
    /* Initialized? */
    if ( !var->ref_cnt ) goto exit_hal_uart_finalize;

    pUart = (UART_T *) NU_MODBASE(pserial_s->uart);

    var->ref_cnt --;

    if (! var->ref_cnt) {

        do {
            UART_Close(pUart);

            //UART_DISABLE_INT(pUart, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
            NVIC_DisableIRQ(modinit->irq_n);

            // Disable IP clock
            CLK_DisableModuleClock(modinit->clkidx);
        } while (0);

        krhino_mutex_del(&var->port_tx_mutex);
        krhino_mutex_del(&var->port_rx_mutex);
        krhino_sem_del(&var->fifo_tx_sem);
        krhino_sem_del(&var->fifo_rx_sem);

        nu_rbuf_destroy(&var->fifo_rbuf_rx);

        aos_free(var->fifo_buf_rx);

        /* Unlink parent and children. */
        var->obj = NULL ;
        var->uart_dev = NULL;
    }

    if (! var->ref_cnt) {
        // Mark this module to be deinited.
        int i = modinit - uart_modinit_tab;
        uart_modinit_mask &= ~(1 << i);
    }

    return HAL_OK;

exit_hal_uart_finalize:

    return HAL_ERROR;

}
