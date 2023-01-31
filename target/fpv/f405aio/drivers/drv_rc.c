/******************************************************************************
 * Copyright 2022 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <firmament.h>

#include "hal/rc/ppm.h"
#include "hal/rc/rc.h"
#include "hal/rc/sbus.h"

#ifndef min //mod by prife
    #define min(x, y) (x < y ? x : y)
#endif

/* capture accuracy is 0.001ms */
#define PPM_DECODER_FREQUENCY 1000000

/* default config for rc device */
#define RC_CONFIG_DEFAULT                      \
    {                                          \
        RC_PROTOCOL_SBUS, /* auto */           \
            16,            /* 6 channel */      \
            0.05f,        /* sample time */    \
            1000,         /* minimal 1000us */ \
            2000,         /* maximal 2000us */ \
    }

static ppm_decoder_t ppm_decoder;
static sbus_decoder_t sbus_decoder;


/* UART GPIO define. */
#define UART_GPIO_TX       GPIO_Pin_6
#define UART_TX_PIN_SOURCE GPIO_PinSource6
#define UART_GPIO_TX_PORT  GPIOB
#define UART_GPIO_TX_RCC   RCC_AHB1Periph_GPIOB

#define UART_GPIO_RX       GPIO_Pin_10
#define UART_RX_PIN_SOURCE GPIO_PinSource10
#define UART_GPIO_RX_PORT  GPIOA
#define UART_GPIO_RX_RCC   RCC_AHB1Periph_GPIOA

#define RCC_APBPeriph_UART RCC_APB2Periph_USART1

#define USART_IRQ          USART1_IRQn
#define USART_NUM          USART1

//#define USE_APB1
#define USE_APB2

void USART1_IRQHandler(void)
{
    uint8_t ch;
    rt_interrupt_enter();
    if (USART_GetITStatus(USART_NUM, USART_IT_RXNE) != RESET) {
        ch = USART_ReceiveData(USART_NUM); /* this process will clear EXNE flag */
        sbus_input(&sbus_decoder, &ch, 1);
        if (!sbus_islock(&sbus_decoder)) {
            sbus_update(&sbus_decoder);
        }
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

static void NVIC_Configuration(IRQn_Type irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the USART Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static rt_err_t usart_configure(USART_TypeDef* USARTx)
{    
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 100000;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_2;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USARTx, &USART_InitStructure); 
    /* Disable USART before configuration */
    USART_Cmd(USARTx, DISABLE);
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
    
    /* Enable USART */
    USART_Cmd(USARTx, ENABLE);

    return RT_EOK;
}

static rt_err_t sbus_lowlevel_init(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(UART_GPIO_RX_RCC, ENABLE);
    RCC_AHB1PeriphClockCmd(UART_GPIO_TX_RCC, ENABLE);
    /* Enable UART1 clock */
    #ifdef USE_APB1
    RCC_APB1PeriphClockCmd(RCC_APBPeriph_UART, ENABLE);
    #else
    RCC_APB2PeriphClockCmd(RCC_APBPeriph_UART, ENABLE);
    #endif

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = UART_GPIO_RX;
    GPIO_Init(UART_GPIO_RX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = UART_GPIO_TX;
    GPIO_Init(UART_GPIO_TX_PORT, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(UART_GPIO_TX_PORT, UART_TX_PIN_SOURCE, GPIO_AF_USART1);
    GPIO_PinAFConfig(UART_GPIO_RX_PORT, UART_RX_PIN_SOURCE, GPIO_AF_USART1);
    
    NVIC_Configuration(USART_IRQ);
    usart_configure(USART_NUM);

    return RT_EOK;
}

static rt_err_t rc_control(rc_dev_t rc, int cmd, void* arg)
{
    switch (cmd) {
    case RC_CMD_CHECK_UPDATE: {
        uint8_t updated = 0;

        if (rc->config.protocol == RC_PROTOCOL_SBUS) {
            updated = sbus_data_ready(&sbus_decoder);
        } else if (rc->config.protocol == RC_PROTOCOL_PPM) {
            updated = ppm_data_ready(&ppm_decoder);
        }

        *(uint8_t*)arg = updated;
    } break;

    default:
        break;
    }

    return RT_EOK;
}

static rt_uint16_t rc_read(rc_dev_t rc, rt_uint16_t chan_mask, rt_uint16_t* chan_val)
{
    uint16_t* index = chan_val;
    rt_uint16_t rb = 0;

    if (rc->config.protocol == RC_PROTOCOL_SBUS) {
        if (sbus_data_ready(&sbus_decoder) == 0) {
            /* no data received, just return */
            return 0;
        }

        sbus_lock(&sbus_decoder);

        for (uint8_t i = 0; i < min(rc->config.channel_num, sbus_decoder.rc_count); i++) {
            *(index++) = sbus_decoder.sbus_val[i];
            rb += 2;
        }
        sbus_data_clear(&sbus_decoder);

        sbus_unlock(&sbus_decoder);
    } else if (rc->config.protocol == RC_PROTOCOL_PPM) {
        if (ppm_data_ready(&ppm_decoder) == 0) {
            /* no data received, just return */
            return 0;
        }

        ppm_lock(&ppm_decoder);

        for (uint8_t i = 0; i < min(rc->config.channel_num, ppm_decoder.total_chan); i++) {
            if (chan_mask & (1 << i)) {
                *(index++) = ppm_decoder.ppm_val[i];
                rb += 2;
            }
        }
        ppm_data_clear(&ppm_decoder);

        ppm_unlock(&ppm_decoder);
    }

    return rb;
}

const static struct rc_ops rc_ops = {
    .rc_configure = NULL,
    .rc_control = rc_control,
    .rc_read = rc_read,
};

static struct rc_device rc_dev = {
    .config = RC_CONFIG_DEFAULT,
    .ops = &rc_ops,
};

rt_err_t drv_rc_init(void)
{
    RT_TRY(sbus_lowlevel_init());
    RT_TRY(sbus_decoder_init(&sbus_decoder));

    RT_CHECK(hal_rc_register(&rc_dev, "rc", RT_DEVICE_FLAG_RDWR, NULL));

    return RT_EOK;
}