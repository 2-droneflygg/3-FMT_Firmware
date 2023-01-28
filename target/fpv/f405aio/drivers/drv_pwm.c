/******************************************************************************
 * Copyright 2020 The Firmament Authors. All Rights Reserved.
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

#include "hal/actuator/actuator.h"

// #define DRV_DBG(...) console_printf(__VA_ARGS__)
#define DRV_DBG(...)

#define PWM_FREQ_50HZ  (50)
#define PWM_FREQ_125HZ (125)
#define PWM_FREQ_250HZ (250)
#define PWM_FREQ_400HZ (400)

#define MAX_PWM_OUT_CHAN      4             // AUX Out has 6 pwm channel
#define TIMER_FREQUENCY       3000000       // Timer frequency: 3M
#define PWM_DEFAULT_FREQUENCY PWM_FREQ_50HZ // pwm default frequqncy
#define VAL_TO_DC(_val)       ((float)(_val * _pwm_freq) / 1000000.0f)
#define DC_TO_VAL(_dc)        (1000000.0f / _pwm_freq * _dc)

#define PWM_ARR(freq) (TIMER_FREQUENCY / freq) // CCR reload value, Timer frequency = 3M/60K = 50 Hz
#define PWM_TIMER(id) (id < 2 ? TIM3 : TIM5)

static int _pwm_freq = PWM_DEFAULT_FREQUENCY;
static float _pwm_fmu_duty_cyc[MAX_PWM_OUT_CHAN] = { 0.00, 0.00, 0.00, 0.00};

typedef void (*timer_func)(TIM_TypeDef*, uint32_t);
timer_func _timer_set_compare[MAX_PWM_OUT_CHAN] = {
    TIM_SetCompare3,
    TIM_SetCompare4,
    TIM_SetCompare4,
    TIM_SetCompare3,
};

static void _pwm_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* TIM5 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    /* GPIOA clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // TIM5 CH3 (PA2) ;CH4 (PA3) 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect TIM5 pins to AF */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);

    /* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* GPIOB clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* TIM3 CH3 (PB0), TIM3 CH4 (PB1) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Connect TIM3 pins to AF */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);

}

static void _pwm_timer_init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    uint16_t PrescalerValue;
    RCC_ClocksTypeDef rcc_clocks;

    /* TIM1CLK = 2 * PCLK2 */
    /* PCLK2 = HCLK / 2 */
    RCC_GetClocksFreq(&rcc_clocks);

    /* Compute the prescaler value, TIM1 frequency = 3M Hz */
    PrescalerValue = (uint16_t)((rcc_clocks.PCLK1_Frequency * 2 / TIMER_FREQUENCY) - 1);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = PWM_ARR(_pwm_freq) - 1; //PWM Frequency = 3M/60K = 50 Hz
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    TIM_OC3Init(TIM5, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);

    TIM_OC4Init(TIM5, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM5, ENABLE);
    //TIM_Cmd(TIM1, ENABLE);

    /* TIM4CLK = 2 * PCLK1 */
    /* PCLK1 = HCLK / 4 */
    PrescalerValue = (uint16_t)((rcc_clocks.PCLK1_Frequency * 2 / TIMER_FREQUENCY) - 1);
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);
    //TIM_Cmd(TIM3, ENABLE);
}

rt_inline void _pwm_write(uint8_t chan_id, float duty_cyc);
rt_err_t _pwm_set_frequency(uint16_t freq_to_set)
{
    if (freq_to_set < PWM_FREQ_50HZ || freq_to_set > PWM_FREQ_400HZ) {
        /* invalid frequency */
        return RT_EINVAL;
    }

    _pwm_freq = freq_to_set;

    _pwm_timer_init();

    /* the timer compare value should be re-configured */
    for (uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
        _pwm_write(i, _pwm_fmu_duty_cyc[i]);
    }

    return RT_EOK;
}

rt_inline void _pwm_write(uint8_t chan_id, float duty_cyc)
{
    _timer_set_compare[chan_id](PWM_TIMER(chan_id), PWM_ARR(_pwm_freq) * duty_cyc);

    _pwm_fmu_duty_cyc[chan_id] = duty_cyc;
}

rt_inline void _pwm_read(uint8_t chan_id, float* duty_cyc)
{
    *duty_cyc = _pwm_fmu_duty_cyc[chan_id];
}

rt_err_t pwm_config(actuator_dev_t dev, const struct actuator_configure* cfg)
{
    DRV_DBG("aux out configured: pwm frequency:%d\n", cfg->pwm_config.pwm_freq);

    if (_pwm_set_frequency(cfg->pwm_config.pwm_freq) != RT_EOK) {
        return RT_ERROR;
    }
    /* update device configuration */
    dev->config = *cfg;

    return RT_EOK;
}

rt_err_t pwm_control(actuator_dev_t dev, int cmd, void* arg)
{
    rt_err_t ret = RT_EOK;

    switch (cmd) {
    case ACT_CMD_CHANNEL_ENABLE:
        /* set to lowest pwm before open */
        for (uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
            _pwm_write(i, VAL_TO_DC(1000));
        }
        TIM_Cmd(TIM5, ENABLE);
        //TIM_CtrlPWMOutputs(TIM5, ENABLE);
        TIM_Cmd(TIM3, ENABLE);
        //TIM_CtrlPWMOutputs(TIM3, ENABLE);
    

        DRV_DBG("aux out enabled\n");
        break;
    case ACT_CMD_CHANNEL_DISABLE:
        TIM_Cmd(TIM5, DISABLE);
        //TIM_CtrlPWMOutputs(TIM5, DISABLE);
        TIM_Cmd(TIM3, DISABLE);
        //TIM_CtrlPWMOutputs(TIM3, DISABLE);

        DRV_DBG("aux out disabled\n");
        break;
    default:
        ret = RT_EINVAL;
        break;
    }

    return ret;
}

rt_size_t pwm_read(actuator_dev_t dev, rt_uint16_t chan_sel, rt_uint16_t* chan_val, rt_size_t size)
{
    rt_uint16_t* index = chan_val;
    float dc;

    for (uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
        if (chan_sel & (1 << i)) {
            _pwm_read(i, &dc);
            *index = DC_TO_VAL(dc);
            index++;
        }
    }

    return size;
}

rt_size_t pwm_write(actuator_dev_t dev, rt_uint16_t chan_sel, const rt_uint16_t* chan_val, rt_size_t size)
{
    const rt_uint16_t* index = chan_val;
    rt_uint16_t val;
    float dc;

    for (uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
        if (chan_sel & (1 << i)) {
            val = *index;
            /* calculate pwm duty cycle */
            dc = VAL_TO_DC(val);
            /* update pwm signal */
            _pwm_write(i, dc);

            index++;
        }
    }

    return size;
}

const static struct actuator_ops _act_ops = {
    .dev_config = pwm_config,
    .dev_control = pwm_control,
    .dev_read = pwm_read,
    .dev_write = pwm_write
};

static struct actuator_device act_dev = {
    .chan_mask = 0x3F,
    .range = { 1000, 2000 },
    .config = {
        .protocol = ACT_PROTOCOL_PWM,
        .chan_num = MAX_PWM_OUT_CHAN,
        .pwm_config = { .pwm_freq = 50 },
        .dshot_config = { 0 } },
    .ops = &_act_ops
};

rt_err_t drv_pwm_init(void)
{
    rt_err_t ret;

    _pwm_gpio_init();
    _pwm_timer_init();

    /* Disable output by default, need ne enabled by uper layer */
    TIM_Cmd(TIM5, DISABLE);
    //TIM_CtrlPWMOutputs(TIM5, DISABLE);
    TIM_Cmd(TIM3, DISABLE);
    //TIM_CtrlPWMOutputs(TIM3, DISABLE);

    /* register actuator hal device */
    ret = hal_actuator_register(&act_dev, "main_out", RT_DEVICE_FLAG_RDWR, NULL);

    return ret;
}
