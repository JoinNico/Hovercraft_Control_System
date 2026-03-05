/*
 * voltage.c
 *
 *  Created on: 2026年3月4日
 *      Author: Administrator1
 */

#include "voltage.h"

float voltage = 0.0f;
uint16_t raw_voltage = 0;

void update_voltage(void)
{
    // 获取滤波后的ADC值
    raw_voltage = adc_mean_filter_convert(ADC1_IN9_B1, 10);

    // 转换为实际电压
    voltage = raw_voltage * ADC_TO_VOLT;
}
