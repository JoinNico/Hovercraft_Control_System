/*
 * voltage.h
 *
 *  Created on: 2026年3月4日
 *      Author: Administrator1
 */

#ifndef VOLTAGE_H_
#define VOLTAGE_H_
#include "zf_common_headfile.h"

// ADC参数
#define ADC_REF_VOLTAGE     3.3f        // 参考电压 3.3V
#define ADC_RESOLUTION      4095.0f      // 12位ADC
#define ADC_BITS            12

// 分压电路参数
#define R_TOP               47.0f        // 上分压电阻 (kΩ)
#define R_BOTTOM            10.0f        // 下分压电阻 (kΩ)
#define DIVIDER_RATIO       ((R_TOP + R_BOTTOM) / R_BOTTOM)  // 分压比

// 转换系数
#define ADC_TO_VOLT         (ADC_REF_VOLTAGE * DIVIDER_RATIO / ADC_RESOLUTION)

extern float voltage;
extern uint16_t raw_voltage;

void update_voltage(void);
static inline float get_voltage(void) {
    return voltage;
}
static inline uint16_t get_raw_voltage(void) {
    return raw_voltage;
}
static inline bool is_voltage_normal(float min_v, float max_v) {
    return (voltage >= min_v) && (voltage <= max_v);
}
#endif /* VOLTAGE_H_ */
