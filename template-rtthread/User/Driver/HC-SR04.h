#ifndef __HC_SR04_H
#define __HC_SR04_H

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

//p2.5接echo，p3.0接trig

//初始化
void init_hc_sr04(void);
//触发测量
void trigger_measure(void);
//转换读数-》param：定时器捕获的计数
float read_hc_sr04(uint32_t countValue);


#endif
