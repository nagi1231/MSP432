#ifndef __HC_SR04_H
#define __HC_SR04_H

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

//p2.5��echo��p3.0��trig

//��ʼ��
void init_hc_sr04(void);
//��������
void trigger_measure(void);
//ת������-��param����ʱ������ļ���
float read_hc_sr04(uint32_t countValue);


#endif