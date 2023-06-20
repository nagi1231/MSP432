/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//***************************************************************************************
//  Blink the LED Demo - Software Toggle P1.0
//
//  Description; Toggle P1.0 inside of a software loop.
//  ACLK = n/a, MCLK = SMCLK = default DCO
//
//                MSP432P4xx
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |             P1.0|-->LED
//
//  E. Chen
//  Texas Instruments, Inc
//  March 2015
//  Built with Code Composer Studio v6
//***************************************************************************************

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "rtthread.h"
#include "HC-SR04.h"
#include "Delay.h"
#include "Serial.h"
#include "HC-05.h"
#include "Encoder.h"
#include "oled.h"
#include "Motor.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"

//��ʱ���ź�Ϊ12Mhz����system_msp432p401r.c������
//rt_thread_t blink_thread = RT_NULL;

//�ź���
//rt_sem_t AbleToConvert;
//extern uint32_t countValue;

//�����˸���߳�
static void blink_entry()
{
	GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
	while(1)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN0);
		rt_thread_mdelay(500);
	}
}

//hcsr�������߳�
static void hcsr_entry()
{
	init_hc_sr04();
	char text[20];
	AbleToConvert=rt_sem_create("AbleToConvert",0,RT_IPC_FLAG_PRIO);
	while(1)
	{
		Interrupt_enableInterrupt(INT_PORT2);
		trigger_measure();
		//����ź���������ͨ�����ڷ��ͣ����ȴ�50tick����ֹ��������жϵ���û����ź�һֱ����
		if(rt_sem_take(AbleToConvert,50)==RT_EOK)
		{
			sprintf(text,"%f\r\n",read_hc_sr04(countValue));
			sendText(text);
			
		}
	}
}

//���������źŵ��߳�
static void hc05_entry()
{
	init_hc05();
	ProcessBtData=rt_sem_create("ProcessBtData",0,RT_IPC_FLAG_PRIO);
	bool connected=false;
	while(1)
	{
		#if MASTER_CAR
		while(!connected){
			sendMsgByBlueTooth("connecting");
		}
		#endif
		if(rt_sem_take(ProcessBtData,RT_WAITING_FOREVER)==RT_EOK)
		{
			//�ӻ��յ������������ź�
			if(!strcmp(btdata,"connecting"))
			{
				sendMsgByBlueTooth("connected\r\n");
				connected=true;
			}
			//�����յ��ӻ�������ȷ�� 
			if(!strcmp(btdata,"connected"))
			{
				connected=true;
				//���͸����Ը�֪�����������
				sendText("bluetooth connected\r\n");
			}
		}
	}
}

void sendEncoderBack()
{
	int encoder_left,encoder_right;
	encoder_left=read_encoder(0);
	encoder_right=read_encoder(1);
	char text[30];
	sprintf(text,"right: %d;left: %d\r\n",encoder_right,encoder_left);
	sendText(text);
}

static void encoder_entry()
{
	init_encoder_left();
	init_encoder_right();
	rt_timer_t EncoderTimer=rt_timer_create("EncoderTimer",sendEncoderBack,RT_NULL,100,RT_TIMER_FLAG_PERIODIC);
	if(EncoderTimer!=RT_NULL)
	{
		rt_timer_start(EncoderTimer);
	}
	
}


static void motor_entry()
{
	init_motor();
	rt_timer_t MotorTimer=rt_timer_create("MotorTimer",set_pwm_trail,RT_NULL,10,RT_TIMER_FLAG_PERIODIC);
	if(MotorTimer!=RT_NULL)
	{
		rt_timer_start(MotorTimer);
	}
}

static void oled_entry()
{
	init();
	OLED_Init();
	OLED_Clear();
	while(1)
	{
    delay_ms(5);
    OLED_ShowString(0,0,(unsigned char *)"  2021  8.4");
    OLED_ShowString(0,2,(unsigned char *)" NUEDC Contest ");
  	delay_ms(25);
	}
}

static void control_entry()
{
	init_encoder_left();
	init_encoder_right();
	init_motor();
	rt_timer_t MotorTimer=rt_timer_create("MotorTimer",set_pwm_trail,RT_NULL,10,RT_TIMER_FLAG_PERIODIC);
	if(MotorTimer!=RT_NULL)
	{
		rt_timer_start(MotorTimer);
	}
}

static void display_entry()
{
	init();
	OLED_Init();
	OLED_Clear();
	char text1[20];
	char text2[20];
	while(1)
	{
		int encoder_left,encoder_right;
		encoder_left=read_encoder(0);
		encoder_right=read_encoder(1);
		sprintf(text1,"r:%2d",encoder_right);
		sprintf(text2,"l:%2d",encoder_left);
		OLED_ShowString(0,0,(unsigned char *)text1);
		OLED_ShowString(0,2,(unsigned char *)text2);
	}
	
}
int main(void)
{
	WDT_A_hold(WDT_A_BASE);
	Interrupt_enableMaster();
	Delay_Init();
  initSerial();
	
	rt_thread_t control_thread=rt_thread_create("control",control_entry,RT_NULL,1024,25,50);
	if(control_thread!=RT_NULL)
	{
		rt_thread_startup(control_thread);
	}

	rt_thread_t display_thread=rt_thread_create("display",display_entry,RT_NULL,1024,25,50);
	if(display_thread!=RT_NULL)
	{
		rt_thread_startup(display_thread);
	}
	//����������Oled�߳�
//	rt_thread_t oled_thread=rt_thread_create("OLED",oled_entry,RT_NULL,1024,25,50);
//	if(oled_thread!=RT_NULL)
//	{
//		rt_thread_startup(oled_thread);
//	}
	
	
	//����������motor�߳�
//	rt_thread_t motorTrail_thread=rt_thread_create("MotorTrail",motor_entry,RT_NULL,1024,25,50);	
//	if(motorTrail_thread!=RT_NULL)
//	{
//		rt_thread_startup(motorTrail_thread);
//	}
	
	
  //����������encoder�߳�
//	rt_thread_t encoder_thread=rt_thread_create("Encoder",encoder_entry,RT_NULL,1024,25,50);
//	if(encoder_thread!=RT_NULL)
//	{
//		rt_thread_startup(encoder_thread);
//	}
	
	
	//����������hcsr�߳�
//	rt_thread_t hcsr_thread=rt_thread_create("HC-SR04",hcsr_entry,RT_NULL,1024,25,50);
//	if(hcsr_thread!=RT_NULL)
//	{
//		rt_thread_startup(hcsr_thread);
//	}
	
	
//	blink_thread=rt_thread_create("blink",blink_entry,RT_NULL,1024,25,5);
//	if(blink_thread!=RT_NULL)
//	{
//		rt_thread_startup(blink_thread);
//	}
}





























//int main(void)
//{
//    volatile uint32_t i;

//    // Stop watchdog timer
//    WDT_A_hold(WDT_A_BASE);

//    // Set P1.0 to output direction
//    GPIO_setAsOutputPin(
//        GPIO_PORT_P1,
//        GPIO_PIN0
//        );

//    while(1)
//    {
//        // Toggle P1.0 output
//        GPIO_toggleOutputOnPin(
//            GPIO_PORT_P1,
//			GPIO_PIN0
//			);

//        // Delay
//        for(i=100000; i>0; i--);
//    }
//}
