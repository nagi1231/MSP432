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
#include "stdio.h"
#include "stdbool.h"
#include "string.h"

//主时钟信号为12Mhz，在system_msp432p401r.c中配置
//rt_thread_t blink_thread = RT_NULL;

//信号量
//rt_sem_t AbleToConvert;
//extern uint32_t countValue;

//红灯闪烁的线程
static void blink_entry()
{
	GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
	while(1)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN0);
		rt_thread_mdelay(500);
	}
}

//hcsr采样的线程
static void hcsr_entry()
{
	init_hc_sr04();
	char text[20];
	AbleToConvert=rt_sem_create("AbleToConvert",0,RT_IPC_FLAG_PRIO);
	while(1)
	{
		Interrupt_enableInterrupt(INT_PORT2);
		trigger_measure();
		//获得信号量，将其通过串口发送；最多等待50tick，防止错过触发中断导致没获得信号一直卡死
		if(rt_sem_take(AbleToConvert,50)==RT_EOK)
		{
			sprintf(text,"%f\r\n",read_hc_sr04(countValue));
			sendText(text);
			
		}
	}
}

//处理蓝牙信号的线程
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
			//从机收到主机的连接信号
			if(!strcmp(btdata,"connecting"))
			{
				sendMsgByBlueTooth("connected\r\n");
				connected=true;
			}
			//主机收到从机的连接确认 
			if(!strcmp(btdata,"connected"))
			{
				connected=true;
				//发送给电脑告知蓝牙连接完毕
				sendText("bluetooth connected\r\n");
			}
		}
	}
}




int main(void)
{
	WDT_A_hold(WDT_A_BASE);
	Interrupt_enableMaster();
	Delay_Init();
  initSerial();
	
	//创建并运行hcsr线程
	rt_thread_t hcsr_thread=rt_thread_create("HC-SR04",hcsr_entry,RT_NULL,1024,25,50);
	if(hcsr_thread!=RT_NULL)
	{
		rt_thread_startup(hcsr_thread);
	}
	
	
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
