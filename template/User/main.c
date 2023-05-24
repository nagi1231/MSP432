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
#include "stdbool.h"
#include "stdio.h"
#include "Serial.h"
#include "HC-SR04.h"
#include "Delay.h"

uint32_t countValue;
char msg[30];

int main(void)
{
	Delay_Init();
	initSerial();
	init_hc_sr04();
	WDT_A_hold(WDT_A_BASE);
	MAP_FPU_enableModule();
  MAP_FPU_enableLazyStacking();
	
	while(1)
	{
		trigger_measure();
		while(GPIO_getInputPinValue(GPIO_PORT_P2,GPIO_PIN5)!=1);
		GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,GPIO_PIN5,GPIO_PRIMARY_MODULE_FUNCTION);
		Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_CONTINUOUS_MODE);
		while(GPIO_getInputPinValue(GPIO_PORT_P2,GPIO_PIN5)!=0);
		countValue=Timer_A_getCaptureCompareCount(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2);
		GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2,GPIO_PIN5);
		Timer_A_clearTimer(TIMER_A0_BASE);
		Timer_A_stopTimer(TIMER_A0_BASE);
		float distance=read_hc_sr04();
		sprintf(msg,"%f\r\n",distance);
		sendText(msg);
		delay_ms(20);
	}
}



//static volatile uint16_t curADCResult;
//static volatile float normalizedADCRes;
//static bool sendTrigger;

//const eUSCI_UART_ConfigV1 uartConfig =
//{
//        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
//        6,                                     // BRDIV = 6
//        8,                                       // UCxBRF = 8
//        0x20,                                       // UCxBRS = 0x20
//        EUSCI_A_UART_NO_PARITY,                  // No Parity
//        EUSCI_A_UART_LSB_FIRST,                  // LSB First
//        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
//        EUSCI_A_UART_MODE,                       // UART mode
//        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
//        EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
//};

//void initSerial()
//{
//	//设置smclk为12mhz
//	CS_setDCOFrequency(12000000);
//	
//	//配置p1.2，p1.3为uart引脚
//	MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
//            GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
//	
//	//初始化配置
//	UART_initModule(EUSCI_A0_BASE, &uartConfig);
//	UART_enableModule(EUSCI_A0_BASE);
//	
//	//使能中断
////	UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
////	Interrupt_enableInterrupt(INT_EUSCIA0);
////	Interrupt_enableMaster();
//}

//void sendText(char *string)
//{
//	while(*string)
//	{
//		UART_transmitData(EUSCI_A0_BASE,*string);
//		string++;
//	}
//}


//int main(void)
//{
//		initSerial();
//		GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
//    /* Initializing Variables */
//    curADCResult = 0;

//    /* Setting Flash wait state */
//    MAP_FlashCtl_setWaitState(FLASH_BANK0, 1);
//    MAP_FlashCtl_setWaitState(FLASH_BANK1, 1);
//    
//    /* Setting DCO to 48MHz  */
//    MAP_PCM_setPowerState(PCM_AM_LDO_VCORE1);
////    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);

//    /* Enabling the FPU for floating point operation */
//    MAP_FPU_enableModule();
//    MAP_FPU_enableLazyStacking();

//    //![Single Sample Mode Configure]
//    /* Initializing ADC (MCLK/1/4) */
//    MAP_ADC14_enableModule();
//    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_64, ADC_DIVIDER_8,
//            0);
//            
//    /* Configuring GPIOs (5.5 A0) */
//    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5,
//    GPIO_TERTIARY_MODULE_FUNCTION);

//    /* Configuring ADC Memory */
//    MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
//    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS,
//    ADC_INPUT_A0, false);

//    /* Configuring Sample Timer */
//    MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

//    /* Enabling/Toggling Conversion */
//    MAP_ADC14_enableConversion();
//    MAP_ADC14_toggleConversionTrigger();
//    //![Single Sample Mode Configure]

//    /* Enabling interrupts */
//    MAP_ADC14_enableInterrupt(ADC_INT0);
//    MAP_Interrupt_enableInterrupt(INT_ADC14);
//    MAP_Interrupt_enableMaster();

//    while (1)
//    {
//        if(sendTrigger){
//					char text[20];
//					sprintf(text,"Voltage is %.2f\r\n",normalizedADCRes);
//					sendText(text);
//					sendTrigger=false;
//				}
//    }
//}


//void ADC14_IRQHandler(void)
//{
//    uint64_t status = MAP_ADC14_getEnabledInterruptStatus();
//    MAP_ADC14_clearInterruptFlag(status);

//    if (ADC_INT0 & status)
//    {
//        curADCResult = MAP_ADC14_getResult(ADC_MEM0);
//        normalizedADCRes = (curADCResult * 3.3) / 16384;
//				GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN0);
//				sendTrigger=true;
////				char text[20];
////					sprintf(text,"Voltage is %.2f\r\n",normalizedADCRes);
////					sendText(text);
//        MAP_ADC14_toggleConversionTrigger();
//    }
//}

