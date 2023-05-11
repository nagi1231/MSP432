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

rt_thread_t blink_thread = RT_NULL;

static void blink_entry()
{
	WDT_A_hold(WDT_A_BASE);
	GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
	while(1)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN0);
		rt_thread_mdelay(500);
	}
}

int main(void)
{
	blink_thread=rt_thread_create("blink",blink_entry,RT_NULL,1024,25,5);
	if(blink_thread!=RT_NULL)
	{
		rt_thread_startup(blink_thread);
	}
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
