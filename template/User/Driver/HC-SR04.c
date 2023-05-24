#include "HC-SR04.h"
#include "stdio.h"
#include "Serial.h"
#include "Delay.h"

extern char msg[];
extern uint32_t countValue;

//p2.5为输入引脚
const Timer_A_CaptureModeConfig hcsrCaptureConfig=
{
	TIMER_A_CAPTURECOMPARE_REGISTER_2,
	TIMER_A_CAPTUREMODE_FALLING_EDGE,
	TIMER_A_CAPTURE_INPUTSELECT_CCIxA,
	TIMER_A_CAPTURE_SYNCHRONOUS,
	TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,
	TIMER_A_OUTPUTMODE_OUTBITVALUE
};
//连续模式，SMCLK为时钟源，12分频————1mhz
const Timer_A_ContinuousModeConfig hcsrContinueConfig=
{
	TIMER_A_CLOCKSOURCE_SMCLK,
	TIMER_A_CLOCKSOURCE_DIVIDER_12,
	TIMER_A_TAIE_INTERRUPT_DISABLE,
	TIMER_A_SKIP_CLEAR
};

void init_hc_sr04(void)
{
	//p2.5下拉输入，上升沿触发外部中断
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2,GPIO_PIN5);
//	GPIO_clearInterruptFlag(GPIO_PORT_P2,GPIO_PIN5);
//	GPIO_selectInterruptEdge(GPIO_PORT_P2,GPIO_PIN5,GPIO_LOW_TO_HIGH_TRANSITION);
//	GPIO_enableInterrupt(GPIO_PORT_P2,GPIO_PIN5);
//	Interrupt_enableInterrupt(INT_PORT2);

	//p3.0输出
	GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN0);
	GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN0);

	//初始化时钟为连续捕获模式
	Timer_A_initCapture(TIMER_A0_BASE,&hcsrCaptureConfig);
	Timer_A_configureContinuousMode(TIMER_A0_BASE,&hcsrContinueConfig);
	sprintf(msg,"init finish");
	sendText(msg);
}
void trigger_measure(void)
{
//	sprintf(msg,"trigger begin");
//	sendText(msg);
	GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN0);
//	__delay_cycles(120);
	delay_ms(1);
	GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN0);
//	sprintf(msg,"trigger finish");
//	sendText(msg);
}
//单位为cm
float read_hc_sr04()
{
	float distance=countValue*0.017;
	return distance;
}


void PORT2_IRQHandler(void)
{
	uint32_t status=GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);
	GPIO_clearInterrupt(GPIO_PORT_P2,status);
	
	if(status&GPIO_PIN5)
	{
//		sprintf(msg,"high interrupt");
//		sendText(msg);
		GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,GPIO_PIN5,GPIO_PRIMARY_MODULE_FUNCTION);
		Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_CONTINUOUS_MODE);
	}
}

void TA0_N_IRQHandler(void)
{
//	sprintf(msg,"count finish");
//	sendText(msg);
	Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2);
	countValue=Timer_A_getCaptureCompareCount(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2);
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2,GPIO_PIN5);
	Timer_A_clearTimer(TIMER_A0_BASE);
	Timer_A_stopTimer(TIMER_A0_BASE);
}
