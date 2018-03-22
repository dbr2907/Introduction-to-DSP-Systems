/*This work is licensed under a
Creative Commons Attribution-NonCommmercial-ShareAlike 4.0 International

This license lets others remix, tweak, and build upon this work non-commercially,
as long as they credit the AUTHOR and license their new creations under the identical terms.
*/

/*
David Josue Barrientos Rojas (2016)
Kaiser-Bessel FIR Low Pass Filter:
Fa=0, Fb=6K, Fs=44100 Hz, Samples=37, Attenuation=80 dB
d[dot]b[dot]gt[at]ieee[dot]org
Universidad de San Carlos de Guatemala
EE School
*/

//********************************Librerias*****************************************//
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
//******************************Librerias Extras************************************//
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
//*********************************Definiciones*************************************//
#define Q 37
//**********************************Variables***************************************//
uint32_t sample;

volatile float memoria[Q];
volatile float coeficientes_pb[Q]={0.000015, 0.000162, 0.000382, 0.000219, -0.000883, -0.002615, -0.003096, 0.000136, 0.007163, 0.013132,
0.009801, -0.007535, -0.031913, -0.043207, -0.018170, 0.052239, 0.150621, 0.237451, 0.272109, 0.237451, 0.150621, 0.052239, -0.018170,
-0.043207, -0.031913, -0.007535, 0.009801, 0.013132, 0.007163, 0.000136, -0.003096, -0.002615, -0.000883, 0.000219, 0.000382, 0.000162, 0.000015};
volatile int signal;
//***********************************Metodos****************************************//
//--------------------Timer Init----------------------//
void Timer_Init(uint32_t Value) {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE, TIMER_A, Value - 1);
	TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
	TimerEnable(TIMER0_BASE, TIMER_A);
}
//--------------------SSI Init----------------------//
//--------MODE 0, Master, 2MHZ and 16 bits----------//
void SSI_Init(void) {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5);
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_TI, SSI_MODE_MASTER, 2000000, 16);
	SSIEnable(SSI0_BASE);
}
//----------------ADC Interrupt Handler----------------//
void DataGet(void) {
    volatile short i=0;
    volatile float out=0;
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, &sample);
    memoria[0]=sample;
    for (i=0;i<Q;i++){out += (memoria[i]*coeficientes_pb[i]);}
    signal=(int)out;
    SSIDataPut(SSI0_BASE, signal);
    for (i=1;i<Q;i++){memoria[Q-i]=memoria[Q-1-i];}
	SSIDataPut(SSI0_BASE, sample);
}

//************************************Main******************************************//
int main(void){
    /****************************CLOCK**********************************/
    SysCtlClockSet(SYSCTL_XTAL_16MHZ|SYSCTL_SYSDIV_4);
	IntMasterEnable();

	Timer_Init(1133);
	SSI_Init();
	/**************************ADC SETUP********************************/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_5);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH11 | ADC_CTL_IE | ADC_CTL_END);
	ADCIntEnable(ADC0_BASE, 3);
	IntEnable(INT_ADC0SS3);

	ADCSequenceEnable(ADC0_BASE, 3);

	while(1){
	}

}


