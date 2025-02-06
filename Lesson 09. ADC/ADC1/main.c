#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_adc.h"              // Keil::Device:StdPeriph Drivers:ADC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM

void RCC_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void TIMER_config(void){
    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV2; //Chia thanh clock nho hon de cap cho timer default: 72MHz (1s tao duoc 72 trieu dao dong) , DIV1: khong chia
    TIM_InitStruct.TIM_Prescaler = 36000; //Sau bao nhieu dao dong thi se dem len 1 lan.  1s = 72M giao dong, gia tri < 65535, neu lon hon thi doi bo chia
    //VD muon 1ms dem len 1 lan thi (10^-3)/(1/36M) = 36000 dao dong
    TIM_InitStruct.TIM_Period  = 0xFFFF;//Dem bao nhieu lan thi reset
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up; //Set mode dem len tu 0
    TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
    
    TIM_Cmd(TIM2, ENABLE); //Cho phep timer2 hoat dong
}

void delay_ms(uint16_t timedelay){
    TIM_SetCounter(TIM2, 0);
    while(TIM_GetCounter(TIM2)<timedelay){}
}

void GPIO_Config(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN; //Analog input
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void ADC_Config(void){
	ADC_InitTypeDef ADC_InitStruct;
	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent; //Hoat dong nhu ADC binh thuong, doc lap voi nhau va khong can kich hoat
	ADC_InitStruct.ADC_NbrOfChannel = 1; //NumberOfChannel so luong kenh can cau hinh (1-16)
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;//Hoat dong o che do Continous hay khong
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //Cau hinh cho phep su dung Trigger (tin hieu de bat dau chuyen doi ADC)
	ADC_InitStruct.ADC_ScanConvMode = DISABLE; //Co su dung Scan de quet nhieu kenh khong
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;//Cau hinh de can le cho Data (ADC 12bit luu vao 16bit bi du 4 bit), ghi vao LSB hay MSB

	ADC_Init(ADC1, &ADC_InitStruct);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	
	ADC_Cmd(ADC1, ENABLE);
	//Bat dau qua trinh chuyen doi (Vi chon che do Continous nen chi can goi 1 lan)
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

uint16_t lightValue, sum, analogValue;
int main() {
	RCC_Config();
	TIMER_config();
	GPIO_Config();
	ADC_Config();
	while(1){
		sum = 0;
		for(int i = 0; i < 10; i++)
		{
			lightValue = ADC_GetConversionValue(ADC1);
			sum += lightValue;
			delay_ms(100);
		}
		analogValue = sum/10;
	}
}
