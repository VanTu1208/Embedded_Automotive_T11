#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h" 
// Keil::Device:StdPeriph Drivers:TIM
uint16_t count;
void RCC_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//Cap xung cho bo chuc nang thay the (Alternative Function I/O)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void TIM_Config(void){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV2; // f = 36Mhz 
	TIM_TimeBaseInitStruct.TIM_Prescaler = 36000-1; //1ms dem len 1 lan
	TIM_TimeBaseInitStruct.TIM_Period =8000-1; //dem 8000 lan thi reset tuc la 5000ms
	
		//Cau hinh ngat cho Timer
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //TIM_IT_Update: Khi Counter dem len het 1 chu khi thi Reset va tao ra ngat
	
	TIM_Cmd(TIM2, ENABLE);

}



void GPIO_Config(void){
	GPIO_InitTypeDef GPIOInitStruct;
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_13; 
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOC, &GPIOInitStruct );    
}

void NVIC_Config(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//Cau hinh so bit cua Preemption Priority vaf Sub Prioriry
	
	NVIC_InitTypeDef NVICInitStruct;

	NVICInitStruct.NVIC_IRQChannel = TIM2_IRQn; //Cau hinh channel ngat la Timer2
	NVICInitStruct.NVIC_IRQChannelPreemptionPriority = 0; //mamg gia tri tu 0-3 vi cai dat 2bit cho Preemption Priority
	NVICInitStruct.NVIC_IRQChannelSubPriority = 0;	//mamg gia tri tu 0-3 vi cai dat 2bit cho Sub Priority
	NVICInitStruct.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVICInitStruct);
}
void delay_ms(uint16_t timedelay){
	count = 0;
	while(count < timedelay);
}

void TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)){ //Kiem tra co ngat TIM_IT_UPDATe
		count++;
	}
	// Clears the TIM2 interrupt pending bit
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}


int main() {
	RCC_Config();
	TIM_Config();
	GPIO_Config();
	NVIC_Config();
	while(1){
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		delay_ms(2000);
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		delay_ms(2000);
	}
}
