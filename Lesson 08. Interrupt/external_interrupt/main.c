#include "stm32f10x.h"                  // Device header
#include "stm32f10x_exti.h"             // Keil::Device:StdPeriph Drivers:EXTI
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM


void RCC_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//Cap xung cho bo chuc nang thay the (Alternative Function I/O)
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
    
    //chia clock cho 2 de 1s tao duoc 36.000.000 dao dong trong 1s, tuc 1 dao dong mat 1/36.000.000
    //Prescaler = 36.000 tuc la voi moi 36.000 dao dong thi dem len 1 lan
    //tuc la mat (1/36.000.000)*(36.000) = 1/1000s = 1ms thi dem len mot lan
}

void delay_ms(uint16_t timedelay){
    TIM_SetCounter(TIM2, 0);
    while(TIM_GetCounter(TIM2)<timedelay){}
}


void GPIO_Config(void){
	GPIO_InitTypeDef GPIOInitStruct;
	//Cau hinh cho chan interrupt PA0
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_IPU; //Input Pull_up
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIOInitStruct);
	//Cau hinh cho PC13
	GPIO_InitTypeDef GPIOInitStruct1;
	GPIOInitStruct1.GPIO_Pin = GPIO_Pin_13; 
	GPIOInitStruct1.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIOInitStruct1.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOC, &GPIOInitStruct1 );    
}

void EXTI_Config(void) {
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0); //PA0
	
	EXTI_InitTypeDef EXTIInitStruct;
	
	EXTIInitStruct.EXTI_Line = EXTI_Line0;//Xac dinh duong ngat (EXTIn) co 18 line va 15 line duoc cau hinh san, 3line mo rong 
	EXTIInitStruct.EXTI_Mode = EXTI_Mode_Interrupt; //Thuc thi ham ngat khi xay ra ngat ---- Even thi khong thuc thi ham ngat
	EXTIInitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//chon canh kich hoat ngat
	EXTIInitStruct.EXTI_LineCmd = ENABLE;
	
	EXTI_Init(&EXTIInitStruct);
}

void NVIC_Config(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//Cau hinh so bit cua Preemption Priority vaf Sub Prioriry
	
	NVIC_InitTypeDef NVICInitStruct;

	NVICInitStruct.NVIC_IRQChannel = EXTI0_IRQn; //Dang cau hinh ngat Line0 nen chon vector ngat EXTI0_IRQn
	NVICInitStruct.NVIC_IRQChannelPreemptionPriority = 1; //mamg gia tri tu 0-3 vi cai dat 2bit cho Preemption Priority
	NVICInitStruct.NVIC_IRQChannelSubPriority = 1;	//mamg gia tri tu 0-3 vi cai dat 2bit cho Sub Priority
	NVICInitStruct.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVICInitStruct);
}

//Ham xu ly khi co ngat line0
void EXTI0_IRQHandler(){	
	if(EXTI_GetITStatus(EXTI_Line0) != RESET){ //Kiem tra co ngat cua Line0
		for(int i = 0; i<10; i++){
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
			delay_ms(200);
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			delay_ms(200);
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line0); //Xoa co ngat cua line0
}


int main(){
	RCC_Config();
	GPIO_Config();
	EXTI_Config();
	NVIC_Config();
	TIMER_config();
	while(1){
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		delay_ms(3000);
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		delay_ms(3000);
  }   
}
