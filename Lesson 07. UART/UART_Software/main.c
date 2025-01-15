#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM


#define TX_Pin GPIO_Pin_0
#define RX_Pin GPIO_Pin_1
#define UART_GPIO GPIOA
#define time_duration 104

void RCC_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void TIMER_config(void){
    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1; //Chia thanh clock nho hon de cap cho timer default: 72MHz (1s tao duoc 72 trieu dao dong) , DIV1: khong chia
    TIM_InitStruct.TIM_Prescaler = 72-1; //Sau bao nhieu dao dong thi se dem len 1 lan.  1s = 72M giao dong, gia tri < 65535, neu lon hon thi doi bo chia
    //VD muon 1us dem len 1 lan thi (10^-6)/(1/36M) = 36000 dao dong
    TIM_InitStruct.TIM_Period  = 0xFFFF;//Dem bao nhieu lan thi reset
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up; //Set mode dem len tu 0
    TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
    
    TIM_Cmd(TIM2, ENABLE); //Cho phep timer2 hoat dong
}

void delay_us(uint16_t timedelay){
    TIM_SetCounter(TIM2, 0);
    while(TIM_GetCounter(TIM2)<timedelay){}
}

void delay_s(uint32_t timedelay){
    TIM_SetCounter(TIM2, 0);
			for(int i = 0; i < timedelay*1000000; i++){
				delay_us(1);
			}
}

void clock(){
	delay_us(time_duration);
}

void GPIO_Config(){
	GPIO_InitTypeDef GPIOInitStruct;
	GPIOInitStruct.GPIO_Pin = RX_Pin;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIOInitStruct);
	//
	GPIOInitStruct.GPIO_Pin = TX_Pin;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIOInitStruct);
}

void UART_Config(){
	GPIO_SetBits(UART_GPIO, TX_Pin);
	delay_us(1);
}


void UARTSoftware_Transmit(char c) {
    // Start bit
    GPIO_ResetBits(GPIOA, TX_Pin); // Tao dieu kien bat dau
    clock();

    // Truyen các bit du lieu (LSB truoc)
    for (int i = 0; i < 8; i++) {
        if (c & (1 << i)) {
            GPIO_SetBits(GPIOA, TX_Pin);
        } else {
            GPIO_ResetBits(GPIOA, TX_Pin);
        }
        clock();
    }

    // Stop bit
    GPIO_SetBits(GPIOA, TX_Pin);
    clock();
}

char UARTSoftware_Receive() {
    char c = 0;
		// Start bit
    
		
		while (GPIO_ReadInputDataBit(GPIOA, RX_Pin) == 1);

		// Ch? m?t n?a th?i gian bit d? vào gi?a Start bit
		delay_us(time_duration + time_duration / 2);

		// Ð?c các bit d? li?u (LSB tru?c)
		for (int i = 0; i < 8; i++) {
				if (GPIO_ReadInputDataBit(GPIOA, RX_Pin)) {
						c |= (1 << i);
				}
				clock(); // Ð?i d?n gi?a bit ti?p theo
		}

		// Ð?i Stop bit
		delay_us(time_duration / 2);
		
		return c;
}

char data[9] = {'V', 'A', 'N', 'T', 'U', 'H', 'A', 'L', 'A'};
int main(){
	RCC_Config();
	GPIO_Config();
	TIMER_config();
	UART_Config();
	for (int i = 0; i<9; i++){
			UARTSoftware_Transmit(data[i]);
		delay_s(1);
	}
		UARTSoftware_Transmit('\n');
		
	while(1){
		UARTSoftware_Transmit(UARTSoftware_Receive());
	}	
}
		
