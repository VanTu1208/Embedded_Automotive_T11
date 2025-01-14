#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM

#define SPI_SCK_Pin GPIO_Pin_0
#define SPI_MISO_Pin GPIO_Pin_1
#define SPI_MOSI_Pin GPIO_Pin_2
#define SPI_CS_Pin GPIO_Pin_3
#define SPI_GPIO GPIOA
#define SPI_RCC RCC_APB2Periph_GPIOA

void RCC_config(void){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		RCC_APB2PeriphClockCmd(SPI_RCC, ENABLE);
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
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_MOSI_Pin|SPI_SCK_Pin| SPI_CS_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
}

void SPISetup(void){
	GPIO_WriteBit(SPI_GPIO, SPI_MISO_Pin, Bit_SET);//Muc gi cung duoc
}

uint8_t SPI_Slave_Receive(void){
	uint8_t dataReceive =0x00;   //0b00000101
															 //0b00000001
	uint8_t temp = 0x00, i=0;
	while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin));//Cho` chan CS = 0
	while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));//Cho xung clock dau tien (bat dau truyen nhan)
	for(i=0; i<8;i++){ 
		if(GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin)){
			while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin)) // Khi co xung clock  
				temp = GPIO_ReadInputDataBit(SPI_GPIO, SPI_MOSI_Pin);// gan gia tri doc duoc tu MOSI vao temp
			//Dich gia tri vua doc duoc vao dataReceive
			dataReceive=dataReceive<<1;
			dataReceive=dataReceive|temp;
    }
		while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));//Cho het xung clock
	}
	return dataReceive;//Data 8 bit
}

uint8_t Num_Receive;
int main(){
    RCC_config();
    TIMER_config();
		GPIO_Config();
		SPISetup();
    TIM_SetCounter(TIM2,0); //Set up gia tri trong thanh ghi dem
    while(1){	
			if(!(GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin))){
				for(int i=0; i<5; i++){
					Num_Receive = SPI_Slave_Receive();
				}
			}
		}
}