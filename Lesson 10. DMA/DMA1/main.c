#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_spi.h"              // Keil::Device:StdPeriph Drivers:SPI
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_dma.h"              // Keil::Device:StdPeriph Drivers:DMA


#define SPI1_NSS  GPIO_Pin_4
#define SPI1_SCK  GPIO_Pin_5
#define SPI1_MISO GPIO_Pin_6
#define SPI1_MOSI GPIO_Pin_7
#define SPI1_GPIO GPIOA
uint8_t ServoDegree[12];

void RCC_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_SPI1| RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
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
    GPIO_InitStructure.GPIO_Pin = SPI1_NSS| SPI1_SCK| SPI1_MISO| SPI1_MOSI;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //Chan hoat dong trong 1 chuc nang thay the nhu I2C, SPI
    GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);

}

void SPI_config(void){
	SPI_InitTypeDef SPI_InitStruct;
	
	SPI_InitStruct.SPI_Mode = SPI_Mode_Slave; //Chon kieu thiet bi master hay slave
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //Quy dinh kieu truyen Song cong, Ban song cong, don cong
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; //Chia tan so xung Clock. Tan so mac dinh 72MHZ
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low; // Khi k cap xung thi SCK = 0
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge; //Tin hieu truyen di o canh xung dau tien
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;//Truyen 8bit hoac 16bit
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_LSB; // Truyen bit trong so cao truoc
	SPI_InitStruct.SPI_CRCPolynomial = 7 ;//Cau hinh Checksum, neu Data8bit thi de 7, neu 16bit thi de 15, co the de mac dinh
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft; //Cau hinh che do chan CS dc quan ly boi Sorfware (1 bit SSI) hay Hardware (1 Pin NSS)

	SPI_Init(SPI1, &SPI_InitStruct);
	SPI_Cmd(SPI1,ENABLE);
        
}
void DMA_Config(void){
	DMA_InitTypeDef DMA_InitStruct;

	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;//Cau hinh dia chi ngoai vi cho DMA (Dia chi de DMA lay data hay truyen data toi)
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)ServoDegree;//Dia chi trong RAM de lay hoac truyen du lieu
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;//Xac dinh ngoai vi la nguon cua data hay dich cua data
	DMA_InitStruct.DMA_BufferSize = 11;//Cau hinh so luong don vi data moi lan truyen
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//Kich thuoc moi data khi truyen (byte8bit, HalfWord 16bit, Word 32bit(doi voi STM32))
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //Kich thuoc data luu vao Memory
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//DMA  truyen xong 1 data, dia chi tiep theo co duoc tang len hay khong (vi chi su dung SPI nen DISABLE)
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable; //Dia chi bien co duoc tang moi khi luu.
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular; //Circular hoac normal
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable; //Co truyen du lieu tu MEM-MEM hay khong
	
	
	DMA_Init(DMA1_Channel2, &DMA_InitStruct);
	DMA_Cmd(DMA1_Channel2,ENABLE);
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
		
	
}


uint8_t SPI_Receive1Byte(void){
	uint8_t temp;//Bien tam
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==1);//Co bao nhan SPI_I2S_FLAG_BSY = 1 khi SPI dang ban, Cho` den khi SPI ranh?
	temp = (uint8_t)SPI_I2S_ReceiveData(SPI1); // Tra ve gia tri doc duoc tren SPI1
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==0); //cho` den khi nhan xong data SPI_I2S_FLAG_RXNE = 1
	return temp;
}



uint16_t i = 0;
int main() {
	RCC_Config();
	SPI_config();
	GPIO_Config();
	DMA_Config();
	TIMER_config();
	while(1){
		if(i > 20) i = 0;
		i++;
	}
}
