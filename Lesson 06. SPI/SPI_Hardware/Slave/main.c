#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_spi.h"              // Keil::Device:StdPeriph Drivers:SPI

#define SPI1_NSS 	GPIO_Pin_4
#define SPI1_SCK	GPIO_Pin_5
#define SPI1_MISO GPIO_Pin_6
#define SPI1_MOSI GPIO_Pin_7
#define SPI1_GPIO GPIOA

void RCC_config(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA, ENABLE);
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

uint8_t SPI_Receive1Byte(uint8_t data_to_send_back){
    uint8_t temp;//Bien tam
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==1);//Co bao nhan SPI_I2S_FLAG_BSY = 1 khi SPI dang ban, Cho` den khi SPI ranh?
    temp = (uint8_t)SPI_I2S_ReceiveData(SPI1); // Tra ve gia tri doc duoc tren SPI1
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==0); //cho` den khi nhan xong data SPI_I2S_FLAG_RXNE = 1
    
		SPI_I2S_SendData(SPI1, data_to_send_back); // G?i d? li?u tr? l?i
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	return temp;
}

uint8_t Num_Receive;
uint8_t k[5] = {12, 13, 14, 15, 16};
int main(){
    RCC_config();
		GPIO_Config();
		SPI_config();
    while(1){   
			while(GPIO_ReadInputDataBit(SPI1_GPIO, SPI1_NSS));
       if(GPIO_ReadInputDataBit(SPI1_GPIO, SPI1_NSS)==0){
          for(int i = 0; i<5; i++){
            Num_Receive = SPI_Receive1Byte(k[i]);
          }
       }
		}

}
