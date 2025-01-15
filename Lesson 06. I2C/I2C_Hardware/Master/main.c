#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_i2c.h"              // Keil::Device:StdPeriph Drivers:I2C

#define I2C_SCL 	GPIO_Pin_6
#define I2C_SDA		GPIO_Pin_7
#define I2C_GPIO 	GPIOB
#define DS1307_ADDRESS 0x50

void RCC_Config(void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
}

void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void TIMER_Config(void) {
    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV2;
    TIM_InitStruct.TIM_Prescaler = 36;
    TIM_InitStruct.TIM_Period  = 0xFFFF;
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
    
    TIM_Cmd(TIM2, ENABLE);
}

void I2C_Config(void) {
	I2C_InitTypeDef I2C_InitStruct;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_ClockSpeed = 400000;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	I2C_Init(I2C1, &I2C_InitStruct);
	I2C_Cmd(I2C1, ENABLE);
}

void delay_us(uint16_t timedelay){
    TIM_SetCounter(TIM2, 0);
    while(TIM_GetCounter(TIM2) < timedelay) {}
}

void delay_ms(uint16_t timedelay){
    TIM_SetCounter(TIM2, 0);
    for(int i = 0; i<1000; ++i)
			while(TIM_GetCounter(TIM2)<timedelay){}
}

void EpromWrite(uint16_t MemAddr, uint8_t SlaveAddr, uint8_t NumByte, uint8_t *pData) {
    uint8_t i;

    for (i = 0; i < NumByte; ++i) {

        I2C_GenerateSTART(I2C1, ENABLE);
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

        I2C_Send7bitAddress(I2C1, SlaveAddr << 1, I2C_Direction_Transmitter);
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

        I2C_SendData(I2C1, (MemAddr + i) >> 8);
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

        I2C_SendData(I2C1, (MemAddr + i));
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

 
        I2C_SendData(I2C1, pData[i]);
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

        I2C_GenerateSTOP(I2C1, ENABLE);

        delay_ms(10);
    }
    return;
}

void EpromRead(uint16_t MemAddr, uint8_t SlaveAddr, uint8_t NumByte, uint8_t *pData) {
    uint8_t i;


    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C1, SlaveAddr << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(I2C1, MemAddr >> 8);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_SendData(I2C1, MemAddr);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));


    I2C_Send7bitAddress(I2C1, (SlaveAddr << 1) | 1, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    for (i = 0; i < NumByte - 1; ++i) {
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
        pData[i] = I2C_ReceiveData(I2C1);
    }

    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    pData[NumByte - 1] = I2C_ReceiveData(I2C1);
    I2C_AcknowledgeConfig(I2C1, DISABLE); // G?i tín hi?u NACK

    I2C_GenerateSTOP(I2C1, ENABLE);

    I2C_AcknowledgeConfig(I2C1, ENABLE);

    return;
}


uint8_t DataSend[8] = {0x17, 0x30, 0x72, 0x08, 0x82, 0x20, 0x38, 0x42};
uint8_t Rov[8] 		 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int main() {
	RCC_Config();
	GPIO_Config();
	TIMER_Config();
	I2C_Config();
	TIM_SetCounter(TIM2,0); 
	EpromWrite(0x0024, 0x50, 8, DataSend);
	while(1){
		EpromRead(0x0024, 0x50, 8, Rov);
	}
    
}
