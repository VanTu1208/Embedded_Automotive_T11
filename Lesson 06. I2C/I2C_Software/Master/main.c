#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM


#define I2C_SCL 	GPIO_Pin_6
#define I2C_SDA		GPIO_Pin_7
#define I2C_GPIO 	GPIOB


#define WRITE_SDA_0 	GPIO_ResetBits(I2C_GPIO, I2C_SDA)
#define WRITE_SDA_1 	GPIO_SetBits(I2C_GPIO, I2C_SDA)
#define WRITE_SCL_0 	GPIO_ResetBits(I2C_GPIO, I2C_SCL)
#define WRITE_SCL_1 	GPIO_SetBits(I2C_GPIO, I2C_SCL)
#define READ_SDA_VAL 	GPIO_ReadInputDataBit(I2C_GPIO, I2C_SDA)//Dung de doc ACK

typedef enum {
	NOT_OK, OK //status la bien co hai gia tri: 0:NOT_OK va 1:OK
} status;

typedef enum {
	NACK, ACK //status la bien co hai gia tri: 0:NOT_OK va 1:OK
} ACK_Bit;


void RCC_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}


void GPIO_Config(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;//SDA,SCL pull up, nen de tranh xung dot nen de mode Open-Drain
	GPIO_InitStructure.GPIO_Pin = I2C_SDA| I2C_SCL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(I2C_GPIO, &GPIO_InitStructure);
}

void TIMER_config(void){
    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV2; //Chia thanh clock nho hon de cap cho timer default: 72MHz (1s tao duoc 72 trieu dao dong) , DIV1: khong chia
    TIM_InitStruct.TIM_Prescaler = 36; //Sau bao nhieu dao dong thi se dem len 1 lan.  1s = 72M giao dong, gia tri < 65535, neu lon hon thi doi bo chia
    //VD muon 1us dem len 1 lan thi (10^-6)/(1/36M) = 36 dao dong
    TIM_InitStruct.TIM_Period  = 0xFFFF;//Dem bao nhieu lan thi reset
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up; //Set mode dem len tu 0
    TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
    
    TIM_Cmd(TIM2, ENABLE); //Cho phep timer2 hoat dong
	
}

void delay_us(uint16_t timedelay){
    TIM_SetCounter(TIM2, 0);
    while(TIM_GetCounter(TIM2)<timedelay){}
}

void delay_ms(uint16_t timedelay){
    TIM_SetCounter(TIM2, 0);
    for(int i = 0; i<1000; ++i)
			while(TIM_GetCounter(TIM2)<timedelay){}
}

void I2C_Config(void){
	//Setup SCL, SDA = 1 khi chua truyen data (pull-up)
	WRITE_SDA_1; 
	delay_us(1);
	WRITE_SCL_1;
	delay_us(1);
}

void I2C_Start(){
	//Dam bao rang SDA va SCL = 1 truoc khi truyen data
	WRITE_SDA_1;
	delay_us(1);	
	WRITE_SCL_1;  	
	delay_us(3);
	//SDA keo xuong 0 truoc SCL	
	WRITE_SDA_0;
	delay_us(3);
	WRITE_SCL_0;
	delay_us(3);
}

void I2C_Stop(){
	
	WRITE_SDA_0;//Setup SDA = 0 truoc de dam bao SCL duoc keo len 1 thi SDA van bang 0
	delay_us(3);
	
	WRITE_SCL_1; 	//SCL set to 1 before SDA.
	delay_us(3);
	WRITE_SDA_1;
	delay_us(3);
}


//Ham gui data
status I2C_Write(uint8_t u8Data){	

	status stRet;
	for(int i=0; i<8; i++){		//Write byte data.
		//xu ly bit va dich du lieu vao chan SDA (7bit address)
		if (u8Data & 0x80)// 0b10000000
		{
			WRITE_SDA_1;
		} else {
			WRITE_SDA_0;
		}
		//Tao xung clock	
		delay_us(3);
		WRITE_SCL_1;
		delay_us(5);
		WRITE_SCL_0;
		delay_us(2);
		
		u8Data <<= 1; //Dich bit
	}
	//Khi truyen xong Data thi cho` ACK, luc nay SDA mac dinh keo len muc 1
	WRITE_SDA_1;					
	delay_us(3);
	
	//Tao 1 xung clock de nhan tin hieu ACK
	WRITE_SCL_1;
	delay_us(3);
	//Neu trong xung clock nhan duoc ACK (SDA = 0) thi tra ve OK
	if (READ_SDA_VAL) {	
		stRet = NOT_OK;				
	} else {
		stRet = OK;					
	}
	//Ket thuc xung clock
	delay_us(2);
	WRITE_SCL_0;
	delay_us(5);
	
	return stRet;
}

//----------------------------------------------------------------------------
//Ham nhan data
uint8_t I2C_Read(ACK_Bit _ACK){	
	uint8_t i;						
	uint8_t u8Ret = 0x00;
	//Dam bao SDA pull_up truoc khi nhan
	WRITE_SDA_1;
	delay_us(3);	
	//Tao 8 clock va doc du lieu ghi vao u8Ret
	for (i = 0; i < 8; ++i) {
		u8Ret <<= 1;
		WRITE_SCL_1;
		delay_us(3);
		if (READ_SDA_VAL) {//Doc du lieu chan ACK
			u8Ret |= 0x01;
		}
		delay_us(2);
		WRITE_SCL_0;
		delay_us(5);
	}
	if (_ACK) {	//Neu tham so truyen vao la ACK thi truyen ACK di 
		WRITE_SDA_0;
	} else {
		WRITE_SDA_1;
	}
	delay_us(3);
	//Tao xung de truyen ACK di
	WRITE_SCL_1;
	delay_us(5);
	WRITE_SCL_0;
	delay_us(5);

	return u8Ret;
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

status EpromWrite(uint16_t MemAddr, uint8_t SlaveAddr, uint8_t NumByte, uint8_t *pData) {
	uint8_t i;
	
	for(i = 0; i< NumByte; ++i){ //Truyen so byte = Numbyte
		I2C_Start();//Tao tin hieu Start
		//Truyen khong thanh cong, tra ve NOT_OK va out
		if(I2C_Write(SlaveAddr<<1) == NOT_OK) { //SlaveAddr<<1: 7bit address dich trai 1 bit = 8bit_address va 1 bit 0 cua R/W: master truyen du lieu 
			I2C_Stop();
			return NOT_OK;
		}
		//Neu truyen di thanh cong (Nhan duoc ACK), master truyen 16bit di 2 lan, moi lan 8bit Memory address muon truyen vao slave
		if(I2C_Write((MemAddr+i)>>8) == NOT_OK) { //MemAddr >> 8 de truyen 8bit MSB(trong so cao) cua MemAddt, +i de moi lan truyen 1 byte se nhay den vi tri o nho khac
			I2C_Stop();
			return NOT_OK;
		}
		if(I2C_Write(MemAddr+i) == NOT_OK) { //Truyen 8bit trong so thap cua MemAddr
			I2C_Stop();
			return NOT_OK;
		}
		//Truyen du lieu muon ghi vao dia chi 16bit o tren
		if(I2C_Write(pData[i]) == NOT_OK) { 
			I2C_Stop();
			return NOT_OK;
		}
		I2C_Stop();
		delay_ms(10);
	}
	return OK;
}

//-----------------------------------------------------------------------------------
status EpromRead(uint16_t MemAddr, uint8_t SlaveAddr, uint8_t NumByte, uint8_t *pData) {
	uint8_t i;
	I2C_Start();//Tao tin hieu Start
	
	//Master gui dia chi Eprom, R/W = 0(Write)
	if(I2C_Write(SlaveAddr<<1) == NOT_OK) {
		I2C_Stop();
		return NOT_OK;
	}
	
	//Gui dia chi can doc
	if(I2C_Write(MemAddr>>8) == NOT_OK) { //MemAddr >> 8 de truyen 8bit MSB(trong so cao) cua MemAddt
		I2C_Stop();
		return NOT_OK;
	}
	if(I2C_Write(MemAddr) == NOT_OK) { //Truyen 8bit trong so thap cua MemAddr
		I2C_Stop();
		return NOT_OK;
	}
	I2C_Start();//Tao tin hieu Start
	//Start Again
	//Gui dia chi Eprom, R/W = 1(Read)
	if(I2C_Write((SlaveAddr<<1)|1) == NOT_OK) {
		I2C_Stop();
		return NOT_OK;
	}
	
	//Doc data tu Eprom (Doc data deu gui lai ACK tru byte cuoi gui lai NACK de ket thuc viec doc
	for(i = 0;i<NumByte;++i){
		pData[i] = I2C_Read(ACK);
	}
	pData[i] = I2C_Read(NACK);
	
	//Stop
	I2C_Stop();
	return OK;
}

uint8_t DataSend[8] = {0x17, 0x30, 0x72, 0x08, 0x82, 0x20, 0x38, 0x42};
uint8_t Rov[8] 		 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int main() {
	RCC_Config();
	GPIO_Config();
	TIMER_config();
	I2C_Config();
	TIM_SetCounter(TIM2,0); 
	EpromWrite(0x0024, 0x50, 8, DataSend);
	while(1){
		while(EpromRead(0x0024, 0x50, 8, Rov) == NOT_OK);
	}
}
