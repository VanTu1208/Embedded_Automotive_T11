#include "stm32f10x.h"                  // Device header
#include "stm32f10x_flash.h"            // Keil::Device:StdPeriph Drivers:Flash
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM

#define Page2Addr 0x08000800
#define Page3Addr 0x08001000
void Flash_Erase(uint32_t pageAddr){
	FLASH_Unlock();//Un lock tat ca vung nho trong Flash
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY));//FLASH_FLAG_BSY = 1, khi Flash dang ban ghi/doc
	FLASH_ErasePage(pageAddr);//Xoa Page theo dia thi duoc truyen vao
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);//Cho den khi xoa hoan tat
	FLASH_Lock();//Lock toan b Flash
}

void Flash_Erase1(uint32_t pageAddr)
{
	FLASH_Unlock();
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY));
	
	FLASH->CR |= FLASH_CR_PER; //Bat xoa Page
	FLASH->AR = pageAddr; // Them dia chi page vao thanh ghi AR
	FLASH->CR |= FLASH_CR_STRT; //Bat dau xoa page
	
	while(FLASH->SR & FLASH_SR_BSY); //Doi den xoa hoan tat
	
	FLASH->CR &=(~FLASH_CR_PER); // Tat xoa page
	
	FLASH_Lock(); //Lock flash
}


void Flash_WriteHalfWord(uint32_t memAddr, uint16_t value){
	FLASH_Unlock();
	
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
	
	FLASH_ProgramHalfWord(memAddr, value);
	
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
	
	FLASH_Lock();
}

void Flash_WriteNumByte(uint32_t address, uint8_t *data, int num)
{
	FLASH_Unlock();
	
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
	
	uint16_t *ptr = (uint16_t*)data;
	
	for(int i=0; i<((num+1)/2); i++){
		FLASH_ProgramHalfWord(address+2*i, *ptr);
		while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
		ptr++;
	}
	
	FLASH_Lock();
}



int main(){
	Flash_Erase(0x08008000);
}
