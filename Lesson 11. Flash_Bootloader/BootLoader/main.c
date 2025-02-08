#include "stm32f10x.h"                  // Device header
#include "stm32f10x_flash.h"            // Keil::Device:StdPeriph Drivers:Flash
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC

#define NewAddr 0x08008000

void Boot(){
	RCC_DeInit();// Vo hieu hoa toan bo cac bus de tranh ngoai vi hoat dong
	
	//Vo hieu hoa toan bo cac qua trinh xu ly loi tranh gay ngat trong qua trinh boot
	SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTENA_Msk| SCB_SHCSR_BUSFAULTENA_Msk| SCB_SHCSR_MEMFAULTENA_Msk);
	
	//Cai dat dia chi cua chuong trinh moi cho Main Stack Pointer de khi reset thi van chay ct moi
	__set_MSP(*(__IO uint32_t*)(NewAddr));
	
	//Cap nhat thanh ghi VTOR de tro den bang vector ngat cua chuong trinh chinh
	SCB->VTOR = NewAddr;
	
	//Lay dia chi Reset Handler cua chuong trinh chinh
	uint32_t JumpAddress = *(__IO uint32_t*)(NewAddr+4);
	
	//Tao con tro ham de tro vao dia chi cua ham reset handler
	void (*reset_handler)(void) = (void (*)(void)) JumpAddress;
	
	//Goi ham reset Handler
	reset_handler();
}

int main(){
	Boot();
}