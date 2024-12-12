#include "stm32f10x.h"                  // Device header
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"      

void RCC_config(void){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}

void GPIO_config(void){
    //Cau hinh chan PC13
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13; // Neu muon them nhieu chan thi su dung phep OR
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_Init(GPIOC, &GPIO_InitStruct );    
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
int main(){
    RCC_config();
    TIMER_config();
    GPIO_config();
    while(1){
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
        delay_ms(1000);
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
        delay_ms(1000);
  }   

}

