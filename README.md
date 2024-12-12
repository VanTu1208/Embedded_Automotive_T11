# Advance-C-Cpp-Algorithm-T11
## Bài 2: GPIO
<details><summary>Xem</summary>
  
### Ba bước cấu hình và sử dụng ngoại vi GPIO

![3 steps](https://i.imgur.com/v00DofK.png)  
**1. Cấp xung CLOCK cho ngoại vi**
```cpp
RCC_APB1PeriphClockCmd: Hàm cấp xung clock cho thanh ghi APB1

RCC_APB2PeriphClockCmd: Hàm cấp xung clock cho thanh ghi APB2

RCC_AHBPeriphClockCmd: Hàm cấp xung clock cho thanh ghi AHB   
```
**Hai tham số của hàm**  
+) Tham số 1: Bus cần cấp xung, ví dụ: RCC_APB2Peripb_GPIOA là bus của GPIOA  
+) Tham số 2: ENABLE: Cấp xung, DISABLE: Không cho phép cấp xung  

**2. Cấu hình ngoại vi**  
Các tham số GPIO được tổ chức trong struct **GPIO_InitTypeDef**, bao gồm:  
```cpp
uint16_t GPIO_Pin; //Chọn pin cần cấu hình, Ví dụ pin 5: GPIO_Pin_5

GPIOSpeed_TypeDef GPIO_Speed; // Chọn tốc độ hoạt động: 2Mhz, 10Mhz hoặc 50Mhz

GPIOMode_TypeDef GPIO_Mode; // Chọn chế độ hoạt động của Pin
```
**3. Sử dụng ngoại vi** 
Có nhiều hàm được xây dựng sẵn trong thư viện **stm32f10x_gpio.h** để giao tiếp với các GPIO
```cpp
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin); //Đọc giá trị  GPIO_Pin_y trong GPIOx được cấu hình là INPUT

uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx); //Đọc giá trị GPIOx được cấu hình là INPUT

uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin); //Đọc giá trị 1  GPIO_Pin_y trong GPIOx được cấu hình là OUTPUT

uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx); // Đọc giá trị GPIOx được cấu hình là OUTPUT

void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin); //Đặt giá trị logic 1 lên GPIO_Pin_y trong GPIOx

void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin); //Đặt giá trị logic 0 lên GPIO_Pin_y trong GPIOx

void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal); // Ghi giá trị "BitVal" vào GPIO_Pin_y trong GPIOx

void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal); // Ghi giá trị "PortVal" vào GPIOx

```
  
**Với x là tên port = (A,B,C,...) và y là số pin = (0,1,2,3,...)** 

### Ví dụ BlinkLed PC13
![2 Leds on STM32F103C8T6](https://i.imgur.com/y7LDm8Z.png)  

**Trên STM32F103C8T6 có hai led**  
+) Led báo nguồn  
+) Led được tích hợp trên chân PC13 có Anode nối VCC, Cathode nối PC13 nên khi PC13 = 0V thì Led sáng

```cpp
#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC

void delay(uint32_t time) {
    for(int i = 0; i < time ; i++);
} // Moi i chay duoc ~1us

void RCC_config(void){ // Hàm cấp xung GPIO
    //Hàm cấp xung clock cho GPIOC
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}

void GPIO_config(void){ // Hàm cấu hình GPIO
    //Cau hinh chan PC13
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; //Output Push-Pull: kéo lên 1 hoặc 0 không cần điện trở nội
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz; // Tốc độ ngoại vi 10Mhz
    
    GPIO_Init(GPIOC, &GPIO_InitStruct );
}

int main(){
    RCC_config();
    GPIO_config();
    while(1) {
        GPIO_SetBits(GPIOC, GPIO_Pin_13); // Đặt bit 1 vào PC13 (Đèn tắt)
        delay(5000000);
        GPIO_ResetBits(GPIOC, GPIO_Pin_13); // Đặt bit 0 vào PC13 (Đèn sáng)
        delay(2000000);
    }   
}

```
**Tại hàm main() gọi hai hàm được thiết lập từ trước với chức năng cấp xung cho GPIOC và cấu hình PC13 làm ngõ ra để điều khiển LED  
Trong hàm while Bật tắt led liên tục sử dụng hàm delay tương đối thay vì sử dụng Timer**

</details>


