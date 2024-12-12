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

> **Ví dụ BlinkLed PC13 **


</details>


