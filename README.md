# Embedded In Automotive - T11
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

## Bài 3: Interrupt - Timer
<details><summary>Xem</summary>
  
### A. Ngắt (Interrupt)
#### 1. Định nghĩa  
- Ngắt là 1 sự kiện khẩn cấp xảy ra trong hay ngoài vi điều khiển. Nó yêu cầu MCU phải dừng chương trình chính (main) và thực thi chương trình ngắt ISR (Interrupt Service Routine - trình phục vụ ngắt).

#### 2. Các loại ngắt thông dụng  
- Mỗi ngắt sẽ có một trình phục vụ ngắt riêng
- Địa chỉ mỗi trình phục vụ ngắt trong bộ nhớ gọi là **vector ngắt**. Mỗi loại vector ngắt có địa chỉ khác nhau.
![Các ngắt khác nhau](https://i.imgur.com/VdLym2V.png)

- Có bốn loại ngắt là Reset, ngắt ngoài, ngắt Timer và ngắt truyền  thông
- Ngắt ngoài có cờ ngắt là IE0, cờ ngắt timer là TF1 và hai loại ngắt này có thể lập trình được
- Ngắt Reset xảy ra khi nhấn nút Reset trên Board

**Thanh ghi PC (Program Counter): chỉ đến lệnh tiếp theo cần thực thi trong chương trình**
![Thanh ghi PC](https://i.imgur.com/eZFuvK5.png)  
Khi chạy chương trình chính, nếu có ngắt xảy ra thì thanh ghi PC sẽ trỏ tới vector ngắt của chương trình ngắt, sau khi thực hiện xong lệnh đang chạy thì sẽ chuyển qua thực thi hàm ngắt, sau đó tiếp tục quay lại thực thi chương trình chính.

#### 3. Ngắt ngoài
Phải cấu hình ngắt cho ngõ vào ngắt, xảy ra khi thay đổi giá trị điện áp trên chân ngắt. Có 4 dạng:  
- **LOW**: kích hoạt ngắt liên tục khi chân ở mức thấp.
- **HIGH**: Kích hoạt liên tục khi chân ở mức cao.
- **RISING**: Kích hoạt khi trạng thái trên chân chuyển từ thấp lên cao.
- **FALLING**: Kích hoạt khi trạng thái trên chân chuyển từ cao xuống thấp.

![Các loại ngắt](https://i.imgur.com/tysVLWl.png)

#### 4. Ngắt Tỉmer

- Xảy ra khi giá trị trong thanh ghi đếm của timer bị tràn (với giá trị được đặt trước nếu không sẽ mặc định tràn khi đếm hết thanh ghi). Sau mỗi lần tràn, cần phải reset giá trị thanh ghi để có thể tạo ngắt tiếp theo.

![Timer](https://i.imgur.com/KYMJFGK.png)

#### 5. Ngắt truyền thông

- Xảy ra khi có sự truyền nhận giữa MCU - MCU hay MCU với ngoại vi. Được sử dụng cho nhiều phương thức như UART, SPI,... Với mục đích đồng bộ hóa truyền nhận

- **Khi MCU1 truyền sẽ tạo ra ngắt để MCU2 lập tức nhận để tránh mất gói tin** 

#### 6. Độ ưu tiên Ngắt
- Khi nhiều ngắt xảy ra, ngắt nào được thực thi trước sẽ dựa vào độ ưu tiên ngắt (Số ưu tiên càng thấp thì có độ ưu tiên càng cao)  
- **Stack Pointer** là thanh ghi trỏ tới đỉnh của vùng stack chứa các địa chỉ trả về của các hàm
**Ví dụ:** ISR2 có độ ưu tiên cao hơn ISR1
- Thực thi hàm main() - xảy ra ngắt 1: Lưu PC tiếp theo (main) vào Stack Pointer và chuyển PC đến ISR1, sau đó thực thi ISR1
- Nếu đang thực thi ISR1 mà ISR2 xảy ra thì lưu PC của ISR1 vào Stack Pointer và Chuyển PC đến ISR2 để thực thi.
- Thực thi xong ISR2 sẽ lấy PC được lưu trong Stack Pointer theo kiểu **Last in - First Out** và sẽ lấy PC của ISR1 thực thi
- Thực thi xong ISR1 sẽ lấy PC của hàm main() và tiếp tục chạy chương trình chính.

### B. Timer
- Timer là 1 mạch digital logic có vai trò đếm mỗi chu kỳ clock (đếm lên hoặc đếm xuống).
- Timer còn có thể hoạt động ở chế độ nhận xung clock từ các tín hiệu ngoài. Ngoài ra còn các chế độ khác như PWM, định thời …vv.
- STM32F103 có 7 timer

**Ví dụ: Bật tắt led PC13**
- Để sử dụng Hardware Timer, thêm thư viện trên STM32F103
```cpp
#include "stm32f10x_tim.h"
```
- Tạo hàm với chức năng cấp xung cho GPIOC và Timer 2: Với GPIOC được kết nối với Bus APB2 còn Timer2 được nối với Bus APB1

```cpp
void RCC_config(void){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}
```
- Cấu hình PC13 đã được học từ bài trước 
```cpp
void GPIO_config(void){
    //Cau hinh chan PC13
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13; // Neu muon them nhieu chan thi su dung phep OR
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_Init(GPIOC, &GPIO_InitStruct );    
}
```
- Tạo hàm cấu hình Timer 2
```cpp
void TIMER_config(void){
    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV2; 
    TIM_InitStruct.TIM_Prescaler = 36000; 
    TIM_InitStruct.TIM_Period  = 0xFFFF;
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
    
    TIM_Cmd(TIM2, ENABLE);
    
}
```
Tần số mặc định của STM32F103C8T6 là 72Mhz ta chia tần số cho 2 ```TIM_CKD_DIV2``` còn 36Mhz
```cpp
 TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV2; 
 ```

Thuộc tính ```TIM_Prescaler```: Sau bao nhiêu dao động thì đếm lên một lần. Ta có tần số 36Mhz tức 1s = 36M dao động. Nên muốn 1ms đếm một lần tức 1ms = 36.000 dao động
```cpp
 TIM_InitStruct.TIM_Prescaler = 36000;
```
Thuộc tính ```TIM_Period```: Đếm đến bao nhiêu thì Reset. Trong bài này ta không sử dụng đến nên để tối đa 0xFFFF
```cpp
TIM_InitStruct.TIM_Period  = 0xFFFF;
```
Thuộc tính ```TIM_CounterMode```: Quy định đếm lên hay xuống
```cpp
 TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
```
Sau khi đã cấu hình, ta gán những thuộc tính này vào Timer cần sử dụng và cho phép hoạt động
```cpp
TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
TIM_Cmd(TIM2, ENABLE);
```
- Tạo hàm delay theo milisecond
```cpp
void delay_ms(uint16_t timedelay){
    TIM_SetCounter(TIM2, 0);
    while(TIM_GetCounter(TIM2)<timedelay){}
}
```
Trước khi đếm phải reset thanh ghi đếm về 0 tránh sai xót bằng lệnh
```cpp
TIM_SetCounter(TIM2, 0);
```
Sau đó đọc giá trị thanh ghi đếm bằng lệnh:
```cpp
TIM_GetCounter(TIM2)
```
Nếu đếm đến giá trị delay cho trước thì bỏ qua câu lệnh while và thực thi lệnh tiếp theo

**Ví dụ:** delay_ms(1000) thì sẽ đếm lên 1000 lần với mỗi lần đếm là 1ms được cấu hình trong Timer, nên sẽ delay được 1s 

- Gọi hàm thực thi trong main() để cấu hình và bật tắt led sau 1 giây.
```cpp
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
```
</details>

## Bài 4: Communication Protocols
<details><summary>Xem</summary>  

### 1. Truyền nhận dữ liệu
Truyền nhận dữ liệu trên MCU là quá trình trao đổi các tín hiệu điện áp biểu diễn thành các bit trên các chân của MCU.

![Truyền data](https://i.imgur.com/goMtf0I.png)

- 3.3V hoặc 5V được biểu diễn bởi bit 1
- 0V được biểu diễn bởi bit 0

Ví dụ MCU1 truyền cho MCU2 ký tự "h" thì MCU1 sẽ chuyển đổi "h" theo mã ASCII có mã nhị phân 0110 1000 và truyền từng bit đến MCU2.

Có hai kiểu truyền dữ liệu: Truyền **nối tiếp** và **song song**
- Truyền song song: Sử dụng nhiều dây, mỗi dây một bit
- Truyền nối tiếp: Sử dụng một dây duy nhất để truyền liên tiếp nhiều bit
**Vấn đề**: Nếu truyền nhiều bit cùng giá trị thì không thể phân biệt các bit. Vì thế, ta cần đến những **chuẩn giao tiếp** (Communication Protocols).

### 2. SPI (Serial Peripheral Interface)
#### Tính chất:
-  Truyền nối tiếp, đồng bộ (Có chân SCK)
- Truyền song công (Có hai dây dữ liệu MOSI - Master Output Slave Input và MISO - Master Input Slave Output)
- Hoạt động ở cơ chế Master - Slave, có chân CS - Chip Select để chọn Slave muốn giao tiếp
![SPI](https://i.imgur.com/wwNFvru.png)

#### Chức năng các chân
- SCK: Chân được điều khiển bởi Master tạo xung Clock gửi cho Slave để đồng bộ truyền nhận dữ liệu.
- CS (Chip Select) hoặc SS (Slave Select): Chân điều khiển bởi Master chọn Slave để giao tiếp. Nếu có nhiều Slave thì có nhiều chân CS.
- MOSI (Master Output Slave Input): Chân truyền dữ liệu từ Master đến Slave.
- MISO (Master Inpur Slave Output): Điều khiển bởi Slave gửi dữ liệu đến Master. 

#### Quá trình truyền dữ liệu
- Trước khi truyền dữ liệu, Master sẽ kéo chân CS của Slave cần giao tiếp xuống 0 để báo hiệu muốn truyền nhận.
- Master sẽ tạo xung Clock để truyền đến Slave kèm theo một bit dữ liệu và đồng thời Slave cũng gửi lại cho Master một bit dữ liệu khác.
- Các thanh ghi sẽ dịch bit và cập nhật giá trị truyền nhận
- Lặp lại quá trình đến khi truyền xong dữ liệu
![Truyền data SPI](https://i.imgur.com/UcLfdyk.png)

#### 4 chế độ hoạt động
Dựa trên Clock Polarity (CPOL) và Clock Phase (CPHA) 
![4 chế độ](https://i.imgur.com/qTfMUcD.png)
- CPOL = 0: Khi không có dữ liệu SCK = 0
- CPOL = 1: Khi không có dữ liệu SCK = 1
- CPHA = 0: Truyền dữ liệu trước khi tạo xung
- CPHA = 1: Truyền dữ liệu sau khi tạo xung

Kết hợp các trường hợp trên ta được bảng sau:
![4 trường hợp](https://i.imgur.com/dy43EC9.png)

### 3. I2C (Inter-Integrated Circuit)

![I2C](https://i.imgur.com/OwtDaI7.png)
#### Tính chất:

- Truyền dữ liệu nối tiếp, đồng bộ
- Hoạt động ở chế độ bán song công
- Hoạt động ở cơ chế Master Slave, sử dụng địa chỉ riêng để phân biệt Slave
- Sử dụng hai dây SDA và SCL để giao tiếp
- Cần có điện trở nối hai dây nối lên nguồn để tạo mức điện áp 3.3/5V khi chưa có dữ liệu vì hai dây thả nổi nên MCU không hiểu được mức điện áp.
- 1 Master tối đa kết nối được 127 Slave
  
#### Chức năng các dây:
- SDA: Dây truyền dữ liệu truyền nhận.
- SCL: Dây đồng bộ truyền nhận dữ liệu do Master điều khiển tạo xung Clock.

#### I2C Data Frame:
![Data Frame](https://i.imgur.com/mZ82odf.png)

- Trước khi truyền/nhận dữ liệu Master sẽ gửi tín hiệu Start Condition: SDA được kéo xuống mức 0 trước sau đó đến SCL xuống mức 0.
- Sau tín hiệu Start, Master sẽ gửi 8 bit gồm 7 địa chỉ và 1 bit Read/Write kèm theo xung Clock cho tất cả Slave nhận. Slave có trùng địa chỉ với địa chỉ được yêu cầu từ Master sẽ đọc bit Read/Write để xác nhận chức năng truyền hoặc nhận dữ liệu từ Master 
    - Nếu R/W = 1: Master đọc dữ liệu từ Slave (Slave chế độ truyền)
    - Nếu R/W = 0: Master ghi dữ liệu đến Slave (Slave chế độ đọc)
- Sau đó Slave có trùng địa chỉ sẽ gửi lại Master một tín hiệu ACK = 0 (SDA = 0), sau đó Master tiếp tục truyền dữ liệu 
    - Nếu không có Slave nào trùng với địa chỉ được yêu cầu từ Master thì không có gói phản hồi ACK (SDA = 1) thì Master dừng việc truyền dữ liệu
- Sau đó, không truyền gì cả, tức SDA = 1
- Trước khi tiếp tục truyền dữ liệu, kéo SDA = 0
- Truyền 8 bit dữ liệu đến Slave. Slave nhận được sẽ tiếp tục truyền lại gói tin phản hồi ACK = 0.
- Master nhận được ACK sẽ kết thúc gói tin bằng cách kéo chân SCL lên 1 trước sau đó kéo chân SDA lên 1

### 4. UART (Universal Asynchronous Receiver-Transmitter)
![UART](https://i.imgur.com/ZrhsvAg.png)
#### Tính chất
- Được gọi là **giao thức truyền thông phần cứng**: Chỉ giao tiếp 1 - 1 giữa hai thiết bị cố định
- Truyền dữ liệu nối tiếp không đồng bộ
- Hoạt động ở chế độ song công

#### Chức năng các chân:
- Tx: Chân truyền dữ liệu điều
- Rx: Chân nhận dữ liệu về
- Vì thế hai các chân được mắc chéo với nhau Tx1 - Rx2, Tx2 - Rx1

#### Không có chân Clock, làm sao để đồng bộ?
- Hai MCU sẽ căn khoảng thời gian giữa hai lần gửi dữ liệu để đồng bộ truyền nhận bằng cách tạo Tỉmer
- Đơn vị thống nhất thời gian giữa hai MCU gọi là **BaudRate** (Số bit truyền trong 1 giây)
- Ví dụ: BaudRate = 9600
    - Tức truyền được 9600bits/s = 9600bits/1000ms
    - Nghĩa là 1 bit truyền đi mất khoảng 0.10467ms
    - Hai MCU sẽ tạo Timer để gửi nhận cùng lúc dữ liệu theo thời gian trên

#### UART Data Frame:
![UART Data Frame](https://i.imgur.com/lv3DRsF.png)
- Khi không có dữ liệu Tx = Rx = 1
- Trước khi truyền dữ liệu: Kéo chân Tx của MCU truyền từ bit 1 xuống bit 0. Sau đó delay một khoảng BaudRate. Bên MCU nhận sẽ đọc chân RX = 0 để kiểm tra có dữ liệu truyền tới hay không.
- Sau đó MCU truyền sẽ dịch dữ liệu và gửi 5 - 9 bits qua chân TX. Bên MCU nhận sẽ đọc liên tiếp 5 - 8 bits dữ liệu sau những khoảng thời gian delay được tạo từ Timer theo BaudRate.
- Sau đó MCU truyền gửi bit kiểm tra lỗi Polarity (có hoặc không), có hai loại:
    - Parity chẵn: Thêm bit 1 hoặc 0 để tổng bit 1 trong data là chẵn.
    - Parity lẻ: Thêm bit 1 hoặc 0 để tổng bit 1 trong data là lẻ.
    - Nhược điểm: Nếu lỗi với số chẵn bit thì không thể kiểm tra được lỗi.
- Bên nhận kiểm tra lỗi bằng cách tính tổng số bit 1 trong data và xem xét loại Parity để xác định đúng Serial.
- Nếu Data sai thì không nhận dữ liệu.
- Sau cùng MCU gửi sẽ gửi tín hiệu Stop bằng cách đưa chân TX từ 0 lên 1.



</details>


## Bài 5: SPI Software & SPI Hardware

<details><summary>Xem</summary>  

### SPI Software

Dùng các chân GPIO và delay để mô phỏng hoạt động của SPI

Có hai bước để thực thi:
- Xác định các chân (có 4 chân)
- Cấu hình GPIO 

#### 1. Xác định các chân SPI 
```cpp
#define SPI_SCK_Pin GPIO_Pin_0
#define SPI_MISO_Pin GPIO_Pin_1
#define SPI_MOSI_Pin GPIO_Pin_2
#define SPI_CS_Pin GPIO_Pin_3
#define SPI_GPIO GPIOA
#define SPI_RCC RCC_APB2Periph_GPIOA
```
Sử dụng các chân trong GPIOA và được cấp xung bởi APB2. Gán các chân GPIO bằng tên các chân dữ liệu SPI.

```cpp
void RCC_config(void){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		RCC_APB2PeriphClockCmd(SPI_RCC, ENABLE);
}
```
Sau đó cấp xung cho SPI_RCC tức GPIOA và cấp cho Timer để tạo hàm delay phục vụ truyền nhận dữ liệu

#### 2. Cấu hình các chân SPI
```cpp
void GPIO_Config(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = SPI_SCK_Pin| SPI_MOSI_Pin| SPI_CS_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
}
```
Đây là cấu hình cho Master nên sẽ có ba chân ngõ ra là SCK, MOSI và CS. Còn lại chân ngõ vào do Slave điều khiển là MISO.

```cpp
void Clock(){
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_SET);
	delay_ms(4);
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_RESET);
	delay_ms(4);
}
```
Mô phỏng xung Clock bằng cách đặt chân SCK lên một khoảng thời gian 4ms sau đó đặt lại mức thấp

```cpp
void SPISetup(void){
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin,  Bit_RESET);
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin,   Bit_SET);
	GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_RESET);//Muc gi cung duoc
}
```
Khởi tạo giá trị đầu cho các chân SPI, SCK chưa có xung nên sẽ là bit 0 (CPOL = 0) và CS ở mức 1 (chưa chọn Slave). Chân MOSI mức gì cũng được và chân MISO master không thể điều khiển.

#### 3. Hàm truyền dữ liệu ở Master
```cpp
void SPI_Master_Transmit(uint8_t u8Data){
	uint8_t u8Mask = 0x80;					
	uint8_t tempData; 
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin, Bit_RESET); 
	delay_ms(1);
	for(int i=0; i<8; i++){
		tempData = u8Data & u8Mask; 
		if(tempData){
			GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_SET);
			delay_ms(1);
		} else{
			GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_RESET);
			delay_ms(1);
		}
		u8Data=u8Data<<1; 
		Clock();
	}
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin, Bit_SET);
	delay_ms(1);
}
```
- Tạo một mặt nạ có giá trị 0b10000000 và tạo một biến đệm để xử lý bit
- Đầu tiên sẽ phải kéo chân CS của Slave muốn giao tiếp xuống 0 và đợi 1ms để đảm Slave nhận được tin hiệu.
- Sau đó gửi 8 bit bằng cách dịch từng bit của ```u8Data``` và phép and với ```u8Mask``` để tìm ra bit đang truyền là bit 0 hay 1 và gán vào ```tempData```
- Cuối cùng là kiểm tra giá trị của ```tempData``` để đặt chân MOSI là cao hay thấp, tương ứng với bit 1 và 0.

```cpp
uint8_t DataTrans[] = {1,7,12,17,89};//Du lieu duoc truyen di
int main(){
    RCC_config();
    TIMER_config();
    GPIO_Config();
    SPISetup();
    while(1){	
			for(int i=0; i<5; i++){
				SPI_Master_Transmit(DataTrans[i]);
				delay_ms(1000);
			}
		}
}
```
Tại hàm main() sẽ gọi lại các hàm cấu hình GPIO và Timer. Sau đó tạo một hàm ```while(1)``` để gửi tuần tự các giá trị của mảng ```DataTrans```

**Tương tự với Master, Slave cũng sẽ cấu hình những thông số trên, chỉ khác là Slave sẽ có ba chân ngõ vào CS, MOSI, SCK và một chân ngõ ra MISO**
```cpp
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
```
4. Hàm nhận dữ liệu ở Slave
```cpp
uint8_t SPI_Slave_Receive(void){
	uint8_t dataReceive =0x00;   
	uint8_t temp = 0x00, i=0;
	while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin));
	while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));
	for(i=0; i<8;i++){ 
		if(GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin)){
			while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin))
				temp = GPIO_ReadInputDataBit(SPI_GPIO, SPI_MOSI_Pin);
			
			dataReceive=dataReceive<<1;
			dataReceive=dataReceive|temp;
    }
		while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));
	}
	return dataReceive;
}
```
Giá trị nhận được ở Slave cũng là 8 bit.
- Đầu tiên tạo một biến để nhận dữ liệu ```dataReceive``` và một biến đệm ```temp```.
- Chờ cho đến khi chân CS được kéo xuống 0
- Chờ đến khi có xung Clock (Có dữ liệu được truyền).
- Dữ liệu nhận được sẽ gán vào biến ```temp``` và dịch vào ```dataReceive```

```cpp
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
```
Tại hàm main sẽ liên tục kiểm tra chân CS và nhận 5 bit dữ liệu từ Master.

### SPI Hardware

Sử dụng các chân SPI đã được tích hợp sẵn trên phần cứng để truyền nhận dữ liệu
Ba bước thực hiện:
- Xác định các chân GPIO của SPI: Bài này ta sử dụng SPI1
    - PA4: CS
    - PA5: SCK
    - PA6: MISO
    - PA7: MOSI
- Cấu hình GPIO cho SPI
- Cấu hình SPI

#### 1. Xác định chân SPI
```cpp
#define SPI1_NSS 	GPIO_Pin_4
#define SPI1_SCK	GPIO_Pin_5
#define SPI1_MISO   GPIO_Pin_6
#define SPI1_MOSI   GPIO_Pin_7
#define SPI1_GPIO   GPIOA

void RCC_config(void){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA, ENABLE);
}
```
Ta sẽ sử dụng SPI với thứ tự chân giống như lý thuyết. Thêm vào đó ta sẽ cấp trực tiếp đến ngoại vi SPI1 thông qua thanh ghi ABP2.

#### 2. Cấu hình chân GPIO cho SPI
```cpp
void GPIO_Config(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = SPI1_NSS| SPI1_SCK| SPI1_MISO| SPI1_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);

}
```
Cấu hình các chân SPI theo kiểu AF_PP: Các chân này sẽ hoạt động với một chức năng thay thế như I2C, SPI, UART...

#### 3. Cấu hình SPI
```cpp
void SPI_config(void){
	SPI_InitTypeDef SPI_InitStruct;
	
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master; 
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; 
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low; 
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;/
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_LSB; 
	SPI_InitStruct.SPI_CRCPolynomial = 7 ;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft; 

	SPI_Init(SPI1, &SPI_InitStruct);
	SPI_Cmd(SPI1,ENABLE);
		
}
```
Đầu tiên, giống với Timer khi sử dụng phải tạo một đối tượng Struct để cấu hình các thông số ```SPI_InitTypeDef SPI_InitStruct;```
- SPI Mode: Xác định thiết bị đang cấu hình là Master hay Slave
- SPI_Direction: Cấu hình kiểu truyền là song công, đơn công hay bán song công
- SPI_BaudRatePrescaler: Bộ chia tần số cho SPI, mặc định 72Mhz
- SPI_CPOL và SPI_CPHA đã được giải thích từ bài 3 và có chức năng cấu hình chế độ truyền của SPI
- SPI_DataSize: Quy định số bit truyền mỗi lần 8 bit hay 16 bit
- SPI_FirstBit: Quy định truyền bit trong số cao trước(LSB) hay bit thấp trước(MSB)
- SPI_CRCPolynomial: Cấu hình CheckSum. Nếu 8 bit thì đặt 7, nếu 16 bit thì đặt là 15
- SPI_NSS: Cấu hình chân CS được quản lý bởi Software(1 biến) hay Hardware(1 pin).
Sau đó gọi hàm Init để cấu hình SPI1 và hàm Cmd để cho phép SPI1 hoạt động

Hai hàm truyền nhận dữ liệu 8/16bits:
- Hàm ```SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data)```, tùy vào cấu hình datasize là 8 hay 16 bit sẽ truyền đi 8 hoặc 16 bit dữ liệu. Hàm nhận 2 tham số là bộ SPI sử dụng và data cần truyền.
- Hàm ```SPI_I2S_ReceiveData(SPI_TypeDef* SPIx)``` trả về giá trị đọc được trên SPIx. Hàm trả về 8 hoặc 16 bit data.

Hàm đọc trạng thái cờ: ```SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG)``` trả về giá trị 1 cờ trong thanh ghi của SPI. Các cờ thường được dùng:
- ```SPI_I2S_FLAG_TXE```: Cờ báo truyền, cờ này sẽ set lên 1 khi truyền xong data trong buffer.
- ```SPI_I2S_FLAG_RXNE```: Cờ báo nhận, cờ này set lên 1 khi nhận xong data.
- ```SPI_I2S_FLAG_BSY```: Cờ báo bận,set lên 1 khi SPI đang bận truyền nhận.

#### 3. Hàm gửi 1 byte đến Slave
```cpp
uint8_t SPI_Send1Byte(uint8_t data){
	uint8_t received_data;
	GPIO_WriteBit(SPI1_GPIO, SPI1_NSS, Bit_RESET); 
 
	SPI_I2S_SendData(SPI1, data);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==0); 
	
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
   received_data = SPI_I2S_ReceiveData(SPI1);
	
	GPIO_WriteBit(SPI1_GPIO, SPI1_NSS, Bit_SET); 
	return received_data; 
}
```
- Đầu tiên trước khi truyền phải kéo chân CS của Slave xuống mức thấp, sau đó gọi hàm ```SPI_I2S_SendData(SPI1, data);```, có hai tham số là bộ SPI và dữ liệu truyền đi để truyền đi 8 bit ```data```
- Sau đó chờ đến khi cờ TXE được kéo lên 1 (truyền xong).
- Theo lý thuyết, Slave cũng sẽ gửi lại data cho Master nên ta sẽ chờ cờ RXNE được kéo lên 1 (nhận xong) và đọc dữ liệu được nhận trong thanh ghi DR của SPI1 bằng hàm ```SPI_I2S_ReceiveData(SPI1);```
- Sau khi đã hoàn thành, đặt lại chân CS lên 1 để bỏ chọn Slave.

**Đối với Slave các thông số cấu hình giống Master nhưng khác ở** ```SPI_Mode = SPI_Mode_Slave```
#### 4. Hàm nhận dữ liệu từ Master
```cpp
uint8_t SPI_Receive1Byte(uint8_t data_to_send_back){
    uint8_t temp;
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==1);//Co bao nhan SPI_I2S_FLAG_BSY = 1 khi SPI dang ban, Cho` den khi SPI ranh?
    temp = (uint8_t)SPI_I2S_ReceiveData(SPI1); // Tra ve gia tri doc duoc tren SPI1
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==0); //cho` den khi nhan xong data SPI_I2S_FLAG_RXNE = 1
    
		SPI_I2S_SendData(SPI1, data_to_send_back); // G?i d? li?u tr? l?i
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	return temp;
}
```
```cpp
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
```
- Đọc chân NSS và chờ đến khi chân được kéo xuống 0
- Sau đó, chờ đến khi SPI1 rảnh bằng cách đọc cờ SPI_I2S_FLAG_BSY. Bằng 0 thì rảnh
- Tiếp theo đó, đọc dữ liệu nhận được từ Master thông qua hàm ```SPI_I2S_ReceiveData(SPI1);```
- Đọc cờ RXNE cho đến khi nhận xong. Sau đó gửi lại dữ liệu đến Master bằng hàm ```SPI_I2S_SendData(SPI1, data_to_send_back);```
- Sau đó chờ đến khi gửi xong TXE == 1 thì kết thúc hàm.

</details>


## Bài 6: I2C Software & I2C Hardware

<details><summary>Xem</summary>  

### I2C Software

Hai bước thực hiện
- Xác định các chân I2C
- Cấu hình GPIO

#### 1. Xác định các chân I2C
```cpp
#define I2C_SCL 	GPIO_Pin_6
#define I2C_SDA		GPIO_Pin_7
#define I2C_GPIO 	GPIOB

void RCC_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

```
Xác định chân SCL là PB6 và SDA là PB7. Sau đó cấp xưng cho GPIOB và Timer2
#### 2. Cấu hình GPIO
```cpp
void GPIO_Config(){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = I2C_SDA| I2C_SCL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(I2C_GPIO, &GPIO_InitStructure);
}
```
Cấu hình SDA và SCL là Output Open-Drain: Có khả năng kéo chân xuống mức 0 và dùng điện trở để kéo lên 1.
Gọi hàm Init để gán thông số vào các chân I2C của GPIOB

#### 3. Cấu hình mô phỏng truyền nhận
```cpp
#define WRITE_SDA_0 	GPIO_ResetBits(I2C_GPIO, I2C_SDA)
#define WRITE_SDA_1 	GPIO_SetBits(I2C_GPIO, I2C_SDA)
#define WRITE_SCL_0 	GPIO_ResetBits(I2C_GPIO, I2C_SCL)
#define WRITE_SCL_1 	GPIO_SetBits(I2C_GPIO, I2C_SCL)
#define READ_SDA_VAL 	GPIO_ReadInputDataBit(I2C_GPIO, I2C_SDA)
```
Tạo các Macro để sử dụng thuận tiện hơn trong việc ghi các chân SDA, SCL và đọc giá trị tại chân SDA.


```cpp
typedef enum {
	NOT_OK, OK //status la bien co hai gia tri: 0:NOT_OK va 1:OK
} status;

typedef enum {
	NACK, ACK //status la bien co hai gia tri: 0:NOT_OK va 1:OK
} ACK_Bit;
```
Tạo hai Enum chứa giá trị được cấu hình của status (truyền thành công?) và ACK_bit(Giá trị của ACK)
```cpp
void I2C_Config(void){
	WRITE_SDA_1; 
	delay_us(1);
	WRITE_SCL_1;
	delay_us(1);
}
```
Cấu hình cho các chân SPI khi chưa truyền dữ liệu thì SDA và SCL đều kéo lên mức 1
```cpp
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
```
Hàm Start: Gán lại hai chân lên 1 trước khi truyền. Sau đó bắt đầu tín hiệu Start bằng cách kéo SDA xuống 0 trước sau đó kéo SCL xuống 0.

```cpp
void I2C_Stop(){
	
	WRITE_SDA_0;
	delay_us(3);
	WRITE_SCL_1; 	//SCL set to 1 before SDA.
	delay_us(3);
	WRITE_SDA_1;
	delay_us(3);
}
``` 
Tương tự hàm Stop sẽ đảm bảo SDA = 0 trước, kéo chân SCL lên 1 trước sau đó kéo SDA lên 1.

```cpp
status I2C_Write(uint8_t u8Data){	

	status stRet;
	for(int i=0; i<8; i++){	
		if (u8Data & 0x80)// 0b10000000
		{
			WRITE_SDA_1;
		} else {
			WRITE_SDA_0;
		}	
		delay_us(3);
		WRITE_SCL_1;
		delay_us(5);
		WRITE_SCL_0;
		delay_us(2);
		
		u8Data <<= 1;
	}
	WRITE_SDA_1;					
	delay_us(3);

	WRITE_SCL_1;
	delay_us(3);

	if (READ_SDA_VAL) {	
		stRet = NOT_OK;				
	} else {
		stRet = OK;					
	}

	delay_us(2);
	WRITE_SCL_0;
	delay_us(5);
	
	return stRet;
}
```
Hàm gửi dữ liệu đến Slave của Master
- Đầu tiên, tạo mặt nạ để and với dữ liệu để tìm ra bit truyền là 1 hay 0
- Sau đó truyền dữ liệu qua SDA với giá trị vừa tìm được kèm theo một xung Clock được tạo bằng cách kéo và hạ chân SCL trong một khoảng delay(5us).
- Dịch một bit và tiếp tục and với mặt nạ để tiếp tục truyền.
- Sau khi truyền 8 bit dữ liệu Master sẽ chờ để nhận ACK. Mặc định SDA được kéo lên 1.
- Tạo xung để nhận ACK từ Slave. Nếu nhận SDA = 0 thì nhận được ACK, nếu SDA = 1 thì không nhận được ACK.
- Hàm trả về trạng thái của ACK nhận được.

```cpp
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
```
Hàm đọc dữ liệu từ Slave
- Tham số truyền vào là ACK_bit để xác nhận muốn đọc tiếp hay dừng lại
- Khởi tạo SDA = 1
- Tạo xung Clock, đồng thời đọc dữ liệu ở chân SDA và dịch vào dữ liệu 8 bit nhận được
- Sau đó kiểm tra tham số ```_ACK``` để xác định có truyền lại ACK hay không kèm theo xung Clock

**Dựa vào khung truyền của từng Slave mà sẽ có những cấu trúc truyền khác nhau**

### I2C Hardware

Các bước thực hiện
- Xác định các chân GPIO của I2C
- Cấu hình GPIO
- Cấu hình I2C

#### 1. Xác định các chân GPIO
```cpp
#define I2C_SCL 	GPIO_Pin_6
#define I2C_SDA		GPIO_Pin_7
#define I2C_GPIO 	GPIOB
#define DS1307_ADDRESS 0x50

void RCC_Config(void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
}
```
Trên STM32F103C8T6 có hai bộ I2C1 và I2C2. Với SPI1 sẽ sử dụng hai chân PB6 và PB7. Và ta sẽ cấp xung cho Timer và I2C1, đồng thời cấp cho GPIOB để GPIO hoạt động.

#### 2. Cấu hình GPIO
```cpp
void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
```
Tương tự với SPI, các GPIO cũng sử dụng mode AF để sử dụng cho chế độ thay thế. Nhưng với I2C sẽ sử dụng Open-Drain
#### 3. Cấu hình I2C
```cpp
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
```
Tương tự các ngoại vi khác, các tham số I2C được cấu hình trong Struct I2C_InitTypeDef:
- I2C_Mode: Cấu hình chế độ hoạt động cho I2C:
	- I2C_Mode_I2C: Chế độ I2C FM(Fast Mode);
	- I2C_Mode_SMBusDevice&I2C_Mode_SMBusHost: Chế độ SM(Slow Mode).
- I2C_ClockSpeed: Cấu hình clock cho I2C, tối đa 100khz với SM và 400khz ở FM.
- I2C_DutyCycle: Cấu hình chu kì nhiệm vụ của xung:
	- I2C_DutyCycle_2: Thời gian xung thấp/xung cao = 2;
	- I2C_DutyCycle_16_9: Thời gian xung thấp/xung cao =16/9;
	Ví dụ:
	- I2C_DutyCycle_2: tLow/tHigh = 2 => tLow = 2tHigh  
		100000khz, 1xung 10us 6.66us low, 3.33 high
	- I2C_DutyCycle_16_9: tLow/tHigh = 16/9 => 9tLow = 16tHigh.
- I2C_OwnAddress1: Cấu hình địa chỉ slave.
- I2C_Ack: Cấu hình ACK, có sử dụng ACK hay không.
- I2C_AcknowledgedAddress: Cấu hình số bit địa chỉ. 7 hoặc 10 bit

**Các hàm truyện nhận I2C**
- Hàm I2C_Send7bitAddress(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction), gửi đi 7 bit address để xác định slave cần giao tiếp. Hướng truyền được xác định bởi I2C_Direction để thêm bit RW.
- Hàm I2C_SendData(I2C_TypeDef* I2Cx, uint8_t Data) gửi đi 8 bit data.
- Hàm I2C_ReceiveData(I2C_TypeDef* I2Cx) trả về 8 bit data.
- Hàm I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT) trả về kết quả kiểm tra I2C_EVENT tương ứng:
	- I2C_EVENT_MASTER_MODE_SELECT: Đợi Bus I2C về chế độ rảnh.
	- I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED: Đợi xác nhận của Slave với yêu cầu nhận của Master.
	- I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED: Đợi xác nhận của Slave với yêu cầu ghi của Master.
	- I2C_EVENT_MASTER_BYTE_TRANSMITTED: Đợi truyền xong 1 byte data từ Master.
	- I2C_EVENT_MASTER_BYTE_RECEIVED: Đợi Master nhận đủ 1 byte data.

```cpp
I2C_GenerateSTART(I2C1, ENABLE);
 //Waiting for flag
 while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
I2C_Send7bitAddress(I2C1, 0x44, I2C_Direction_Transmitter);
//And check the transmitting
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

```
Hàm truyền địa chỉ chọn Slave:
- Tạo tín hiệu bắt đầu bằng hàm ```I2C_GenerateSTART```
- Đợi ACK từ Slave
- Và truyền 7 bit địa chỉ đến các Slave trong mạng để chọn Slave cần giao tiếp
- Chờ đến khi truyền xong và có xác nhận từ Slave

```cpp
void Send_I2C_Data(uint8_t data)
{
	I2C_SendData(I2C1, data);
	// wait for the data trasnmitted flag
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

```
Hàm gửi 8 bit dữ liệu
- Gửi data qua I2C1 và chờ đến khi hoàn tất truyền
```cpp
uint8_t Read_I2C_Data(){
	
	uint8_t data = I2C_ReceiveData(I2C1);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	return data;
}

```
Hàm nhận 8 bit dữ liệu

Sử dụng các hàm trên theo các câu trúc khác nhau để giao tiếp với slave

</details>



## Bài 7: UART Software - UART Hardware
<details><summary>Xem</summary>  

### UART Software
Sử dụng các GPIO để mô phỏng quá trình truyền nhận của UART

Có hai bước để thực hiện:
- Xác định các chân UART
- Cấu hình GPIO
```cpp
#define TX_Pin GPIO_Pin_0
#define RX_Pin GPIO_Pin_1
#define UART_GPIO GPIOA
#define time_duration 104

void RCC_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}
```
Chúng ta sẽ sử dụng A0 làm TX và A1 làm RX và cấp xung cho chúng.
```cpp
void delay_us(uint16_t timedelay){
    TIM_SetCounter(TIM2, 0);
    while(TIM_GetCounter(TIM2)<timedelay){}
}

void delay_s(uint32_t timedelay){
    TIM_SetCounter(TIM2, 0);
			for(int i = 0; i < timedelay*1000000; i++){
				delay_us(1);
			}
}
```
Tạo hai hàm delay sử dụng Tỉmer2 đã được đề cập từ bài trước.

```
void clock(){
	delay_us(time_duration);
}
```
Một hàm tạo delay, là khoảng thời gian giữa hai lần truyền dữ liệu để hai bên đồng bộ.

```time_duration``` là khoảng thời gian delay khi chọn Baudrate = 9600.
- BaudRate = 9600
- Tức truyền được 9600bits/s = 9600bits/1000ms
- Nghĩa là 1 bit truyền đi mất khoảng 0.10467ms
- Vì thế ```time_duration = 104``` để xấp xỉ.

```cpp
void GPIO_Config(){
	GPIO_InitTypeDef GPIOInitStruct;
	GPIOInitStruct.GPIO_Pin = RX_Pin;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIOInitStruct);
	//
	GPIOInitStruct.GPIO_Pin = TX_Pin;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIOInitStruct);
}
```
Ta sẽ cấu hình RX kiểu Input Floating để làm ngõ vào và TX Output Push-Pull để tạo ngõ ra.

```cpp
void UART_Config(){
	GPIO_SetBits(UART_GPIO, TX_Pin);
	delay_us(1);
}
```
Và khi chưa truyền dữ liệu thì cả hai chân được nối mức cao.

**Hàm mô phỏng gửi dữ liệu kí tự**
```cpp
void UARTSoftware_Transmit(char c) {
    // Start bit
    GPIO_ResetBits(GPIOA, TX_Pin); // Tao dieu kien bat dau
    clock();

    // Truyen các bit du lieu (LSB truoc)
    for (int i = 0; i < 8; i++) {
        if (c & (1 << i)) {
            GPIO_SetBits(GPIOA, TX_Pin);
        } else {
            GPIO_ResetBits(GPIOA, TX_Pin);
        }
        clock();
    }

    // Stop bit
    GPIO_SetBits(GPIOA, TX_Pin);
    clock();
}
```

- Trước khi gửi, phải kéo chân TX xuống mức và delay một chu kỳ để bắt đầu tín hiệu Start
- Sau đó sẽ truyền lần lượt 8 bits dữ liệu bằng cách dịch và set chân TX theo giá trị vừa tìm và delay trong một khoảng chu kỳ
- Sau khi truyền xong 8 bits dữ liệu sẽ truyền bits Stop tức kéo chân TX lên mức 1 để kết thúc truyền nhận (với trường hợp không dùng Parity).

**Hàm mô phỏng nhận ký tự**
```cpp
char UARTSoftware_Receive() {
    char c = 0;
		// Start bit
		while (GPIO_ReadInputDataBit(GPIOA, RX_Pin) == 1);

		delay_us(time_duration + time_duration / 2);

		for (int i = 0; i < 8; i++) {
				if (GPIO_ReadInputDataBit(GPIOA, RX_Pin)) {
						c |= (1 << i);
				}
				clock(); 
		}

		// Đợi Stop bit
		delay_us(time_duration / 2);
		
		return c;
}
```

- Tại bên nhận sẽ chờ cho đến khi nhận được tín hiệu Start
- Sau đó sẽ delay 1.5 chu kỳ để khi đọc dữ liệu, sẽ đọc tại vị trí giữa của một bit, nơi tín hiệu đã ổn định
- Lần lượt nhận 8 bits dữ liệu tại chân RX và dịch vào ```c```
- Delay nửa chu kỳ cuối để đợi Stop bit và trả về ký tự ```c``` là dữ liệu nhận được

```cpp
char data[9] = {'V', 'A', 'N', 'T', 'U', 'H', 'A', 'L', 'A'};
int main(){
	RCC_Config();
	GPIO_Config();
	TIMER_config();
	UART_Config();
	for (int i = 0; i<9; i++){
			UARTSoftware_Transmit(data[i]);
		delay_s(1);
	}
		UARTSoftware_Transmit('\n');
		
	while(1){
		UARTSoftware_Transmit(UARTSoftware_Receive());
	}	
}
```
Hàm test sẽ lần lượt gửi 9 ký tự qua UART và sau đó sẽ chờ để nhận dữ liệu nhận về của UART.


### UART Hardware
Sử dụng UART được tích hợp trên phần cứng của vi điều khiển.

Các bước thực hiện
- Xác định các chân GPIO của UART
- Cấu hình GPIO
- Cấu hình UART

Ở bài này chúng ta sẽ sử dụng **USART1** có hai chân **PA9 - TX** và **PA10 - RX**

Để sử dụng UART, ta phải thêm thư viện ``` "stm32f10x_usart.h"```

**Cấu hình GPIO**
```cpp
void RCC_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void GPIO_Config(void){
	GPIO_InitTypeDef GPIOInitStruct;
	
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_10; //Chan RX
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;// Neu 2 chan de AF thi bi ngan mach
	GPIO_Init(GPIOA, &GPIOInitStruct);
	
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_9; //Chan TX
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIOInitStruct);
}
```
Đầu tiên phải cấp xung GPIOA để sử dụng các chân và cấp thêm cho bộ USART1.

Chân RX được cấu hình ```GPIO_Mode_IN_FLOATING``` và ngõ ra TX được cấu hình ```GPIO_Mode_AF_PP```. Không cấu hình RX là AF_PP vì sẽ bị ngắn mạch hai chân RX và TX.

**Cấu hình UART**
```cpp
void UART_Config(void){
	USART_InitTypeDef UARTInitStruct;
	UARTInitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //Cau hinh che do: ca truyen va nhan (song cong)
	UARTInitStruct.USART_BaudRate = 115200; //Cau hinh toc do bit
	UARTInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Cau hinh kiem soat luong truyen du lieu tranh viec tran bo dem
	UARTInitStruct.USART_WordLength = USART_WordLength_8b; //Truyen du lieu 8 hoac 9 bit
	UARTInitStruct.USART_Parity = USART_Parity_No;
	UARTInitStruct.USART_StopBits = USART_StopBits_1;
	
	USART_Init(USART1, &UARTInitStruct);
	
	USART_Cmd(USART1, ENABLE);
	
}
```
- ```USART_Mode```: Cấu hình Mode của vi điều khiển là nhận hay gửi hoặc cả hai khi sử dụng phép OR.

- ```USART_BaudRate```: Cấu hình BaudRate để đồng bộ truyền nhận dữ liệu giữa hai vi điều khiển. Phổ biến nhất là 9600 và 115200.

- ```USART_HardwareFlowControl```: Dùng để bật chế độ kiểm soát luồng truyền để tránh việc tràn bộ đệm. 
    Có các giá trị sauL
    ```
    USART_HardwareFlowControl_None       
    USART_HardwareFlowControl_RTS    // Request to Send   
    USART_HardwareFlowControl_CTS    // Clear to Send
    USART_HardwareFlowControl_RTS_CTS
    ```

    RTS (Request To Send - Yêu cầu gửi)

    - Được điều khiển bởi thiết bị gửi dữ liệu.
    - Khi thiết bị gửi dữ liệu sẵn sàng để truyền, nó kéo RTS xuống mức thấp (Active Low).
    - Khi bộ đệm của thiết bị nhận đầy, nó kéo RTS lên mức cao, báo cho thiết bị gửi tạm dừng truyền dữ liệu.

    CTS (Clear To Send - Cho phép gửi)

    - Được điều khiển bởi thiết bị nhận dữ liệu.
    - Khi bộ đệm của thiết bị nhận còn trống, nó kéo CTS xuống mức thấp, cho phép thiết bị gửi tiếp tục truyền dữ liệu.
    - Nếu bộ đệm đầy, thiết bị nhận kéo CTS lên mức cao, yêu cầu dừng gửi dữ liệu.
    Thiết bị A muốn gửi dữ liệu:
    -   Kiểm tra CTS từ thiết bị B:
    - Nếu CTS = LOW, thiết bị A có thể gửi dữ liệu.
    - Nếu CTS = HIGH, thiết bị A tạm dừng gửi để tránh tràn bộ đệm.
    Thiết bị B nhận dữ liệu:
    - Khi bộ đệm gần đầy, RTS = HIGH, báo hiệu thiết bị A tạm dừng truyền dữ liệu.
    - Khi bộ đệm có không gian trống, RTS = LOW, cho phép tiếp tục truyền.

- ```USART_WordLength```: Khai báo dữ liệu truyền là 8 bits hoặc 9 bits
- ```USART_StopBits```: Cấu hình số lượng Stopbit
    ```
    USART_StopBits_1
    USART_StopBits_0_5 
    USART_StopBits_2    
    USART_StopBits_1_5    
    ```
- ```USART_Parity```: Cấu hình sử dụng Parity, có hai loại là chẵn hoặc lẽ, có thể không cần sử dụng
    ```
    #define USART_Parity_No
    #define USART_Parity_Even
    #define USART_Parity_Odd 
    ```

**Một số hàm được thiết lập sẵn trong UART**
- Hàm ```USART_SendData(USART_TypeDef* USARTx, uint16_t Data)``` truyền data từ UARTx. Data này đã được thêm bit chẵn/lẻ tùy cấu hình.
- Hàm ```USART_ReceiveData(USART_TypeDef* USARTx)``` nhận data từ UARTx.
- Hàm ```USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG)``` trả về trạng thái cờ USART_FLAG tương ứng:
    ```
    USART_FLAG_TXE: Cờ truyền, set lên 1 nếu quá trình truyền hoàn tất.
    USART_FLAG_RXNE: Cờ nhận, set lên 1 nếu quá trình nhận hoàn tất.
    USART_FLAG_IDLE: Cờ báo đường truyền đang ở chế độ Idle.
    USART_FLAG_PE: Cờ báo lỗi Parity.
    ```


**Hàm gửi một ký tự**
```cpp
void UART_SendChar(USART_TypeDef *USARTx, char data){
	
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE)==RESET); // Cho khi thanh ghi DR trong de chen du lieu moi de gui
	
	USART_SendData(USARTx, data);
	
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);//Cho den khi truyen thanh cong
	
	
}
```
- Đợi đến khi thanh ghi DR trống đến thêm dữ liệu mới và gửi
- Gửi dữ liệu bằng hàm ```USART_SendData(USARTx, data);```

- Sau đó chờ đến khi truyền thành công

**Hàm nhận một ký tự**
```cpp
char UART_ReceiveChar(USART_TypeDef *USARTx){
	char tmp = 0x00;
	
	while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE)==RESET);
	
	tmp = USART_ReceiveData(USARTx);
	
	return tmp;
}
```
- Chờ đến khi nhận hoạt tất
- Nhận Data từ UARTx gán vào tmp.


**Test**
```cpp
uint8_t DataTrans[] = {'V','A','N','T','U'};//Du lieu duoc truyen di
int main() {
	RCC_Config();
	GPIO_Config();
	UART_Config();
	TIMER_config();	
	for(int i = 0; i<5; ++i){
			delay_ms(1998);
			UART_SendChar(USART1, DataTrans[i]);
			delay_ms(2);
		}
	UART_SendChar(USART1, '\n');

	while(1){
		UART_SendChar(USART1,UART_ReceiveChar(USART1));
	}
}
```
Sử dụng main() để test bằng cách gửi 5 bytes dữ liệu qua USART1 và sau đó chờ để nhận dữ liệu về.




</details>


## Bài 8: Interrupt
<details><summary>Xem</summary>  

### Ngắt ngoài

Các bước thực hiện
- Xác định chân Ngắt
- Cấu hình GPIO

Để sử dụng được ngắt ngoài, ngoài bật clock cho GPIO tương ứng cần bật thêm clock cho AFIO.
```cpp
void RCC_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}
```
Cấu hình ngõ vào Ngắt
```cpp
void GPIO_Config(void){
	GPIO_InitTypeDef GPIOInitStruct;
	//Cau hinh cho chan interrupt PA0
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_IPU; //Input Pull_up
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIOInitStruct);
}
```
Ta sẽ cấu hình chân ngắt là Input và có thể cấu hình thêm trở kéo lên/xuống tùy theo cạnh ngắt được sử dụng.

**Cấu trúc**
![Cấu trúc external interrupt](https://i.imgur.com/efZpWTc.png)

Vi điều khiển sẽ có 16 Line ngắt từ 0-15, mỗi Line sẽ tương ứng với một nhóm các chân GPIO. Ví dụ Line0 sẽ nhận tín hiệu ngắt từ PA0, PB0,...

Để sử dụng chân GPIO nào để làm ngắt cho Line thì phải nối Line với chân đó.   
**Hạn chế sử dụng hai GPIO trên cùng một Line ngắt vì khi có ngắt sẽ không biết GPIO nào tạo ngắt**.

**Cấu hình ngắt**
```cpp
void EXTI_Config(void) {
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0); //PA0
	
	EXTI_InitTypeDef EXTIInitStruct;
	
	EXTIInitStruct.EXTI_Line = EXTI_Line0;//Xac dinh duong ngat (EXTIn) co 18 line va 15 line duoc cau hinh san, 3line mo rong 
	EXTIInitStruct.EXTI_Mode = EXTI_Mode_Interrupt; //Thuc thi ham ngat khi xay ra ngat ---- Even thi khong thuc thi ham ngat
	EXTIInitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//chon canh kich hoat ngat
	EXTIInitStruct.EXTI_LineCmd = ENABLE;
	
	EXTI_Init(&EXTIInitStruct);
}
```
- Hàm ```GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource)``` cấu hình chân ở chế độ sử dụng ngắt ngoài:
    ```GPIO_PortSource```: Chọn Port để sử dụng làm nguồn cho ngắt ngoài.  
    ```GPIO_PinSource```: Chọn Pin để cấu hình.

Tương tự với các ngoại vi khác, External Interrupt cũng sẽ có một Struct ```Struct EXTI_InitTypeDef``` giữ các thuộc tính của ngắt.
- ```EXTI_Line```: Cấu hình Line ngắt, EXTI_Line0-15
- ```EXTI_Mode```: Chế độ hoạt động cho ngắt ngoài. Có hai chế độ:
    - ```EXTI_Mode_Interrupt```: Khi ngắt xảy ra sẽ tạo ra ngắt và chạy hàm thực thi.
    - ```EXTI_Mode_Event```: Khi có ngắt chỉ báo cho CPU biết chứ không thực thi hàm phục vụ ngắt.
- ```EXTI_Trigger```: Xác định khi nào xảy ra Ngắt
    - ```EXTI_Trigger_Rising```: Ngắt cạnh lên
    - ```EXTI_Trigger_Falling```: Ngắt cạnh xuống
    - ```EXTI_Trigger_Rising_Falling```: Cả hai
- ```EXTI_LineCmd```: Cho phép ngắt ở Line đã cấu hình.
- ```EXTI_Init(&EXTIInitStruct)```: Lưu cài đặt vào thanh ghi.

**Cấu hình bộ quản lý ngắt**

NVIC (Nested Vectored Interrupt Controller) chịu trách nhiệm quản lý và xử lý các ngắt, NVIC cho phép MCU xử lý nhiều ngắt từ nhiều nguồn khác nhau, có ưu tiên ngắt và hổ trợ ngắt lồng nhau. Số ưu tiên càng nhỏ thì độ ưu tiên càng lớn.

- Priority Group xác định cách phân chia bit giữa Preemption Priority và SubPriority bằng cách sử dụng hàm ```NVIC_PriorityGroupConfig(uint32_t PriorityGroup)```. Trong đó:
    - Preemption Priority xác định mức độ ưu tiên chính của ngắt và quy định ngắt nào có thể được lồng vào.
    - SubPriority: khi các ngắt có cùng mức Preemption Preemption, thì sẽ xem xét tới SubPriority.

![PriorityGrouip](https://i.imgur.com/mxtLW1k.png)

Sẽ có **4 bits** dùng để quan lý hai loại ưu tiên trên và hàm ```NVIC_PriorityGroupConfig``` sẽ chia 4 bits này ra để cấu hình.  
Ví dụ khi cấu hình là ```NVIC_PriorityGroup_1``` Thì một bit sẽ được sử dụng cho Preemption Priority và 3 bits sẽ dùng cho SubPriority.

Bộ NVIC cấu hình các tham số ngắt và quản lý các vecto ngắt. Các tham số được cấu hình trong ```NVIC_InitTypeDef```, bao gồm:
- ```NVIC_IRQChannel```: Xác định vector ngắt của kênh ngắt cần được cấu hình. Với vector ngắt là địa chỉ của hàm phục vụ ngắt tương ứng.
    - Các Line0 đến Line4 sẽ được phân vào các vector ngắt riêng tương ứng EXTI0 -> EXTI4, 
    - Line5->Line9 được phân vào vector ngắt EXTI9_5,
    - Line10->Line15 được phân vào vecotr EXTI15_10.



- ```NVIC_IRQChannelPreemptionPriority```: Cấu hình độ ưu tiên của ngắt.
- ```NVIC_IRQChannelSubPriority```: Cấu hình độ ưu tiên phụ.
- ```NVIC_IRQChannelCmd```: Cho phép ngắt.

```cpp
void NVIC_Config(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//Cau hinh so bit cua Preemption Priority vaf Sub Prioriry
	
	NVIC_InitTypeDef NVICInitStruct;

	NVICInitStruct.NVIC_IRQChannel = EXTI0_IRQn; //Dang cau hinh ngat Line0 nen chon vector ngat EXTI0_IRQn
	NVICInitStruct.NVIC_IRQChannelPreemptionPriority = 1; //mamg gia tri tu 0-3 vi cai dat 2bit cho Preemption Priority
	NVICInitStruct.NVIC_IRQChannelSubPriority = 1;	//mamg gia tri tu 0-3 vi cai dat 2bit cho Sub Priority
	NVICInitStruct.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVICInitStruct);
}
```

**Hàm phục vụ ngắt ngoài**

Trong hàm phục vụ ngắt ngoài, chúng ta sẽ thực hiện:
- Kiểm tra ngắt đến từ line nào, có đúng là line cần thực thi hay không?
- Thực hiện các lệnh, các hàm.
- Xóa cờ ngắt ở line.

Ngắt trên từng line có hàm phục riêng của từng line. Có tên cố định: EXTIx_IRQHandler() (x là line ngắt tương ứng). Và hàm này đã được định nghĩa sẵn trên hệ thống ở dạng WEAK tức là có thể ghi đè được.  
```cpp
EXPORT  EXTI0_IRQHandler           [WEAK]  
EXPORT  EXTI1_IRQHandler           [WEAK]  
EXPORT  EXTI2_IRQHandler           [WEAK]
EXPORT  EXTI3_IRQHandler           [WEAK]
EXPORT  EXTI4_IRQHandler           [WEAK]
```

- Hàm ```EXTI_GetITStatus(EXTI_Linex)```, Kiểm tra cờ ngắt của line x tương ứng.   
- Hàm ```EXTI_ClearITPendingBit(EXTI_Linex)```: Xóa cờ ngắt ở line x.

Cú pháp:
```cpp
void EXTI0_IRQHandler()
{	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{

	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}
```

Ví dụ:
```cpp
//Ham xu ly khi co ngat line0
void EXTI0_IRQHandler(){	
	if(EXTI_GetITStatus(EXTI_Line0) != RESET){ //Kiem tra co ngat cua Line0
		for(int i = 0; i<5; i++){
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
			delay_ms(200);
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			delay_ms(200);
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line0); //Xoa co ngat cua line0
}


int main(){
	RCC_Config();
	GPIO_Config();
	EXTI_Config();
	NVIC_Config();
	TIMER_config();
	while(1){
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		delay_ms(3000);
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		delay_ms(3000);
  }   
}
```

Led tại PC13 sẽ nhấp nháy chậm, khi có ngắt tại PA0 cạnh xuống sẽ nhấp nháy nhanh 5 lần.

### Ngắt Timer

Tương tự với External Interrupt thì Timer Interrupt cũng cần phải cấp xung cho bộ AFIO.

**Cấu hình ngắt Timer**
```cpp
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV2; // f = 36Mhz 
	TIM_TimeBaseInitStruct.TIM_Prescaler = 36000-1; //1ms đếm lên một lần
	TIM_TimeBaseInitStruct.TIM_Period =5000-1; //Chu kỳ 5000 tức đếm lên 5000 lần thì reset tức 5s.
	
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);


	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //TIM_IT_Update: Khi Timer cập nhật lại bộ đếm về 0 thì tạo ngắt.
	

	TIM_Cmd(TIM2, ENABLE);
```

**Cấu hình NVIC**
```cpp
void NVIC_Config(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//Cau hinh so bit cua Preemption Priority vaf Sub Prioriry
	
	NVIC_InitTypeDef NVICInitStruct;

	NVICInitStruct.NVIC_IRQChannel = TIM2_IRQn; //Cau hinh channel ngat la Timer2
	NVICInitStruct.NVIC_IRQChannelPreemptionPriority = 0; //mamg gia tri tu 0-3 vi cai dat 2bit cho Preemption Priority
	NVICInitStruct.NVIC_IRQChannelSubPriority = 0;	//mamg gia tri tu 0-3 vi cai dat 2bit cho Sub Priority
	NVICInitStruct.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVICInitStruct);
}
```

**Test delay**
```cpp
void delay_ms(uint16_t timedelay){
	count = 0;
	while(count < timedelay);
}

void TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)){ //Kiem tra co ngat TIM_IT_UPDATe
		count++;
	}
	// Clears the TIM2 interrupt pending bit
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}


int main() {
	RCC_Config();
	TIM_Config();
	GPIO_Config();
	NVIC_Config();
	while(1){
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		delay_ms(2000);
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		delay_ms(2000);
	}
}
```
Sử dụng ngắt Timer để tạo delay nhấp nháy led PC13.

### Comunication Interrupt.
STM32F1 hỗ trọ các ngắt cho các giao thức truyền nhận như SPI, I2C, UART…  
Ở bài này ta sẽ ví dụ với UART ngắt.  
Các ngắt ở SPI, I2C… sẽ được cấu hình tương tự như UART.  

**Cấu hình UART**
```cpp
void UART_Config(void){
	USART_InitTypeDef UARTInitStruct;
	UARTInitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //Cau hinh che do: ca truyen va nhan (song cong)
	UARTInitStruct.USART_BaudRate = 9600; //Cau hinh toc do bit
	UARTInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Cau hinh kiem soat luong truyen du lieu tranh viec tran bo dem
	UARTInitStruct.USART_WordLength = USART_WordLength_8b; //Truyen du lieu 8 hoac 9 bit
	UARTInitStruct.USART_Parity = USART_Parity_No;
	UARTInitStruct.USART_StopBits = USART_StopBits_1;
	
	USART_Init(USART1, &UARTInitStruct);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);// Khi co du lieu gui toi thi goi ham ngat
	
	USART_Cmd(USART1, ENABLE);
}
```
- Hàm ```USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState)``` gồm ba tham số:
    - ```USART_TypeDef* USARTx```: Bộ UART cần cấu hình.
    - ```uint16_t USART_IT```: Chọn nguồn ngắt UART, Có nhiều nguồn ngắt từ UART, ở bài này ta chú ý đến ngắt truyền (USART_IT_TXE) và ngắt nhận (USART_IT_RXNE).  
    - ```FunctionalState NewState```: Cho phép ngắt.

**Cấu hình NVIC tương tự**
```cpp
void NVIC_Config(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStruct;

	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

}
```

**Hàm phục vụ ngắt để nhận dữ liệu**

Hàm ```USARTx_IRQHandler()``` sẽ được gọi nếu xảy ra ngắt trên Line ngắt UART đã cấu hình.

Hàm ```USART_GetITStatus``` kiểm tra các cờ ngắt UART. Hàm này nhận 2 tham số là bộ USART và cờ tương ứng cần kiểm tra:  
- ```USART_IT_RXNE```: Cờ ngắt nhận, cờ này set lên 1 khi bộ USART phát hiện data truyền tới.
- ```USART_IT_TXE```: Cờ ngắt truyền, cờ này set lên 1 khi USART truyền data xong.  
Có thể xóa cờ ngắt, gọi hàm ```USART_ClearITPendingBit``` để đảm bảo không còn ngắt trên line (thông thường cờ ngắt sẽ tự động xóa).

```cpp

uint8_t UART_ReceiveChar(USART_TypeDef *USARTx){
	uint8_t  data = 0x00;
	while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE)==RESET); //cho co nhan bat len 1 ( nhan hoan tat) thi tra ve gia tri data
	data = USART_ReceiveData(USARTx);
	return data;
}

uint8_t Data_Receive;

void USART1_IRQHandler(){
	if(USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET){
		Data_Receive = UART_ReceiveChar(USART1);
	}
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}

uint16_t count = 0;
int main() {
	RCC_Config();
	GPIO_Config();
	UART_Config();
	TIMER_config();
	NVIC_Config();
	while(1){
		count++;
		delay_ms(1000);
	}
}
```
Tại hàm phục vụ ngắt sẽ thực thi khi có dữ liệu được nhận tại USART1 để không bị mất dữ liệu và tại chương trình chính sẽ cộng biến Count lên 1.


</details>



## Bài 9: ADC - Analog Digital Converter
<details><summary>Xem</summary>  

Vi điều khiển cũng như hầu hết các thiết bị điện tử ngày nay đều sử dụng tín hiệu kĩ thuật số, dựa trên các bit nhị phân để hoạt động.  
Tín hiệu thì không chỉ luôn luôn là tín hiệu số, đa số tín hiệu trong thực tế ở dạng tương tự và liên tục, vì thế để các thiết bị hiểu được cần phải chuyển đổi nó về dạng số.

![ADC](https://i.imgur.com/1hRVRvb.png)

ADC (Analog-to-Digital Converter) là 1 mạch điện tử lấy điện áp tương tự làm đầu vào và chuyển đổi nó thành dữ liệu số (1 giá trị đại diện cho mức điện áp trong mã nhị phân).

Khả năng chuyển đổi của ADC được quyết định bởi 2 yếu tố chính:
- Độ phân giải: Số bit mà ADC sử dụng để mã hóa tín hiệu. Có thể xem như là số mức mà tín hiệu tương tự được biểu diễn. ADC có **độ phân giải càng cao** thì cho ra kết quả chuyển đổi **càng chi tiết**. 

- Tần số/chu kì lấy mẫu: tốc độ/khoảng thời gian giữa 2 lần mã hóa. **Tần số lấy mẫu càng lớn** thì tín hiệu sau khi chuyển đổi sẽ có **độ chính xác càng cao**. Khả năng tái tạo lại tín hiệu càng chính xác.  
	```Tần số lấy mẫu = 1/(thời gian lấy mẫu+ Tg chuyển đổi.)```

	**Tần số lấy mẫu phải lớn hơn tần số của tín hiệu ít nhất 2 lần để đảm bảo độ chính xác khi khôi phục lại tín hiệu.**

### ADC trong STM32

STM32F103C8 có 2 bộ ADC đó là ADC1 và ADC2 với nhiều mode hoạt động.  
Kết quả chuyển đổi được lưu trữ trong thanh ghi 16 bit. 
- Độ phân giải 12 bit.
- Có các ngắt hỗ trợ.
- Có thể điều khiển hoạt động ADC bằng xung Trigger.
- Thời gian chuyển đổi nhanh : 1us tại tần số 65Mhz.
- Có bộ DMA giúp tăng tốc độ xử lí.

**Các chế độ của ADC**  
Có 16 kênh ADC nhưng tại một thời điểm chỉ có một kênh được chuyển đổi dữ liệu
- **Regular Conversion**:
	- **Single**: ADC chỉ đọc 1 kênh duy nhất, và chỉ đọc khi nào được yêu cầu.
	- **Single Continuous**: ADC sẽ đọc một kênh duy nhất, nhưng đọc dữ liệu nhiều lần liên tiếp (Có thể được biết đến như sử dụng DMA để đọc dữ liệu và ghi vào bộ nhớ). 
	- Scan: **Multi-Channels**: Quét qua và đọc dữ liệu nhiều kênh, nhưng chỉ đọc khi nào được yêu cầu.
	- Scan: **Continuous Multi-Channels Repeat**: Quét qua và đọc dữ liệu nhiều kênh, nhưng đọc liên tiếp nhiều lần giống như Single Continous. 
- **Injected Conversion**:  
	Trong trường hợp nhiều kênh hoạt động. Khi kênh có mức độ ưu tiên cao hơn có thể tạo ra một **Injected Trigger**. Khi gặp Injected Trigger thì ngay lập tức kênh đang hoạt động bị ngưng lại để kênh được ưu tiên kia có thể hoạt động.

### Đọc giá trị Analog từ biến trở

![Biến trở](https://i.imgur.com/Q6cnkUC.png)

**Cấp xung cho ADC**
```cpp
void RCC_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}
```
Cấp xung cho bộ ADC và GPIO để cử dụng Pin, đồng thời cũng phải cấp cho bộ AFIO.

**Cấu hình GPIO**
```cpp
void GPIO_Config(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN; //Analog input
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}
```
Cấu hình chân PA0 với chức năng Analog Input ```GPIO_Mode_AIN```

**Cấu hình ADC**
```cpp
void ADC_Config(void){
	ADC_InitTypeDef ADC_InitStruct;
	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent; //Hoat dong nhu ADC binh thuong, doc lap voi nhau va khong can kich hoat
	ADC_InitStruct.ADC_NbrOfChannel = 1; //NumberOfChannel so luong kenh can cau hinh (1-16)
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;//Hoat dong o che do Continous hay khong
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //Cau hinh cho phep su dung Trigger (tin hieu de bat dau chuyen doi ADC)
	ADC_InitStruct.ADC_ScanConvMode = DISABLE; //Co su dung Scan de quet nhieu kenh khong
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;//Cau hinh de can le cho Data (ADC 12bit luu vao 16bit bi du 4 bit), ghi vao LSB hay MSB

	ADC_Init(ADC1, &ADC_InitStruct);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	
	ADC_Cmd(ADC1, ENABLE);
	//Bat dau qua trinh chuyen doi (Vi chon che do Continous nen chi can goi 1 lan)
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
```
- ```ADC_Mode```: Cấu hình chế độ hoạt động cho bộ ADC
	```
	ADC_Mode_Independent: Các kênh hoạt động độc lập với nhau.                 
	ADC_Mode_RegInjecSimult: Sử dụng cả hai mode Regular và Injected.                      
	ADC_Mode_InjecSimult: Hoạt động chế độ Injected                  
	ADC_Mode_RegSimult: Hoạt động ở chế độ Regular                                      
	```

- ```ADC_NbrOfChannel```: Số lượng ADC cần cấu hình 1-16
- ```ADC_ContinuousConvMode```: Có chuyển đổi ADC liên tục hay không ```ENABLE``` hoặc ```DISABLE```		

- ```ADC_ExternalTrigConv```: Cấu hình sử dụng kích chuyển đổi ADC ở bên ngoài như Timer, External Trigger.
- ```ADC_ScanConvMode```: Có sử dụng Mode Scan để quét qua nhiều kênh hay không
- ```ADC_DataAlign```: Cấu hình căn lề cho data. Vì bộ ADC xuất ra giá trị 12bit, được lưu vào biến 16 hoặc 32 bit nên phải căn lề các bit về trái hoặc phải. Nếu căn phải thì dữ liệu đọc giữ nguyên, căn trái dữ liệu đọc gấp 16 lần thực tế nếu được lưu vào biến 16 bits

- Ngoài các tham số trên, cần cấu hình thêm thời gian lấy mẫu, thứ tự kênh ADC khi quét, ```ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime)```
	- ```Rank```: Ưu tiên của kênh ADC. Cao nhất 1, thấp nhất 16
	- ```SampleTime```: Thời gian lấy mẫu tín hiệu.
		```
		ADC_SampleTime_1Cycles5: Sample time equal to 1.5 cycles
		ADC_SampleTime_7Cycles5: Sample time equal to 7.5 cycles
		ADC_SampleTime_13Cycles5: Sample time equal to 13.5 cycles
		ADC_SampleTime_41Cycles5: Sample time equal to 41.5 cycles	
		ADC_SampleTime_55Cycles5: Sample time equal to 55.5 cycles	
		ADC_SampleTime_71Cycles5: Sample time equal to 71.5 cycles	
		ADC_SampleTime_239Cycles5: Sample time equal to 239.5 cycles	
		```
- ```ADC_SoftwareStartConvCmd(ADCx, NewState)```: Bắt đầu quá trình chuyển đổi

- ```ADC_SoftwareStartConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState)```: Bắt đầu quá trình chuyển đổi.
- ```ADC_GetConversionValue(ADC_TypeDef* ADCx)```: Đọc giá trị chuyển đổi được ở các kênh ADC tuần tự.
- ```ADC_GetDualModeConversionValue(void)```: Trả về giá trị chuyển đổi cuối cùng của ADC1, ADC2 ở chế độ kép. Trả về thanh ghi 32 bits, 16 bits đầu của ADC2 và 16 bits sau của ADC1

### Bộ lọc Kalman
Bộ lọc Kalman, được Rudolf (Rudy) E. Kálmán công bố năm 1960, là thuật toán sử dụng chuỗi các giá trị đo lường, bị ảnh hưởng bởi nhiễu hoặc sai số để ước đoán biến số nhằm tăng độ chính xác.
![Kalman](https://i.imgur.com/2wJ7ubG.png)

Gọi hàm ```SimpleKalmanFilter(float mea_e, float est_e, float q))```; Để khởi các giá trị sai số ước tính, sai số đo lường và sai số quá trình ban đầu.
	- ```mea_e```: Sai số đo lường của phần cứng
	- ```est_e```: Sai số dự đoán.
	- ```q```: Sai số nhiễu quá trình từ tính hiệu đến bộ chuyển đổi ADC.


</details>


## Bài 10: DMA - Direct Memory Access
<details><summary>Xem</summary>  

**Khi truyền nhận dữ liệu không dùng DMA**  

CPU sẽ điều khiển việc trao đổi data giữa Peripheral (UART, I2C, SPI, ...) và bộ nhớ (RAM) qua các đường bus. 

CPU phải lấy lệnh từ bộ nhớ (FLASH) để thực thi các lệnh của chương trình. Vì vậy, khi cần truyền dữ liệu liên tục giữa Peripheral và RAM, CPU sẽ bị chiếm dụng, và không có thời gian làm các công việc khác, hoặc có thể gây miss dữ liệu khi transfer. 


### Khái niệm DMA

DMA – Direct memory access được sử dụng với mục đích truyền data với tốc độ cao từ thiết bị ngoại vi đến bộ nhớ cũng như từ bộ nhớ đến bộ nhớ mà không cần thông qua CPU.

DMA có thể điều khiển data truyền từ:
- Bộ nhớ đến Peripheral 
- Ngược lại, Periph đến Bộ nhớ.
- Giữa 2 vùng nhớ.
mà không thông qua data bus  của CPU, giữ cho tài nguyên của CPU được rảnh rỗi cho các thao tác khác. Đồng thời tránh việc data nhận về từ ngoại vi bị mất mát.

![DMA](https://i.imgur.com/wfdkwZu.png)

CPU sẽ kích hoạt bộ DMA để làm việc thông qua Handshake, Sau đó data sẽ được trao đổi thông qua bộ DMA Controller. Sau đó DMA sẽ báo cho CPU tín hiệu hoàn thành và CPU có thể sử dụng các dữ liệu đó.

![DMA Block Diagram](https://i.imgur.com/9DcK9TS.png)

CPU(1) cấu hình và kích hoạt DMA(4) hoạt động

Các ngoại vi(5) sẽ sử dụng DMA Request(6) để yêu cầu DMA(4) gửi nhận dữ liệu từ ngoại vi. DMA Request có thể là yêu cầu ngoại vi gửi dữ liệu đến SRAM hoặc từ SRAM đến ngoại vi.

DMA(4) sẽ tiền hành thực hiện yêu cầu từ DMA Request(6)

Lấy dữ liệu từ SRAM(2) thông qua Bus Matrix(3) <-> đi qua các đường bus ngoại vi <-> truy cập các thanh ghi của ngoại vi(5).

Khi kết thúc, DMA(4) kích hoạt ngắt báo cho CPU(1) biết quá trình truyền nhận hoàn tất.

### DMA trong STM32

STM32F103C8T6 có hai bộ DMA. DMA1 gồm 7 kênh, DMA2 gồm 5 kênh.

![DMA STM32](https://i.imgur.com/zTySk0H.png)

- Các kênh đều có thể được cấu hình riêng biệt.
- Mỗi kênh có thể phục vụ cho nhiều ngoại vi khác nhau nhưng không đồng thời.
- Có 4 mức ưu tiên có thể lập trình cho mỗi kênh.
- Kích thước data được sử dụng là 1 Byte, 2 Byte (Half Word) hoặc 4byte (Word)
- Hỗ trợ việc lặp lại liên tục Data.
- 5 cờ báo ngắt (```DMA Half Transfer, DMA Transfer complete, DMA Transfer Error, DMA FIFO Error, Direct Mode Error```).
- Quyền truy cập tới Flash, SRAM, APB1, APB2, AHB.
- Số lượng data có thể lập trình được lên tới 65535.
- Đối với DMA2, mỗi luồng đều hỗ trợ để chuyển dữ liệu từ bộ nhớ đến bộ nhớ.

DMA có hai chế độ hoạt động

- Normal mode: Với chế độ này, DMA truyền dữ liệu cho tới khi truyền đủ 1 lượng dữ liệu giới hạn đã khai báo DMA sẽ dừng hoạt động. Muốn nó tiếp tục hoạt động thì phải khởi động lại
 
- Circular mode: Với chế độ này, Khi DMA truyền đủ 1 lượng dữ liệu giới hạn đã khai báo thì nó sẽ truyền tiếp về vị trí ban đầu (Cơ chế như Ring buffer).

### Sử dụng DMA

**Cấp xung cho DMA thông qua bus AHB**
```cpp
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_SPI1| RCC_APB2Periph_AFIO, ENABLE);
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
```

**Cấu hình DMA**

Các tham số cho bộ DMA được cấu hình trong struct DMA_InitTypeDef. Gồm:

- ```DMA_PeripheralBaseAddr```: Cấu hình địa chỉ của ngoại vi cho DMA. Đây là địa chỉ mà DMA sẽ lấy data hoặc truyền data tới cho ngoại vi.
- ```DMA MemoryBaseAddr```: Cấu hình địa chỉ vùng nhớ cần ghi/ đọc data	.
- ```DMA_DIR```: Cấu hình hướng truyền DMA, từ ngoại vi tới vùng nhớ ```PeripheralSRC``` hay từ vùng nhớ tới ngoại vi ```PeripheralDST```.
- ```DMA_BufferSize```: Cấu hình kích cỡ buffer. Số lượng bytes dữ liệu muốn gửi/nhận qua DMA.
- ```DMA_PeripheralInc```: Cấu hình địa chỉ ngoại vi có tăng sau khi truyền DMA hay không.
- ```DMA Memory Inc```: Cấu hình địa chỉ bộ nhớ có tăng lên sau khi truyền DMA hay không.
- ```DMA_PeripheralDataSize```: Cấu hình độ lớn data của ngoại vi.
- ```DMA_MemoryDataSize```: Cấu hình độ lớn data của bộ nhớ, ```Byte```, ```halfWord``` hoặc ```Word```
- ```DMA_Mode```: Cấu hình mode hoạt động là Normal hay Circular 
- ```DMA_Priority```: Cấu hình độ ưu tiên cho kênh DMA.
- ```DMA_M2M```: Kích hoạt sử dụng truyền từ bộ nhớ đếm bộ nhớ cho kênh DMA ```Enable``` hoặc ```Disable```

Sau khi cấu hình cho DMA xong, chỉ cần gọi hàm DMACmd cho ngoại vi tương ứng. Bộ DMA sẽ tự động truyền nhận data cũng như ghi dữ liệu vào vùng nhớ cụ thể. 

```cpp
DMA_Init(DMA1_Channel2, &DMA_InitStruct);
DMA_Cmd(DMA1_Channel2, ENABLE);
SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
```
- ```Channel2```: Ứng với ngoại vi SPI1, RX nhận.
- Hàm ```SPI_I2S_DMACmd()``` cho phép truyền nhận DMA của bộ SPI.

Ngoài ra còn các hàm để áp dụng các ngoại vi khác 
```cpp
void SPI_I2S_DMACmd(SPI_TypeDef* SPIx, uint16_t SPI_I2S_DMAReq, FunctionalState NewState)
void I2C_DMACmd(I2C_TypeDef* I2Cx, FunctionalState NewState)
void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState)
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState)
```

### PWM - Pulse Width Modulation
Trong điều khiển động cơ servo, tín hiệu PWM (Điều chế độ rộng xung) được sử dụng để chỉ định góc mà động cơ servo sẽ xoay đến. Tín hiệu PWM có hai yếu tố quan trọng:

- **Tần số**: Là số lần tín hiệu lặp lại trong một giây. Đối với servo, tần số thông thường là 50Hz (tức là, chu kỳ lặp lại sau mỗi 20ms).  
- **Độ rộng xung** (Pulse Width ): Là thời gian tín hiệu ở mức cao trong mỗi chu kỳ. Độ rộng xung thường được đo bằng microsecond (µs) và quyết định góc mà servo sẽ xoay đến.

Tỉ lệ độ rộng xung với chu kì xung gọi là chu kì nhiệm vụ(Duty Cycle).

![Duty Cycle](https://i.imgur.com/Lw6k4H9.png)

Đối với hầu hết các servo, độ rộng xung PWM để xoay đến góc 0 độ là khoảng 1000µs, và để xoay đến 180 độ là khoảng 2000µs. Các giá trị này có thể thay đổi tùy thuộc vào loại servo.

Đối với hầu hết các servo, độ rộng xung PWM để xoay đến góc 0 độ là khoảng 1000µs, và để xoay đến 180 độ là khoảng 2000µs. Các giá trị này có thể thay đổi tùy thuộc vào loại servo.

![Servo](https://i.imgur.com/w21VKen.png)

**Công thức tính toán độ rộng xung**

```pulseWidth = MIN_PULSE_WIDTH + (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) * angle / 180```

- ```MIN_PULSE_WIDTH``` là độ rộng xung cho góc 0 độ (thường là 1000µs).
- ```MAX_PULSE_WIDTH``` là độ rộng xung cho góc 180 độ (thường là 2000µs).
- ```angle``` là góc mà servo xoay đến.


</details>


## Bài 11: Flash - Bootloader 

<details><summary>Xem</summary>  

![3MEM](https://i.imgur.com/oD2TG7T.png)

- RAM dùng để lưu những biến thay đổi liên tục trong quá trình hoạt động
- Flash dùng để lưu trữ mã lệnh của chương trình, những biến cố định
- Eeprom lưu trữ những giá trị lâu dài

### Flash

STM32F1 không có EEPROM mà chỉ được cung cấp sẵn 128/64Kb Flash.  
Được chia nhỏ thành các Bank (với bộ nhớ có kích thước lớn), mỗi Bank gồm nhiều Page, mỗi Page có kích thước 1KB để dễ dàng quản lý.  
Trước khi ghi phải xóa, mỗi lần xóa phải **xóa cả page** thành ```0XFF```.  
Khi ghi dữ liệu, chỉ có thể **ghi Halfword(16 bits) hoặc Word(32 bits)**.  
Giới hạn số lần xóa/ghi

**Các vùng nhớ của Flash**

![Flash](https://i.imgur.com/s67xSc4.png)

Thông thường chương trình sẽ được nạp vào vùng nhớ bắt đầu ở 0x08000000, vùng nhớ phía sau sẽ là trống và người dùng có thể lưu trữ dữ liệu ở vùng này.  
Thư viện Std cung cấp các hàm để giao tiếp với Flash trong Module Flash. File "stm32f10x_flash.h".

**Xóa một Page**
![PageDelete](https://i.imgur.com/OWWxTKU.png)

- Đầu tiên, kiểm tra cờ ```FLASH_CR_LOCK``` của Flash, nếu Cờ này đang được bật, Flash đang ở chế độ Lock và cần phải được Unlock trước khi sử dụng.
- Sau khi Flash đã Unlock, cờ ```FLASH_CR_PER``` được set lên 1 để cho phép xóa page.
- Gán dịa chỉ của Page cần xóa vào thanh ghi ```AR```.
- Set bit ```FLASH_CR_STRT``` lên 1 để bắt đầu quá trình xóa Page.
- Kiểm tra cờ ```FLASH_SR_BSY``` được reset về 0 khi quá trình xóa hoàn tất.
- Reset cờ ```FLASH_CR_PER``` để vô hiệu hóa tính năng xóa Page
- Lock Flash

Các hàm LOCK, UNLOCK Flash:
- ```void FLASH_Unlock(void)```: Hàm này Unlock cho tất cả vùng nhớ trong Flash.
- ```void FLASH_UnlockBank1(void)```: Hàm này chỉ Unlock cho Bank đầu tiên. Vì SMT32F103C8T6 chỉ có 1 Bank duy nhất nên chức năng tương tự hàm trên.
- ```void FLASH_UnlockBank2(void)```: Unlock cho Bank thứ 2.
- ```void FLASH_Lock(void)```: Lock bộ điều khiển xóa Flash cho toàn bộ vùng nhớ Flash.
- ```void FLASH_LockBank1(void)``` và ```void FLASH_LockBank2(void)```: Lock bộ điều khiển xóa Flash cho Bank 1 hoặc 2.

Các Hàm xóa Flash:
- ```FLASH_Status FLASH_EraseAllBank1Pages(void)```: Xóa tất cả các Page trong Bank 1 của Flash. 
- ```FLASH_Status FLASH_EraseAllBank2Pages(void)```: Xóa tất cả các Page trong Bank 2 của Flash. 
- ```FLASH_Status FLASH_EraseAllPages(void)```: Xóa toàn bộ Flash.
- ```FLASH_Status FLASH_ErasePage(uint32_t Page_Address)```: Xóa 1 page cụ thể trong Flash, cụ thể là Page bắt đầu bằng địa chỉ Page_Address.


Hàm xóa bằng thanh ghi
```cpp
void Flash_Erase1(uint32_t pageAddr)
{
	FLASH_Unlock();
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY));
	
	FLASH->CR |= FLASH_CR_PER; //Bat xoa Page
	FLASH->AR = pageAddr; // Them dia chi page vao thanh ghi AR
	FLASH->CR |= FLASH_CR_STRT; //Bat dau xoa page
	
	while(FLASH->SR && FLASH_SR_BSY); //Doi den xoa hoan tat
	
	FLASH->CR &=(~FLASH_CR_PER); // Tat xoa page
	
	FLASH_Lock(); //Lock flash
}
```

Hàm xóa sử dụng hàm có sẵn trong thư viện Flash
```cpp
void Flash_Erase(uint32_t pageAddr){
	FLASH_Unlock();//Un lock tat ca vung nho trong Flash

	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY));//FLASH_FLAG_BSY = 1, khi Flash dang ban ghi/doc

	FLASH_ErasePage(pageAddr);//Xoa Page theo dia thi duoc truyen vao

	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);//Cho den khi xoa hoan tat

	FLASH_Lock();//Lock toan bo Flash
}
```

**Ghi giá trị vào Flash**

![FlashWrite](https://i.imgur.com/BMPjISr.png)

- Tương tự quá trình xóa, đầu tiên Cờ ```FLASH_CR_LOCK``` được kiểm tra.
- Sau khi xác nhận đã Unlock, CỜ ```FLASH_CR_PG``` được set lên 1 để cho phép ghi.
- Ghi dữ liệu vào địa chỉ Flash sử dụng con trỏ.
- Kiểm tra cờ ```FLASH_CR_BSY``` để đợi quá trình ghi hoàn tất.
- Lock Flash.

- ```FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data)```:  Ghi dữ liệu vào vùng nhớ Address với kích thước mỗi 2 byte (Halfword).
- ```FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data)```: Ghi dữ liệu vào vùng nhớ Address với kích thước mỗi 4 byte (Word).
- ```FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG)```: hàm này trả về trạng thái của Flag. Ở bài này ta sẽ dùng hàm này để kiểm tra cờ.
- ```FLASH_FLAG_BSY```: Cờ này báo hiệu rằng Flash đang bận (Xóa/Ghi) nếu được set lên 1. 

```cpp
void Flash_WriteHalfWord(uint32_t memAddr, uint16_t value){
	FLASH_Unlock();
	
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
	
	FLASH_ProgramHalfWord(memAddr, value);
	
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
	
	FLASH_Lock();
}
```

### Bootloader
Quá trình cập nhật phần mềm từ xa, không thể yêu cầu người dùng mang thiết bị đến cửa hàng để nạp lại chương trình =>> Dùng phương pháp nạp phần mềm từ xa qua Wifi, Bluetooth.

![](https://i.imgur.com/yi125KJ.png)

![](https://i.imgur.com/u0CWU5Z.png)

Chương trình nạp từ UART được lưu ở vùng nhớ khác, MCU không thể tự nhận biết và tìm đến để thực thi được.

Vì thế, cần 1 chương trình lưu tại ```0x08000000``` để MCU có thể nhảy đến chương trình đó. Chương trình đặt tại ```0x08000000``` được gọi là Bootloader

Bootloader là chương trình chạy đầu tiên khi khởi động, thường gồm 2 loại:
- Bootloader do nhà sản xuất cung cấp.
- Bootloader do người dùng tự viết.

Trên vi điều khiển có hai tín hiệu dùng để điều khiển địa chỉ để sử dụng Bootloader
![](https://i.imgur.com/DH8yhiA.png)

**Quá trình từ khi cấp nguồn hoặc nhấn reset đến khi thực thi hàm ```main()```**
- MCU đọc giá trị BOOT0 và BOOT1 để xác định vị trí bắt đầu đọc dữ liệu trong bộ nhớ.
- Địa chỉ khởi đầu của vùng nhớ được lưu vào thanh ghi PC (Program Counter) để thực hiện đọc lệnh.
- Giá trị tại ô nhớ đầu tiên được lấy để khởi tạo MSP (Main Stack Pointer).
- Thanh ghi PC chuyển đến ô nhớ tiếp theo - **ô nhớ thứ 2** , nơi chứa địa chỉ của Reset_Handler - ngắt có độ ưu tiên cao nhất.
- Chương trình nhảy đến **Reset_Handler** để thực thi các nhiệm vụ sau:
    - Khởi tạo hệ thống.
    - Sao chép dữ liệu (biến) từ FLASH sang RAM.
    - Gọi hàm main().

![](https://i.imgur.com/Cf6wh3J.png)

- Sau khi reset, vi điều khiển nhảy đến Reset_Handler() mặc định, từ đó thực thi hàm main() của chương trình Bootloader.
- Chương trình Bootloader sẽ khởi tạo lại MSP (Main Stack Pointer) bằng cách lấy dữ liệu từ ô nhớ đầu tiên của vùng lưu trữ chương trình Application.
- Gọi hàm Bootloader(), hàm này sẽ thiết lập thanh ghi ```SCB_VTOR``` theo địa chỉ của Application cần nhảy đến, tức là: ```SCB → VTOR``` = Firmware address(địa chỉ của chương trình chính).
- Vi điều khiển tiếp tục nhảy đến ô nhớ tiếp theo, chính là Reset_Handler của chương trình Application.
- Tại đây, Firmware mới bắt đầu chạy. 

Do vi xử lý đã nhận diện Reset_Handler() tại địa chỉ mới, nên nếu tiếp tục nhấn reset, hệ thống vẫn hoạt động trong Application mà không quay lại Bootloader.

**Hàm Boot**
```cpp
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
```

</details>


## Bài 12: Lý thuyết CAN - Controller Area Network

<details><summary>Xem</summary>  

Giao thức CAN (Controller Area Network) là một giao thức truyền thông được sử dụng rộng rãi trong các hệ thống nhúng, đặc biệt là trong lĩnh vực ô tô và các ứng dụng công nghiệp. CAN cho phép các vi điều khiển và các thiết bị khác nhau giao tiếp với nhau mà không cần có máy tính chủ.

So sánh với các chuẩn giao tiếp khác
- SPI: Khi nhiều Slave thì yêu cầu phần cứng cao hơn vì phải thêm các chân CS
- UART: thì chỉ có thể giao tiếp giữa hai phần cứng với nhau
- I2C: Mỗi lần truyền chỉ truyền 8 bit và truyền các bit địa chỉ. Ngoài ra I2C chỉ có các xác nhận ACK để đảm bảo đã truyền thành công chứ không có cơ chế phát hiện lỗi.

Vì thế, chúng ta sẽ sử dụng **CAN** với các ưu điểm sau:
- Cho phép nhiều hệ thống điều khiển (ECU) giao tiếp với nhau trên một bus truyền thông chung.
- Giảm số lượng dây dẫn, giúp tiết kiệm chi phí và tăng độ tin cậy của hệ thống.
- Không yêu cầu máy tính chủ (master) để điều phối các thiết bị. Các thiết bị trên bus CAN có thể truyền dữ liệu bất cứ lúc nào, với cơ chế arbitrage tự động để tránh xung đột dữ liệu.
- Độ tin cậy cao, đảm bảo việc phát hiện lỗi tự động thông qua cơ chế kiểm tra và sửa lỗi. Điều này cực kỳ quan trọng trong các hệ thống ô tô yêu cầu an toàn cao.

### Kiến trúc mạng CAN

![](https://i.imgur.com/PGz5JdP.png)

Giao thức CAN sử dụng topology bus để kết nối các thiết bị với nhau, nghĩa là tất cả các thiết bị (node) đều được kết nối song song vào một cặp dây truyền thông chung được gọi là CAN bus.

CAN bus này gồm hai dây tín hiệu chính, là:
- CAN_H (CAN High): Dây tín hiệu cao.
- CAN_L (CAN Low): Dây tín hiệu thấp.
tạo thành 1 Bus, các thiết bị được nối chung trên 2 dây này và gọi là 1 **node** trong mạng.

Các tín hiệu truyền qua bus CAN là **tín hiệu vi sai** (differential signaling), nghĩa là thông tin được mã hóa dựa trên sự **chênh lệch điện áp giữa hai dây CAN_H và CAN_L**. Điều này giúp mạng CAN chống lại nhiễu từ môi trường và duy trì tín hiệu ổn định trên đường truyền.


**Trạng thái 0 - dominant và 1 - recessive**  
Bus CAN định nghĩa hai trạng thái điện áp là “dominant” và “recessive”, tương ứng với hai trạng thái bit là 0 và 1. Hai trạng thái này sẽ được xử lý bởi Transceiver của Node. Có 2 loại CAN tương ứng với các giá trị điện áp khác nhau là **CAN low speed và CAN high speed**.

![](https://i.imgur.com/mwQxkEw.png)

- Nếu độ chênh lệch điện áp giữa CAN_H và CAN_L > 2.5V thì là bit dominant
- Nếu độ chênh lệch < 0.5V thì đó là bit recessive

**Bit dominant sẽ có độ ưu tiên cao hơn bit recessive**

### Cơ chế giảm nhiễu của CAN

Khi có nhiễu xuất hiện, sẽ làm giá trị vi sai lệch và làm thay đổi bit. Vì thế phải có cơ chế giảm nhiễu.

**Xoắn hai dây CAN_H và CAN_L lại với nhau**

![](https://i.imgur.com/4ptJ8dU.png)

- Giảm thiểu nhiễu từ môi trường bên ngoài: Khi có nhiễu tác động là vào hai đoạn dây, khi bị xoắn lại với nhau thì cả hai dây sẽ chịu tác động với giá trị nhiễu là như nhau. Khi đó khi tính độ chệnh lệch điện áp, cả hai dây cùng tăng hoặc giảm áp do nhiễu, vì thế giá trị tính được sẽ không thay đổi.
- Giảm thiểu nhiễu xuyên âm: Việc xoắn đôi các dây giúp giảm hiện tượng này bằng cách phân tán nhiễu xuyên âm ra khắp chiều dài của cáp.

**Gắn Termination resistor (Điện trở kết cuối) 120Ohm ở hai đầu đoạn dây**  
- Mỗi đầu của bus CAN cần một điện trở kết cuối với giá trị 120Ω để ngăn chặn hiện tượng phản xạ tín hiệu. Nếu không có điện trở này, tín hiệu có thể bị phản xạ lại từ các đầu cuối mở, gây ra nhiễu và làm hỏng dữ liệu.
- Điện trở giá trị 120Ω  bởi vì để hấp thụ toàn bộ sóng phản xạ thì điện trở kết cuối phải bằng giá trị của trở kháng đặc tính của đường dây, giá trị này phụ thuộc vào các yếu tố vật lý như chiều dài, chất liệu, tiết diện của đường dây.


### Cấu trúc của CAN node

![](https://i.imgur.com/mkB9WHA.png)

Mạng CAN được hình thành bởi một nhóm các node được kết nối song song với nhau thông qua topology bus và không cần máy tính chủ.  
Một Node sẽ bao gồm các thành phần:

- **Bộ điều khiển CAN (CAN Controller)**: Đây là thành phần chính trong node CAN, có nhiệm vụ xử lý toàn bộ giao tiếp CAN.
    - Gửi và nhận thông điệp CAN.
    - Điều khiển truy cập vào bus CAN (arbitration).
    - Phát hiện và xử lý các lỗi truyền thông CAN.
    - Kiểm soát việc truyền lại thông điệp khi gặp lỗi.
    - Cung cấp giao diện giữa các vi điều khiển và bus CAN.
- **Transceiver CAN (CAN Transceiver)**: Chuyển đổi tín hiệu số từ bộ điều khiển CAN thành tín hiệu điện áp dạng differential (CAN_H và CAN_L) để gửi lên bus CAN và ngược lại
    - Đảm bảo tín hiệu truyền và nhận trên bus CAN có độ chính xác và tốc độ cao.
- **Vi điều khiển (Microcontroller)**: là thành phần trung tâm điều khiển hoạt động của node CAN.
    - Đọc và xử lý thông điệp CAN được nhận.
    - Tạo ra thông điệp CAN để truyền đi.
    - Quản lý các khung dữ liệu, bit arbitration và quá trình xử lý lỗi.
    - Điều khiển hành vi của node (ví dụ: bật/tắt node, reset node khi gặp lỗi bus-off).




### Nguyên tắt hoạt động

- Thông điệp dữ liệu được truyền từ bất kỳ nút nào trên bus CAN **không chứa địa chỉ** của nút truyền hoặc của bất kỳ nút nhận dự kiến nào.
- Thay vào đó, nội dung của thông điệp được gắn nhãn bởi một số nhận dạng **(ID)** là duy nhất trên toàn mạng. 
- Tất cả các nút khác trên mạng đều nhận được thông điệp và mỗi nút thực hiện kiểm tra sự chấp nhận trên mã ID để xác định xem thông điệp có liên quan đến nút đó hay không. Nếu thông điệp có liên quan, nó sẽ được xử lý, nếu không thì nó bị bỏ qua


**Xảy ra trường hợp nếu nhiều Node muốn gửi dữ liệu cùng lúc thì cần phải Tranh chấp quyền gửi (Arbitration)**  

Một đặc điểm quan trọng của mạng CAN là khả năng giải quyết tranh chấp quyền gửi dữ liệu giữa các node. Nếu có nhiều node cùng muốn gửi dữ liệu lên bus cùng một lúc, cơ chế arbitration sẽ được thực hiện:
- Mỗi thông điệp CAN có một ID ưu tiên. Node nào có thông điệp với ID ưu tiên thấp hơn (tức có độ ưu tiên cao hơn) sẽ chiếm quyền truy cập bus và gửi thông điệp trước.
- Những node khác có ID ưu tiên cao hơn sẽ tự động dừng lại và chờ lượt tiếp theo để gửi thông điệp.
- Quá trình arbitration diễn ra mà không gây mất dữ liệu hay làm gián đoạn các thiết bị khác, vì thế mạng CAN là một hệ thống non-destructive (không gây mất dữ liệu).

Khi phân xử, bit ID có giá trị **0 sẽ được ưu tiên hơn**.
Node sẽ dựa vào giá trị điện áp trên bus để quyết định.

![](https://i.imgur.com/p1kBRjn.png)

Ở ví dụ trên, nếu ba Node cùng muốn gửi dữ liệu thì sẽ cần phải tranh chấp.

Tại bit 1 tới bit 6 thì các Node gửi cùng giá trị nên được phép gửi tiếp. Nhưng tại bit thứ 7, Node 2 gửi bit recessive nhưng Node 1 và 3 gửi bit dominant, theo nguyên tắt bit 0 ưu tiên hơn thì Node hai phải vào trạng  thái Listen Only để đọc và chờ đến khi hoàn tất thông điệp này thì mới tiếp tục tranh chấp và gửi lại.

Để nhận bit được quyền ưu tiên, Khi Node trên điện áp lên hai dây CAN_H và CAN_L thì sẽ nhận lại giá trị được truyền đi trên dây. Nhưng khi Node truyền đi bit recessive tức CAN_H 1.75V và CAN_L 3.25V nhưng nhận lại giá trị dominant CAN_H 4V và CAN_L 1V thì Node đó biết đã có Node có thông điệp được ưu tiên hơn đang truyền và mình sẽ vào trạng thái chờ.

### Các loại Frame truyền trong CAN

Có 4 loại Frame là Data Frame, Remote Frame, Error Frame và Overload Frame.


#### Data Frame (Truyền Data)
Data Frame là khung phổ biến nhất được sử dụng trong giao thức CAN (Controller Area Network). Nó được sử dụng để truyền dữ liệu thực tế giữa các node trên mạng CAN.   
Data Frame bao gồm các thông tin về địa chỉ của node gửi và nhận, kích thước dữ liệu, và chính dữ liệu cần truyền. Frame này giúp đảm bảo rằng dữ liệu sẽ được truyền đúng cách và bảo vệ khỏi lỗi bằng cách sử dụng mã kiểm tra CRC.

![](https://i.imgur.com/MhhfK9f.png)

- **Start of Frame (SOF)**: 1 bit để báo hiệu bắt đầu của khung truyền.
- **Arbitration Field**: Là nhân tố được so sánh trong cơ chế phân quyền. 
    - Chứa ID của thông điệp cần gửi, có thể là 11-bit (Standard Frame) hoặc 29-bit (Extend Frame), giúp phân biệt giữa các node. 
    - Trường này cũng chứa bit RTR để xác định kiểu khung là Data Frame (Dominant) hay Remote Frame(Recessive).
- **Control Field**: Chứa DLC (Data Length Code), chỉ ra số byte dữ liệu trong khung, từ 0 đến 8 byte.
    - Bit r: là bit bảo tồn và không có chức năng gì cả
    - Bit IDE để nhận định bit thuộc CAN tiêu chuẩn (Dominant) hoạc CAN mở rộng (Recessive)
- **Data Field**: Chứa dữ liệu thực tế cần truyền. Độ dài từ 0 đến 8 byte tùy thuộc vào giá trị của DLC.
- **CRC Field (Cyclic Redundancy Check)**: Dùng để phát hiện lỗi trong quá trình truyền thông qua mạng.
    - CRC delimetier: Để phân cách trường CRC với ACK
- **ACK Field (Acknowledgment)**: Node nhận sẽ gửi tín hiệu ACK để xác nhận rằng dữ liệu đã được nhận thành công.
    - ACK slot: Bit recessive, khi nhận thành công thì bên nhận sẽ chờ đến thời gian gửi ACK Slot để kéo đường dây xuống bit dominant (ưu tiên hơn), khi này bên gửi sẽ đọc lại Bus bit 0 để xác nhận đã nhận thành công. Nếu vẫn bằng recessive thì vẫn chưa nhận được
    - ACK delimetier để phân cách ACK với EOF
- **End of Frame (EOF)**: 7 bit kết thúc khung.


#### Remote Frame (Yêu cầu Data)

Remote Frame được sử dụng khi một node trên mạng CAN yêu cầu dữ liệu từ một node khác. Thay vì chứa dữ liệu thực, Remote Frame chứa ID của node cần yêu cầu, cùng với bit điều khiển RTR (Remote Transmission Request).  
Remote Frame thường được sử dụng trong các hệ thống mà một node có thể yêu cầu thông tin từ một node khác mà không có dữ liệu nào được truyền ngay lập tức.  
**Node nhận yêu cầu sẽ trả lời bằng một Data Frame chứa dữ liệu cần thiết với ID giống với ID của Remote Frame trước đó.**

![](https://i.imgur.com/sLMffac.png)

Cấu trúc là tương tự với Data Frame nhưng tại Arbitration Field có bit RTR là bit Recessive để xác định đây là gói tin Remote. Vì thế khi có hai gói tin Data và Remote được truyền đi với cùng ID thì gói tin Data có ưu tiên cao hơn vì bit RTR là dominant.  
Ngoài ra, gói tin Remote **không có Data Field**.
 
Ý nghĩa các trường khác đều giống với Data Frame.
- Start of Frame (SOF): 1 bit báo hiệu bắt đầu của khung.
- Arbitration Field: Bao gồm định danh (11-bit hoặc 29-bit Identifier) và bit RTR được đặt thành 1 để báo đây là Remote Frame.
- Control Field: Chứa DLC (Data Length Code), chỉ ra số byte dữ liệu mà Data Frame sẽ chứa.
- CRC Field (Cyclic Redundancy Check): Được sử dụng để kiểm tra tính toàn vẹn của khung.
- ACK Field (Acknowledgment): Sử dụng để xác nhận rằng khung đã được nhận.
- End of Frame (EOF): 7 bit kết thúc khung.

#### Error Frame

- Error Frame được sử dụng khi một node phát hiện ra lỗi trong quá trình truyền dữ liệu. Nó được gửi để thông báo cho các node khác rằng có lỗi đã xảy ra trên bus. Bất kỳ node nào phát hiện ra lỗi đều có thể gửi Error Frame.
- Error Frame có vai trò rất quan trọng trong việc duy trì độ tin cậy của mạng CAN. Khi một lỗi xảy ra, Error Frame sẽ báo hiệu để các node khác biết rằng thông điệp vừa được truyền không hợp lệ và cần được truyền lại.

**Các lỗi xuất hiện trong CAN**

- **Bit Error**
    - Bit Error xảy ra khi một node gửi một bit (dominant hoặc recessive) lên bus và nhận lại một bit khác với giá trị mong đợi. 
    - Trong mạng CAN, mỗi node không chỉ gửi dữ liệu mà còn tự lắng nghe các tín hiệu trên bus để kiểm tra sự đồng bộ.
    - Nguyên nhân:
        - Nếu một node gửi một bit recessive (1) nhưng nhận lại bit dominant (0) từ bus, node này sẽ phát hiện ra lỗi.
        - Điều này có thể xảy ra khi một node khác có ưu tiên cao hơn trên bus đang truyền dữ liệu, hoặc do tín hiệu bị nhiễu.
- **Stuff Error**
    - Stuff Error xảy ra khi có hơn 5 bit liên tiếp cùng giá trị (tất cả đều là 0 hoặc tất cả đều là 1) trên bus CAN. Điều này vi phạm quy tắc **bit stuffing** của giao thức CAN.
    - Quy tắc bit stuffing: Trong mạng CAN, sau mỗi chuỗi 5 bit giống nhau liên tiếp, một bit ngược giá trị (ngược với giá trị của các bit trước đó) phải được thêm vào để đảm bảo tính đồng bộ và tránh nhiễu tín hiệu. Nếu quy tắc này bị vi phạm, lỗi sẽ xảy ra.
    - Nguyên nhân: Vi phạm quy tắc bit stuffing có thể do lỗi trong quá trình truyền tín hiệu hoặc do thiết bị không tuân theo quy chuẩn CAN.
- **CRC Error**
    - CRC Error xảy ra khi có sai lệch trong quá trình kiểm tra CRC (Cyclic Redundancy Check), được sử dụng để phát hiện lỗi trong dữ liệu truyền qua bus.
    - Cơ chế CRC:
        - Trong mỗi khung dữ liệu CAN, có một CRC Field được sử dụng để kiểm tra tính toàn vẹn của dữ liệu. Trường này chứa giá trị CRC, được tính toán dựa trên nội dung của thông điệp.
        - Node nhận sẽ tính toán lại giá trị CRC của dữ liệu nhận được và so sánh với CRC trong trường CRC Field. Nếu hai giá trị này không khớp, một CRC error sẽ được phát hiện.
    - Nguyên nhân: Lỗi CRC có thể xảy ra do nhiễu tín hiệu trong quá trình truyền dữ liệu hoặc do lỗi phần cứng trong node gửi hoặc nhận.
- **Form Error**
    - Form Error xảy ra khi cấu trúc khung dữ liệu không tuân theo quy chuẩn của giao thức CAN. Mỗi khung dữ liệu trong CAN phải tuân theo một cấu trúc định sẵn, bao gồm Start of Frame (SOF), Arbitration Field, Control Field, Data Field, CRC Field, ACK Field, và End of Frame (EOF).
    - Nguyên nhân: Nếu một node nhận thấy có lỗi trong định dạng của bất kỳ trường nào trong khung dữ liệu, đặc biệt là các bit trong EOF hoặc ACK Field, nó sẽ phát hiện Form Error.
- **Acknowledgment Error**
    - Acknowledgment Error (ACK Error) xảy ra khi node gửi thông điệp lên bus mà không nhận được bit ACK từ bất kỳ node nào trên mạng.
    - Cơ chế ACK trong CAN:
        - Khi một node gửi thành công một khung dữ liệu, các node nhận phải gửi một bit ACK dominant (0) để xác nhận rằng dữ liệu đã được nhận chính xác.
        - Nếu không có node nào gửi bit ACK, node gửi sẽ phát hiện ACK Error và phải truyền lại thông điệp.
    - Nguyên nhân:
        - Thiết bị nhận có thể không hoạt động đúng cách hoặc không kết nối đúng vào bus CAN.
        - Tín hiệu ACK có thể bị nhiễu hoặc lỗi phần cứng.
        - Id của gói tin không trùng phù hợp với bất kỳ node nào trong mạng.


- Error Frame gồm hai phần:
    - **Error Flag** là chuỗi từ 6 đến 12 bit dominant, báo hiệu trạng thái lỗi.   
    - **Error Delimiter** là chuỗi 8 bit recessive, kết thúc Error Frame.
![](https://i.imgur.com/isTCb1L.png)

Khi phát hiện lỗi, các node trong mạng CAN sẽ tự động chuyển đổi giữa ba trạng thái lỗi để đảm bảo hệ thống hoạt động ổn định và không gây gián đoạn cho bus.

Các trạng thái lỗi được xác định bởi **Error Flag**:
- **Error Active**: Trong trạng thái Error Active, **node vẫn có khả năng tham gia đầy đủ vào quá trình truyền thông và có thể phát hiện lỗi**. Nếu node phát hiện lỗi, nó sẽ gửi một Error Frame để thông báo cho các node khác trên bus rằng đã xảy ra lỗi một **Error Flag gồm 6 bit dominant liên tiếp**.
    - Ví dụ: Một cảm biến nhiệt độ trong hệ thống công nghiệp phát hiện một Bit Error do tín hiệu CAN bị nhiễu. Node phát hiện lỗi này sẽ phát ra một Active Error Frame với 6 bit dominant. Các node khác trên mạng sẽ tạm ngừng giao tiếp và xử lý lại quá trình truyền thông.
- **Error Passive**
    - Nếu một node **phát hiện quá nhiều lỗi**, nó sẽ chuyển sang trạng thái Error Passive. Trong trạng thái này, **node vẫn có thể tham gia truyền thông**, nhưng nếu phát hiện lỗi, nó sẽ không gửi Error Frame mạnh mẽ như trong trạng thái Error Active. Điều này giúp tránh gây gián đoạn lớn cho bus khi node gặp sự cố thường xuyên. **Phát ra Error Flag 12 bit gồm 6 bit dominant và 6 bit recessive**
- Trong trạng thái Error Passive, node vẫn có thể nhận và gửi thông điệp nhưng sẽ hạn chế việc can thiệp vào quá trình truyền thông của các node khác. Node chỉ gửi Error Frame yếu hơn để thông báo lỗi, và không ảnh hưởng đến quá trình truyền thông của các node khác.
- **Bus Off**: Khi một node **gặp quá nhiều lỗi nghiêm trọng**, nó sẽ chuyển sang trạng thái Bus Off. Trong trạng thái này, node sẽ **hoàn toàn ngắt kết nối khỏi bus CAN và không thể tham gia vào quá trình truyền hay nhận dữ liệu**. Node chỉ có thể **được kết nối lại vào bus sau khi được khởi động lại (restart) hoặc reset bởi phần mềm**.
    - Bus Off là trạng thái an toàn, ngăn chặn một node bị lỗi nặng gây ra sự cố nghiêm trọng cho toàn bộ hệ thống CAN.

**Chuyển đổi giữa các trạng thái Active và Passive**
- **Error Counter**: Các node CAN quản lý một Error Counter để theo dõi số lượng lỗi mà chúng gặp phải. Mỗi khi một lỗi được phát hiện, giá trị của Error Counter sẽ tăng lên.
- Khi Error Counter vượt qua **ngưỡng 127**, node sẽ chuyển từ trạng thái **Active Error sang Passive Error**. Trong trạng thái này, node sẽ phát ra Passive Error Frame nếu phát hiện lỗi.
- Nếu Error Counter tiếp tục tăng và vượt **ngưỡng 255**, node sẽ chuyển sang trạng thái **Bus Off**, tức là node sẽ bị loại khỏi mạng CAN và không thể giao tiếp thêm cho đến khi được reset lại.



#### Overload Frame

Overload Frame là một loại khung đặc biệt trong giao thức CAN được sử dụng để trì hoãn việc truyền dữ liệu khi một node trong mạng CAN cần thêm thời gian để xử lý.  
Khung này không chứa dữ liệu, mà chỉ báo hiệu rằng một node đang quá tải và cần thời gian trước khi tiếp tục giao tiếp.  
Mục tiêu của Overload Frame là ngăn không cho các khung khác được truyền quá nhanh, giúp node bị quá tải có đủ thời gian để xử lý các khung trước đó.  

Overload Frame không phải do người dùng phát ra, mà được tự động phát ra bởi phần cứng CAN khi cần thiết. Node CAN sẽ phát ra Overload Frame khi một trong các điều kiện sau xảy ra:
- Node không thể xử lý tiếp dữ liệu do buffer đã đầy hoặc cần thêm thời gian xử lý dữ liệu.
- Node không thể nhận khung mới do có quá trình xử lý nội bộ cần hoàn thành trước.

![](https://i.imgur.com/sI9oA2u.png)

Cấu trúc của Overload Frame:
- Overload Flag: 6 bit dominant (bit 0) để báo hiệu trạng thái quá tải.
- Overload Delimiter: 8 bit recessive (bit 1), để phân tách khung quá tải với các khung khác và báo hiệu kết thúc Overload Frame.


### Tốc độ truyền và giới hạn vật lý của CAN

Giao thức CAN được thiết kế để hoạt động hiệu quả trong các hệ thống nhúng với khả năng truyền thông dữ liệu tin cậy và độ trễ thấp.  
Một số yếu tố quan trọng ảnh hưởng đến hiệu suất của mạng CAN bao gồm **tốc độ truyền (baud rate), chiều dài của bus**. Các yếu tố này liên quan chặt chẽ với nhau, ảnh hưởng đến khả năng truyền tín hiệu trên bus CAN.

#### Tốc độ baud của CAN liên hệ tới chiều dài

**Tốc độ baud** là tốc độ truyền dữ liệu trên bus CAN, thường được đo bằng kbps hoặc Mbps. Tốc độ baud quyết định tốc độ truyền thông giữa các thiết bị trên mạng và phụ thuộc vào khả năng xử lý của hệ thống cũng như chiều dài của bus.

- Mạng CAN hỗ trợ dải tốc độ baud từ 10 kbps đến 1 Mbps.
    - 10 kbps: Tốc độ thấp nhất, thường được sử dụng cho các hệ thống có yêu cầu truyền thông chậm, nhưng cần truyền xa.
    - 1 Mbps: Tốc độ cao nhất, thường được sử dụng trong các ứng dụng yêu cầu truyền thông nhanh, chẳng hạn như trong hệ thống ô tô hoặc robot.

**Tốc độ càng cao, độ trễ càng thấp, chiều dài tối đa đường dây giảm**

Giải thích:  
- Tốc độ truyền càng cao, chiều dài bus càng ngắn: Điều này do thời gian lan truyền tín hiệu trên dây dẫn cần phải nhỏ hơn một khoảng thời gian nhất định để đảm bảo tất cả các node trên bus có thể nhận được tín hiệu đồng bộ.
- Khi tốc độ baud tăng lên, thời gian bit ngắn lại, nghĩa là tín hiệu phải đến các node nhận nhanh hơn. Do đó, chiều dài tối đa của bus phải giảm để đảm bảo thời gian lan truyền tín hiệu phù hợp với tốc độ baud.

#### Cài đặt tốc độ truyền

Để có thể cài đặt tốc độ truyền, ta sẽ chỉnh sửa thời gian mà một bit được tồn tại trên bus. Được chia thành 4 giai đoạn

![](https://i.imgur.com/kj27OHX.png)

- **Sync Segment:** là đoạn đầu tiên của mỗi bit và có độ dài cố định là **1 time quanta (TQ)**. Đoạn này giúp **đồng bộ hóa tất cả các node trên bus**. Nó đảm bảo rằng tất cả các node đều nhận biết sự bắt đầu của một bit tại cùng một thời điểm. Node nhận sẽ phát hiện ra cạnh thay đổi (rising edge hoặc falling edge) của tín hiệu CAN tại đoạn này để điều chỉnh thời gian của chính nó, đảm bảo đồng bộ với các node khác trên bus.
- **Propagation Segment**: Đoạn này bù đắp thời gian cần thiết để tín hiệu lan truyền qua bus CAN từ node gửi đến node nhận. Tín hiệu cần thời gian để di chuyển từ một node đến một node khác, và thời gian này **phụ thuộc vào chiều dài của bus và tốc độ truyền**. Propagation Segment được cấu hình sao cho nó bao gồm độ trễ lan truyền và thời gian trễ xử lý của cả phần cứng CAN. **Node nhận càng xa Node gửi thì Propagation Segment càng lớn**.

- **Phase Segment 1 (PS1) và Phase Segment 2 (PS2)**: Hai phân đoạn này được sử dụng để đồng bộ hóa tín hiệu và bù đắp cho các sai lệch về thời gian hoặc độ trễ nhỏ trong quá trình truyền để quyết định **Sample Point**. 
    - **PS1** là đoạn thời gian trước điểm lấy mẫu. Đoạn này cho phép node điều chỉnh thời gian đọc tín hiệu để đồng bộ với tín hiệu thực tế trên bus, đảm bảo Sample Point nằm trong khoảng tín hiệu ổn định.
    - **PS2** là đoạn thời gian sau điểm lấy mẫu. Nó có thể được kéo dài hoặc thu ngắn để bù trừ sự sai lệch nhỏ giữa các node, giữ cho tất cả node đồng bộ với nhau. Đây là đoạn để kết thúc 1 bit và sẵn sàng để truyền/nhận bit tiếp theo.

**Điểm lấy mẫu (Sample Point):**
Là thời điểm mà tín hiệu trên bus CAN được đọc để xác định giá trị của một bit (dominant hoặc recessive). Mẫu thường được lấy ở vị trí cuối mỗi bit để đảm bảo tín hiệu đã ổn định.
- Vị trí của điểm mẫu (sample point) được tính toán dựa trên tỷ lệ phần trăm trong một khoảng thời gian bit. Điểm mẫu lý tưởng thường nằm ở khoảng **75% đến 90%** thời gian của một bit.

Vậy, tổng thời gian của một bit trong giao thức CAN là tổng của các phân đoạn thời gian (Sync Segment, Propagation Segment, PS1, và PS2), tính bằng TQ:  
```Bit Time = Sync Segment + Propagation Segment + PS1 + PS2 (TQ)```  
Và tốc độ baud (bit rate) được tính như sau:  
```Tốc độ baud = 1/Bit Time (bps)```

### Cơ chế bộ lọc

Trong CAN, các node có thể nhận rất nhiều thông điệp, nhưng không phải tất cả thông điệp đều liên quan đến tất cả các node. Bộ lọc CAN cho phép các node lựa chọn và chỉ nhận những thông điệp cần thiết, dựa trên **ID** của thông điệp hoặc các tiêu chí khác.

**Vai trò của bộ lọc CAN:**
- Lựa chọn thông điệp: Bộ lọc CAN giúp các node lọc ra những thông điệp cần thiết và bỏ qua những thông điệp không liên quan. Điều này giúp giảm tải cho vi điều khiển, vì nó chỉ xử lý những dữ liệu mà nó cần.
- Giảm băng thông: Bằng cách chỉ nhận những thông điệp có ID cụ thể, node có thể giảm khối lượng dữ liệu cần xử lý, giúp mạng hoạt động hiệu quả hơn.

Bộ lọc CAN hoạt động dựa trên hai thành phần chính:
- **Mask (Mặt nạ):** là một dãy bit mà các bit có giá trị **1 sẽ được kiểm tra**, còn các bit có giá trị **0 sẽ bị bỏ qua**. Điều này cho phép chỉ định cụ thể những phần của ID thông điệp mà node sẽ quan tâm, trong khi bỏ qua các phần không quan trọng. Mask giúp xác định phạm vi ID mà node quan tâm.

- **Filter (Bộ lọc):** Các bit được phép kiểm tra thông qua Mask sẽ được so sánh với Filter. Nếu ID thông điệp trùng khớp với giá trị của bộ lọc (sau khi áp dụng mask), thông điệp sẽ được chấp nhận. Nếu không trùng khớp, thông điệp sẽ bị bỏ qua, node sẽ không xử lý nó.

#### Ví dụ:
Một Node cần nhận thông điệp có ID trong khoảng từ 0x100 (001 0000 0000) đến 0x1FF(001 1111 1111) và không quan tâm đến các thông điệp có ID nằm ngoài phạm vi này.  
Để đạt được điều này, chúng ta có thể cấu hình bộ lọc CAN như sau:  
- Mask 0x700 (111 0000 0000) có nghĩa là chỉ có 3 bit đầu tiên của ID thông điệp sẽ được so sánh với filter. Các bit khác (bit từ 8 trở xuống) sẽ không được kiểm tra. Các gói tin sau khi qua mặt nạ sẽ là xxx 0000 0000
- Filter 0x100 (001 0000 0000), sau đó dùng phép trừ, nếu bằng 0 có nghĩa là node sẽ chấp nhận những thông điệp có 3 bit đầu tiên là 001, tức là thông điệp có ID nằm trong khoảng từ 0x100 đến 0x1FF.

### Các phiên bản của CAN

Giao thức CAN đã phát triển qua nhiều phiên bản để đáp ứng nhu cầu ngày càng cao trong các ứng dụng nhúng, đặc biệt là trong ngành công nghiệp ô tô và tự động hóa. Các phiên bản mở rộng của CAN bao gồm CAN 2.0A, CAN 2.0B, và CAN FD (Flexible Data-rate). Mỗi phiên bản có những cải tiến để hỗ trợ các yêu cầu khác nhau về độ ưu tiên, dung lượng dữ liệu và tốc độ truyền tải.

#### CAN 2.0A
- 11-bit identifier (ID): Phiên bản CAN 2.0A sử dụng định dạng ID dài 11 bit. Đây là một trong những yếu tố quan trọng để xác định mức độ ưu tiên của thông điệp trong quá trình truyền dữ liệu. Với 11 bit, có thể biểu diễn 2^11 = 2048 ID khác nhau.
- Khả năng ưu tiên: Mỗi thông điệp trong mạng CAN đều có một ID xác định mức độ ưu tiên của nó. ID càng thấp, mức độ ưu tiên càng cao.
- Truyền dữ liệu: Dữ liệu có thể truyền đi qua bus CAN với kích thước tối đa là 8 byte mỗi khung. Điều này là đặc trưng của CAN 2.0A, giúp quản lý hiệu quả băng thông và độ trễ.

#### CAN 2.0B (Extended CAN)

- 29-bit identifier (ID): Phiên bản CAN 2.0B mở rộng định dạng ID từ 11 bit trong CAN 2.0A lên 29 bit. Với 29 bit, có thể biểu diễn 2^29 ID khác nhau, cho phép phân bổ số lượng ID lớn hơn và hỗ trợ các hệ thống phức tạp với nhiều node hơn.
- Tương thích ngược: CAN 2.0B vẫn tương thích ngược với CAN 2.0A, có nghĩa là các node sử dụng CAN 2.0B có thể hiểu được và giao tiếp với các node sử dụng CAN 2.0A. - Các node CAN 2.0B có thể nhận diện giữa các khung dữ liệu tiêu chuẩn và khung dữ liệu mở rộng thông qua bit điều khiển IDE (Identifier Extension).
- Khả năng truyền dữ liệu: Giống như CAN 2.0A, CAN 2.0B cũng giới hạn khung dữ liệu tối đa là 8 byte, nhưng sự khác biệt chính nằm ở số lượng ID có thể được sử dụng để định danh các thiết bị trên mạng.

![](https://i.imgur.com/mU3tMT6.png)

- **Start of Frame (SOF)**: Đây là một bit duy nhất đánh dấu bắt đầu của một frame.
- **Identifier (ID)**: 29-bit được chia thành hai phần:
    - **Base ID:** 11-bit giống như trong Standard Frame.
    - **Extended ID:** 18-bit mở rộng để cung cấp tổng cộng 29-bit nhận dạng.
- **Remote Transmission Request (RTR):** Bit này chỉ định liệu frame là một Data Frame hay Remote Frame.
    - RTR = 0: Đây là Data Frame.
    - RTR = 1: Đây là Remote Frame (yêu cầu dữ liệu từ một node khác).
- **Identifier Extension (IDE):** Bit này phân biệt giữa Standard Frame (IDE = 0) và Extended Frame (IDE = 1).
- **Reserved Bit (r0, r1):** Những bit dự phòng để phục vụ cho các phiên bản tương lai của chuẩn CAN.
- **Data Length Code (DLC):** 4-bit chỉ định số lượng byte dữ liệu trong Data Field (từ 0 đến 8 byte).
- **Data Field:** Chứa dữ liệu thực tế mà bạn muốn truyền. Độ dài từ 0 đến 8 byte (mỗi byte 8-bit).
- **Cyclic Redundancy Check (CRC):** Trường này chứa 15-bit CRC và 1-bit CRC Delimiter. CRC được dùng để kiểm tra lỗi trong frame.
- **ACK Slot**: 1-bit dành cho node nhận phát tín hiệu xác nhận (ACK) nếu frame được nhận mà không có lỗi.
- **End of Frame (EOF):** Kết thúc frame, gồm 7 bit liên tiếp có giá trị "1".
- **Intermission Frame Space (IFS):** 3-bit dành cho thời gian nghỉ giữa hai frame truyền liên tiếp.

#### CAN FD (Flexible Data-rate)

CAN FD là một phiên bản cải tiến của giao thức CAN tiêu chuẩn, được phát triển để giải quyết các hạn chế về tốc độ truyền dữ liệu và kích thước khung dữ liệu trong các phiên bản trước đó. CAN FD là viết tắt của Flexible Data-rate, tức là tốc độ dữ liệu linh hoạt, và có những cải tiến lớn so với CAN 2.0A và 2.0B.

Đặc điểm chính của CAN FD:
- Truyền dữ liệu với tốc độ cao hơn: CAN FD cho phép tốc độ truyền dữ liệu nhanh hơn nhiều so với CAN tiêu chuẩn, với **tốc độ có thể lên tới 8 Mbps** trong giai đoạn truyền dữ liệu (Data Phase). Trong CAN tiêu chuẩn, tốc độ truyền tối đa bị giới hạn ở 1 Mbps.
- Kích thước khung dữ liệu lớn hơn: Trong khi CAN tiêu chuẩn chỉ hỗ trợ tối đa 8 byte dữ liệu trong mỗi khung, CAN FD có thể **truyền tới 64 byte** dữ liệu trong một khung. Điều này giúp giảm số lượng khung cần thiết để truyền một lượng lớn dữ liệu, từ đó giảm độ trễ và tăng hiệu suất.
- Tốc độ truyền linh hoạt: CAN FD sử dụng hai tốc độ truyền khác nhau trong cùng một khung: một tốc độ chậm hơn cho Arbitration Phase (giai đoạn tranh chấp quyền gửi), và một tốc độ nhanh hơn cho Data Phase (giai đoạn truyền dữ liệu). Điều này giúp đảm bảo tính tương thích và hiệu quả của hệ thống.
- Tương thích ngược: CAN FD vẫn giữ được khả năng tương thích ngược với các phiên bản CAN cũ như CAN 2.0A và CAN 2.0B, giúp các hệ thống sử dụng cả CAN tiêu chuẩn và CAN FD có thể hoạt động song song.




</details>


## Bài 13: LIN

<details><summary>Xem</summary>  

LIN (Local Interconnect Network) là một giao thức phụ trong Ô tô, sinh ra nhằm mục đích giảm thiểu sự phức tạp và chi phí trong việc truyền thông giữa các thiết bị điện tử đơn giản trong xe

LIN thường được sử dụng trong các ứng dụng điều khiển chức năng không an toàn của xe như điều khiển cửa sổ, đèn, gương,...

![](https://i.imgur.com/mFWatV5.png)

Trên xe ô tô, có sự kết hợp của CAN và LIN, các hệ thống phụ sử dụng LIN sẽ không trực tiếp giao tiếp với nhau mà sẽ trao đổi thông qua CAN, mỗi hệ thống LIN sẽ có một LIN master để kết nối vào CAN bus.

### Các đặc điểm của LIN

![](https://i.imgur.com/9KYCGdH.png)

- LIN hoạt động theo mô hình Master - Slave
- Slave chỉ phản hồi khi có yêu cầu từ Master
- LIN master node sẽ quét và gửi yêu cầu liên tục đến tất cả các LIN slave nodes để đọc thông tin từ Slave, nếu có thì Slave sẽ trả về, không thì bỏ qua
- Sau đó Master sẽ gửi dữ liệu lên CAN bus để truyền đến các hệ thống LIN khác để thực thi
- LIN sử dụng giao thức UART để truyền/nhận dữ liệu với khung truyền **1 start bit - 8 data bits - 1 stop bit**
- LIN hoạt động ở hai mức là **0 và 12V**

![](https://i.imgur.com/g096nDr.png)

### Cấu trúc mỗi node

![](https://i.imgur.com/K47tyIC.png)

**Chức năng các chân của TJA1028**
- TXD (Transmit Data): Chân đầu vào dữ liệu truyền từ bộ vi điều khiển.
- RXD (Receive Data): Chân đầu ra dữ liệu nhận từ bus LIN đến bộ vi điều khiển.
- LIN: Chân kết nối với đường truyền bus LIN.
- VBAT: Chân cấp nguồn cho module LIN từ nguồn cung cấp chính (thường là pin xe hơi,Vecu = 12V).
- GND: Chân nối đất (ground).
- EN (Enable): Chân cho phép kích hoạt bộ transceiver.
- INH (Inhibit): Chân điều khiển bật/tắt nguồn ngoại vi, dùng để bật hoặc tắt thiết bị hoặc mạch khác.
- WAKE: Chân cho phép đánh thức hệ thống từ chế độ ngủ (sleep mode).
### Khung truyền của LIN

![](https://i.imgur.com/hA22a9s.png)

**Header** là gói tin được gửi từ Master (có ba dạng TX, RX và ignore) và **Response** được gửi từ Slave hoặc Master tùy thuộc vào kiểu của Header

- Header
    - Break: Master gửi 14 bits giống nhau để bắt đầu khung dữ liệu
    - Sync: 8 bit để đồng bộ hóa (0x55 = 01010101). Master sẽ gửi 8 bit tượng trưng cho xung Clock, các Slave sẽ đọc và điều chỉnh tốc độ xung theo tốc độ của Master
    - ID: 6 bit ID + 2 bit parity(một bit kiểm tra chẵn, một bit kiểm tra lẻ). Xác định loại dữ liệu và Slave cần phản hồi.
- Response: Nếu gói Header là TX thì Response được gửi từ Master đến Slave. Nếu gói là RX thì Slave sẽ gửi gói tin đến Master.
    - Data: 16 - 64 bits tức 2 - 8 bytes dữ liệu chứa nội dung thông điệp truyền đi
    - Checksum: 8 bits để phát hiện lỗi trong quá trình truyền.


### Quá trình truyền
- Master sẽ gửi Header cho các Slave trước.
    - Nếu Master yêu cầu dữ liệu từ Slave, thì Slave sẽ gửi lại Response cho Master.
    - Nếu Master muốn gửi dữ liệu cho Slave, thì sẽ gửi kèm Response cho Slave.
- Slave sẽ kiểm tra ID có khớp với mình không, nếu có thì xử lý, nếu không thì bỏ qua.

</details>




## AUTOSAR Classic

<details><summary>Xem</summary>  


**AUTOSAR** (**AUT**omotive **O**pen **S**ystem **AR**chitecture) là một tiêu chuẩn toàn cầu cho phát triển phần mềm ô tô với mục tiêu là chuẩn hóa kiến trúc phần mềm cho các hệ thống điều khiển điện tử (ECU) trong ô tô, nhằm tăng tính khả chuyển, khả mở rộng và giảm chi phí phát triển.

![Architechure](https://i.imgur.com/Cd7S24G.png)

Các Task sẽ chạy song song với nhau và trong một thời điểm chỉ có 1 task được thực thi để tránh việc làm sai xót dữ liệu, tất cả các Task được quản lý thông qua hệ điều hành (Operating System).

**So sánh**
![Sosanh](https://i.imgur.com/FwJT071.png)

- Các Task được viết song song nên có sự đồng nhất và khả năng tái sử dụng cao. Đồng thời việc bảo trì cũng dễ dàng hơn so với việc chương trình hoạt động tuần tự vì các Task được cấu hình riêng biệt.

### Kiến trúc AUTOSAR

Được chia làm 4 tầng
- **Application Layer**: Bao gồm các thành phần phần mềm ứng dụng, thực hiện các chức năng cụ thể của xe (như kiểm soát động cơ, phanh, v.v).
- **Runtime Environment(RTE)**: Là lớp trung gian giữa phần mềm ứng dụng và phần mềm cơ bản, giúp phần mềm ứng dụng có thể giao tiếp với nhau một cách chuẩn hóa (liên kết SWC và BSW).
- **Basic Software (BSW)**: Là phần mềm cơ bản, bao gồm các thành phần phần mềm tiêu chuẩn để quản lý các chức năng hệ thống, giao tiếp và điều khiển phần cứng.
- **Microcontroller** 

#### Application Layer

Bao gồm nhiều khối phần mềm ứng dụng (Software Component - SWC). **Mỗi SWC thực hiện 1 chức năng cụ thể** trong hệ thống ECU. Ví dụ: điều khiển động cơ, phanh, túi khí,...

![anh](https://i.imgur.com/ECiUKwP.png)

SWC chỉ quan tâm đến **logic** (tính toán số học, khởi động thế nào), không cần quan tâm đến phần cứng. Tuy nhiên, SWC vẫn có thể giao tiếp với nhau và giao tiếp với phần cứng thông qua RTE.

#### Runtime Environment(RTE)

RTE đóng vai trò trung gian, quan trọng trong việc kết nối các Software Components (SWC) trền tâng Application Layer và Basic Software (BSW) thông qua một kiến trúc trừu tượng.

Chức năng:
- **Truyền thông tin giữa các SWCs**: RTE cung cấp cơ chế truyền thông để các thành phần phần mềm (SWCs) có thể trao đổi dữ liệu hoặc gọi dịch vụ với nhau mà không cần biết chi tiết về các phần còn lại của hệ thống. 
- **Kết nối SWCs với BSW**: RTE cung cấp giao diện để các SWCs có thể tương tác với BSW. Điều này giúp các SWCs có thể sử dụng các dịch vụ hoặc điều khiển phần cứng một cách dễ dàng.
- Hỗ trợ việc lập lịch và điều phối thực thi của các SWCs theo các sự kiện hoặc chu kỳ định sẵn. 

#### Basic Software
Basic Software (BSW) là một trong ba thành phần chính của kiến trúc AUTOSAR, đóng vai trò nền tảng để hỗ trợ phần mềm ứng dụng (SWC) hoạt động trên phần cứng. BSW cung cấp các dịch vụ cơ bản như quản lý phần cứng, giao tiếp, chẩn đoán, và các dịch vụ hệ thống.

BSW được chia thành 3 tầng:
- Service Layer.
- ECU Abstraction Layer.
- Microcontroller Abstraction Layer - MCAL.

**Service Layer**:
Đây là lớp cao nhất trong BSW, cung cấp các dịch vụ hệ thống và tiện ích cho các phần mềm ứng dụng (SWC) và các lớp khác của BSW.
Là nơi **lưu trữ hệ điều hành của hệ thống tại lớp (System Services)**. Các dịch vụ khác bao gồm quản lý thời gian thực, chẩn đoán, quản lý lỗi, quản lý nguồn, bộ nhớ v.v.

**ECU Abstraction Layer**:
Lớp này cung cấp một giao diện trừu tượng cho tất cả các thiết bị ngoại vi và phần cứng cụ thể của ECU. Nó ẩn đi sự khác biệt về phần cứng của các thiết bị ngoại vi khác nhau và cung cấp một giao diện tiêu chuẩn cho các lớp bên trên (Service Layer và SWC). **Là nơi giao tiếp BSW với tầng Application thông qua RTE**

**MCAL (Microcontroller Abstraction Layer):**
Đây là lớp thấp nhất trong BSW, cung cấp giao diện trừu tượng để tương tác trực tiếp với các thành phần phần cứng của vi điều khiển, chẳng hạn như bộ xử lý trung tâm (CPU), các thiết bị ngoại vi tích hợp (như ADC, PWM, UART), và các bộ định thời (timer).

Với mỗi dòng vi điều khiển khác nhau, MCAL sẽ là khác nhau nhưng các lớp ở trên như Application Layer, RTE,... sẽ được giống nhau. Vì thế khi muốn chuyển hệ thống sang một hệ thống khác, ta chỉ cần thay đổi MCAL phù hợp với chức năng phần cứng vi điều khiển mới.

Vì thế, để các lớp bên trên có thể kết nối được với MCAL, chúng ta cần phải có một quy chuẩn chung để cấu hình các chức năng phần cứng bên trong MCAL.

</details>

## MCAL 

### DIO Driver
<details><summary>Xem</summary>  
**Cấu hình chân**:
- Các chân trong STM32 được gọi là GPIO nhưng với MCAL, tổng quát cho tất cả vi điều kiển, chúng được gọi là **DIO Channels**.
- Tương tự với port thì được gọi là **DIO Ports**.
- **DIO Channels Groups** là một nhóm các chân không xác định

**Imported types**: Những kiểu dữ liệu cần phải thêm vào để viết Driver, gồm:
![](https://i.imgur.com/HBAwd7x.png)

- File header: Std_Types.h là file chứa các định nghĩa, các kiểu dữ liệu được cấu hình riêng cho vi điều khiển
- Một vài kiểu dữ liệu được cung cấp như:
    - **Std_ReturnType**
    ```cpp
    typedef uint8_t Std_ReturnType;

    #define E_OK        0x00U   /**< Thao tác thành công */
    #define E_NOT_OK    0x01U   /**< Thao tác thất bại */

    ```
    Ngoài ra, một vài định nghĩa khác như:
    ```cpp 
    /* ============================================= */
    /*           Logical State Definitions          */
    /* ============================================= */

    /**
    * @brief Định nghĩa các trạng thái logic cao và thấp
    * @details Được sử dụng cho các tín hiệu đầu vào/đầu ra.
    */
    #define STD_HIGH    0x01U   /**< Trạng thái logic cao */
    #define STD_LOW     0x00U   /**< Trạng thái logic thấp */
    /* ============================================= */
    /*            Null Pointer Definition            */
    /* ============================================= */

    /**
    * @brief Định nghĩa con trỏ NULL
    * @details Con trỏ NULL là con trỏ trỏ đến địa chỉ 0.
    */

    #ifndef NULL_PTR
        #define NULL_PTR ((void*)0)   /**< Định nghĩa con trỏ NULL */
    #endif
    /* ============================================= */
    /*         Platform Independent Data Types       */
    /* ============================================= */

    /**
    * @brief Các kiểu dữ liệu độc lập với nền tảng
    * @details Định nghĩa rõ ràng về kích thước và dấu của các kiểu dữ liệu
    */
    typedef unsigned char      uint8;   /**< Số nguyên không dấu 8-bit */
    typedef signed char        sint8;   /**< Số nguyên có dấu 8-bit */

    typedef unsigned short     uint16;  /**< Số nguyên không dấu 16-bit */
    typedef signed short       sint16;  /**< Số nguyên có dấu 16-bit */

    typedef unsigned long      uint32;  /**< Số nguyên không dấu 32-bit */
    typedef signed long        sint32;  /**< Số nguyên có dấu 32-bit */

    typedef unsigned long long uint64;  /**< Số nguyên không dấu 64-bit */
    typedef signed long long   sint64;  /**< Số nguyên có dấu 64-bit */

    typedef float              float32; /**< Kiểu số thực 32-bit */
    typedef double             float64; /**< Kiểu số thực 64-bit */

    ```

    Std_returnType sẽ định nghĩa lại các kiểu dữ liệu từ những nguồn gốc như uint8_t, uint16_t... thành những kiểu phù hợp với tiêu chuẩn AUTOSAR và làm cho chương trình được sắp xếp để dễ dàng xử lý.
    
    -  **Std_VersionInfoType**
    Lưu trữ dữ liệu của phiên bản phần mềm hiện tại của Module

### Type Definitions:

- **```Dio_ChannelType```**

![](https://i.imgur.com/dmGLmuT.png)

```Dio_ChannelType``` là một kiểu dữ liệu được cấu hình từ gốc uint có chức năng đánh số ID cho từng chân (channel) trên vi điều khiển

Ví dụ trên STM32, các chân được phân biệt theo 2 tiêu chí Port - Pin Number như chân A0 được gọi là PA0, chân B 13 được gọi là PB13. Nhưng đối với AUTOSAR chỉ được phân biệt bởi một thuộc tinh là ID.  
Ví dụ với STM32, Các chân PA0 - PA15 sẽ được đánh số từ 0 - 15, tiếp theo đó các số từ PB0 - PB15 được đánh số từ 16 - 31.

Các vi điều khiển khác nhau sẽ có một kiểu định nghĩa chân khác nhau và MCAL sẽ là chuẩn chung có thể định nghĩa lại các chân đó theo kiểu dữ liệu chuẩn của AUTOSAR để cung cấp cho các lớp trên

- **```Dio_PortType```**

![](https://i.imgur.com/xOB8DBv.png)

Tương tự với ```Dio_ChannelType``` thì ```Dio_PortType``` cũng là một kiểu dữ liệu dạng số để xác định ID của các Port

Với STM32 sẽ là A, B, C, D và AUTOSAR sẽ định nghĩa lại là 0, 1, 2, 3...


- **```Dio_ChannelGroupType```**

![](https://i.imgur.com/FgbrS5y.png)


- ```Dio_ChannelGroupType``` là một Struct để xác định một nhóm các chân của vi điều khiển, gồm ba thuộc tính
    - ```mask``` có kiểu uint8/16/32 dùng để xác định vị trí của ChannelGroup
    - ```offset```: kiểu dữ liệu unit8, Độ dời tính từ vị trí gốc Channel 0 của Port
        - Ví dụ: Nếu group là A3 A4 A5 A6 thì độ dời là 3
    - ```port``` nhóm này nằm ở port nào kiểu Dio_PortType

- **```Dio_LevelType```** 


![](https://i.imgur.com/vPMJksj.png)
![](https://i.imgur.com/793LYDn.png)

```Dio_LevelType``` là một kiểu dữ liệu có gốc uint8 dùng để xác định mức điện áp của một Channel, có thể là INPUT hoặc OUTPUT

- ```STD_LOW``` tương ứng với 0V
- ```STD_HIGH``` tương ứng với 3.3V hoặc 5V


- **```Dio_PortLevelType```** 

![](https://i.imgur.com/TSChfwl.png)

là một kiểu dữ liệu có gốc là uint, chứa 16 hoặc 32bit dữ liệu để hiển thị mức logic của các chân trong một Port



### Function Definitions

#### **```Dio_ReadChannel```**

![](https://i.imgur.com/FAuRRYm.png)

- Chức năng: Đọc tín hiệu điện áp của một Channel
- Cú pháp
    ```
    Dio_LevelType Dio_ReadChannel(Dio_ChannelType ChannelId)
    ```
- Kiểu trả về: Dio_LevelType là STD_HIGH hoặc STD_LOW
- Tham số truyền vào: ```Dio_ChannelType ChannelId``` là Id của Channel muốn đọc điện áp

Với ChannelId được truyền vào, chúng ta sẽ phân tích và biển đổi thành Port và Pin để MCU hiểu và gọi những Function hoặc sử dụng thanh ghi để đọc giá trị điện áp và trả về.


#### **```Dio_WriteChannel```**


![](https://i.imgur.com/SFrLzQq.png)

- Chức năng: Đặt mức điện áp cho một Channel
- Cú pháp
    ```
    void Dio_WriteChannel (Dio_ChannelType ChannelId,Dio_LevelType Level)
    ```
- Tham số truyền vào:
    - ```Dio_ChannelType ChannelId```: Channel cần điều khiển
    - ```Dio_LevelType Level```: Mức điện áp để đặt cho Channel đó
- Không có giá trị trả về.

#### **```Dio_ReadPort```**

![](https://i.imgur.com/r067VIJ.png)

- Chức năng: Đọc tín hiệu của tất cả Channel có trong Port
- Cú pháp
    ```
    Dio_PortLevelType Dio_ReadPort (Dio_PortType PortId)
    ```
- Tham số truyền vào ```Dio_PortType PortId```: Port cần đọc giá trị
- Kiểu trả về: ```Dio_PortLevelType``` là 16/32 bit tương ứng với giá trị từng Channel của port.

#### **```Dio_WritePort```**

![](https://i.imgur.com/xEpMvWj.png)

- Chức năng: Ghi giá trị điện áp cho các chân của một Port
- Cú pháp 
    ```
    void Dio_WritePort (Dio_PortType PortId, Dio_PortLevelType Level)
    ```
- Tham số truyền vào: 
    - ```Dio_PortType PortId```: Port của vi điều khiển
    - ```Dio_PortLevelType Level```: Giá trị uint để cấu hình cho Port.

- Không có dữ liệu trả về.

#### **```Dio_ReadChannelGroup```**

![](https://i.imgur.com/TlsFSxW.png)

- Chức năng: Đọc giá trị điện áp của một GroupChannel
- Cú pháp:
    ```
    Dio_PortLevelType Dio_ReadChannelGroup (
    const Dio_ChannelGroupType* ChannelGroupIdPtr
    )
    ```
- Tham số truyền vào là một Struct chứa các thuộc tính mask, offset, Port của ChannelGroup
- Trả về giá trị thuộc kiểu dữ liệu ```Dio_PortLevelType```

#### **```Dio_WriteChannelGroup```**

![](https://i.imgur.com/5LTGqff.png)

- Chức năng: Ghi mức điện áp vào các chân của ChannelGroup
- Gồm hai tham số là GroupChannel và Level của Channel đó
- Không có kiểu trả về

#### **```Dio_GetVersionInfo```**

![](https://i.imgur.com/fbfUxZC.png)

- Chức năng: Lấy thông tin phiên bản hiện tại của hệ thống
- Gán các giá trị phiên bản hiện tại vào Struct Std_VersionInfoType
- Không có kiểu dữ liệu trả về.

#### **```Dio_FlipChannel```** 

![](https://i.imgur.com/biUkHy7.png)
![](https://i.imgur.com/4sfQphE.png)

- Chức năng: Lật mức điện áp tại 1 Channel
- Trả về mức điện áp của chân vừa đảo


</details>

