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







