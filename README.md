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


















