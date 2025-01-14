#define RCC_APB2ENR *((unsigned int  *)0x40021018)// ep kieu o nho ve unsigned int va doc gia tri tai o nho do
#define GPIOC_CRH *((unsigned int  *)0x40011004) // doc gia tri thanh ghi GPIOC 
#define GPIOC_ODR *((unsigned int *)0x4001100C)


void delay(unsigned int time) {
    for(int i = 0; i < time ; i++);
} 



int main(){
	RCC_APB2ENR |= (1 << 4);
	
	GPIOC_CRH |= (1 << 20) | (1 << 21); //MODE PC13 = 11
	
	GPIOC_CRH &= ~(1 << 22) | (1 << 23); //CNF PC13 = 00
	
	while(1){
	GPIOC_ODR |= 1 << 13;
	delay(3000000);
	
	GPIOC_ODR &= ~(1 << 13);
	delay(3000000);
	}
}




