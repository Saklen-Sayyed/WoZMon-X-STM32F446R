#include "main.h"
#include "stm32f4xx_it.h"

void TIM2_IRQHandler(void){
	TIM2->SR    &= (~(1));
	//FOR FUTURE TIMEOUT PURPOSES
}

void USART3_IRQHandler(void){

	if(USART3_RXFLAG & SET){
		if(mode == COMMAND){
			command = USART3->DR;
			uart_tx(command);
		}
		else if(mode == DISPLAY_DATA) recieve_address();
		else if(mode ==   WRITE_DATA) recieve_new_data();
	}
}
