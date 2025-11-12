#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "main.h"

const    uint8_t  version[]    = "\nMemory_Monitor_WozMon_V0.1";
const    uint8_t  hex_error[]  = "\nERR : BAD HEX\n";
const    uint8_t  enter_add[]  = " ENTER ADDRESS :\n";
const    uint8_t  terminal[]   = "\nWoZMon>";
const    uint8_t  help_msg[]   = "ENTER -H- FOR HELP\n";
const    uint8_t  helpln1[]	   = "\n1) H - HELP:)";
const    uint8_t  helpln2[]	   = "\n2) D - READ A LOCATION";
const    uint8_t  helplnd[]	   = "\n           ACCESSIBLE LOCATIONS:";
const    uint8_t  helplnd1[]   = "\n             -FLASH(READ/EXEC): 0x08000000-0x0807FFFF";
const    uint8_t  helplnd2[]   = "\n             -SRAM (READ/WRITE/EXEC): 0x20000000-0x2002FFFF";
const    uint8_t  helplnd3[]   = "\n             -SYSTEM ROM (READ/EXEC): 0x1FFF0000-0x1FFF77FF";
const    uint8_t  helplnd4[]   = "\n             -PERIPHERALS (RISKY/BUS FAULT): 0x40000000-0x5FFFFFFF";
const    uint8_t  helpln3[]	   = "\n2) E - WRITE A LOCATION";
const    uint8_t  helplne[]	   = "\n           ACCESSIBLE LOCATIONS:";
const    uint8_t  helplne1[]   = "\n             -SRAM (READ/WRITE/EXEC): 0x20000000-0x2002FFFF";
volatile uint8_t  address[10]  = {'\0'};
volatile uint8_t  new_data[2]  = {'\0'};
volatile uint8_t  indx         = 0;
volatile uint8_t  mode         = 0;
volatile uint8_t  command      = 0;
volatile uint8_t  last_command = 0;
         uint8_t *current_add = NULL;
         uint8_t *last_add    = 0;


int main(void){

	SystemClock_Config();
	peipheral_clk_en();
	gpioc_config();
	uart3_config();
	sw_init();
	tim2_init();
	uint8_t* p=(uint8_t*)0x20002000;

	*p=0x55;

	uart_str_tx(version);
	uart_str_tx(terminal);
	uart_str_tx(help_msg);
	uart_str_tx(terminal);
	while(1){
		if     ((command == help)  || (command == HELP  )) display_help();
		else if((command == dump)  || (command == DUMP  )) dump_data();
		else if((command == write) || (command == WRITE )) write_data();
	}
}

void display_help(void){

	uart_str_tx(terminal);
	uart_str_tx(helpln1);
	uart_str_tx(helpln2);
	uart_str_tx(helplnd);
	uart_str_tx(helplnd1);
	uart_str_tx(helplnd2);
	uart_str_tx(helplnd3);
	uart_str_tx(helplnd4);
	uart_str_tx(helpln3);
	uart_str_tx(helplne);
	uart_str_tx(helplne1);
	uart_str_tx(terminal);
	command = COMMAND;

}

void dump_data(void){

	uint32_t address_buffer =  0;

	last_command = command;
	command      = NASCENT;
	mode 		 = DISPLAY_DATA;

	uart_str_tx(enter_add);
	uart_str_tx(terminal);

	while(mode == DISPLAY_DATA); //POLLING FOR ADDRESS
	address_buffer = shift_add();
	indx    	   =  0;
	current_add    = (uint8_t*)address_buffer;
	last_add	   =  current_add;

	uart_tx('\n');
	uart_str_tx(terminal);

	if((address_buffer >= 0x08000000 &&  address_buffer <= 0x0807FFFF) || //FLASH(READ/EXEC)
	   (address_buffer >= 0x20000000 &&  address_buffer <= 0x2002FFFF) || //SRAM (READ/WRITE/EXEC)
	   (address_buffer >= 0x1FFF0000 &&  address_buffer <= 0x1FFF77FF) || //SYSTEM ROM (READ/EXEC)
	   (address_buffer >= 0x40000000 &&  address_buffer <= 0x5FFFFFFF))   //PERIPHERALS (RISKY/BUS FAULT)
		print_data(current_add,address_buffer);
	else
		uart_str_tx((const unsigned char*)"Invalid Access");

	address_buffer=NASCENT;
}

void write_data(void){

	uint32_t address_buffer =  0;
	uint8_t new_dat=0;

	last_command = command;
	command      = NASCENT;
	mode 		 = DISPLAY_DATA;

	uart_str_tx(enter_add);
	uart_str_tx(terminal);

	while(mode == DISPLAY_DATA); //POLLING FOR ADDRESS
	mode = WRITE_DATA;
	uart_str_tx(terminal);
	address_buffer = shift_add();
	indx    	   =  0;
	current_add    = (uint8_t*)address_buffer;
	last_add	   =  current_add;

	uart_str_tx((const unsigned char*)"NEW DATA:");
	uart_str_tx(terminal);

	while(mode == WRITE_DATA);

	new_dat  = (new_data[1] & 0x0f);
	new_dat |= ((new_data[0] << 4) & 0xf0);

	*(current_add) = new_dat;
}

void recieve_address(void){

	uint8_t temp_buff = (USART3->DR);

	if(((temp_buff>= ('0')) && (temp_buff<= ('9')))){  //0-9
		address[indx] = ((temp_buff)-ZERO_ASCII);
		uart_tx(address[indx]+ZERO_ASCII);
		indx++;
	}
	else if(((temp_buff >= 'A') && (temp_buff <= 'F'))){
		address[indx] = ((temp_buff)- CAP_ASCII);
		uart_tx(address[indx] + CAP_ASCII);
		indx++;
	}
	else if(((temp_buff >= 'a') && (temp_buff <= 'f'))){
		address[indx] = ((temp_buff)-SMALL_ASCII);
		uart_tx(address[indx]+SMALL_ASCII);
		indx++;
	}
	else{
		uart_str_tx(hex_error);
		uart_str_tx((const unsigned char*)"\nEnter Again:");
		indx=0;
	}

	if(indx > 7){
		mode  = COMMAND;
		indx = 0;
	}
}

void recieve_new_data(void){

	uint8_t temp_buff = (USART3->DR);

		if(((temp_buff>= ('0')) && (temp_buff<= ('9')))){  //0-9
			new_data[indx] = ((temp_buff)-ZERO_ASCII);
			uart_tx(new_data[indx]+ZERO_ASCII);
			indx++;
		}
		else if(((temp_buff >= 'A') && (temp_buff <= 'F'))){
			new_data[indx] = ((temp_buff)- CAP_ASCII);
			uart_tx(new_data[indx] + CAP_ASCII);
			indx++;
		}
		else if(((temp_buff >= 'a') && (temp_buff <= 'f'))){
			new_data[indx] = ((temp_buff)-SMALL_ASCII);
			uart_tx(new_data[indx]+SMALL_ASCII);
			indx++;
		}
		else{
			uart_str_tx(hex_error);
			uart_str_tx((const unsigned char*)"\nEnter Again:");
			indx=0;
		}

		if(indx > 1){
			mode  = COMMAND;
			indx = 0;
			uart_str_tx((const unsigned char*)"\nDONE\n");
		}
}

uint32_t shift_add(void){

	uint8_t  shift_bit  = 0;
	uint32_t temp_add32 = 0;

	for(indx=0,shift_bit=28; indx < 8; indx++,shift_bit-=4){
		temp_add32 |=  (address[indx] << shift_bit);
	}
	indx=0;
	return temp_add32;
}

void print_data(uint8_t *addr,uint32_t addr_buff){

	uint8_t dump_data;
	uart_tx('\n');

	for(uint8_t n_location = 0; n_location < 8; n_location++,addr++,addr_buff++){
		dump_data = *addr;
		print_address(addr_buff);

		if(((dump_data>>4) & 0x0f) < (10)) uart_tx(((dump_data>>4) & 0x0f) +(ZERO_ASCII));
		else    					 	   uart_tx(((dump_data>>4) & 0x0f) + CAP_ASCII );
		if((dump_data & 0x0f) < (10))      uart_tx((dump_data & 0x0f) +(ZERO_ASCII));
		else 					 	 	   uart_tx((dump_data & 0x0f) + CAP_ASCII );
		uart_tx('\n');
	}
}

void print_address(uint32_t address32){

	uart_tx('0');
	uart_tx('X');
	for(uint8_t shift_bit=28, term=0; term < 8; shift_bit -= 4,term++){
		if(((address32 >> shift_bit )  & 0X0F) >= (10))
			uart_tx((((address32 >> shift_bit)  & 0X0F) + CAP_ASCII));
		else
			uart_tx((((address32 >> shift_bit)  & 0X0F) + ZERO_ASCII));
	}
	uart_tx(':');
	uart_tx(' ');
}

void peipheral_clk_en(void){

	RCC->APB1ENR |=(1<<0);  //TIM2 CLOCK ENABLE
	RCC->AHB1ENR |=(1<<2);  //GPIOC CLOCK ENABLE
	RCC->AHB1ENR |=(1<<1);	//GPIOB CLOCK ENABLE
	RCC->APB1ENR |=(1<<18); //USART3 CLOCK ENABLE
}

void gpioc_config(){

	GPIOC->MODER  &= ~(0xF << 20);
	GPIOC->MODER  |=  (0xA << 20); //10_11 ALTERNATE FUNCTION
	GPIOC->OTYPER  =   0X0000;     //10_11 PUSH PULL
	GPIOC->OSPEEDR =   0X00F00000; //10_11 HIGH SPEED
	GPIOC->PUPDR   =   0X00000000; //10_11 NO PULL
	GPIOC->AFR[1] &= ~(0xFF << 8);
	GPIOC->AFR[1] |=  (0x77 << 8); //10_USART3_TX 11_USART3_RX
}

void uart3_config(void){

	/*<====: TX CONFIG :====>*/
	USART3->CR1  = (1<<13);
	USART3->CR2  = 0;
	USART3->CR3  = 0;
	USART3->BRR  = (104<<4) + 3;  //(DIV_MANTISSA + DIV_FRACTION) 9600 BAUD
	USART3->CR1 |= (1 << 3);      // TE = 1 (Transmitter enable)
	USART3->CR1 |= (1 << 13);     // UE = 1 (USART enable)

	/*<====: RX CONFIG :====>*/
	USART3->CR1 |= (1<<5); //RECIEVE INTERRUPT ENABLE
	USART3->CR1 |= (1<<2); //RX ENABLE

	NVIC_SetPriority(USART3_IRQn, 0);
	NVIC_EnableIRQ(USART3_IRQn);

}

void uart_tx(unsigned char data) {

    while (!(USART3->SR & (1 << 7))) ;  // Waiting for TXE
    USART3->DR = data;
    while (!(USART3->SR & (1 << 6))) ;  // Waiting for TC
}

void uart_str_tx(const unsigned char* str){

	while((*str) != '\0'){
		uart_tx(*str);
		str++;
	}
}

void sw_init(void){

	GPIOB->MODER   &= ~(3<<0);
	GPIOB->OTYPER  &= ~(1<<0);
	GPIOC->OSPEEDR &= ~(1<<0);
	GPIOB->PUPDR   &= ~(3<<0);
}

void tim2_init(void){

	TIM2->CR1  &=(~(1<<1)); //UDIS=0 ENABLE UPDATE EVENT
	TIM2->CR1  |=(1<<2);    //URS=1  ONLY OVERFLOW/UNDERFLOW
	TIM2->CR1  &=(~(1<<3)); //OPM=0  CONTINOUS COUNTER
	TIM2->CR1  &=(~(1<<4)); //DIR=0  UPCOUNTER
	TIM2->CR1  &=(~(3<<6)); //CMS[1:0]=00 EDGE-ALIGNED
	TIM2->DIER |=(1<<0);    //UIE=1 UPDATE INTERRUPT ENABLE "TIM2->SR" BIT0 UIF==INTERRUPT FLAG

	TIM2->CNT   = 0;        //COUNTER VALUE
	TIM2->PSC   = 16000-1;  //PRESCALER 16Mhz/16000=1KHz
	TIM2->ARR   = 1 ;       //AUTO RELOAD REGISTER 1mS SYSTEM TIMER

	TIM2->CR1  |= 1;        //COUNTER/TIMER2 ENABLE

	NVIC_SetPriority(TIM2_IRQn, 1);
	NVIC_EnableIRQ(TIM2_IRQn);
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


