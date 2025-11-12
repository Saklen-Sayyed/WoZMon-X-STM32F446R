#include "stm32f4xx_hal.h"

#define help		      'h'
#define HELP              'H'
#define dump              'd'
#define DUMP              'D'
#define write			  'e'
#define WRITE             'E'
#define NASCENT            0
#define COMMAND            0
#define DISPLAY_DATA       1
#define WRITE_DATA		   2

#define USART3_RXFLAG  (USART3->SR)
#define SET			   ((1<<5))
#define ZERO_ASCII        '0'
#define CAP_ASCII		  55
#define SMALL_ASCII       87

void SystemClock_Config (void);
void gpioc_config       (void);
void uart3_config       (void);
void uart_tx            (unsigned char data);
void uart_str_tx        (const unsigned char* str);
void sw_init            (void);
void peipheral_clk_en   (void);
void tim2_init          (void);
void print_address      (uint32_t address32);
void display_help       (void);
void dump_data			(void);
void recieve_address	(void);
void print_address		(uint32_t address32);
void print_data         (uint8_t *addr,uint32_t addr_buff);
uint32_t shift_add      (void);
void recieve_new_data   (void);
void write_data         (void);
