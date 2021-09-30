#include "stm32f0xx.h"
#include <stdio.h> 
#define DEVICE_CAN_ID				0x703U 	// 0x701 - рама, 0x702 - стрела, 0x703 - отвал
#define NUM_OF_PAGES				31 		// Количество страниц в МК	STM32F042F4 - 15
																	//	STM32F042F6 - 31
																	//	STM32F072C8 - 31
																	//	STM32F072C8 - 31
																	//	STM32F072CB - 63
#define CAN_PIN_PORTA

//#define CAN_PIN_PORTB

#define CAN_Id_Standard             ((uint32_t)0x00000000)  /*!< Standard Id */
#define CAN_Id_Extended             ((uint32_t)0x00000004)  /*!< Extended Id */

#define	WAIT_HOST    				0
#define IDLE        				1
#define PAGE_PROG    				2
#define BOOT    					3


// Flash configuration
#define MAIN_PROGRAM_START_ADDRESS 	(uint32_t)0x08001000
#define MAIN_PROGRAM_PAGE_NUMBER 	2

#define NUM_OF_FREE_PAGES 			(NUM_OF_PAGES - MAIN_PROGRAM_PAGE_NUMBER)

// CAN identifiers
#define DEVICE_CAN_TX_ID			0x100

#define CMD_HOST_INIT				0x01
#define CMD_PAGE_PROG				0x02
#define CMD_BOOT					0x03

#define CAN_RESP_OK					0x01
#define CAN_RESP_ERROR_CRC			0x02
#define CAN_RESP_ERROR_Memory		0x03

#define FLASH_ER_PRG_TIMEOUT    	((uint32_t)0x000B0000)


#define countof(a)					(uint8_t)(sizeof(a) / sizeof(*(a)))
	
uint8_t search = 0, search_ARROW = 0, ready_to_send = 0, search_RECEIVE1 = 0, RECEIVE1 = 0, command = 0xFF, search_COMMAND = 0, search_PAGE = 0, PAGE_wait = 0;
uint8_t size_of_Page_arr[5], size_of_Page_parcel_cnt = 0, crc_cnt = 0;
uint16_t  size_of_Page = 0, received_Page_bytes = 0, Page_buf_cnt = 0;
	
volatile	uint32_t 				ticks_delay = 0;
volatile 	uint8_t 				Data_CRC[5];
uint8_t              				PageBuffer[FLASH_PAGE_SIZE];
volatile 	int                 	PageBufferPtr;
volatile 	uint8_t             	PageIndex;
volatile 	uint32_t            	PageCRC;
volatile 	uint8_t             	blState;

typedef void 						(*pFunction)(void);
pFunction    						JumpAddress;

typedef enum
{
	FLASH_BUSY = 1,
	FLASH_ERROR_WRP,
	FLASH_ERROR_PROGRAM,
	FLASH_COMPLETE,
	FLASH_TIMEOUT
}FLASH_Status;

void Configure_USART1(void);
void Configure_GPIO_USART1(void);
void Init_RCC(void);
void delay_ms(uint32_t milliseconds);
void Remap_pin(void);
void CRC_Init(void);
uint32_t CRC_Calculate(uint32_t pBuffer[], uint32_t BufferLength);
void DeInit(void);
void JumpToApplication(void);
void FLASH_Unlock(void);
void copy_Buffer(uint8_t *dst, uint8_t *src, uint16_t size);
void clear_Buffer(uint8_t *BUFER, uint16_t size);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
const char char_map[10] =																		// Карта
{
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'
};
uint8_t negative_flag;
uint8_t ies;
uint8_t char_cnt;
volatile uint16_t number1;
uint16_t char_to_int (char *cti)													// Функция перевода из строки в число
{
	negative_flag = 0;
	ies = 0;
	number1 = 0;
	char_cnt = 0;

	while (cti[char_cnt] != 0)
	{
		for (ies = 0; ies < 10; ies++)
		{
			if (cti[0] == '-') negative_flag = 1;
			if (cti[char_cnt] == char_map[ies]) 
			{
				number1 = number1 * 10 + ies ;
				break;
			}
		}
		char_cnt++;
	}
	
	if (negative_flag) number1 *= -1;
	
	return number1;
}

void send_Uart(USART_TypeDef* USARTx, uint8_t c)                				// Отправка симовла по USART, GPS
{
    while((USART1->ISR & USART_ISR_TC) == USART_ISR_TC);
    USART1->TDR = c;
}
char print_buffer[30];
void TransmitResponsePacket(char *s)           					// Отправка строки AT команд
{
	uint8_t i = 0;
	clear_Buffer((uint8_t *)print_buffer, countof(print_buffer));
	sprintf(print_buffer, "AT+CIPSEND=1,4\r");
	while (print_buffer[i] != 0) send_Uart(USART1, print_buffer[i++]);
	while (!ready_to_send) continue;
	
	ready_to_send = 0;
	delay_ms(50);
    while (*s != 0) send_Uart(USART1, *s++);
	delay_ms(50);
	USART1->ICR |= USART_ICR_TCCF;
}

int main(void)
{
	SysTick_Config(48000);/* 1ms config with HSE 8MHz*/
	Init_RCC();
	Remap_pin();

	Configure_GPIO_USART1();
	Configure_USART1();
	CRC_Init();
	
	delay_ms(500);

	if (blState == WAIT_HOST) 
	{
		JumpToApplication();
	}
	
	while (1)
	{
		
		switch(command)
		{
			case CMD_HOST_INIT:
								blState = IDLE;
								TransmitResponsePacket("$OK1");
								delay_ms(500);
			break;

			case CMD_PAGE_PROG:
								if (blState == IDLE) 
								{
									clear_Buffer(PageBuffer, countof(PageBuffer)); // Заполнение страницы нулями
									PageCRC = Data_CRC[4] << 24 | Data_CRC[3] << 16 | Data_CRC[2] << 8 | Data_CRC[1];
									PageIndex = Data_CRC[0];
									blState = PAGE_PROG;
									PageBufferPtr = 0;
									TransmitResponsePacket("$OK2");
								} 
								else 
								{
								// Should never get here
								}
			break;

			case CMD_BOOT:
								TransmitResponsePacket("$OK3");
								blState = BOOT;

			break;

			default:
			break;
		}
		
		if (blState == PAGE_PROG)
		{
//			copy_Buffer(&PageBuffer[PageBufferPtr], CAN_RX.Data, CAN_RX.DLC);
//			PageBufferPtr += CAN_RX.DLC;

			if (PageBufferPtr == FLASH_PAGE_SIZE) 
			{
				NVIC_DisableIRQ(USART1_IRQn);
					
				uint32_t crc = CRC_Calculate((uint32_t*)PageBuffer, FLASH_PAGE_SIZE / 4);

				if (PageIndex <= NUM_OF_FREE_PAGES)
				{
					
					if (crc == PageCRC)
					{
						FLASH_Unlock();

						FLASH_ErasePage(MAIN_PROGRAM_START_ADDRESS + PageIndex * FLASH_PAGE_SIZE);

						for (int i = 0; i < FLASH_PAGE_SIZE; i += 4)
						{
							FLASH_ProgramWord(MAIN_PROGRAM_START_ADDRESS + PageIndex * FLASH_PAGE_SIZE + i, *(uint32_t*)&PageBuffer[i]);
						}

						FLASH->CR |= FLASH_CR_LOCK;

						TransmitResponsePacket("$OK2");
					}
					else 
					{
						TransmitResponsePacket("$ER1");
					}
				}
				else
				{
					TransmitResponsePacket("$ER2");
				}

				blState = IDLE;

				NVIC_EnableIRQ(USART1_IRQn);
			}
		}

		
		
		if (blState == BOOT) 
		{
			delay_ms(50);
			JumpToApplication();
		}
	}
}

__INLINE void DeInit(void)
{
	CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
	CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
	CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_CANEN);
	
	CLEAR_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
	CLEAR_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);
	CLEAR_BIT(RCC->AHBENR, RCC_AHBENR_CRCEN);
}

__INLINE void JumpToApplication(void)
{
	__disable_irq();
	JumpAddress = *(__IO pFunction*)(MAIN_PROGRAM_START_ADDRESS + 4);
	__set_MSP(*(__IO uint32_t*) MAIN_PROGRAM_START_ADDRESS);
	DeInit();
	JumpAddress();
}

__INLINE void FLASH_Unlock(void)
{
	if((FLASH->CR & FLASH_CR_LOCK) != RESET)
	{
		FLASH->KEYR = FLASH_FKEY1;
		FLASH->KEYR = FLASH_FKEY2;
	}
}

FLASH_Status FLASH_GetStatus(void)
{
	FLASH_Status FLASHstatus = FLASH_COMPLETE;

	if((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) 
	{
		FLASHstatus = FLASH_BUSY;
	}
	else 
	{  
		if((FLASH->SR & (uint32_t)FLASH_SR_WRPERR)!= (uint32_t)0x00)
		{ 
			FLASHstatus = FLASH_ERROR_WRP;
		}
		else 
		{
			if((FLASH->SR & (uint32_t)(FLASH_SR_PGERR)) != (uint32_t)0x00)
			{
				FLASHstatus = FLASH_ERROR_PROGRAM; 
			}
			else
			{
				FLASHstatus = FLASH_COMPLETE;
			}
		}
	}
	/* Return the FLASH Status */
	return FLASHstatus;
}

FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout)
{ 
	FLASH_Status status = FLASH_COMPLETE;

	/* Check for the FLASH Status */
	status = FLASH_GetStatus();

	/* Wait for a FLASH operation to complete or a TIMEOUT to occur */
	while((status == FLASH_BUSY) && (Timeout != 0x00))
	{
		status = FLASH_GetStatus();
		Timeout--;
	}

	if(Timeout == 0x00 )
	{
		status = FLASH_TIMEOUT;
	}
	/* Return the operation status */
	return status;
}

FLASH_Status FLASH_ErasePage(uint32_t Page_Address)
{
	FLASH_Status status = FLASH_COMPLETE;

	/* Wait for last operation to be completed */
	status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

	if(status == FLASH_COMPLETE)
	{ 
		/* If the previous operation is completed, proceed to erase the page */
		FLASH->CR |= FLASH_CR_PER;
		FLASH->AR  = Page_Address;
		FLASH->CR |= FLASH_CR_STRT;

		/* Wait for last operation to be completed */
		status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

		/* Disable the PER Bit */
		FLASH->CR &= ~FLASH_CR_PER;
	}

	/* Return the Erase Status */
	return status;
}

FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data)
{
	FLASH_Status status = FLASH_COMPLETE;
	__IO uint32_t tmp = 0;

	/* Wait for last operation to be completed */
	status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

	if(status == FLASH_COMPLETE)
	{
		/* If the previous operation is completed, proceed to program the new first 
		half word */
		FLASH->CR |= FLASH_CR_PG;

		*(__IO uint16_t*)Address = (uint16_t)Data;

		/* Wait for last operation to be completed */
		status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

		if(status == FLASH_COMPLETE)
		{
			/* If the previous operation is completed, proceed to program the new second 
			half word */
			tmp = Address + 2;

			*(__IO uint16_t*) tmp = Data >> 16;

			/* Wait for last operation to be completed */
			status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

			/* Disable the PG Bit */
			FLASH->CR &= ~FLASH_CR_PG;
		}
		else
		{
			/* Disable the PG Bit */
			FLASH->CR &= ~FLASH_CR_PG;
		}
	}

	/* Return the Program Status */
	return status;
}

__INLINE void CRC_Init()
{	
	SET_BIT(RCC->AHBENR, RCC_AHBENR_CRCEN);
	
	CRC->INIT = 0xFFFFFFFFU;
}

uint32_t CRC_Calculate(uint32_t pBuffer[], uint32_t BufferLength)
{
	uint32_t index; /* CRC input data buffer index */

	CRC->CR |= CRC_CR_RESET;

	for (index = 0U; index < BufferLength; index++)
	{
		CRC->DR = pBuffer[index];
	}

	return CRC->DR; /* Return the CRC computed value */
}

void delay_ms(uint32_t milliseconds) 
{
	uint32_t start = ticks_delay;
	while((ticks_delay - start) < milliseconds);
}

__INLINE void Remap_pin(void)
{
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
	
	#if defined (STM32F042x4) || defined (STM32F042x6)
		SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;
	#endif	
}

__INLINE void Init_RCC(void)
{
	NVIC_EnableIRQ(RCC_CRS_IRQn); /* Enable Interrupt on RCC*/
	NVIC_SetPriority(RCC_CRS_IRQn,0); /* Set priority for RCC */

	RCC->CIR |= RCC_CIR_HSERDYIE; /* Enable interrupt on HSE ready */  
	RCC->CR |= RCC_CR_CSSON | RCC_CR_HSEON; /* Enable the CSS, Enable HSE */  

	if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) /* Test if PLL is used as System clock */
	{          
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE) /* Wait for HSE switched */
		{
				
		}
	}
	RCC->CR &= (uint32_t)(~RCC_CR_PLLON);/* Disable the PLL */        
	while((RCC->CR & RCC_CR_PLLRDY) != 0) /* Wait until PLLRDY is cleared */
	{
		
	}
	RCC->CFGR &= ~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLSRC);
	RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL6); /* HSE clock selected as PLL entry clock source, Set the PLL multiplier to 6 */
	RCC->CR |= RCC_CR_PLLON; /* Enable the PLL */
	while((RCC->CR & RCC_CR_PLLRDY) == 0) /* Wait until PLLRDY is set */
	{

	}
	RCC->CFGR &= (uint32_t) (~RCC_CFGR_SW);
	RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_PLL); /* Select PLL as system clock */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) /* Wait until the PLL is switched on */
	{

	}

	RCC->CFGR &= ~RCC_CFGR_HPRE;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	RCC->CFGR &= ~RCC_CFGR_PPRE;
	RCC->CFGR |= RCC_CFGR_PPRE_DIV2;	
}

__INLINE void Configure_GPIO_CAN(void)
{
	#if defined(CAN_PIN_PORTA)
		/* Enable the peripheral clock of GPIOA */
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

		/* GPIO configuration for CAN signals */
		/* (1) Select AF mode (10) on PA11 and PA12 */
		/* (2) AF4 for CAN signals */
		GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12))\
					 | (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1); /* (1) */
		GPIOA->AFR[1] = (GPIOA->AFR[1] &~ (GPIO_AFRH_AFRH3 | GPIO_AFRH_AFRH4))\
					  | (4 << (3 * 4)) | (4 << (4 * 4)); /* (2) */
	#elif defined(CAN_PIN_PORTB)
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
		GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9))\
					 | (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);
		GPIOB->AFR[1] = (GPIOB->AFR[1] &~ (GPIO_AFRH_AFRH0 | GPIO_AFRH_AFRH1))\
					  | (4 << (0 * 4)) | (4 << (1 * 4));
	#endif	
}

__INLINE void Configure_GPIO_USART1(void)
{
	/* Enable the peripheral clock of GPIOA */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* GPIO configuration for USART1 signals */
	/* (1) Select AF mode (10) on PA9 and PA10 */
	/* (2) AF1 for USART1 signals */
	GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9|GPIO_MODER_MODER10))\
					| (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1); /* (1) */
	GPIOA->AFR[1] = (GPIOA->AFR[1] &~ (GPIO_AFRH_AFRH1 | GPIO_AFRH_AFRH2))\
					| (1 << (1 * 4)) | (1 << (2 * 4)); /* (2) */
}

/**
  * @brief  This function configures USART1.
  * @param  None
  * @retval None
  */
__INLINE void Configure_USART1(void)
{
	/* Enable the peripheral clock USART1 */
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	/* Configure USART1 */
	/* (1) oversampling by 16, 9600 baud */
	/* (2) 8 data bit, 1 start bit, 1 stop bit, no parity, reception mode */
	USART1->BRR = 480000 / 96; /* (1) */
	USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE; /* (2) */

	/* Configure IT */
	/* (3) Set priority for USART1_IRQn */
	/* (4) Enable USART1_IRQn */
	NVIC_SetPriority(USART1_IRQn, 0); /* (3) */
	NVIC_EnableIRQ(USART1_IRQn); /* (4) */
}

void clear_Buffer(uint8_t *BUFER, uint16_t size)
{
	uint16_t i = 0;
	for (i = 0; i < size; i++) BUFER[i] = 0;
}

void copy_Buffer(uint8_t *dst, uint8_t *src, uint16_t size)
{
	for(uint16_t i = 0; i < size; i++ )
		((uint8_t*)dst)[i] = ((uint8_t*)src)[i];
}


/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

void SVC_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void NMI_Handler(void)
{
	if ((RCC->CIR & RCC_CIR_CSSF) != 0)
	{
	/* Report the error */
		RCC->CIR |= RCC_CIR_CSSC; /* Clear the flag */
	}
}

void RCC_CRS_IRQHandler(void)
{
	if ((RCC->CIR & RCC_CIR_HSERDYF) != 0) /* Check the flag HSE ready */
	{
		RCC->CIR |= RCC_CIR_HSERDYC; /* Clear the flag HSE ready */    
		RCC->CFGR = ((RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_0); /* Switch the system clock to HSE */
	}
	else
	{
	/* Report an error */
	}
}

void SysTick_Handler(void)
{
	ticks_delay++;
}

void USART1_IRQHandler(void)
{
	uint8_t chartoreceive = 0;

	if((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
	{
		chartoreceive = (uint8_t)(USART1->RDR); /* Receive data, clear flag */
		
		// Идентификатор посылки от 0 сервера
		switch (chartoreceive)
		{
			case '$' : search++; break;
			case 'F' : search == 1 ? (search++) : (search = 0); break;
			case 'W' : search == 2 ? (search++) : (search = 0); break;
			
			default: search = 0;
		}
		if (search == 3) 
		{ 
			command = 0x01;	
			search = 0;			
		}
		
		if (command == 0x00)
		{
			Data_CRC[crc_cnt++] = chartoreceive;
			if (crc_cnt == 0x05)
			{
				command = CMD_PAGE_PROG;
				crc_cnt = 0;
			}
		}
		
		if (search_COMMAND == 2) 
		{ 
			command = chartoreceive;	
			search_COMMAND = 0;			
		}
		
		switch (chartoreceive)
		{
			case '$' : search_COMMAND++; break;
			case 'C' : search_COMMAND == 1 ? (search_COMMAND++) : (search_COMMAND = 0); break;
			
			default: search_COMMAND = 0;
		}

		
		// Идентификатор хорошего ответа
		switch (chartoreceive)
		{
			case '>' : search_ARROW++; break;
			case ' ' : search_ARROW == 1 ? (search_ARROW++) : (search_ARROW = 0); break;
			
			default: search_ARROW = 0;
		}
		if (search_ARROW == 2) 
		{
			ready_to_send = 1;
			search_ARROW = 0;
		}
		// ************************************

		if (RECEIVE1)
		{

			if (chartoreceive != ':' && size_of_Page_parcel_cnt < 4) 
			{
				size_of_Page_arr[size_of_Page_parcel_cnt++] = chartoreceive;		
			}
			else
			{
				size_of_Page = char_to_int((char *)size_of_Page_arr);
				if (size_of_Page > FLASH_PAGE_SIZE)
				{
					size_of_Page = 0;
				}
				RECEIVE1 = 0;
				size_of_Page_parcel_cnt = 0;
				clear_Buffer(size_of_Page_arr, 4);
			}
				
		}
		
		if (size_of_Page)
		{
			received_Page_bytes++;
			if (received_Page_bytes > 3)
			{
				PageBuffer[Page_buf_cnt++] = chartoreceive;
				
			}
					
			if (Page_buf_cnt == size_of_Page)
			{
				PageBufferPtr = Page_buf_cnt;
				size_of_Page = 0;
				Page_buf_cnt = 0;
				received_Page_bytes = 0;
			}
		}
		
		if (blState == PAGE_PROG)
		{
			// Идентификатор посылки от 0 сервера
			switch (chartoreceive)
			{
				case '+' : search_RECEIVE1++; break;
				case 'R' : search_RECEIVE1 == 1 ? (search_RECEIVE1++) : (search_RECEIVE1 = 0); break;
				case 'E' : search_RECEIVE1 == 2 || search_RECEIVE1 == 4 || search_RECEIVE1 == 7 ? (search_RECEIVE1++) : (search_RECEIVE1 = 0); break;
				case 'C' : search_RECEIVE1 == 3 ? (search_RECEIVE1++) : (search_RECEIVE1 = 0); break;
				case 'I' : search_RECEIVE1 == 5 ? (search_RECEIVE1++) : (search_RECEIVE1 = 0); break;
				case 'V' : search_RECEIVE1 == 6 ? (search_RECEIVE1++) : (search_RECEIVE1 = 0); break;
				case ',' : search_RECEIVE1 == 8 || search_RECEIVE1 == 10 ? (search_RECEIVE1++) : (search_RECEIVE1 = 0); break;
				case '1' : search_RECEIVE1 == 9 ? (search_RECEIVE1++) : (search_RECEIVE1 = 0); break;
				
				default: search_RECEIVE1 = 0;
			}
			if (search_RECEIVE1 == 11) 
			{ 
				RECEIVE1 = 1;			
				search_RECEIVE1 = 0;
			}
		}		
	}
}
