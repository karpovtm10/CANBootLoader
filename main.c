#include "stm32f0xx.h"

#define DEVICE_CAN_ID				0x703U //0x701 - рама, 0x702 - стрела, 0x703 - отвал
#define NUM_OF_PAGES				31 //Количество страниц в МК 	STM32F042F4 - 15
																											//STM32F042F6 - 31
																											//STM32F072C8 - 31
																											//STM32F072CB - 63

#define CAN_PIN_PORTA
#define HELLO_BRUNCH				222

#define DELETE_HELLO_MAIN			0xFF
#define CREATE_HELLO_MAIN			0xAA
#define CHECK_HELLO_MAIN			0xBB

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
	
volatile	uint32_t 				ticks_delay = 0;
volatile 	uint8_t 				CAN_TX_Data[8];
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

typedef struct
{
	uint32_t StdId;
	uint32_t ExtId;
	uint8_t IDE;
	uint8_t Data[8];
	uint32_t DLC;
} CanRxMsg;

CanRxMsg CAN_RX;

typedef struct
{
	uint32_t StdId; 
	uint32_t ExtId; 
	uint8_t IDE;
	uint8_t Data[8];
} CanTxMsg;

CanTxMsg CAN_TX;

void Init_RCC(void);
void ConfigureGPIO(void);
void Configure_GPIO_CAN(void);
void Configure_CAN(void);
void CAN_Transmit(void);
void delay_ms(uint32_t milliseconds);
void Remap_pin(void);
void CRC_Init(void);
uint32_t CRC_Calculate(uint32_t pBuffer[], uint32_t BufferLength);
void DeInit(void);
void JumpToApplication(void);

int main(void)
{
	SysTick_Config(48000);/* 1ms config with HSE 8MHz*/
	Init_RCC();
	Remap_pin();

	Configure_GPIO_CAN();
	Configure_CAN();
	CRC_Init();
	
	delay_ms(500);

	if (blState == WAIT_HOST) 
	{
		JumpToApplication();
	}
	
	while (1)
	{
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

void TransmitResponsePacket(uint8_t response)
{
	CAN_TX_Data[0] = response;
	
	CAN_Transmit();	
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

void CAN_Transmit(void)
{
	if((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0)/* check mailbox 0 is empty */
	{
		CAN->sTxMailBox[0].TDTR = 8; /* fill data length = 1 */
		
		CAN->sTxMailBox[0].TDLR = (	((uint32_t)CAN_TX_Data[3] << 24) | 
									((uint32_t)CAN_TX_Data[2] << 16) |
									((uint32_t)CAN_TX_Data[1] << 8) | 
									((uint32_t)CAN_TX_Data[0]));
		CAN->sTxMailBox[0].TDHR = (	((uint32_t)CAN_TX_Data[7] << 24) | 
									((uint32_t)CAN_TX_Data[6] << 16) |
									((uint32_t)CAN_TX_Data[5] << 8) |
									((uint32_t)CAN_TX_Data[4]));
		
		CAN->sTxMailBox[0].TIR = (uint32_t)(DEVICE_CAN_TX_ID << 21 | CAN_TI0R_TXRQ); /* fill Id field and request a transmission */
	}
	else
	{
		CAN->TSR |= CAN_TSR_ABRQ0; /* abort transmission if not empty */
	}	
}

void CAN_Receive(CanRxMsg* RxMessage)
{
	RxMessage->IDE = (uint8_t)0x04 & CAN->sFIFOMailBox[0].RIR;

	if (RxMessage->IDE == CAN_Id_Standard)
		RxMessage->StdId = (uint32_t)0x000007FF & (CAN->sFIFOMailBox[0].RIR >> 21);
	else if (RxMessage->IDE == CAN_Id_Extended)
		RxMessage->ExtId = (uint32_t)0x1FFFFFFF & (CAN->sFIFOMailBox[0].RIR >> 3);

	RxMessage->DLC = (uint8_t)0x0F & CAN->sFIFOMailBox[0].RDTR;

	RxMessage->Data[0] = (uint8_t)0xFF & CAN->sFIFOMailBox[0].RDLR;
	RxMessage->Data[1] = (uint8_t)0xFF & (CAN->sFIFOMailBox[0].RDLR >> 8);
	RxMessage->Data[2] = (uint8_t)0xFF & (CAN->sFIFOMailBox[0].RDLR >> 16);
	RxMessage->Data[3] = (uint8_t)0xFF & (CAN->sFIFOMailBox[0].RDLR >> 24);
	RxMessage->Data[4] = (uint8_t)0xFF & CAN->sFIFOMailBox[0].RDHR;
	RxMessage->Data[5] = (uint8_t)0xFF & (CAN->sFIFOMailBox[0].RDHR >> 8);
	RxMessage->Data[6] = (uint8_t)0xFF & (CAN->sFIFOMailBox[0].RDHR >> 16);
	RxMessage->Data[7] = (uint8_t)0xFF & (CAN->sFIFOMailBox[0].RDHR >> 24);

	CAN->RF0R |= CAN_RF0R_RFOM0; /* release FIFO */
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

__INLINE void Configure_CAN(void)
{
	/* Enable the peripheral clock CAN */
	RCC->APB1ENR |= RCC_APB1ENR_CANEN;

	/* Configure CAN */
	/* (1) Enter CAN init mode to write the configuration */
	/* (2) Wait the init mode entering */
	/* (3) Exit sleep mode */
	/* (4) Loopback mode, set timing to 1Mb/s: BS1 = 4, BS2 = 3, prescaler = 6 */
	/* (5) Leave init mode */
	/* (6) Wait the init mode leaving */
	/* (7) Enter filter init mode, (16-bit + mask, filter 0 for FIFO 0) */
	/* (8) Acivate filter 0 */
	/* (9) Identifier list mode */
	/* (12) Leave filter init */
	/* (13) Set FIFO0 message pending IT enable */
	CAN->MCR |= CAN_MCR_INRQ; /* (1) */
	while((CAN->MSR & CAN_MSR_INAK)!=CAN_MSR_INAK) /* (2) */
	{ 
	/* add time out here for a robust application */
	}
	CAN->MCR |= CAN_MCR_ABOM;
	CAN->MCR &=~ CAN_MCR_SLEEP; /* (3) */
	CAN->BTR = 0;
	CAN->BTR |= 3 << 20 | 2 << 16 | 11 << 0; /* (4) */ 
	CAN->MCR &=~ CAN_MCR_INRQ; /* (5) */
	while((CAN->MSR & CAN_MSR_INAK)==CAN_MSR_INAK) /* (6) */
	{ 
	/* add time out here for a robust application */
	}  
	CAN->FMR = CAN_FMR_FINIT; /* (7) */ 
	CAN->FA1R = CAN_FA1R_FACT0; /* (8) */

	CAN->FM1R = CAN_FM1R_FBM0; /* (9) */
	CAN->sFilterRegister[0].FR1 = 0 << 5 | DEVICE_CAN_ID << (16+5); /* (10) */ 


	CAN->FMR &=~ CAN_FMR_FINIT; /* (12) */
	CAN->IER |= CAN_IER_FMPIE0; /* (13) */

	/* Configure IT */
	/* (14) Set priority for CAN_IRQn */
	/* (15) Enable CAN_IRQn */
	NVIC_SetPriority(CEC_CAN_IRQn, 0); /* (16) */
	NVIC_EnableIRQ(CEC_CAN_IRQn); /* (17) */
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

void CEC_CAN_IRQHandler(void)
{
	CAN_Receive(&CAN_RX);

	if (CAN_RX.StdId != DEVICE_CAN_ID) 
	{
		return;
	}

	if (blState == PAGE_PROG)
	{
		copy_Buffer(&PageBuffer[PageBufferPtr], CAN_RX.Data, CAN_RX.DLC);
		PageBufferPtr += CAN_RX.DLC;
		
		if (PageBufferPtr == FLASH_PAGE_SIZE) 
		{
			NVIC_DisableIRQ(CEC_CAN_IRQn);
				
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

					TransmitResponsePacket(CAN_RESP_OK);
				}
				else 
				{
					TransmitResponsePacket(CAN_RESP_ERROR_CRC);
				}
			}
			else
			{
				TransmitResponsePacket(CAN_RESP_ERROR_Memory);
			}

			blState = IDLE;

			NVIC_EnableIRQ(CEC_CAN_IRQn);
		}
	CAN_Receive(&CAN_RX);
	return;
	}

	switch(CAN_RX.Data[0])
	{
		case CMD_HOST_INIT:
							blState = IDLE;
							TransmitResponsePacket(CAN_RESP_OK);
		break;
		
		case CMD_PAGE_PROG:
							if (blState == IDLE) 
							{
								clear_Buffer(PageBuffer, countof(PageBuffer)); //заполнения массива нулями
								PageCRC = CAN_RX.Data[5] << 24 | CAN_RX.Data[4] << 16 | CAN_RX.Data[3] << 8 | CAN_RX.Data[2];
								PageIndex = CAN_RX.Data[1];
								blState = PAGE_PROG;
								PageBufferPtr = 0;
							} 
							else 
							{
							// Should never get here
							}
		break;
		
		case CMD_BOOT:
							TransmitResponsePacket(CAN_RESP_OK);
							blState = BOOT;
	  
		break;
		
		default:
		break;
	}
}
