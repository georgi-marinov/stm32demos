#include "stm32f0xx.h"

// Define HSE crystal and calc PLL multiply for 48Mhz
#define HSE_MHZ_VALUE 4UL
#define RCC_CFGR_PLLMULX (((48UL / HSE_MHZ_VALUE) - 2UL) << 18)

// Defines blink rate interval
#define BLINK_INTERVAL 100

// Keep elapsed millis
volatile uint32_t Millis = 0;


// Function prototypes
void GPIO_Config(void);
void SystemInit(void); // !!! This function is called before main in startup file

// Main function
int main(void)
{
	uint32_t hbeat;

	GPIO_Config();

	hbeat = Millis;

	for(;;)
	{
		if((Millis - hbeat) > BLINK_INTERVAL)
		{
			hbeat = Millis;
			GPIOB->ODR ^= GPIO_ODR_1;
		}
	}
}

// Configure system clock to use PLL from HSE to 48Mhz from 4Mhz crystal
// !!! This function is called before main in startup file
void SystemInit(void)
{
	// Flash prefetch buffer enable
	FLASH->ACR |= FLASH_ACR_PRFTBE;

	// System configuration controller clock enable
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Power interface clock enable
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	// System Clock > 24Mhz so latency must be set to 1
	if((FLASH->ACR & FLASH_ACR_LATENCY) == 0)
	{
		FLASH->ACR |= 1UL;
	}

	// Enable HSE and wait to stabilize
	if((RCC->CR & RCC_CR_HSEON) != RCC_CR_HSEON)
	{
		RCC->CR |= RCC_CR_HSEON;
		while((RCC->CR & RCC_CR_HSERDY) == 0);
	}

	// Configure PLL to bring 48Mhz system clock
	if((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
	{
		// Disable PLL. Required before configure.
		RCC->CR &= ~(RCC_CR_PLLON);

		// Wait to stop
		while((RCC->CR & RCC_CR_PLLRDY) != 0);

		// Remove predivider for PLL
		RCC->CFGR2 = 0UL;

		// Set multiplier of 12 and PLL source from HSE
		MODIFY_REG(RCC->CFGR, (RCC_CFGR_PLLMUL | RCC_CFGR_PLLSRC), (RCC_CFGR_PLLMULX | RCC_CFGR_PLLSRC_HSE_PREDIV));

		// Enable PLL
		RCC->CR |= RCC_CR_PLLON;

		// Wait to stabilize
		while((RCC->CR & RCC_CR_PLLRDY) == 0);
	}

	// Set APB(PCLK) and AHB prescallers to 1
	RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE);

	// Set PLL as system clock
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

	// Wait to switch PLL as system clock
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	// Set and start system tick timer with 1ms interval
	SysTick_Config(48000);

	// Enable system tick timer IRQ
	NVIC_SetPriority(SysTick_IRQn, 0);
	NVIC_EnableIRQ(SysTick_IRQn);
}

void GPIO_Config(void)
{
	// Enable GPIOA and GPIOB clock
	RCC->AHBENR |= (RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOAEN);

	// Set GPIOB.1 Output
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER1, GPIO_MODER_MODER1_0);

	// GPIOB.1 - Push Pull
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_1);

	// GPIOB.1 - Low speed
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR1);

	// GPIOB.1 - No pullups
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
}



void SysTick_Handler(void)
{
	Millis++;
}
