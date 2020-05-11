#include "stm32f0xx.h"

/*
 * Pinout
 * PB.1 - LED
 * PA.0 - X Axis
 * PA.1 - Y Axis
 * PA.7 - Servo SG90 (Timer 17 / CH1 / GPIO AF5)
 */


// Define HSE crystal and calc PLL multiply for 48Mhz
#define HSE_MHZ_VALUE 4UL
#define RCC_CFGR_PLLMULX (((48UL / HSE_MHZ_VALUE) - 2UL) << 18)

// Defines blink rate interval
#define BLINK_INTERVAL 500

// Keep elapsed millis
volatile uint32_t Millis = 0;

// X Axis, Y Axis, Temperature, Vref
uint16_t ADC_array[4];

// Function prototypes
void GPIO_Config(void);
void SystemInit(void); // !!! This function is called before main in startup file
void ADC_Config(void);
void DMA_Config(void);
void PWM_Config(void);

// Main function
int main(void) {
	uint32_t hbeat, frate;

	GPIO_Config();
	ADC_Config();
	DMA_Config();
	PWM_Config();

	ADC1->CR |= ADC_CR_ADSTART;

	hbeat = Millis;
	frate = Millis;

	for (;;) {
		// Toggle led
		if ((Millis - hbeat) > BLINK_INTERVAL) {
			hbeat = Millis;
			GPIOB->ODR ^= GPIO_ODR_1;
		}

		// Update servo on 5ms
		if ((Millis - frate) > 5) {
			frate = Millis;
			TIM17->CCR1 = (uint16_t) (((ADC_array[0] * 200) / 4095) + 50);
		}
	}
}

// Configure system clock to use PLL from HSE to 48Mhz from 4Mhz crystal
// !!! This function is called before main in startup file
void SystemInit(void) {
	// Flash prefetch buffer enable
	FLASH->ACR |= FLASH_ACR_PRFTBE;

	// System configuration controller clock enable
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Power interface clock enable
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	// System Clock > 24Mhz so latency must be set to 1
	if ((FLASH->ACR & FLASH_ACR_LATENCY) == 0) {
		FLASH->ACR |= 1UL;
	}

	// Enable HSE and wait to stabilize
	if ((RCC->CR & RCC_CR_HSEON) != RCC_CR_HSEON) {
		RCC->CR |= RCC_CR_HSEON;
		while ((RCC->CR & RCC_CR_HSERDY) == 0)
			;
	}

	// Configure PLL to bring 48Mhz system clock
	if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
		// Disable PLL. Required before configure.
		RCC->CR &= ~(RCC_CR_PLLON);

		// Wait to stop
		while ((RCC->CR & RCC_CR_PLLRDY) != 0)
			;

		// Remove predivider for PLL
		RCC->CFGR2 = 0UL;

		// Set multiplier of 12 and PLL source from HSE
		MODIFY_REG(RCC->CFGR, (RCC_CFGR_PLLMUL | RCC_CFGR_PLLSRC),
				(RCC_CFGR_PLLMULX | RCC_CFGR_PLLSRC_HSE_PREDIV));

		// Enable PLL
		RCC->CR |= RCC_CR_PLLON;

		// Wait to stabilize
		while ((RCC->CR & RCC_CR_PLLRDY) == 0)
			;
	}

	// Set APB(PCLK) and AHB prescallers to 1
	RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE);

	// Set PLL as system clock
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

	// Wait to switch PLL as system clock
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
		;

	// Set and start system tick timer with 1ms interval
	SysTick_Config(48000);

	// Enable system tick timer IRQ
	NVIC_SetPriority(SysTick_IRQn, 0);
	NVIC_EnableIRQ(SysTick_IRQn);
}

void GPIO_Config(void) {
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

/*
 * tSAR = 12.5
 * tSMPL = 239.5
 * tCONV = 21 us / per channel
 */
void ADC_Config(void) {

	// Enable ADC clock
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	// Set ADC clock source PCLK / 4 = 12Mhz
	MODIFY_REG(ADC1->CFGR2, ADC_CFGR2_CKMODE, ADC_CFGR2_CKMODE_1);

	// Stop before calibrate
	if ((ADC1->CR & ADC_CR_ADEN) != 0) {
		ADC1->CR &= ~ADC_CR_ADEN;
	}

	// Calibrate
	ADC1->CR |= ADC_CR_ADCAL;
	while ((ADC1->CR & ADC_CR_ADCAL) != 0)
		;

	// PA.0 PA.1 - Analog
	GPIOA->MODER |= (GPIO_MODER_MODER0 | GPIO_MODER_MODER1);

	// Enable ADC
	ADC1->CR |= ADC_CR_ADEN;
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0)
		;

	// Continuous conversion mode / Overrun management enable
	ADC1->CFGR1 |= (ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD);

	// Sequence channel selection
	// Channel 16 - Internal Temperature sensor
	// Channel 17 - Vref
	ADC1->CHSELR |= (ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL16
			| ADC_CHSELR_CHSEL17);

	// Set sample rate 239.5 ADC clock cycles
	ADC1->SMPR |= (ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2);

	// Temperature sensor enable, VREFINT enable
	ADC->CCR |= (ADC_CCR_TSEN | ADC_CCR_VREFEN);
}

void DMA_Config(void) {
	RCC->AHBENR |= RCC_AHBENR_DMAEN;
	ADC1->CFGR1 |= ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN;

	DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR));
	DMA1_Channel1->CMAR = (uint32_t) (ADC_array);
	DMA1_Channel1->CNDTR = 4;
	DMA1_Channel1->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_CIRC
			| DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
	DMA1_Channel1->CCR |= DMA_CCR_EN;

	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
}

void PWM_Config(void) {
	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER7, GPIO_MODER_MODER7_1); // PA7 - AF
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7;					// High speed
	GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFRL7_Pos);

	TIM17->PSC = 479; // 100 000 Hz / 0.01ms / clk
	TIM17->ARR = 2000; // 50Hz / 20ms
	TIM17->CCR1 = 150; // Compare initial value [50 .. 250]
	TIM17->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM17->CCER |= TIM_CCER_CC1E;
	TIM17->BDTR |= TIM_BDTR_MOE;
	TIM17->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

void ADC1_IRQHandler(void) {
	// Global ADC handler
}

void DMA1_Channel1_IRQHandler(void) {
	if ((DMA1->ISR & DMA_ISR_TEIF1) != 0) {
		// DMA Transfer error handler
		DMA1->IFCR |= DMA_IFCR_CTEIF1;
	}

	if ((DMA1->ISR & DMA_ISR_TCIF1) != 0) {
		// DMA Transfer complete handler
		// !! This sould be invoked on every 84uS with 4 adc channels
		DMA1->IFCR |= DMA_IFCR_CTCIF1;
	}
}

void SysTick_Handler(void) {
	Millis++;
}
