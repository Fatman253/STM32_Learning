#include "main.h"
#include "usb_device.h"
#include "stm32f1xx.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
#define micros()  ((HAL_GetTick() * 1000) + ((72000 - SysTick->VAL) / 72))
#define millis()  HAL_GetTick();

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config_HSE_72MHz();
void Error_Handler(void);


int main(void)
{

  HAL_Init();
  SystemClock_Config_HSE_72MHz();
  MX_USB_DEVICE_Init();
  
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  GPIOB->CRL &= ~(0xF << 0);    // Clear old settings
  GPIOB->CRL |= (0xB << 0);     // Set 0xB (Binary 1011)
  // 10 = Alternate Function Push Pull
  // 11 = OUTPUT Mode 50 Mhz Max

  TIM3->PSC = 71;    // Prescaler: 72MHz / (71+1) = 1 MHz Timer Clock (1us per tick)
    TIM3->ARR = 1000;  // Auto Reload: 1 MHz / 1000 = 1000 Hz (1 kHz PWM freq)

    // 5. Configure Channel 3 (PB0) for PWM Mode 1
    // CCMR2 controls Channel 3 and 4. 
    // OC3M bits (6:4) = 110 (PWM Mode 1)
    // OC3PE bit (3)   = 1   (Preload Enable)
    TIM3->CCMR2 |= (6 << 4) | (1 << 3);

    // 6. Enable Output for Channel 3
    // CCER register, Bit 8 (CC3E)
    TIM3->CCER |= (1 << 8);

    // 7. Set Duty Cycle (Brightness)
    // CCR3 is the "Compare" value for Channel 3.
    // 500 / 1000 (ARR) = 50% Duty Cycle
    TIM3->CCR3 = 100;

    // 8. Enable the Timer Counter
    TIM3->CR1 |= TIM_CR1_CEN;


  while (1)
  {
    HAL_GetTick();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void SystemClock_Config_HSE_72MHz(){

  /* Setting Core Clock
  1. Use HSE Clock (External CLock) 8 Mhz
  2. Use PLL
  3. Setting Multiplier 9. 8 * 9 = 72 Mhz -> Getting Max Freq
  4. Setting Switch System Clock to PLLCLK -> Sysclk = 72 Mhz
  5. Setting AHB /1 -> HCLK -> 72 Mhz
  6. Setting APB1 /2 -> APB1 -> 36 Mhz
  7. Setting APB2 /1 -> APB2 -> 72 Mhz
  8. Setting USB /1.5 -> USB -> 48 Mhz
  */


  // 1. Enable HSE and Wait Ready
  RCC->CR |= RCC_CR_HSEON; // Bit 16 (HSE ON)
  while(!(RCC->CR & RCC_CR_HSERDY)); // BIT 17 (HSE_Ready)

  // 2. Configure Flash
  FLASH->ACR |= FLASH_ACR_PRFTBE;
  FLASH->ACR &= ~FLASH_ACR_LATENCY;
  FLASH->ACR |= FLASH_ACR_LATENCY_2;

  // 3. Configure Prescalers (AHB, APB1, APB2, USB)
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // BIT 7:4 0000 (AHB /1)
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 /2 So it get 36Mhz(APB1 Max)
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // APB2 /1 So it get 72Mhz(APB2 Max)
  RCC->CFGR &= ~(RCC_CFGR_USBPRE); // USB Prescaler /1.5 So it get 48Mhz(USB Max)

  // 4. Configure PLL
  RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL | RCC_CFGR_PLLXTPRE);
  RCC->CFGR |= RCC_CFGR_PLLMULL9; //PLL Multiply 9 (8 * 9 = 72 Mhz)
  RCC->CFGR |= RCC_CFGR_PLLSRC; // PLL Select Source HSE (0 = HSI, 1 = HSE)
  // PLLXTPRE (HSE Prediv /1) (0 = /1, 1 = /2)

  // 5. Enable PLL
  RCC->CR |= RCC_CR_PLLON; // BIT 24 PLL On
  while(!(RCC->CR & RCC_CR_PLLRDY)); // BIT 25 PLL Ready

  // 6. Switch System Clock to PLL
  RCC->CFGR &= ~(RCC_CFGR_SW); // Reset Switch State
  RCC->CFGR |= RCC_CFGR_SW_PLL; // Switch Select PLL
  while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait BIT 3:2 Switch Clock Ready

  // 7. Update HAL Global Variable
  SystemCoreClock = 72000000;

  // 8. Re-configure SysTick for 72MHz (1ms tick)
  // Without this, HAL_Delay(1000) will only wait ~110ms
  SysTick_Config(72000000 / 1000);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
      // If you get stuck here, something went wrong with HAL initialization
  }
}