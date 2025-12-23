#include "main.h"
#include "stm32f1xx.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "string.h"  // <--- ADD THIS
#include "stdio.h"   // <--- ADD THIS (Optional, for sprintf)



/* Private typedef -----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct __attribute__((packed)) {
    uint32_t timestamp;   // 4 Bytes (millis)
    uint16_t pwm_value;   // 2 Bytes (CCR3)
    uint8_t  status_flag; // 1 Byte  (Example: On/Off)
} RobotState;             // Total Size: 7 Bytes
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
#define micros()  ((HAL_GetTick() * 1000) + ((72000 - SysTick->VAL) / 72))
#define millis()  HAL_GetTick();

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config_HSE_72MHz();
void Error_Handler(void);

void USB_ForceReEnumeration(void) {
    // 1. Enable Port A Clock
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // 2. Configure PA12 as General Purpose Output Push-Pull
    // PA12 is in CRH (Control Register High)
    // Clear bits 19:16 (for PA12)
    GPIOA->CRH &= ~(0xF << 16); 
    // Set to 0x3 (Binary 0011): Output mode 50 MHz, Push-Pull
    GPIOA->CRH |= (0x3 << 16); 

    // 3. Drive PA12 LOW (Simulate unplugging)
    GPIOA->BRR = (1 << 12); 

    // 4. Wait for PC to notice (10ms is usually enough)
    HAL_Delay(10); 

    // 5. Important: Reset PA12 to Input/Floating state
    // So the USB hardware can take over later
    GPIOA->CRH &= ~(0xF << 16); // Clear bits
    GPIOA->CRH |= (0x4 << 16);  // 0x4 (Binary 0100): Floating Input
}

extern PCD_HandleTypeDef hpcd_USB_FS; // or hpcd_USB_OTG_FS for F4 chips

// Fast Send Function
void Fast_USB_Send(uint8_t *data, uint32_t len)
{
    // 1. Check if the USB Endpoint 1 (IN) is ready to send.
    // We check the hardware status directly, skipping middleware flags.
    // '0x81' is the standard address for CDC Data IN (Device -> PC)
    if ((hpcd_USB_FS.IN_ep[1].is_in == 1) && 
        (hpcd_USB_FS.IN_ep[1].xfer_count == 0)) 
    {
         // 2. Call the Low-Level Driver directly
         // This bypasses the USBD_CDC middleware overhead
         HAL_PCD_EP_Transmit(&hpcd_USB_FS, 0x81, data, len);
    }
}



int main(void)
{
  HAL_Init();
  SystemClock_Config_HSE_72MHz();
  SystemCoreClockUpdate();
  USB_ForceReEnumeration();
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

  // 1. Define a message buffer
    char msg[64]; 
  RobotState myRobot;
    // 2. Format the message (Example: Sending the current millis)
    // We use sprintf to combine text and numbers
  while (1)
  {
    
    // 1. Fill the struct with real data
    myRobot.timestamp   = HAL_GetTick();
    myRobot.pwm_value   = (uint16_t)TIM3->CCR3++;
    if(TIM3->CCR3 > 1000) TIM3->CCR3 = 0;
    myRobot.status_flag = 1;                    // Example status

    // 2. Send the Struct
    // We cast the ADDRESS of the struct (&myRobot) to a (uint8_t*) pointer.
    // We use sizeof(RobotState) so it knows exactly how many bytes to send.
    Fast_USB_Send((uint8_t*)&myRobot, sizeof(RobotState));

    // 3. Delay to prevent flooding
    HAL_Delay(50);
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
  RCC->CFGR |= (RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC); // PLL Select Source HSE (0 = HSI, 1 = HSE)
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