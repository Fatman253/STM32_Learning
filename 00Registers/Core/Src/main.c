    #include "main.h"
    #include "stm32f1xx.h"
    #include "usb_device.h"
    #include "usbd_cdc_if.h"
    #include "string.h"  // <--- ADD THIS
    #include "stdio.h"   // <--- ADD THIS (Optional, for sprintf)



    /* Private typedef -----------------------------------------------------------*/
    /* Private typedef -----------------------------------------------------------*/

    // Page 63 (Last Page of 64KB Flash)
    #define FLASH_PAGE_ADDR  0x0800FC00 
    #define PAGE_SIZE        1024
    #define MAGIC_CODE       0xDEADBEEF // Safety check

    // Structure for Saved Settings
    typedef struct {
        uint16_t saved_pwm;
        uint32_t magic;      // To verify data validity
    } RobotConfig;


    typedef struct __attribute__((packed)) {
    uint32_t timestamp;   // 4 Bytes
    uint16_t pwm_value;   // 2 Bytes
    uint8_t  status_flag; // 1 Byte
    int16_t  encoder_pos; // 2 Bytes <--- ADD THIS (int16 allows negative numbers)
    } RobotState;          // Total Size: 7 Bytes

    // --- PID STRUCTURE ---
typedef struct {
    // Tuning Parameters
    float Kp;
    float Ki;
    float Kd;
    
    // Limits
    int16_t  output_limit;    // Max PWM (e.g., 1000)
    float    integral_limit;  // Prevent "Windup" (e.g., 800)
    
    // Internal State
    float    integral_sum;
    float    prev_error;
    int16_t  prev_encoder;    // To calculate speed
    float    filtered_speed;  // Smoothed speed (Crucial for low PPR)
    uint32_t last_time;       // To track 50ms loop
} SpeedPID;
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

    /* FLASH FUNCTIONS -----------------------------------------------------------*/

    // 1. Load Settings from Flash
    void Flash_Load_Smart(RobotConfig *cfg) {
        uint32_t curr_addr = FLASH_PAGE_ADDR;
        uint32_t last_valid_addr = 0;

        // Scan the page to find the LATEST valid entry
        while (curr_addr < (FLASH_PAGE_ADDR + PAGE_SIZE)) {
            RobotConfig *temp = (RobotConfig*)curr_addr;
            if (temp->magic == MAGIC_CODE) { 
                last_valid_addr = curr_addr; // Found one!
            } else if (temp->magic == 0xFFFFFFFF) {
                break; // Found empty space, stop searching
            }
            curr_addr += sizeof(RobotConfig); // Check next slot
        }

        if (last_valid_addr != 0) {
            *cfg = *(RobotConfig*)last_valid_addr; // Load data
        } else {
            cfg->saved_pwm = 100; // Default value if Flash is empty
        }
    }

    // 2. Save Settings to Flash (Smart Append)
    void Flash_Save_Smart(RobotConfig *cfg) {
        // 1. Find the next Empty Slot
        uint32_t dest_addr = FLASH_PAGE_ADDR;
        
        while (dest_addr < (FLASH_PAGE_ADDR + PAGE_SIZE)) {
            // If we find 0xFFFFFFFF, this slot is empty
            if (*(uint32_t*)dest_addr == 0xFFFFFFFF) {
                break;
            }
            dest_addr += sizeof(RobotConfig);
        }

        // --- THE FIX IS HERE ---
        // Check if adding our data size goes PAST the end of the page
        if (dest_addr + sizeof(RobotConfig) > (FLASH_PAGE_ADDR + PAGE_SIZE)) {
            
            // UNLOCK
            if (FLASH->CR & FLASH_CR_LOCK) {
                FLASH->KEYR = 0x45670123U;
                FLASH->KEYR = 0xCDEF89ABU;
            }

            // ERASE PAGE
            while (FLASH->SR & FLASH_SR_BSY);
            FLASH->CR |= FLASH_CR_PER; 
            FLASH->AR  = FLASH_PAGE_ADDR; 
            FLASH->CR |= FLASH_CR_STRT;
            while (FLASH->SR & FLASH_SR_BSY);
            FLASH->CR &= ~FLASH_CR_PER;
            
            // Reset to the beginning
            dest_addr = FLASH_PAGE_ADDR;
        }
        // -----------------------

        // 2. Write Data
        if (FLASH->CR & FLASH_CR_LOCK) {
            FLASH->KEYR = 0x45670123U;
            FLASH->KEYR = 0xCDEF89ABU;
        }

        FLASH->CR |= FLASH_CR_PG; // Program Mode
        uint16_t *data = (uint16_t*)cfg;
        
        // Write 16-bits at a time
        for (int i = 0; i < sizeof(RobotConfig) / 2; i++) {
            *(__IO uint16_t*)dest_addr = data[i];
            while (FLASH->SR & FLASH_SR_BSY);
            dest_addr += 2;
        }

        FLASH->CR &= ~FLASH_CR_PG; // Disable PG
        FLASH->CR |= FLASH_CR_LOCK; // Lock
    }

    void MX_TIM2_Encoder_Init(void) {
    // 1. Enable Clocks for TIM2 and GPIOA
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // Enable Timer 2 Clock
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // Enable Port A Clock (If not already enabled)

    // 2. Configure PA0 and PA1 as Input Floating
    // CRL Bits: PA0 (3:0), PA1 (7:4)
    // Mode = 00 (Input), CNF = 01 (Floating Input) -> 0x4
    GPIOA->CRL &= ~(0xFF << 0); // Clear settings for PA0 and PA1
    GPIOA->CRL |=  (0x44 << 0); // Set PA0 and PA1 to Input Floating (0x4)

    // 3. Configure Timer 2 for Encoder Mode
    // CCMR1 Register: Configure CC1S and CC2S bits
    // CC1S (1:0) = 01 (Map IC1 to TI1)
    // CC2S (9:8) = 01 (Map IC2 to TI2)
    TIM2->CCMR1 |= (1 << 0) | (1 << 8);

    // Optional: Add Input Filter (Debounce) if your encoder is noisy
    // IC1F (7:4) = 1111 (0xF) -> Max Filter
    // IC2F (15:12) = 1111 (0xF) -> Max Filter
    TIM2->CCMR1 |= (0xF << 4) | (0xF << 12);

    // CCER Register: Polarity (Standard is non-inverted, so leave 0)
    // TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);

    // SMCR Register: Select Encoder Mode 3
    // SMS (2:0) = 011 (Mode 3: Count on both TI1 and TI2 edges)
    TIM2->SMCR |= (3 << 0);

    // 4. Set Max Count (Auto Reload)
    TIM2->ARR = 0xFFFF; // 65535 (Full 16-bit range)
    
    // 5. Start the Timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

void MX_TIM3_PWM_Init(void) {
    // 1. Enable Clocks
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // Timer 3
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // Port A (PA4, PA5, PA6)
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;  // Port B (PB0)

    // 2. Configure GPIOs
    
    // --- PA4 & PA5 (GPIO Output Push-Pull) ---
    // PA4: CRL Bits 19:16 -> 0011 (0x3)
    // PA5: CRL Bits 23:20 -> 0011 (0x3)
    GPIOA->CRL &= ~(0xFF << 16); // Clear PA4, PA5
    GPIOA->CRL |=  (0x33 << 16); // Set Output 50MHz, Push-Pull

    // --- PA6 (PWM Output - TIM3_CH1) ---
    // PA6: CRL Bits 27:24 -> 1011 (0xB)
    // Mode=11 (50MHz), CNF=10 (Alt Func Push-Pull)
    GPIOA->CRL &= ~(0xF << 24);
    GPIOA->CRL |=  (0xB << 24);

    // --- PB0 (PWM Output - TIM3_CH3) --- 
    // (Keeping your old one if you still need it)
    GPIOB->CRL &= ~(0xF << 0);
    GPIOB->CRL |=  (0xB << 0);

    // 3. Configure TIM3 Time Base
    TIM3->PSC = 2;    // 72MHz / 72 = 1MHz Clock
    TIM3->ARR = 1000;  // 1MHz / 1000 = 1kHz PWM Frequency

    // 4. Configure Channel 1 (PA6)
    // CCMR1: OC1M=110 (PWM Mode 1), OC1PE=1 (Preload)
    TIM3->CCMR1 |= (6 << 4) | (1 << 3);
    // CCER: Enable Output (CC1E)
    TIM3->CCER |= TIM_CCER_CC1E;

    // 5. Configure Channel 3 (PB0 - Your old one)
    // CCMR2: OC3M=110 (PWM Mode 1), OC3PE=1
    TIM3->CCMR2 |= (6 << 4) | (1 << 3);
    // CCER: Enable Output (CC3E)
    TIM3->CCER |= TIM_CCER_CC3E;

    // 6. Start Timer
    TIM3->CR1 |= TIM_CR1_CEN;
}

// Valid Input: -1000 (Full Reverse) to 1000 (Full Forward)
// --- 1. INITIALIZE PID ---
void PID_Init(SpeedPID *pid, float p, float i, float d) {
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->output_limit = 1000; // Matches TIM3->ARR
    pid->integral_limit = 300.0f; 
    
    // Reset History
    pid->integral_sum = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_encoder = 0;
    pid->filtered_speed = 0.0f;
    pid->last_time = 0;
}

// --- 2. COMPUTE PID (Run this every 50ms!) ---
int16_t PID_Compute(SpeedPID *pid, int16_t current_encoder, float target_speed) {
    
    // A. Calculate Raw Speed (Delta)
    // The cast to (int16_t) handles 0->65535 wrap-around automatically
    int16_t raw_speed = (int16_t)(current_encoder - pid->prev_encoder);
    pid->prev_encoder = current_encoder;

    // B. LOW PASS FILTER (Essential for your 500 PPR)
    // Alpha 0.2 means: 20% New Data, 80% Old Data. Makes it smooth.
    float alpha = 0.2f; 
    pid->filtered_speed = (alpha * raw_speed) + ((1.0f - alpha) * pid->filtered_speed);

    // C. Calculate Error
    float error = target_speed - pid->filtered_speed;

    // D. Proportional
    float P = pid->Kp * error;

    // E. Integral (With Clamp)
    pid->integral_sum += (pid->Ki * error);
    if (pid->integral_sum > pid->integral_limit) pid->integral_sum = pid->integral_limit;
    if (pid->integral_sum < -pid->integral_limit) pid->integral_sum = -pid->integral_limit;

    // F. Derivative
    // Note: D-term usually makes 7PPR encoders noisy. Start with Kd=0.
    float D = pid->Kd * (error - pid->prev_error);
    pid->prev_error = error;

    // G. Total Output
    float output = P + pid->integral_sum + D;

    // H. Clamp to Motor Max Limits
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return (int16_t)output;
}

// --- 3. APPLY TO MOTOR (PA4/PA5/PA6) ---
void Motor_SetSpeed(int16_t speed) {
    // Handle Direction
    if (speed > 0) {
        GPIOA->BSRR = (1 << 4);  // PA4 HIGH (Forward)
        GPIOA->BRR  = (1 << 5);  // PA5 LOW
    } else if (speed < 0) {
        GPIOA->BRR  = (1 << 4);  // PA4 LOW
        GPIOA->BSRR = (1 << 5);  // PA5 HIGH (Reverse)
        speed = -speed;          // Make PWM positive
    } else {
        GPIOA->BRR = (1 << 4) | (1 << 5); // Stop/Brake
    }

    // Apply PWM to PA6 (TIM3 Channel 1)
    if (speed > 1000) speed = 1000;
    TIM3->CCR1 = (uint16_t)speed;
}


    int main(void) {
    // --- SYSTEM INIT ---
    HAL_Init();
    HAL_Delay(200);
    SystemClock_Config_HSE_72MHz();
    SystemCoreClockUpdate();
    USB_ForceReEnumeration();
    MX_USB_DEVICE_Init();
    
    // --- HARDWARE INIT ---
    MX_TIM2_Encoder_Init(); // Setup Encoder (PA0/PA1)
    MX_TIM3_PWM_Init();     // Setup Motor PWM (PA4/PA5/PA6)
    
    // --- PID INIT ---
    SpeedPID myPID;
    // STARTING VALUES: Kp=2.0, Ki=0.5, Kd=0.0
    // Tip: Increase Kp if motor is too lazy. Increase Ki to fix error.
    PID_Init(&myPID, 20.0f, 60.0f, 0.0f); 

    RobotState myRobot;
    
    // Target Speed: 30 Encoder Counts per 50ms
    // 30 counts/50ms = 600 counts/sec ~= 0.3 Rev/sec (approx 20 RPM)
    float target_speed = 20.0f; 

    while (1) {
        uint32_t now = HAL_GetTick();

        // --- RUN PID LOOP (20Hz = Every 50ms) ---
        if (now - myPID.last_time >= 50) {
            myPID.last_time = now;
            
            // 1. Get Current Encoder Position
            int16_t current_pos = (int16_t)TIM2->CNT;

            // 2. Compute PID Output
            int16_t motor_cmd = PID_Compute(&myPID, current_pos, target_speed);

            // 3. Drive the Motor
            Motor_SetSpeed(motor_cmd);

            // 4. Debugging via USB
            myRobot.timestamp   = now;
            myRobot.pwm_value   = (uint16_t)(myPID.filtered_speed * 10.0f);
            // Instead of sending 0.5 as 0, this sends it as 5
            myRobot.status_flag = 3;
            myRobot.encoder_pos = current_pos; // Need to use updated struct for this
            
            Fast_USB_Send((uint8_t*)&myRobot, sizeof(RobotState));
        }
        
        // --- NON-CRITICAL TASKS ---
        // (Blink LED, Check Buttons, etc.)
    }
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