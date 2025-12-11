#include "stm32f4xx.h"
#include <stdio.h>
#include <stdbool.h>

#define GREEN_LED (1U<<0)  // PB0
#define RED_LED   (1U<<1)  // PB1
#define BUZZER    (1U<<3)  // PA3

// HC-SR04 Pins
#define TRIG_PIN (1U<<1)   // PA1
#define ECHO_PIN (1U<<2)   // PA2

// I2C LCD (using I2C1)
// SDA -> PB7
// SCL -> PB6
#define LCD_I2C_ADDR  0x27  // Common I2C LCD address (try 0x3F if 0x27 doesn't work)

volatile uint32_t ms_ticks = 0;

// SysTick handler
void SysTick_Handler(void){
    ms_ticks++;
}

// Delay in ms
void delay_ms(uint32_t ms){
    uint32_t start = ms_ticks;
    while((ms_ticks - start) < ms);
}

// Simple microsecond delay
void delay_us(uint32_t us){
    us *= 4;
    while(us--){
        __NOP();
    }
}

// --- I2C Functions ---
void i2c_init(void){
    // Enable I2C1 and GPIOB clock
    RCC->APB1ENR |= (1<<21);  // I2C1 clock enable
    RCC->AHB1ENR |= (1<<1);   // GPIOB clock enable
    
    // Configure PB6 (SCL) and PB7 (SDA) as alternate function
    GPIOB->MODER &= ~((3<<12) | (3<<14));  // Clear
    GPIOB->MODER |= (2<<12) | (2<<14);     // Alternate function mode
    
    // Set alternate function to AF4 (I2C1)
    GPIOB->AFR[0] &= ~((0xF<<24) | (0xF<<28));  // Clear AF for PB6 and PB7
    GPIOB->AFR[0] |= (4<<24) | (4<<28);         // AF4 for I2C1
    
    // Open-drain output type
    GPIOB->OTYPER |= (1<<6) | (1<<7);
    
    // High speed
    GPIOB->OSPEEDR |= (3<<12) | (3<<14);
    
    // Pull-up resistors
    GPIOB->PUPDR &= ~((3<<12) | (3<<14));
    GPIOB->PUPDR |= (1<<12) | (1<<14);
    
    // Reset I2C1
    I2C1->CR1 = 0x8000;
    I2C1->CR1 = 0;
    
    // Configure I2C1 (assuming 16MHz APB1 clock)
    I2C1->CR2 = 16;  // 16MHz peripheral clock
    I2C1->CCR = 80;  // 100kHz standard mode (16MHz / (2 * 100kHz))
    I2C1->TRISE = 17; // Maximum rise time
    
    // Enable I2C1
    I2C1->CR1 |= (1<<0);
}

void i2c_start(void){
    I2C1->CR1 |= (1<<8);  // Generate START
    while(!(I2C1->SR1 & (1<<0)));  // Wait for START condition
}

void i2c_stop(void){
    I2C1->CR1 |= (1<<9);  // Generate STOP
}

void i2c_write_addr(uint8_t addr){
    I2C1->DR = addr;
    while(!(I2C1->SR1 & (1<<1)));  // Wait for ADDR flag
    (void)I2C1->SR2;  // Clear ADDR by reading SR2
}

void i2c_write_data(uint8_t data){
    while(!(I2C1->SR1 & (1<<7)));  // Wait until TXE (transmit empty)
    I2C1->DR = data;
    while(!(I2C1->SR1 & (1<<2)));  // Wait for BTF (byte transfer finished)
}

// --- I2C LCD Functions ---
void lcd_send_cmd(uint8_t cmd){
    uint8_t data_u, data_l;
    uint8_t data_t[4];
    
    data_u = (cmd & 0xF0);
    data_l = ((cmd << 4) & 0xF0);
    
    data_t[0] = data_u | 0x0C;  // en=1, rs=0
    data_t[1] = data_u | 0x08;  // en=0, rs=0
    data_t[2] = data_l | 0x0C;  // en=1, rs=0
    data_t[3] = data_l | 0x08;  // en=0, rs=0
    
    i2c_start();
    i2c_write_addr(LCD_I2C_ADDR << 1);
    for(int i = 0; i < 4; i++){
        i2c_write_data(data_t[i]);
    }
    i2c_stop();
    delay_ms(2);
}

void lcd_send_data(uint8_t data){
    uint8_t data_u, data_l;
    uint8_t data_t[4];
    
    data_u = (data & 0xF0);
    data_l = ((data << 4) & 0xF0);
    
    data_t[0] = data_u | 0x0D;  // en=1, rs=1
    data_t[1] = data_u | 0x09;  // en=0, rs=1
    data_t[2] = data_l | 0x0D;  // en=1, rs=1
    data_t[3] = data_l | 0x09;  // en=0, rs=1
    
    i2c_start();
    i2c_write_addr(LCD_I2C_ADDR << 1);
    for(int i = 0; i < 4; i++){
        i2c_write_data(data_t[i]);
    }
    i2c_stop();
    delay_us(100);
}

void lcd_init(void){
    delay_ms(50);
    
    lcd_send_cmd(0x30);
    delay_ms(5);
    lcd_send_cmd(0x30);
    delay_ms(1);
    lcd_send_cmd(0x30);
    delay_ms(1);
    lcd_send_cmd(0x20);  // 4-bit mode
    delay_ms(1);
    
    lcd_send_cmd(0x28);  // 4-bit, 2 lines, 5x8
    lcd_send_cmd(0x08);  // Display off
    lcd_send_cmd(0x01);  // Clear display
    delay_ms(5);
    lcd_send_cmd(0x06);  // Entry mode
    lcd_send_cmd(0x0C);  // Display on, cursor off
}

void lcd_clear(void){
    lcd_send_cmd(0x01);
    delay_ms(5);
}

void lcd_set_cursor(uint8_t row, uint8_t col){
    uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;
    lcd_send_cmd(addr);
}

void lcd_print(char* str){
    while(*str){
        lcd_send_data(*str++);
    }
}

void lcd_print_distance(float d){
    char buf[17];
    lcd_set_cursor(0, 0);
    
    if(d >= 2.0f && d <= 400.0f){
        sprintf(buf, "Dist:%5.1fcm     ", d);
    } else {
        sprintf(buf, "No Object Detect");
    }
    lcd_print(buf);
    
    lcd_set_cursor(1, 0);
    if(d < 2.0f || d > 400.0f){
        sprintf(buf, "Status: NO OBJ  ");
    }
    else if(d > 15){
        sprintf(buf, "Status: SAFE    ");
    }
    else if(d > 7){
        sprintf(buf, "Status: WARNING ");
    }
    else{
        sprintf(buf, "Status: DANGER! ");
    }
    lcd_print(buf);
}

// --- HC-SR04 measurement ---
float measure_distance_cm(void){
    uint32_t pulse_width = 0;
    uint32_t timeout;
    
    // Send trigger pulse
    GPIOA->ODR &= ~TRIG_PIN;
    delay_us(2);
    GPIOA->ODR |= TRIG_PIN;
    delay_us(10);
    GPIOA->ODR &= ~TRIG_PIN;
    
    // Wait for echo pin to go HIGH
    timeout = 10000;
    while(!(GPIOA->IDR & ECHO_PIN) && timeout > 0){
        timeout--;
        delay_us(1);
    }
    
    if(timeout == 0){
        return 999.9f;
    }
    
    // Measure pulse width
    timeout = 30000;
    pulse_width = 0;
    while((GPIOA->IDR & ECHO_PIN) && timeout > 0){
        pulse_width++;
        timeout--;
        delay_us(1);
    }
    
    if(timeout == 0){
        return 999.9f;
    }
    
    // Calculate distance
    float distance = (float)pulse_width / 58.0f;
    
    if(distance < 2.0f || distance > 400.0f){
        return 999.9f;
    }
    
    return distance;
}

// --- GPIO Initialization ---
void init_pins(void){
    RCC->AHB1ENR |= (1U<<0) | (1U<<1);
    delay_ms(2);
    
    // LEDs (PB0, PB1)
    GPIOB->MODER &= ~((3U<<0) | (3U<<2));
    GPIOB->MODER |= (1U<<0) | (1U<<2);
    GPIOB->OTYPER &= ~(GREEN_LED | RED_LED);
    GPIOB->OSPEEDR |= (3U<<0) | (3U<<2);
    GPIOB->PUPDR &= ~((3U<<0) | (3U<<2));
    GPIOB->ODR &= ~(GREEN_LED | RED_LED);
    
    // Buzzer (PA3)
    GPIOA->MODER &= ~(3U<<6);
    GPIOA->MODER |= (1U<<6);
    GPIOA->OTYPER &= ~BUZZER;
    GPIOA->OSPEEDR |= (3U<<6);
    GPIOA->PUPDR &= ~(3U<<6);
    GPIOA->ODR &= ~BUZZER;
    
    // TRIG (PA1)
    GPIOA->MODER &= ~(3U<<2);
    GPIOA->MODER |= (1U<<2);
    GPIOA->OTYPER &= ~TRIG_PIN;
    GPIOA->OSPEEDR |= (3U<<2);
    GPIOA->PUPDR &= ~(3U<<2);
    GPIOA->ODR &= ~TRIG_PIN;
    
    // ECHO (PA2)
    GPIOA->MODER &= ~(3U<<4);
    GPIOA->PUPDR &= ~(3U<<4);
}

// --- LED + Buzzer Control Logic ---
void update_outputs(float d){
    static uint32_t last_toggle = 0;
    static bool blink_state = false;
    
    if(d < 2.0f || d > 400.0f){
        // NO VALID OBJECT - Everything OFF
        GPIOB->ODR &= ~(GREEN_LED | RED_LED);
        GPIOA->ODR &= ~BUZZER;
        blink_state = false;
    }
    else if(d > 15){ 
        // SAFE ZONE: Object is FAR (> 15cm)
        GPIOB->ODR |= GREEN_LED;
        GPIOB->ODR &= ~RED_LED;
        GPIOA->ODR &= ~BUZZER;
        blink_state = false;
    }
    else if(d > 7 && d <= 15){ 
        // WARNING ZONE: Object is at MEDIUM distance (7-15cm)
        GPIOB->ODR &= ~GREEN_LED;
        
        if((ms_ticks - last_toggle) >= 400){
            blink_state = !blink_state;
            
            if(blink_state){
                GPIOB->ODR |= RED_LED;
                GPIOA->ODR |= BUZZER;
            }
            else{
                GPIOB->ODR &= ~RED_LED;
                GPIOA->ODR &= ~BUZZER;
            }
            
            last_toggle = ms_ticks;
        }
    }
    else{ 
        // DANGER ZONE: Object is VERY CLOSE (< 7cm)
        GPIOB->ODR &= ~GREEN_LED;
        GPIOB->ODR |= RED_LED;
        GPIOA->ODR |= BUZZER;
        blink_state = false;
    }
}

// --- Main ---
int main(void){
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);
    
    init_pins();
    delay_ms(100);
    
    // Quick hardware test - blink once each
    GPIOB->ODR |= GREEN_LED;
    delay_ms(200);
    GPIOB->ODR &= ~GREEN_LED;
    delay_ms(100);
    
    GPIOB->ODR |= RED_LED;
    delay_ms(200);
    GPIOB->ODR &= ~RED_LED;
    delay_ms(100);
    
    GPIOA->ODR |= BUZZER;
    delay_ms(200);
    GPIOA->ODR &= ~BUZZER;
    delay_ms(300);
    
    // Initialize I2C LCD
    i2c_init();
    delay_ms(100);
    lcd_init();
    delay_ms(50);
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Proximity Alarm");
    lcd_set_cursor(1, 0);
    lcd_print("Ready...");
    delay_ms(2000);
    lcd_clear();
    
    uint32_t last_measurement = 0;
    
    while(1){
        if((ms_ticks - last_measurement) >= 150){
            float distance = measure_distance_cm();
            
            update_outputs(distance);
            lcd_print_distance(distance);
            
            last_measurement = ms_ticks;
        }
        
        delay_ms(10);
    }
}
