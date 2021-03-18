#include "common.h"
#include "MK60_ftm.h"
#include "MK60_gpio.h"
#include "MK60_PIT.h"
#include "VCAN_LED.h"
#include "m_LED.h"

/* 内部函数声明 */
static void LED_WATER(uint16 time);

/*  @Function     m_LED_And_BEEP_Init
 *	@Description  开机LED初始化
 *  @CallyBy      main.c
 */
void m_LED_And_BEEP_Init()
{
    gpio_init (BEEP_PORT, GPO, 0);
    led_init (LED_MAX); 
    
    BEEP_OFF;
}

/******************************************************************************
 *
 *  函数功能：LED亮度循环 -- 1~100 100~1 1~100...
 *  返回值：亮度值(1~100)
 *
*******************************************************************************/
uint8 Light_Water()
{
    static unsigned int i = 0;
    static unsigned int plus_flag = 0;
    uint8 light_t;

    i = i%100;
    
    if(plus_flag%2 == 0)
        light_t = (i+1);
    else
        light_t = (100-i);

    if(i == 99)
        plus_flag++; 

    i++;

    return light_t;
}

/*  @Function     LED_WATER
 *	@Description  流水灯测试
 *  @CallyBy      m_LED_And_BEEP_Init of m_LED.c
 */
void LED_WATER(uint16 time)
{
    led (LED0,LED_ON);
    led (LED1,LED_OFF);
    led (LED2,LED_OFF);
    led (LED3,LED_OFF);
    
    DELAY_MS(time);
    
    led (LED0,LED_OFF);
    led (LED1,LED_ON);
    led (LED2,LED_OFF);
    led (LED3,LED_OFF);
    
    DELAY_MS(time);
    
    led (LED0,LED_OFF);
    led (LED1,LED_OFF);
    led (LED2,LED_ON);
    led (LED3,LED_OFF);
    
    DELAY_MS(time);
    
    led (LED0,LED_OFF);
    led (LED1,LED_OFF);
    led (LED2,LED_OFF);
    led (LED3,LED_ON);

    DELAY_MS(time);
    
    led (LED0,LED_OFF);
    led (LED1,LED_OFF);
    led (LED2,LED_OFF);
    led (LED3,LED_OFF); 
    

}

