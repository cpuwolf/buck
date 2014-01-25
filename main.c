/**
  ******************************************************************************
  * @file TIM1_7PWM_Output\main.c
  * @brief This file contains the main function for TIM1 7 PWM Output example.
  * @author  MCD Application Team
  * @version  V2.0.1
  * @date     18-November-2011
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

#include <stdio.h>
/**
  * @addtogroup TIM1_7PWM_Output
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define CCR1_Val  ((uint16_t)800)
#define MAX_Val  ((uint16_t)8095)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void TIM1_Config(void);
static int uart_init(uint32_t baudrate);
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
    GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
    /* Toggles LED */
    GPIO_WriteLow(GPIOE, GPIO_PIN_5);

    uart_init(115200);

    printf("\nGo\n");

    /* TIM1 configuration -----------------------------------------*/
    TIM1_Config();

    while (1)
    {}
}

static void ADC_Config()
{
    /*  Init GPIO for ADC1 */
    GPIO_Init(GPIOB, GPIO_PIN_0, GPIO_MODE_IN_FL_NO_IT);

    /* De-Init ADC peripheral*/
    ADC1_DeInit();

    /* Init ADC2 peripheral */
    ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_0, ADC1_PRESSEL_FCPU_D18, \
              ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_LEFT, ADC1_SCHMITTTRIG_CHANNEL0,\
              DISABLE);

    /* Disable EOC interrupt */
    ADC1_ITConfig(ADC1_IT_EOCIE, DISABLE);


}

static uint16_t ADC_Read()
{
    uint16_t Conversion_Value;
    ADC1_StartConversion();
    while(ADC1_GetFlagStatus(ADC1_FLAG_EOC)==RESET) {
        __wait_for_interrupt();
    };
    Conversion_Value = ADC1_GetConversionValue();
    ADC1_ITConfig(ADC1_IT_EOC, DISABLE);
    return Conversion_Value;
}
/**
  * @brief  Configure TIM1 to generate 7 PWM signals with 4 different duty cycles
  * @param  None
  * @retval None
  */
static void TIM1_Config(void)
{
    uint16_t ICValue;
    uint16_t rpm;

    TIM1_DeInit();

    /* Time Base configuration 10KHz=100us*/
    /* gernal speed of PC fan
       1000rpm=16.66Hz=60ms=60000us*/
    /*
    TIM1_Period = 4095
    TIM1_Prescaler = 1
    TIM1_CounterMode = TIM1_COUNTERMODE_UP
    TIM1_RepetitionCounter = 0
    */

    TIM1_TimeBaseInit(200-1, TIM1_COUNTERMODE_UP, MAX_Val, 0);

    /* Channel 1 in PWM mode */

    /*
    TIM1_OCMode = TIM1_OCMODE_PWM2
    TIM1_OutputState = TIM1_OUTPUTSTATE_ENABLE
    TIM1_OutputNState = TIM1_OUTPUTNSTATE_ENABLE
    TIM1_Pulse = CCR1_Val
    TIM1_OCPolarity = TIM1_OCPOLARITY_LOW
    TIM1_OCNPolarity = TIM1_OCNPOLARITY_HIGH
    TIM1_OCIdleState = TIM1_OCIDLESTATE_SET
    TIM1_OCNIdleState = TIM1_OCIDLESTATE_RESET

    */
    TIM1_OC1Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_ENABLE,
                 CCR1_Val, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                 TIM1_OCNIDLESTATE_RESET);

    /* Channel 2 in Capure/Compare mode */
    TIM1_ICInit( TIM1_CHANNEL_2, TIM1_ICPOLARITY_RISING, TIM1_ICSELECTION_DIRECTTI,
                 TIM1_ICPSC_DIV1, 0x0);

    TIM1_SelectInputTrigger(TIM1_TS_TI2FP2);
    TIM1_SelectSlaveMode(TIM1_SLAVEMODE_RESET);



    /* TIM1 counter enable */
    TIM1_Cmd(ENABLE);
    /* TIM1 Main Output Enable */
    TIM1_CtrlPWMOutputs(ENABLE);


    /* Clear CC2 Flag*/
    TIM1_ClearFlag(TIM1_FLAG_CC2);

read_speed:
    /* wait a capture on CC2 */
    while((TIM1->SR1 & TIM1_FLAG_CC2) != TIM1_FLAG_CC2);
    /* Get CCR2 value*/
    ICValue = TIM1_GetCapture2();
    TIM1_ClearFlag(TIM1_FLAG_CC2);

    if(ICValue > 65535) {
        printf("Invalid value\n");
    } else {
        rpm = 10000/ICValue;
        rpm *= 60;
        printf("[%d]rpm %d\n", ICValue, rpm);
    }

    goto read_speed;


}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

/*
 * Initialize the UART to requested baudrate, tx/rx, 8N1.
 */
static int uart_init(uint32_t baudrate)
{
    int status;

    /**
     * Set up UART2 for putting out debug messages.
     * This the UART used on STM8S Discovery, change if required.
     */
    UART2_DeInit();
    UART2_Init (baudrate, UART2_WORDLENGTH_8D, UART2_STOPBITS_1, UART2_PARITY_NO,
                UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);

    status = 0;

    /* Finished */
    return (status);
}


static char uart_putchar (char c)
{

    /* Convert \n to \r\n */
    if (c == '\n')
        putchar('\r');

    /* Write a character to the UART2 */
    UART2_SendData8(c);

    /* Loop until the end of transmission */
    while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET)
        ;


    return (c);
}
__root size_t __write(int handle, const unsigned char *buf, size_t bufSize)
{
    size_t chars_written = 0;

    /* Ignore flushes */
    if (handle == -1)
    {
        chars_written = (size_t)0;
    }
    /* Only allow stdout/stderr output */
    else if ((handle != 1) && (handle != 2))
    {
        chars_written = (size_t)-1;
    }
    /* Parameters OK, call the low-level character output routine */
    else
    {
        while (chars_written < bufSize)
        {
            uart_putchar (buf[chars_written]);
            chars_written++;
        }
    }

    return (chars_written);
}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
