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
#include <stm8s.h>

#include <stdio.h>
#include <stdlib.h>
/**
  * @addtogroup TIM1_7PWM_Output
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define CCR1_Val  ((uint16_t)199)
#define MAX_Val  ((uint16_t)200)
//#define BLUETOOTH


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint16_t tacho_count = 0;
static uint16_t Conversion_Value;
static struct {
  uint8_t buf[256];
  uint8_t head;
  uint8_t tail;
  uint8_t size;
} ringbuf;
static uint8_t rxline[256];
static uint8_t * prxline = rxline;
/* Private function prototypes -----------------------------------------------*/
static void TIM1_Config(void);
static void TIM3_Config(void);
static void PWM_init(void);
static int uart_init(uint32_t baudrate);
static void ADC_Config();
#ifdef BLUETOOTH
static void bluetooth(void);
#endif
static void Myloop(void);
/* Private functions ---------------------------------------------------------*/
static void ringbuf_init()
{
  memset(&ringbuf,0, sizeof(ringbuf));
}
static void ringbuf_put(uint8_t c)
{
  if(ringbuf.tail<256) {
    ringbuf.buf[ringbuf.tail] = c;
    ringbuf.tail++;
  }
  disableInterrupts();
  ringbuf.size+=1;
  enableInterrupts();
  /*adjust tail*/
  if(ringbuf.tail>=256) {
    ringbuf.tail = 0;
  }
}
static int8_t ringbuf_get(uint8_t *pc)
{
  int8_t ret = 0;
  if (ringbuf.size > 0) {
    if (ringbuf.head<256) {
      *pc = ringbuf.buf[ringbuf.head];
      ringbuf.head++;
      disableInterrupts();
      ringbuf.size-=1;
      enableInterrupts();
      ret = 1;
    }
  }
  
  /*adjust tail*/
  if(ringbuf.head>=256) {
    ringbuf.head = 0;
  }
  return ret;
}

static int16_t GetString(void)
{
    int16_t val=-1;
    uint8_t c;
    while(ringbuf_get(&c)){
      *prxline = c;
      //debug: printf("%c", c);
      prxline++;
      if((c=='\r') || (c=='\n')){
        *prxline = 0;
        /*print*/
        if(strlen(rxline)>0) {
          printf("RX: %s\n", rxline);
          val = atoi(rxline);
        }
        /*reset pointer*/
        prxline = rxline;
      }
    }
    
    return val;
}
/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
    ringbuf_init();
    ADC_Config();

    /*heart beat led*/
    GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_WriteLow(GPIOE, GPIO_PIN_5);

    /* PC fan tacho signal */
    GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_IN_FL_IT);
    //GPIO_ExternalPullUpConfig(GPIOD, GPIO_PIN_2, ENABLE);
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_FALL_ONLY);
    EXTI_SetTLISensitivity(EXTI_TLISENSITIVITY_FALL_ONLY);

    /* PWM output signal */
    GPIO_Init(GPIOC, GPIO_PIN_1, GPIO_MODE_OUT_OD_HIZ_SLOW);
    /* full speed */
    GPIO_WriteLow(GPIOC, GPIO_PIN_0);

    uart_init(115200);
#ifdef BLUETOOTH
    bluetooth();
#else
    printf("\nGo\n");
#endif
    /* PWM configuration */
    PWM_init();
    /* TIM3 heart beat */
    TIM3_Config();

    /* Enable general inUnhandled exceptionUnhandled exceptionerrupts */
    enableInterrupts();

    Myloop();

    while (1)
    {
    }
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
        //__wait_for_interrupt();
    };
    Conversion_Value = ADC1_GetConversionValue();
    //ADC1_ITConfig(ADC1_IT_EOC, DISABLE);
    return Conversion_Value;
}
/**
  * @brief  Configure TIM1 to generate 7 PWM signals with 4 different duty cycles
  * the microcontroller can vary the output voltage.
  * In this implementation the pulse rate is 2.5KHz and the pulse width varies between zero and 170uS
  * to give an output between zero and 12V
  * @param  None
  * @retval None
  */
static void TIM1_Config(void)
{

    TIM1_DeInit();

    /*
       clock src 2MHz = 500ns
       1) 500ns * 200 = 100000ns = 100us
       2) 500ns * 2 = 1us
       Time Base configuration 10KHz=100us
       general speed of PC fan
       1000rpm=16.66Hz=60ms=60000us*/

    TIM1_TimeBaseInit(4-1, TIM1_COUNTERMODE_UP, MAX_Val, 0);

    /* Channel 1 in PWM mode */
    TIM1_OC1Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_ENABLE,
                 CCR1_Val, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                 TIM1_OCNIDLESTATE_RESET);
#if 0
    /* Channel 3/4 in Capure/Compare mode */
    TIM1_ICInit( TIM1_CHANNEL_3, TIM1_ICPOLARITY_RISING, TIM1_ICSELECTION_DIRECTTI,
                 TIM1_ICPSC_DIV1, 0x0);

    TIM1_SelectInputTrigger(TIM1_TS_TI2FP2);
    TIM1_SelectSlaveMode(TIM1_SLAVEMODE_RESET);

#endif

    /* TIM1 counter enable */
    TIM1_Cmd(ENABLE);
#if 1
    /* TIM1 Main Output Enable */
    TIM1_CtrlPWMOutputs(ENABLE);


    /* Clear CC3 Flag*/
    TIM1_ClearFlag(TIM1_FLAG_CC3);
#endif
}

static void PWM_init(void)
{
    TIM1_Config();
}
static void PWM_update(uint8_t percentage)
{
    uint16_t tim_val;
    static uint16_t tim_val_old=0;

    /* test only */
    tim_val = percentage;
    tim_val = tim_val * MAX_Val /100;

    /* update PWM */
    if(tim_val != tim_val_old)
    {
        if(tim_val <= MAX_Val)
        {
            TIM1_SetCompare1(tim_val);
            printf("PWM:%u\n", tim_val);
            tim_val_old = tim_val;
        }
    }
}
void tacho_handler(void)
{
    if ((GPIO_ReadInputData(GPIOD) & GPIO_PIN_2) == 0)
    {
        tacho_count++;
    }
}
void TIM3_update(void)
{
    uint16_t tacho = tacho_count;
    printf("RPM:%u ADC:%u\n", tacho*10, Conversion_Value);
    //disableInterrupts();
    tacho_count = 0;
    //enableInterrupts();
    /*cpuwolf: LED breathe*/
    GPIO_WriteReverse(GPIOE, GPIO_PIN_5);
    /* Cleat Interrupt Pending bit */
    TIM3_ClearITPendingBit(TIM3_IT_UPDATE);
}
/* pin PD3 */
static void TIM3_Config(void)
{
    TIM3_DeInit();
    /*
       clock src 2MHz = 500ns
       500ns * 32768 = 16.386ms
       16.386ms * 61 = 1000ms = 1s
       Time Base configuration 10KHz=100us

       reference general PC fan speed
       1000rpm=16.66Hz=60ms=60000us
    */
    TIM3_TimeBaseInit(TIM3_PRESCALER_32768, 61*6-1);
    /* Clear TIM3 update flag */
    TIM3_ClearFlag(TIM3_FLAG_UPDATE);
    /* Enable update interrupt */
    TIM3_ITConfig(TIM3_IT_UPDATE, ENABLE);
#if 0
    /* Channel 1/2 in Capure/Compare mode */
    TIM3_ICInit( TIM3_CHANNEL_1, TIM3_ICPOLARITY_RISING, TIM3_ICSELECTION_DIRECTTI,
                 TIM3_ICPSC_DIV1, 0x0);
    TIM3_ICInit( TIM3_CHANNEL_2, TIM3_ICPOLARITY_FALLING, TIM3_ICSELECTION_INDIRECTTI,
                 TIM3_ICPSC_DIV1, 0x0);
#endif

    /* TIM3 counter enable */
    TIM3_Cmd(ENABLE);
#if 0
    /* Clear CC3 Flag*/
    TIM3_ClearFlag(TIM3_FLAG_CC1|TIM3_FLAG_CC2);
#endif
}


void uart_rx_irq(void)
{
    ringbuf_put(UART2_ReceiveData8());
}
static void Myloop(void)
{
    uint16_t rasing_value;
    uint16_t falling_value;
    uint16_t rpm;
    int16_t tim_val, tim_val_old=0;

    uint8_t i;
read_speed:

     tim_val = GetString();
     if(tim_val > 0) {
        /* left input will be ignored */
        //UART2_ClearFlag(UART2_FLAG_RXNE);
    
        PWM_update(tim_val);
     }
#if 0
    /* wait a capture on CC1, CC2 */
    if((TIM1->SR2 & TIM1_FLAG_CC1) != TIM1_FLAG_CC1)
    {
        /* Get CCR1 value*/
        rasing_value = TIM3_GetCapture1();
        TIM3_ClearFlag(TIM3_FLAG_CC1);
    }
    if((TIM1->SR2 & TIM1_FLAG_CC2) != TIM1_FLAG_CC2)
    {
        /* Get CCR2 value*/
        falling_value = TIM3_GetCapture2();
        TIM3_ClearFlag(TIM3_FLAG_CC2);
    }
#endif
    Conversion_Value = ADC_Read();

    //if(falling_value > rasing_value)
    //    printf("[%u]ADC 0x%x\n", falling_value - rasing_value, Conversion_Value);

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
    /* Enable UART2 Receive interrupt */
    UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);

    status = 0;

    /* Finished */
    return (status);
}

#ifdef BLUETOOTH
static void bluetooth(void)
{

}

#endif

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
