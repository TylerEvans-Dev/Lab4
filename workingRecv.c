/**
  *
  * Tyler Evans U1313811
  * U1313811
  *
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
// Const section
const int length = 5;
const int errorLen =21;
/*
 1234567891234567891
 error inccorect key
*/

void _Error_Handler(char * file, int line);

void SystemClock_Config(void);

/**
 @function sendChar
 @param s ->String
 @discussion function sends a charcter to the USART.
 */
void sendChar(char s){
    /*
     this bit of code seems wrong.
     */
    // sends the string we desire
    USART3->TDR = s;
    //GPIOC->ODR ^= (1<<8);
}
/**
 @function transString
 @param vals
 @brief function transmits a string
 */
///TODO FIX THIS FUNCTION TO CHECK VALUES.
void transString(char  vals[], int flag){
    int i = 0;
    while(vals[i] != '\0'){
        sendChar(vals[i]);
        if (flag == 1){
            i++;
        }
    }
}


void isTyped(char s){
    char errorMessage[] ={'E','r','r','o','r',' ',  'i', 'n', 'c', 'c', 'o', 'r', 'e', 'c', 't', ' ', 'k','e', 'y', '\n', '\0'};
    switch(s){
        case 'r':
            GPIOC->ODR ^= (1<<6);
            break;
        case 'b':
            GPIOC->ODR ^= (1<<7);
            break;
        case 'o':
            GPIOC->ODR ^= (1<<8);
            break;
        case 'g':
            GPIOC->ODR ^= (1<<9);
        default:
            transString(errorMessage, 1);
            break;
    }
}

char isWrote(){
    char val = '\0';
    val = USART3->RDR;
    return val;
}




/*
 useful things
 bit follows for the LED
 RED = 6
 BLUE = 7
 ORANGE = 8
 GREEN = 9
 */

int main(void)
{
    char shortMessage[] = {'\t', 'H', 'E', 'Y', '\n', '\0'};
    char errorMessage[] ={'E','r','r','o','r',' ',  'i', 'n', 'c', 'c', 'o', 'r', 'e', 'c', 't', ' ', 'k','e', 'y', '\n', '\0'};
    char begMessage[] = {'CMD:'};
    HAL_Init(); // Reset of all peripherals, init the Flash and Systick
    SystemClock_Config(); //Configure the system clock
    /* This example uses HAL library calls to control
     the GPIOC peripheral. You’ll be redoing this code
     with hardware register access. */
    
    //__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
    //HERE IS THE RCC CLOCK ENABLE PIN REGEISTER DONE
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // done to find the clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // set up the clock for GIPOB
    RCC->APB1ENR |= (1<<18); // enabling the clock for USART 3.
    /*
     */
    
    
    //sets everything to zero in the pins
    // GPIOC is the GPIO_x where the pin is located.
    GPIOC->MODER &= 0; // sets the mode
    GPIOC->OTYPER &= 0; // sets what type push pull
    GPIOC->OSPEEDR &= 0; // sets the speed
    GPIOC->PUPDR &= 0; // sets the pulldown/pullup resitor
    
    //sets all the values in modder to the correct pin into input mode.
    GPIOC->MODER |= (1<< 12) | (1 << 14) | (1<< 16) | (1 << 18); //configures what pins for use setting up the mode
    /*here is GPIOA stuff*/
    GPIOC->MODER |= (1<<18) | (1<<16) | (1<<14) | (1<<12);
    /*
     GPIOB enabled here
     */
    GPIOB->MODER &=0;
    GPIOB->OTYPER &= 0;
    GPIOB->OSPEEDR &= 0;
    GPIOB->PUPDR &= 0;
    /*
     Here GPIOB is modifed in order to function as a
     USART 3 TX(transmitter) and RX(reciver)
     to do so
     PB10 is TX and PB11 is RX
     1.GPIOB needs to be the alternate function
     2.GPIOB AFR needs AF4 to be in the 9 and 10 pin
     3.The next thing that needs to be done is setting the baud rate
     4.enable the USART
     */
    
    //THIS is enabling the alternate mode PB10 and PB11 both should have 10 in the bits
    GPIOB->MODER |= (1<<23) | (1<<21); //enables the alternate function
    GPIOB->MODER &= ~((1<<22) | (1<<20));
    //here is the AFR it is in the HIGH regeister.
    //TODO CHANGE THIS HERE.
    GPIOB->AFR[1] |= (0<<11)| (1<<10) | (0<<9) |(0<<8); // 0100 to enable AF4
    GPIOB->AFR[1] |= (0<<15) | (1<<14) | (0<<13) | (0<<12); // 0100 enable AF4 for the pi
    
    
    
    
    // USART requires that the baud rate meets (8*10^6) / (115200) = 694 (clck speed / 11520)
    USART3->BRR = HAL_RCC_GetHCLKFreq()/ 115200;
    USART3->CR1 |= (1<<7);
    USART3->CR1 |= (1<<3) | (1<<2); //enabling the bits with value of 1, in location 3 as its transmiter and 2 as it is reciver 1
    // done on seperate line but USART needs to be enabled.
    USART3->CR1 |= (1<<0);
    //GPIOC->ODR |= (1<<9); //switiching on green for seeing it turn on.
    //GPIOC->ODR |= (1<<8); // switching it on for the Orange LED to turn on
    while(1) {
        int flagPassed = 0;
        //HAL_Delay(200);
        //GPIOC->ODR ^= (1<<9);
        /*
         Right here what I am doing is taking the mask and anding it with the
         bit in address 16 and if there is a value there it will be busy otherwise it will
         be able to continue on in execution however this is wrong it should be bit 7 as this is the transmison and not the Busy flag. Although busy will show that it is busy what we really care about is sending data and when the transmison is clear.
         */
        for(int k = 0; k < 4; k++){
            while(!(USART3->ISR & (1<<5))){
                //GPIOC->ODR ^= (1<<7);
                //GPIOC->ODR ^= (1<<6);
                //HAL_Delay(200);
                
            }
            
        }
            while(!(USART3->ISR & (1<<5))){
                //GPIOC->ODR ^= (1<<7);
                //GPIOC->ODR ^= (1<<6);
                //HAL_Delay(200);
                
            }
        isTyped(isWrote());
      
    }

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

