#include "watchdog.h"
#include "main.h"
#include "luos.h"

static IWDG_HandleTypeDef   IwdgHandle;


uint16_t tmpCC4[2] = {0, 0};
__IO TIM_HandleTypeDef Input_Handle;
__IO uint32_t uwCaptureNumber = 0;
__IO uint32_t uwLsiFreq = 0;


static uint32_t GetLSIFrequency(void);

void watchdog_Init(void)
{

  /*##-2- Get the LSI frequency: TIM14 is used to measure the LSI frequency ###*/
  uwLsiFreq = GetLSIFrequency();
  
  /*##-3- Configure & Start the IWDG peripheral #########################################*/
  /* Set counter reload value to obtain 2 sec. IWDG TimeOut.
     IWDG counter clock Frequency = uwLsiFreq
     Set Prescaler to 32 (IWDG_PRESCALER_32)
     Timeout Period = (Reload Counter Value * 16) / uwLsiFreq
     So Set Reload Counter Value = (1 * uwLsiFreq) / 16 */
  IwdgHandle.Instance = IWDG;
  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_32;
  IwdgHandle.Init.Reload = (uwLsiFreq / 16);
  IwdgHandle.Init.Window = IWDG_WINDOW_DISABLE;

  if(HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

void watchdog_Refresh(void)
{
    /* Refresh IWDG: reload counter */
    if(HAL_IWDG_Refresh(&IwdgHandle) != HAL_OK)
    {
        /* Refresh Error */
        Error_Handler();
    }
}


/**
  * @brief  Configures TIM14 to measure the LSI oscillator frequency. 
  * @param  None
  * @retval LSI Frequency
  */
static uint32_t GetLSIFrequency(void)
{
  TIM_IC_InitTypeDef    TIMInput_Config;


  /* Configure the TIM peripheral *********************************************/ 
  /* Set TIMx instance */  
  Input_Handle.Instance = TIM14;
  
  /* TIM14 configuration: Input Capture mode ---------------------
     The LSI oscillator is connected to TIM14 CH1.
     The Rising edge is used as active edge.
     The TIM14 CCR1 is used to compute the frequency value. 
  ------------------------------------------------------------ */
  Input_Handle.Init.Prescaler         = 0;
  Input_Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  Input_Handle.Init.Period            = 0xFFFF;
  Input_Handle.Init.ClockDivision     = 0;
  Input_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_IC_Init((TIM_HandleTypeDef *)&Input_Handle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Connect internally the TIM14_CH1 Input Capture to the LSI clock output */
  HAL_TIMEx_RemapConfig((TIM_HandleTypeDef *)&Input_Handle, TIM_TIM14_MCO);

  /* Connect internally the MCO to LSI */
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_LSI, RCC_MCODIV_1);

  /* Configure the Input Capture of channel 1 */
  TIMInput_Config.ICPolarity  = TIM_ICPOLARITY_RISING;
  TIMInput_Config.ICSelection = TIM_ICSELECTION_DIRECTTI;
  TIMInput_Config.ICPrescaler = TIM_ICPSC_DIV8;
  TIMInput_Config.ICFilter    = 0;
  if(HAL_TIM_IC_ConfigChannel((TIM_HandleTypeDef *)&Input_Handle, &TIMInput_Config, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Start the TIM Input Capture measurement in interrupt mode */
  if(HAL_TIM_IC_Start_IT((TIM_HandleTypeDef *)&Input_Handle, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Wait until the TIM14 get 2 LSI edges */
  while(uwCaptureNumber != 2)
  {
  }

  /* Disable TIM14 CC1 Interrupt Request */
  HAL_TIM_IC_Stop_IT((TIM_HandleTypeDef *)&Input_Handle, TIM_CHANNEL_1);
  
  /* Deinitialize the TIM14 peripheral registers to their default reset values */
  HAL_TIM_IC_DeInit((TIM_HandleTypeDef *)&Input_Handle);

  return uwLsiFreq;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint32_t lsiperiod = 0;

  /* Get the Input Capture value */
  tmpCC4[uwCaptureNumber++] = HAL_TIM_ReadCapturedValue((TIM_HandleTypeDef *)&Input_Handle, TIM_CHANNEL_1);

  if (uwCaptureNumber >= 2)
  {
    /* Compute the period length */
    lsiperiod = (uint16_t)(0xFFFF - tmpCC4[0] + tmpCC4[1] + 1);

    /* Frequency computation */ 
    uwLsiFreq = (uint32_t) SystemCoreClock / lsiperiod;
    uwLsiFreq *= 8;
  }
}