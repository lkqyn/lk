#include "iwdg/iwdg.h"

IWDG_HandleTypeDef hiwdg;

/**
 @brief �������Ź���ʼ��
 500ms��λ.
*/
void IWDG_config(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
  hiwdg.Init.Reload = 2499;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    //Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */
    
  /* USER CODE END IWDG_Init 2 */

}

void IWDG_task(void *pv)
{
    while(1)
    {
        IWDG_RELOAD();
        
    }
}


