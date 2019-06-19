
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <string.h>
#include <ctype.h>
#include <stdbool.h>

typedef enum torque_sensor_signals
{
  SINE,
  COSINE,
  TORQUE
} ts_signal;

//Global Variables
uint16_t adc_val[3]; //Torque Sensor Values
uint8_t tx_buffer[50];
volatile uint16_t ppm = 0;

int parseInt(char *str, int len);
void setPPM(uint16_t *pulseLength);

int main(void)
{
  // Peripheral Initialization
  system_peripheral_init();
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val, 3);

  setPPM(1000);

  while (1)
  {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
    HAL_Delay(100);
  }
}

void CDC_ReceiveCallBack(uint8_t *buf, uint32_t len)
{
  char tx_buffer[50];
  char rx_buffer[50];
  strncpy(rx_buffer, buf, len);
  rx_buffer[len] = '\0';
  ppm = parseInt(rx_buffer, strlen(rx_buffer));
  setPPM(&ppm);
  sprintf(tx_buffer, "Setting PPM : %dms\r\n", ppm, 50);
  // sprintf(tx_buffer, "Received Buffer Size: %d\n Buffer : %s", len, rx_buffer, 50);
  // sprintf(tx_buffer, "Received : %s\n", rx_buffer, 50);

  CDC_Transmit_FS(tx_buffer, strlen(tx_buffer));
}

int parseInt(char *str, int len)
{
  int parsed = 0;
  char ch;
  bool negative = false, parsing = false;

  for (int i = 0; i < len; i++)
  {
    ch = str[i];
    if (isdigit(ch) != 0)
    {
      if (!parsing && i != 0 && str[i - 1] == '-')
        negative = true;
      parsing = true;
      parsed = (parsed * 10) + (ch - '0');
    }
    else if (parsing)
      break;
  }
  return ((negative) ? -parsed : parsed);
}

void setPPM(uint16_t *pulseLength)
{
  if (*pulseLength < 1000)
    *pulseLength = 1000;
  else if (*pulseLength > 2000)
    *pulseLength = 2000;
  htim1.Instance->CCR1 = *pulseLength;
}
//////////////////////////////////////////////////////////////////////
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
