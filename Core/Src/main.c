  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"

#include "stdio.h"
#include "string.h"

#define ACCEL_PERIOD_MS   1000
#define TEMP_PERIOD_MS    1000
#define MAG_PERIOD_MS	1000
#define GYRO_PERIOD_MS 	1000
#define LED_PERIOD_MS  	1000
#define PRESS_WINDOW_MS  500

#define SINGLE_PRESS 0
#define DOUBLE_PRESS 1

#define BUTTON_IDLE 0
#define BUTTON_WAIT 1

#define PRINT_RESUME 0
#define PRINT_PAUSE 1

//UART STuffss
static void UART1_Init(void);
UART_HandleTypeDef huart1;

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)

//volatile uint8_t  press_count      = 0;
//volatile uint32_t first_press_tick = 0;
//volatile uint32_t last_press_tick  = 0;
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if (GPIO_Pin == USER_BUTTON_PIN)    // PC13, blue button
//	{
//		uint32_t now = HAL_GetTick();
//
//		if (now - last_press_tick < DEBOUNCE_MS)
//			return;                     // contact bounce, not a real press
//		last_press_tick = now;
//
//		if (press_count == 1 && (now - first_press_tick) < PRESS_WINDOW_MS)
//		{
//			press_count = 2;            // second press inside the window
//		}
//		else if (press_count == 0)
//		{
//			press_count = 1;            // first press: start the window
//			first_press_tick = now;
//		}
//		// press_count == 2: main loop hasn't handled it yet, ignore extras
//	}
//}






volatile uint8_t button_pressed = 0;
volatile uint32_t last_pressed = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == USER_BUTTON_PIN) {
		button_pressed = 1;
		last_pressed = HAL_GetTick();
	}
}


int battlemode = 0;
int main(void)
{

	initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Peripheral initializations using BSP functions */
	BSP_ACCELERO_Init();
	BSP_TSENSOR_Init();
	BSP_MAGNETO_Init();
	BSP_GYRO_Init();
	BSP_LED_Init(LED2);
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

	UART1_Init();

//	uint8_t paused = 0;

	uint32_t last_accel = HAL_GetTick();
	uint32_t last_temp  = HAL_GetTick();
	uint32_t last_mag  = HAL_GetTick();
	uint32_t last_gyro = HAL_GetTick();
	uint32_t last_led  = HAL_GetTick();


	while (1)
	{
		uint32_t now = HAL_GetTick();
		__disable_irq();
		static uint8_t button_state = BUTTON_IDLE;
		static uint8_t press_type = SINGLE_PRESS;

		switch (button_state) {
		case BUTTON_IDLE:
			if (button_pressed) {
				button_state = BUTTON_WAIT;
				button_pressed = 0;
			}
			break;

		case BUTTON_WAIT:
			if (now - last_pressed >= PRESS_WINDOW_MS) {
				button_state = BUTTON_IDLE;
				press_type = SINGLE_PRESS;
			} else if (button_pressed) {
				button_state = BUTTON_IDLE;
				press_type = DOUBLE_PRESS;
				printf("Double Pressed\n");
				battlemode += 1;
				button_pressed = 0;
			}
			break;
		}

		static uint8_t print_state = PRINT_RESUME;

		switch (print_state) {
		case (PRINT_RESUME ):
			if (press_type == DOUBLE_PRESS&& battlemode % 2 ==1) {
				print_state = PRINT_PAUSE;
			}
		case (PRINT_PAUSE):
			if (press_type == DOUBLE_PRESS&& battlemode % 2 ==0) {
				print_state = PRINT_RESUME;
			}
		}
		__enable_irq();



//		__disable_irq();                // stop the ISR changing press_count mid-check
//				if (press_count == 2)
//				{
//					paused = 1;                 // double press: pause (no change if already paused)
//					press_count = 0;
//				}
//				else if (press_count == 1 && (now - first_press_tick) >= PRESS_WINDOW_MS)
//				{
//					paused = 0;                 // window expired with one press: resume
//					press_count = 0;
//				}
//				__enable_irq();

		if (now - last_led >= LED_PERIOD_MS)        // e.g. toggled at 470 → next at 720
		    {
		      last_led += LED_PERIOD_MS;
		      BSP_LED_Toggle(LED2);
		    }


		if (now - last_accel >= ACCEL_PERIOD_MS)
		{
			last_accel += ACCEL_PERIOD_MS;

			float accel_data[3];
			int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
			BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
			// the function above returns 16 bit integers which are acceleration in mg (9.8/1000 m/s^2).
			// Converting to float to print the actual acceleration.
			accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f);
			accel_data[1] = (float)accel_data_i16[1] * (9.8/1000.0f);
			accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);

			if (print_state == PRINT_RESUME){

				char buf[100];
				sprintf(buf ,"Accel X : %f; Accel Y : %f; Accel Z : %f\n", accel_data[0], accel_data[1], accel_data[2]);
			}
		}

		if (now - last_temp >= TEMP_PERIOD_MS){
			last_temp += TEMP_PERIOD_MS;
			float temp_data = BSP_TSENSOR_ReadTemp();
			if (print_state == PRINT_RESUME){
				char buf[100];
				int len = sprintf(buf, "Temperature : %.2f\r\n", temp_data);
				HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 0xFFFF);
			}
		}

		if (now - last_mag >= MAG_PERIOD_MS){
			last_mag += MAG_PERIOD_MS;
			int16_t mag_data_i16[3] = { 0 };       // X, Y, Z
			BSP_MAGNETO_GetXYZ(mag_data_i16);
			if (print_state == PRINT_RESUME){
				char message_print[100];        // UART transmit buffer. See the comment in the line above.
				sprintf(message_print,"Mag X : %d; Mag Y : %d; Mag Z : %d (mGauss)\r\n", mag_data_i16[0], mag_data_i16[1], mag_data_i16[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); //Sending in normal mode
			}
		}

		if (now - last_gyro >= GYRO_PERIOD_MS){
			last_gyro += GYRO_PERIOD_MS;
			float gyro_data_i16[3] = {0};
			BSP_GYRO_GetXYZ(gyro_data_i16);
			if (print_state == PRINT_RESUME){
				char message_print[100];        // UART transmit buffer. See the comment in the line above.
				sprintf(message_print,"Gyro X : %d; Gyro Y : %d; Gyro Z : %d\r\n\n", gyro_data_i16[0], gyro_data_i16[1], gyro_data_i16[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}
		}

	}

}

//UART stuff from LAB
static void UART1_Init(void)
{
        /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
        __HAL_RCC_GPIOB_CLK_ENABLE();
         __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Configuring UART1 */
        huart1.Instance = USART1;
        huart1.Init.BaudRate = 115200;
        huart1.Init.WordLength = UART_WORDLENGTH_8B;
        huart1.Init.StopBits = UART_STOPBITS_1;
        huart1.Init.Parity = UART_PARITY_NONE;
        huart1.Init.Mode = UART_MODE_TX_RX;
        huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart1.Init.OverSampling = UART_OVERSAMPLING_16;
        huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
        if (HAL_UART_Init(&huart1) != HAL_OK)
        {
          while(1);
        }

}



