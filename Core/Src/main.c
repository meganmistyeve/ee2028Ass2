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
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"

#include "stdio.h"
<<<<<<< Updated upstream
#include "string.h"

#define ACCEL_PERIOD_MS   1000
#define TEMP_PERIOD_MS    1000
#define MAG_PERIOD_MS	1000
#define GYRO_PERIOD_MS 	1000
#define PRESSURE_PERIOD_MS  	1000
#define HUMIDITY_PERIOD_MS  	1000
#define LED_PERIOD_MS  	1000
=======
#include <math.h>

#define LED_PERIOD_MS  	500
>>>>>>> Stashed changes
#define PRESS_WINDOW_MS  500
#define TELEMETRY_PERIOD_MS 1000

#define STANDBY_MODE 0
#define BATTLE_MODE  1

#define T_LOW   20.0f    // C
#define T_HIGH  40.0f
#define P_LOW   1000.0f  // hPa, pressure drops about 0.12 hPa per metre of height, so 1000 hPa is roughly 60 m above the lab. 1030 hPa is above normal weather.
#define P_HIGH  1030.0f
#define H_LOW   30.0f    // %RH
#define H_HIGH  70.0f
#define A_LOW   5.0f     // m/s^2 (Z-axis) About 9.8 when level. Below 5 means a tilt of more than about 60°. Above 15 means a jolt of more than 1.5 g.
#define A_HIGH  15.0f
#define G_HIGH  3.0f     // rad/s About 170 °/s, which counts as uncontrolled spinning
#define M_HIGH  300.0f   // uT

#define SINGLE_PRESS 0
#define DOUBLE_PRESS 1

#define BUTTON_IDLE 0
#define BUTTON_WAIT 1

<<<<<<< Updated upstream
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






=======
extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)

>>>>>>> Stashed changes
volatile uint8_t button_pressed = 0;
volatile uint32_t last_pressed = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == USER_BUTTON_PIN) {
		if (HAL_GetTick() - last_pressed < 50) return;   // ignore contact bounce
		button_pressed = 1;
		last_pressed = HAL_GetTick();
	}
}

<<<<<<< Updated upstream
//color code
#define BATTLE_RED  "\033[38;2;245;42;75m"
#define STANDBY_GREEN  "\033[38;2;158;255;89m"
#define RESET       "\033[0m"
=======
static uint8_t check(const char *name, float v, float lo, float hi, const char *unit)
{
    if (v > hi) {
        printf("%s: %.2f %s, exceeds threshold of %.2f %s\r\n", name, v, unit, hi, unit);
        return 1;
    }
    if (v < lo) {
        printf("%s: %.2f %s, below threshold of %.2f %s\r\n", name, v, unit, lo, unit);
        return 1;
    }
    return 0;
}
>>>>>>> Stashed changes

int battlemode = 0;
int main(void)
{

	initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Peripheral initializations using BSP functions */
	BSP_ACCELERO_Init();
	BSP_PSENSOR_Init();
	BSP_HSENSOR_Init();
	BSP_TSENSOR_Init();
	BSP_MAGNETO_Init();
	BSP_GYRO_Init();
	BSP_HSENSOR_Init();
	BSP_PSENSOR_Init();
	BSP_LED_Init(LED2);
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

<<<<<<< Updated upstream
	UART1_Init();

//	uint8_t paused = 0;

	uint32_t last_accel = HAL_GetTick();
	uint32_t last_temp  = HAL_GetTick();
	uint32_t last_mag  = HAL_GetTick();
	uint32_t last_gyro = HAL_GetTick();
	uint32_t last_led  = HAL_GetTick();
	uint32_t last_pressure  = HAL_GetTick();
	uint32_t last_humidity  = HAL_GetTick();
=======
	uint32_t last_led  = HAL_GetTick();
	uint32_t last_telemetry = HAL_GetTick();
>>>>>>> Stashed changes

	//printf("Entering STANDBY MODE\r\n");

	char standby_print[100];        // UART transmit buffer. See the comment in the line above.
	sprintf(standby_print,"Entering STANDBY MODE\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)standby_print, strlen(standby_print),0xFFFF);

	uint8_t mode = STANDBY_MODE;
	BSP_LED_On(LED2);

	while (1)
	{
		uint32_t now = HAL_GetTick();
		__disable_irq();
		static uint8_t button_state = BUTTON_IDLE;
		static uint8_t press_type = SINGLE_PRESS;

		uint8_t press_detected = 0;

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
				press_detected = 1;
			} else if (button_pressed) {
				button_state = BUTTON_IDLE;
				press_type = DOUBLE_PRESS;
				battlemode += 1;
				if(battlemode % 2 == 1){

					char message_print[100];        // UART transmit buffer. See the comment in the line above.
					sprintf(message_print, BATTLE_RED"Battle Mode Engaged"RESET"\r\n\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); //Sending in normal mode

				}else{

					char message_print[100];        // UART transmit buffer. See the comment in the line above.
					sprintf(message_print,STANDBY_GREEN "Stand By Mode Engaged"RESET"\r\n\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); //Sending in normal mode

				}
				button_pressed = 0;
				press_detected = 1;
			}
			break;
		}

<<<<<<< Updated upstream
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
=======
>>>>>>> Stashed changes
		__enable_irq();

		if (press_detected && press_type == DOUBLE_PRESS) {
			if (mode == STANDBY_MODE) {
				mode = BATTLE_MODE;
				printf("Entering BATTLE MODE\r\n");
				last_led = now;
			} else {
				mode = STANDBY_MODE;
				printf("Entering STANDBY MODE\r\n");
				BSP_LED_On(LED2);
			}
		} else if (press_detected && press_type == SINGLE_PRESS) {
			if (mode == BATTLE_MODE) {
				// LASERGUN_CHARGING goes here
			}
		}

		if (mode == BATTLE_MODE && now - last_led >= LED_PERIOD_MS) {
			last_led += LED_PERIOD_MS;
			BSP_LED_Toggle(LED2);
		}

		if (now - last_telemetry >= TELEMETRY_PERIOD_MS)
		{
		    last_telemetry += TELEMETRY_PERIOD_MS;

		    float t = BSP_TSENSOR_ReadTemp();
		    float p = BSP_PSENSOR_ReadPressure();
		    float h = BSP_HSENSOR_ReadHumidity();

<<<<<<< Updated upstream
			if (print_state == PRINT_RESUME){

				char message_print[100];        //Remember \r\n it goes down a line from the start
				sprintf(message_print, "Accel X : %.2f; Accel Y : %.2f; Accel Z : %.2f (m/s^2)\r\n", accel_data[0], accel_data[1], accel_data[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); //Sending in normal mode

			}
		}

		if (now - last_temp >= TEMP_PERIOD_MS){
			last_temp += TEMP_PERIOD_MS;
			float temp_data = BSP_TSENSOR_ReadTemp();
			if (print_state == PRINT_RESUME){
				char message_print[100];        // UART transmit buffer. See the comment in the line above.
				sprintf(message_print, "Temperature : %.2f (degC)\r\n", temp_data);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); //Sending in normal mode
			}
		}

		if (now - last_pressure >= PRESSURE_PERIOD_MS){
			last_pressure += PRESSURE_PERIOD_MS;
			float temp_data = BSP_PSENSOR_ReadPressure();
			if (print_state == PRINT_RESUME){
				char message_print[100];        // UART transmit buffer. See the comment in the line above.
				sprintf(message_print, "Pressure : %.2f (hPa)\r\n", temp_data);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); //Sending in normal mode
			}
		}

		if (now - last_humidity >= HUMIDITY_PERIOD_MS){
			last_humidity += HUMIDITY_PERIOD_MS;
			float temp_data = BSP_HSENSOR_ReadHumidity();
			if (print_state == PRINT_RESUME){
				char message_print[100];
				sprintf(message_print, "Humidity : %.2f (%%RH)\r\n", temp_data);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
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
				sprintf(message_print,"Gyro X : %.2f; Gyro Y : %.2f; Gyro Z : %.2f \r\n\n", gyro_data_i16[0], gyro_data_i16[1], gyro_data_i16[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}
		}

=======
		    int16_t accel_data_i16[3] = {0};
		    BSP_ACCELERO_AccGetXYZ(accel_data_i16);
		    float a = accel_data_i16[2] * (9.8f / 1000.0f);

		    float gyro_data_i16[3] = {0};
		    BSP_GYRO_GetXYZ(gyro_data_i16);
		    float gx = gyro_data_i16[0] / 1000.0f * (float)M_PI / 180.0f;
		    float gy = gyro_data_i16[1] / 1000.0f * (float)M_PI / 180.0f;
		    float gz = gyro_data_i16[2] / 1000.0f * (float)M_PI / 180.0f;
		    float g = sqrtf(gx * gx + gy * gy + gz * gz);

		    int16_t mag_data_i16[3] = {0};
		    BSP_MAGNETO_GetXYZ(mag_data_i16);
		    float mx = mag_data_i16[0] * 0.1f;
		    float my = mag_data_i16[1] * 0.1f;
		    float mz = mag_data_i16[2] * 0.1f;
		    float m = sqrtf(mx * mx + my * my + mz * mz);

		    uint8_t violated = 0;
		    violated |= check("T", t, T_LOW, T_HIGH, "C");
		    violated |= check("P", p, P_LOW, P_HIGH, "hPa");
		    violated |= check("H", h, H_LOW, H_HIGH, "%RH");
		    violated |= check("A", a, A_LOW, A_HIGH, "m/s^2");
		    violated |= check("G", g, -1.0f, G_HIGH, "rad/s");   // magnitude can't be negative, so no low limit
		    violated |= check("M", m, -1.0f, M_HIGH, "uT");

		    if (!violated) {
		        // printf("T: %.2f C, P: %.2f hPa, H: %.2f %%RH, A: %.2f m/s^2, G: %.2f rad/s, M: %.2f uT\r\n",
		        //        t, p, h, a, g, m);

				char violated_print[100];        // UART transmit buffer. See the comment in the line above.
				sprintf(violated_print,"T: %.2f C, P: %.2f hPa, H: %.2f %%RH, A: %.2f m/s^2, G: %.2f rad/s, M: %.2f uT\r\n",
		               t, p, h, a, g, m);
				HAL_UART_Transmit(&huart1, (uint8_t*)violated_print, strlen(violated_print),0xFFFF);
		    }
		}
>>>>>>> Stashed changes
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



