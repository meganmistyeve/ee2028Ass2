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

#include "stdio.h"
#include "string.h"
#include "math.h"

#define ACCEL_PERIOD_MS   1000
#define TEMP_PERIOD_MS    1000
#define MAG_PERIOD_MS	1000
#define GYRO_PERIOD_MS 	1000
#define PRESSURE_PERIOD_MS  	1000
#define HUMIDITY_PERIOD_MS  	1000
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

//color code
#define BATTLE_RED  "\033[38;2;245;42;75m"
#define STANDBY_GREEN  "\033[38;2;158;255;89m"
#define SHOTFIRED_ORANG  "\033[38;2;255;175;56m"
#define RESET       "\033[0m"
#define DAMAGE_PINK  "\033[38;2;247;77;159m"
#define CAPTAIN_GOLD  "\033[38;2;255;219;13m"
//declaring numbers
//states
int last_of_ee2028 = 0;
int battlemode = 0;

//laser
const int laserMax = 10;
int laserCount = 0;
//tilt
float rot_around_x = 0;
float rot_around_y = 0;
uint32_t last_tilt_tick = 0;
#define TILT_PERIOD_MS  20
uint8_t  tilt_ready = 0;          // 1 once tiltInit() has a good starting angle
uint32_t last_tilt_check = 0;
uint32_t attacked_at = 0;         // when it rolled over (for the 10 s rescue)
uint32_t last_attack_msg = 0;

uint32_t tiltInit(void){


	float accel_data[3];
	int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
	BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
	// the function above returns 16 bit integers which are acceleration in mg (9.8/1000 m/s^2).
	// Converting to float to print the actual acceleration.
	accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f);//x
	accel_data[1] = (float)accel_data_i16[1] * (9.8/1000.0f);//y
	accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);///z


	//check if its moving or gravity
	float mag = sqrtf(accel_data[0]*accel_data[0] + accel_data[1]*accel_data[1] + accel_data[2]*accel_data[2]);
	if (mag < 6.9f || mag > 12.7f) return 0;



	rot_around_x = atan2f(accel_data[1],accel_data[2])*180/3.141592653589793;
	rot_around_y = atan2f(-accel_data[0],sqrtf(accel_data[1]*accel_data[1]+accel_data[2]*accel_data[2]))*180/3.141592653589793;
	last_tilt_tick = HAL_GetTick();
	return 1;
}

uint32_t tiltcheck(void){
	uint32_t tih = HAL_GetTick();
	float dt = (tih - last_tilt_tick) / 1000.0f;
	last_tilt_tick = tih;
	float gyro[3];
	BSP_GYRO_GetXYZ(gyro);

	rot_around_x += (gyro[0]) / 1000.0f * dt;
	rot_around_y += (gyro[1]) / 1000.0f * dt;

	return (fabsf(rot_around_x) >= 120 || fabsf(rot_around_y) >= 120);
}


















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
	BSP_LED_Init(LED2);
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

	UART1_Init();

//	uint8_t paused = 0;

	uint32_t last_accel = HAL_GetTick();
	uint32_t last_temp  = HAL_GetTick();
	uint32_t last_mag  = HAL_GetTick();
	uint32_t last_gyro = HAL_GetTick();
	uint32_t last_led  = HAL_GetTick();
	uint32_t last_pressure  = HAL_GetTick();
	uint32_t last_humidity  = HAL_GetTick();

	while (1)
	{

		if(laserCount >= laserMax){
			laserCount = laserMax;
		}
		if (laserCount <=0){
			laserCount = 0;
		}
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

				if (battlemode % 2 == 1 && last_of_ee2028 % 2 == 0)
				{
					laserCount += 3;
					char message_print[100];        // UART transmit buffer. See the comment in the line above.
					sprintf(message_print, "Added 3 Charges; Canister: [%d/10]\r\n\n", laserCount);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); //Sending in normal mode



					if (laserCount >= 5)
					{
						laserCount -= 5;        // UART transmit buffer. See the comment in the line above.
						sprintf(message_print, SHOTFIRED_ORANG"Shot Fired!!!"RESET"; Canister: [%d/10]\r\n\n", laserCount);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); //Sending in normal mode


					}
				}


			} else if (button_pressed) {
				button_state = BUTTON_IDLE;
				press_type = DOUBLE_PRESS;
				button_pressed = 0;
				char message_print[100];

				if (last_of_ee2028 % 2 == 1) {
					last_of_ee2028 += 1;
					tilt_ready = 0;
					sprintf(message_print, STANDBY_GREEN"Drone Rescued! Back to Battle Mode"RESET"\r\n\n");
				} else {
					battlemode += 1;
					tilt_ready = 0;
					if (battlemode % 2 == 1)
						sprintf(message_print, BATTLE_RED"Battle Mode Engaged"RESET"\r\n\n");
					else
						sprintf(message_print, STANDBY_GREEN"Stand By Mode Engaged"RESET"\r\n\n");
				}
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); //Sending in normal mode
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




		if (battlemode % 2 == 1 && last_of_ee2028 % 2 == 0) {
			if (!tilt_ready) {
				tilt_ready = tiltInit();                     // retries until the board is still
				last_tilt_check = now;
			} else if (now - last_tilt_check >= TILT_PERIOD_MS) {
				last_tilt_check = now;
				if (tiltcheck()) {                           // past 120 degrees
					last_of_ee2028 += 1;                     // odd = Last of EE2028
					attacked_at = now;
					tilt_ready = 0;

					char message_print[100];
					sprintf(message_print, DAMAGE_PINK"Critically Damaged; Entered Last_Of_2028 Mode"RESET"\r\n\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				}
			}
		}

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








		if (battlemode % 2 == 1 && last_of_ee2028 % 2 == 1) {
			if (now - attacked_at >= 10000) {
				char message_print[100];		// UART transmit buffer. See the comment in the line above.
				sprintf(message_print, CAPTAIN_GOLD"O Captain!\r\nMy Captain"RESET"\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);//Sending in normal mode

				BSP_LED_Off(LED2);
				HAL_UART_DeInit(&huart1);
				__disable_irq();
				while (1) {
				}
			}
			if (now - last_attack_msg >= 1000) {
				last_attack_msg = now;
				char message_print[100];
				sprintf(message_print, DAMAGE_PINK"Drone Was Attacked! "RESET"%lu\r\n", (unsigned long)((10000 - (now - attacked_at) + 999) / 1000));
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}
		}

		/* LED */
		if (battlemode % 2 == 1 && last_of_ee2028 % 2 == 1) {
			if (now - last_led >= 250) {
				last_led += 250;
				BSP_LED_Toggle(LED2);
			}
		}

		else if (battlemode % 2 == 1) {

			if (now - last_led >= 500) {
				last_led += 500;
				BSP_LED_Toggle(LED2);
			}
		}
		else {
			BSP_LED_On(LED2);
			last_led = now;
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



