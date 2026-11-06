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
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#define LED_PERIOD_MS        500     // toggle every 500 ms = 1 Hz blink
#define LED_ATTACKED_MS      250
#define PRESS_WINDOW_MS      500
#define TELEMETRY_PERIOD_MS  1000

#define STANDBY_MODE 0
#define BATTLE_MODE  1

#define T_LOW   20.0f    // C
#define T_HIGH  40.0f
#define P_LOW   1000.0f  // hPa, pressure drops about 0.12 hPa per metre of height, so 1000 hPa is roughly 60 m above the lab. 1030 hPa is above normal weather.
#define P_HIGH  1030.0f
#define H_LOW   30.0f    // %RH
#define H_HIGH  70.0f
#define A_LOW   5.0f     // m/s^2 (Z-axis) About 9.8 when level. Below 5 means a tilt of more than about 60 deg. Above 15 means a jolt of more than 1.5 g.
#define A_HIGH  15.0f
#define G_HIGH  3.0f     // rad/s About 170 deg/s, which counts as uncontrolled spinning
#define M_HIGH  300.0f   // uT

#define SINGLE_PRESS 0
#define DOUBLE_PRESS 1

#define BUTTON_IDLE 0
#define BUTTON_WAIT 1

// Sensor indices for the threshold/violation tables below
#define SENS_T 0		//temperature
#define SENS_P 1		//pressure
#define SENS_H 2		//humidity
#define SENS_A 3		//accelerometer
#define SENS_G 4		//gyroscope
#define SENS_M 5		//magnetometer
#define NUM_SENSORS 6	//total number of sensors

// Threshold tables (a low limit of -1 means "no low limit")
static const char  *sensor_name[NUM_SENSORS] = { "T", "P", "H", "A", "G", "M" };
static const char  *sensor_unit[NUM_SENSORS] = { "C", "hPa", "%RH", "m/s^2", "rad/s", "uT" };
static const float  sensor_low[NUM_SENSORS]  = { T_LOW,  P_LOW,  H_LOW,  A_LOW,  -1.0f,  -1.0f  }; //G and M will always be positive because they're square roots, "-1" is put here
static const float  sensor_high[NUM_SENSORS] = { T_HIGH, P_HIGH, H_HIGH, A_HIGH, G_HIGH, M_HIGH };

// Violation tracking for the status report
static uint8_t  was_violated[NUM_SENSORS]    = {0};   // out of range at the last reading?
static uint32_t violation_count[NUM_SENSORS] = {0};   // number of separate violations since power-on

// ANSI colour codes for the terminal
#define BATTLE_RED       "\033[38;2;245;42;75m"
#define STANDBY_GREEN    "\033[38;2;158;255;89m"
#define SHOTFIRED_ORANG  "\033[38;2;255;175;56m"
#define DAMAGE_PINK      "\033[38;2;247;77;159m"
#define CAPTAIN_GOLD     "\033[38;2;255;219;13m"
#define COLOR_RESET      "\033[0m"

// UART
UART_HandleTypeDef huart1;
static void UART1_Init(void);

// printf-style output over UART1
static void uart_printf(const char *fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len < 0) return;
    if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;   // message was cut off
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 0xFFFF);
}

volatile uint8_t button_pressed = 0;
volatile uint32_t last_pressed = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == USER_BUTTON_PIN) {
		if (HAL_GetTick() - last_pressed < 50) return;   // ignore contact bounce
		button_pressed = 1;
		last_pressed = HAL_GetTick();
	}
}

int last_of_ee2028 = 0;

const int laserMax = 10;
int laserCount = 0;

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
//checking if its actually upright before the thing.. rescue else it wont save
uint8_t board_upright(void){
	int16_t a[3] = { 0 };
	BSP_ACCELERO_AccGetXYZ(a);
	float x = (float)a[0] * (9.8f / 1000.0f);
	float y = (float)a[1] * (9.8f / 1000.0f);
	float z = (float)a[2] * (9.8f / 1000.0f);

	float mag = sqrtf(x*x + y*y + z*z);
	if (mag < 6.9f || mag > 12.7f) return 0;

	return z > 0.0f;
}

// Checks one sensor against its limits, prints a violation line if needed,
// and counts each new violation once (not once per second while it lasts).
static uint8_t check(uint8_t id, float v)
{
    uint8_t bad = 0;

    if (v > sensor_high[id]) {
        uart_printf("%s: %.2f %s, exceeds threshold of %.2f %s\r\n",
                    sensor_name[id], v, sensor_unit[id], sensor_high[id], sensor_unit[id]);
        bad = 1;
    } else if (v < sensor_low[id]) {
        uart_printf("%s: %.2f %s, below threshold of %.2f %s\r\n",
                    sensor_name[id], v, sensor_unit[id], sensor_low[id], sensor_unit[id]);
        bad = 1;
    }

    if (bad && !was_violated[id]) {
        violation_count[id]++;               // in range -> out of range: a new violation
    }
    was_violated[id] = bad;

    return bad;
}

// One-off status report, sent on a single press in STANDBY_MODE
static void print_status_report(void)
{
    uint32_t s = HAL_GetTick() / 1000;       // uptime in seconds
    uint32_t total = 0;

    uart_printf("\r\n========== STATUS REPORT ==========\r\n");
    uart_printf("Uptime: %02lu:%02lu:%02lu\r\n", s / 3600, (s / 60) % 60, s % 60); //00:04:37 means the board has been running for 4 minutes and 37 seconds
    uart_printf("Thresholds and violations since power-on:\r\n");

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        const char *now_state = was_violated[i] ? "OUT OF RANGE" : "OK";

        if (sensor_low[i] < 0.0f) {          // no low limit
            uart_printf("  %s: below %.2f %s | violations: %lu | now: %s\r\n",
                        sensor_name[i], sensor_high[i], sensor_unit[i],
                        violation_count[i], now_state);
        } else {
            uart_printf("  %s: %.2f to %.2f %s | violations: %lu | now: %s\r\n",
                        sensor_name[i], sensor_low[i], sensor_high[i], sensor_unit[i],
                        violation_count[i], now_state);
        }
        total += violation_count[i];
    }

    uart_printf("Total violations: %lu\r\n", total);
    uart_printf("===================================\r\n\r\n");
}

// Prints one line of the startup check, e.g. "  Pressure      OK"
static void report_init(const char *name, uint8_t ok)
{
    uart_printf("  %-13s %s\r\n", name, ok ? "OK" : "FAILED");
}

int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	UART1_Init();                    // start UART first so the check results can be printed

	uart_printf(STANDBY_GREEN "Entering STANDBY MODE" COLOR_RESET "\r\n");

	/* Sensor startup check: each BSP init returns OK (0) if the sensor responded */
	uart_printf("\r\nSensor startup check:\r\n");
	uint8_t fails = 0;
	uint8_t ok;

	ok = (BSP_ACCELERO_Init() == ACCELERO_OK); report_init("Accelerometer", ok); fails += !ok;
	ok = (BSP_GYRO_Init()     == GYRO_OK);     report_init("Gyroscope", ok);     fails += !ok;
	ok = (BSP_MAGNETO_Init()  == MAGNETO_OK);  report_init("Magnetometer", ok);  fails += !ok;
	ok = (BSP_TSENSOR_Init()  == TSENSOR_OK);  report_init("Temperature", ok);   fails += !ok;
	ok = (BSP_HSENSOR_Init()  == HSENSOR_OK);  report_init("Humidity", ok);      fails += !ok;
	ok = (BSP_PSENSOR_Init()  == PSENSOR_OK);  report_init("Pressure", ok);      fails += !ok;

	if (fails == 0) {
		uart_printf("All sensors OK\r\n\r\n");
	} else {
		uart_printf("WARNING: %d sensor(s) failed to start, readings may be invalid\r\n\r\n", fails);
	}

	BSP_LED_Init(LED2);
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

	// Reference pressure at power-on (average of 10 readings for stability)
	    	float p0 = 0.0f;
	    	for (int i = 0; i < 10; i++) {
	    		HAL_Delay(50);
	    		p0 += BSP_PSENSOR_ReadPressure();
	    	}
	    	p0 /= 10.0f;
	    	float alt_filt = 0.0f;      // smoothed altitude, starts at 0 m
	    	float t_f = 0.0f, h_f = 0.0f, m_f = 0.0f;   // smoothed T, H, M
	    	uint8_t filt_ready = 0;                     // 0 until the first reading

	uint32_t last_led       = HAL_GetTick();
	uint32_t last_telemetry = HAL_GetTick();

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
				button_pressed = 0;
				press_detected = 1;
			}
			break;
		}

		__enable_irq();

		if (press_detected && press_type == DOUBLE_PRESS) {
			if (mode == BATTLE_MODE && last_of_ee2028 % 2 == 1) {
				if (board_upright()) {
					last_of_ee2028 += 1;
					tilt_ready = 0;
					uart_printf(STANDBY_GREEN "Drone Rescued! Back to Battle Mode" COLOR_RESET "\r\n\n");
				} else {
					uart_printf(DAMAGE_PINK "Rescue failed: drone is upside down and unstable" COLOR_RESET "\r\n\n");
				}
			} else if (mode == STANDBY_MODE) {
				mode = BATTLE_MODE;
				tilt_ready = 0;
				uart_printf(BATTLE_RED "Entering BATTLE MODE" COLOR_RESET "\r\n\n");
				last_led = now;                  // restart blink timing
			} else {
				mode = STANDBY_MODE;
				uart_printf(STANDBY_GREEN "Entering STANDBY MODE" COLOR_RESET "\r\n");
				BSP_LED_On(LED2);
			}
		} else if (press_detected && press_type == SINGLE_PRESS) {
			if (mode == BATTLE_MODE) {
				if (last_of_ee2028 % 2 == 0) {
					laserCount += 3;
					if (laserCount > laserMax) laserCount = laserMax;
					uart_printf("Added 3 Charges; Canister: [%d/10]\r\n\n", laserCount);

					if (laserCount >= 5) {
						laserCount -= 5;
						uart_printf(SHOTFIRED_ORANG "Shot Fired!!!" COLOR_RESET "; Canister: [%d/10]\r\n\n", laserCount);
					}
				}
			} else {
				print_status_report();           // STANDBY: user button single press
			}
		}

		if (mode == BATTLE_MODE && last_of_ee2028 % 2 == 0) {
			if (!tilt_ready) {
				tilt_ready = tiltInit();                     // retries until the board is still
				last_tilt_check = now;
			} else if (now - last_tilt_check >= TILT_PERIOD_MS) {
				last_tilt_check = now;
				if (tiltcheck()) {                           // past 120 degrees
					last_of_ee2028 += 1;                     // odd = Last of EE2028
					attacked_at = now;
					tilt_ready = 0;
					uart_printf(DAMAGE_PINK "Critically Damaged; Entered Last_Of_2028 Mode" COLOR_RESET "\r\n\n");
				}
			}
		}

		if (mode == BATTLE_MODE && last_of_ee2028 % 2 == 1) {
			if (now - attacked_at >= 10000) {
				uart_printf("\r\n" CAPTAIN_GOLD "O Captain!\r\nMy Captain" COLOR_RESET "\r\n");
				BSP_LED_Off(LED2);
				HAL_UART_DeInit(&huart1);
				__disable_irq();
				while (1) {
				}
			}
			if (now - last_attack_msg >= 1000) {
				last_attack_msg = now;
				uart_printf(DAMAGE_PINK "Drone Was Attacked! " COLOR_RESET "%lu\r\n", (unsigned long)((10000 - (now - attacked_at) + 999) / 1000));
			}
		}

		/* LED */
		if (mode == BATTLE_MODE && last_of_ee2028 % 2 == 1) {
			if (now - last_led >= LED_ATTACKED_MS) {
				last_led += LED_ATTACKED_MS;
				BSP_LED_Toggle(LED2);
			}
		} else if (mode == BATTLE_MODE) {
			if (now - last_led >= LED_PERIOD_MS) {
				last_led += LED_PERIOD_MS;
				BSP_LED_Toggle(LED2);
			}
		}
		else {
			BSP_LED_On(LED2);
			last_led = now;
		}

		if (mode != STANDBY_MODE) {
			last_telemetry = now;
		} else if (now - last_telemetry >= TELEMETRY_PERIOD_MS)
		{
		    last_telemetry += TELEMETRY_PERIOD_MS;

		    float t = BSP_TSENSOR_ReadTemp();        // C
		    float p = BSP_PSENSOR_ReadPressure();    // hPa
		    float alt = 44330.0f * (1.0f - powf(p / p0, 1.0f / 5.255f));   // m, relative to start, calculate height
		    alt_filt += 0.2f * (alt - alt_filt);   // exponential moving average
		    float h = BSP_HSENSOR_ReadHumidity();    // %RH

		    int16_t accel_data_i16[3] = {0};         // mg
		    BSP_ACCELERO_AccGetXYZ(accel_data_i16);
		    float a = accel_data_i16[2] * (9.8f / 1000.0f);   // m/s^2, Z-axis

		    float gyro_data_i16[3] = {0};            // float mdps, not int16
		    BSP_GYRO_GetXYZ(gyro_data_i16);
		    float gx = gyro_data_i16[0] / 1000.0f * (float)M_PI / 180.0f;   // rad/s
		    float gy = gyro_data_i16[1] / 1000.0f * (float)M_PI / 180.0f;
		    float gz = gyro_data_i16[2] / 1000.0f * (float)M_PI / 180.0f;
		    float g = sqrtf(gx * gx + gy * gy + gz * gz);

		    int16_t mag_data_i16[3] = {0};           // mGauss
		    BSP_MAGNETO_GetXYZ(mag_data_i16);
		    float mx = mag_data_i16[0] * 0.1f;       // uT (1 mG = 0.1 uT)
		    float my = mag_data_i16[1] * 0.1f;
		    float mz = mag_data_i16[2] * 0.1f;
		    float m = sqrtf(mx * mx + my * my + mz * mz);

		    if (!filt_ready) {          // first reading: start the filters at the real value
		        t_f = t;
		        h_f = h;
		        m_f = m;
		        filt_ready = 1;
		    } else {
		        t_f += 0.2f * (t - t_f);
		        h_f += 0.2f * (h - h_f);
		        m_f += 0.2f * (m - m_f);
		    }

		    uint8_t violated = 0;
		    violated |= check(SENS_T, t_f);
		    violated |= check(SENS_H, h_f);
		    violated |= check(SENS_M, m_f);
		    violated |= check(SENS_P, p);
		    violated |= check(SENS_A, a);
		    violated |= check(SENS_G, g);

		    if (!violated) {
		        uart_printf("T: %.2f C, P: %.2f hPa (Alt: %.1f m), H: %.2f %%RH, A: %.2f m/s^2, G: %.2f rad/s, M: %.2f uT\r\n\n",
		                    t_f, p, alt_filt, h_f, a, g, m_f);
		    }
		}
	}

}

// UART stuff from LAB
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
//the days blurred together...
