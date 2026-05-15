/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
bno055_t       bno;
error_bno      bno_err;
bno055_euler_t euler;
bno055_vec4_t  quat;
bno055_vec3_t  accel;
bno055_vec3_t  linear_accel;
bno055_vec3_t  gyro;
bno055_vec3_t  mag;
bno055_vec3_t  gravity;
uint8_t        calib_stat = 0;
uint32_t       bno_seq = 0;
char           bno_json[1024];
uint8_t        bno_addr_7bit = BNO_ADDR_ALT;

// -------------------------------------------------------------------
// CALIBRATION PROFILE
// Once you achieve SYS/GYR/ACC/MAG all = 3, copy the array from the 
// terminal and paste it here. Reflash to start calibrated instantly.
// -------------------------------------------------------------------
uint8_t current_calib[22];
uint8_t saved_calib[22] = {233, 255, 33, 0, 220, 255, 192, 255, 57, 4, 124, 253, 254, 255, 255, 255, 0, 0, 232, 3, 159, 2};
bool use_saved_calib = false;
bool profile_ready = false; 

// Timing
uint32_t last_print_tick = 0;
uint32_t last_retry_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BNO_JSON_PERIOD_MS 20U
#define BNO_ERROR_PERIOD_MS 1000U
#define BNO_RETRY_PERIOD_MS 3000U
#define CDC_TX_TIMEOUT_MS 20U

static int append_text(char *buf, size_t size, int offset, const char *fmt, ...)
{
    if (offset < 0 || (size_t)offset >= size) {
        return offset;
    }

    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(&buf[offset], size - (size_t)offset, fmt, args);
    va_end(args);

    if (written < 0) {
        return offset;
    }

    if ((size_t)written >= size - (size_t)offset) {
        return (int)size - 1;
    }

    return offset + written;
}

static int append_fixed(char *buf, size_t size, int offset, float value, int32_t scale, uint8_t digits)
{
    int32_t scaled = (int32_t)((value * (float)scale) + ((value >= 0.0f) ? 0.5f : -0.5f));

    if (scaled < 0) {
        offset = append_text(buf, size, offset, "-");
        scaled = -scaled;
    }

    if (digits == 6U) {
        return append_text(buf, size, offset, "%ld.%06ld", (long)(scaled / scale), (long)(scaled % scale));
    }

    return append_text(buf, size, offset, "%ld.%04ld", (long)(scaled / scale), (long)(scaled % scale));
}

static int append_fixed4(char *buf, size_t size, int offset, float value)
{
    return append_fixed(buf, size, offset, value, 10000L, 4U);
}

static int append_fixed6(char *buf, size_t size, int offset, float value)
{
    return append_fixed(buf, size, offset, value, 1000000L, 6U);
}

static int append_vec3_json(char *buf, size_t size, int offset, const char *name, const bno055_vec3_t *value, const char *units)
{
    offset = append_text(buf, size, offset, "\"%s\":{\"x\":", name);
    offset = append_fixed4(buf, size, offset, value->x);
    offset = append_text(buf, size, offset, ",\"y\":");
    offset = append_fixed4(buf, size, offset, value->y);
    offset = append_text(buf, size, offset, ",\"z\":");
    offset = append_fixed4(buf, size, offset, value->z);
    return append_text(buf, size, offset, ",\"units\":\"%s\"}", units);
}

static bool cdc_write(const char *buf, uint16_t len)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < CDC_TX_TIMEOUT_MS) {
        uint8_t status = CDC_Transmit_FS((uint8_t *)buf, len);
        if (status == USBD_OK) {
            return true;
        }

        if (status != USBD_BUSY) {
            return false;
        }
    }

    return false;
}

static void configure_bno055(uint8_t addr_7bit)
{
    memset(&bno, 0, sizeof(bno));
    bno.i2c  = &hi2c1;
    bno.addr = addr_7bit;
    bno.mode = BNO_MODE_NDOF;

    bno._acc_unit = BNO_ACC_UNITSEL_M_S2;
    bno._gyr_unit = BNO_GYR_UNIT_DPS;
    bno._eul_unit = BNO_EUL_UNIT_DEG;
    bno._temp_unit= BNO_TEMP_UNIT_C;
    bno_addr_7bit = addr_7bit;
}

static bool saved_calibration_available(void)
{
    for (int i = 0; i < 22; i++) {
        if (saved_calib[i] != 0) {
            return true;
        }
    }

    return false;
}

static void finish_bno055_init(void)
{
    profile_ready = false;

    if (use_saved_calib && saved_calibration_available()) {
        printf("Loading saved calibration profile...\r\n");
        bno055_set_calibration(&bno, saved_calib);
    }

    bno055_ext_crystal(&bno, true);
    printf("BNO055 OK at I2C 0x%02X (NDOF Mode). Calibrate gyro, accelerometer, and magnetometer.\r\n", bno_addr_7bit);
}

static error_bno init_bno055_any_address(void)
{
    const uint8_t addresses[] = {BNO_ADDR_ALT, BNO_ADDR};
    error_bno last_error = BNO_ERR_I2C;

    for (uint32_t i = 0; i < sizeof(addresses); i++) {
        configure_bno055(addresses[i]);
        last_error = bno055_init(&bno);
        if (last_error == BNO_OK) {
            finish_bno055_init();
            return BNO_OK;
        }
    }

    return last_error;
}

static void print_bno055_error_json(void)
{
    int offset = append_text(bno_json, sizeof(bno_json), 0,
                             "{\"version\":1,\"source\":\"bno055\",\"seq\":%lu,\"timestampMs\":%lu,"
                             "\"status\":\"error\",\"errorCode\":%d,\"message\":\"BNO055 init failed\","
                             "\"addr7bit\":%u}\r\n",
                             (unsigned long)bno_seq++,
                             (unsigned long)HAL_GetTick(),
                             (int)bno_err,
                             (unsigned)bno_addr_7bit);

    if (offset > 0) {
        cdc_write(bno_json, (uint16_t)offset);
    }
}

static void print_bno055_json(void)
{
    int offset = 0;

    bno055_euler(&bno, &euler);
    bno055_quaternion(&bno, &quat);
    bno055_acc(&bno, &accel);
    bno055_linear_acc(&bno, &linear_accel);
    bno055_gyro(&bno, &gyro);
    bno055_mag(&bno, &mag);
    bno055_gravity(&bno, &gravity);
    bno055_read_regs(bno, BNO_CALIB_STAT, &calib_stat, 1);

    uint8_t s = (calib_stat >> 6) & 0x03;
    uint8_t g = (calib_stat >> 4) & 0x03;
    uint8_t a = (calib_stat >> 2) & 0x03;
    uint8_t m = calib_stat & 0x03;

    offset = append_text(bno_json, sizeof(bno_json), offset,
                         "{\"version\":1,\"source\":\"bno055\",\"seq\":%lu,\"timestampMs\":%lu,",
                         (unsigned long)bno_seq++,
                         (unsigned long)HAL_GetTick());

    offset = append_text(bno_json, sizeof(bno_json), offset, "\"euler\":{\"roll\":");
    offset = append_fixed4(bno_json, sizeof(bno_json), offset, euler.roll);
    offset = append_text(bno_json, sizeof(bno_json), offset, ",\"pitch\":");
    offset = append_fixed4(bno_json, sizeof(bno_json), offset, euler.pitch);
    offset = append_text(bno_json, sizeof(bno_json), offset, ",\"yaw\":");
    offset = append_fixed4(bno_json, sizeof(bno_json), offset, euler.yaw);
    offset = append_text(bno_json, sizeof(bno_json), offset, ",\"units\":\"deg\"},");

    offset = append_text(bno_json, sizeof(bno_json), offset, "\"quaternion\":{\"w\":");
    offset = append_fixed6(bno_json, sizeof(bno_json), offset, quat.w);
    offset = append_text(bno_json, sizeof(bno_json), offset, ",\"x\":");
    offset = append_fixed6(bno_json, sizeof(bno_json), offset, quat.x);
    offset = append_text(bno_json, sizeof(bno_json), offset, ",\"y\":");
    offset = append_fixed6(bno_json, sizeof(bno_json), offset, quat.y);
    offset = append_text(bno_json, sizeof(bno_json), offset, ",\"z\":");
    offset = append_fixed6(bno_json, sizeof(bno_json), offset, quat.z);
    offset = append_text(bno_json, sizeof(bno_json), offset, "},");

    offset = append_vec3_json(bno_json, sizeof(bno_json), offset, "accelerometer", &accel, "m/s^2");
    offset = append_text(bno_json, sizeof(bno_json), offset, ",");
    offset = append_vec3_json(bno_json, sizeof(bno_json), offset, "linearAcceleration", &linear_accel, "m/s^2");
    offset = append_text(bno_json, sizeof(bno_json), offset, ",");
    offset = append_vec3_json(bno_json, sizeof(bno_json), offset, "gyroscope", &gyro, "deg/s");
    offset = append_text(bno_json, sizeof(bno_json), offset, ",");
    offset = append_vec3_json(bno_json, sizeof(bno_json), offset, "magnetometer", &mag, "uT");
    offset = append_text(bno_json, sizeof(bno_json), offset, ",");
    offset = append_vec3_json(bno_json, sizeof(bno_json), offset, "gravity", &gravity, "m/s^2");

    offset = append_text(bno_json, sizeof(bno_json), offset,
                         ",\"calibration\":{\"system\":%u,\"gyro\":%u,\"accelerometer\":%u,\"magnetometer\":%u}}\r\n",
                         s, g, a, m);

    if (offset > 0) {
        cdc_write(bno_json, (uint16_t)offset);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(700);
  bno_err = init_bno055_any_address();
  last_retry_tick = HAL_GetTick();

  if (bno_err != BNO_OK) {
      printf("BNO055 FAILED: %d\r\n", bno_err);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (bno_err == BNO_OK) {
        if (HAL_GetTick() - last_print_tick >= BNO_JSON_PERIOD_MS) {
            last_print_tick = HAL_GetTick();

            print_bno055_json();

            uint8_t s = (calib_stat >> 6) & 0x03;
            uint8_t g = (calib_stat >> 4) & 0x03;
            uint8_t a = (calib_stat >> 2) & 0x03;
            uint8_t m = calib_stat & 0x03;

            // Once fully calibrated, print the profile for the user to save
            if (s == 3 && g == 3 && a == 3 && m == 3 && !profile_ready) {
                bno055_get_calibration(&bno, current_calib);
                
                printf("\r\n--- FULL CALIBRATION REACHED! COPY THIS LINE INTO main.c ---\r\n");
                printf("uint8_t saved_calib[22] = {");
                for(int i=0; i<22; i++) {
                    printf("%d%s", current_calib[i], (i < 21) ? ", " : "");
                }
                printf("};\r\n");
                printf("------------------------------------------------------------\r\n\r\n");
                
                profile_ready = true;
            }
        }
    } else {
        if (HAL_GetTick() - last_print_tick >= BNO_ERROR_PERIOD_MS) {
            last_print_tick = HAL_GetTick();
            print_bno055_error_json();
        }

        if (HAL_GetTick() - last_retry_tick >= BNO_RETRY_PERIOD_MS) {
            last_retry_tick = HAL_GetTick();
            bno_err = init_bno055_any_address();
        }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
    UNUSED(file);

    if (len <= 0) {
        return 0;
    }

    cdc_write(ptr, (uint16_t)len);
    return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
