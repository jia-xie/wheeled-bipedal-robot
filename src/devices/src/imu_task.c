/**
 * @file       imu_task.c/h
 * @brief      use bmi088 to calculate the euler angle.
 *
 * @note        change sample frequency in fusion
 */

#include "imu_task.h"
#include "Fusion.h"
#include "user_math.h"

static void imu_cmd_spi_dma(void);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
void IMU_Task_Init(IMU_t *imu);
void IMU_Task_Process(IMU_t *imu);
void IMU_Task_Temp();

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim10;

static TaskHandle_t imu_task_local_handler;
uint16_t imu_up_time_ms = 0;
#define IMU_READY_TIME_MS (3000)
uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};

/* Hardware Controlled */
volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

IMU_t g_imu;
PID_t g_imu_temp_pid;
FusionAhrs g_fusion_ahrs;
float gravity[3] = {0, 0, 9.81f};
void IMU_Task(void const *pvParameters)
{
    IMU_Task_Init(&g_imu);
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);
    while (1)
    {
        /* Wait for SPI DMA @ref HAL_GPIO_EXTI_Callback() */
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
            ;
        if (imu_up_time_ms == IMU_READY_TIME_MS)
        {
            g_imu.imu_ready_flag = 1;
        }
        else
        {
            imu_up_time_ms++;
        }
        IMU_Task_Process(&g_imu);
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

void IMU_Task_Init(IMU_t *imu)
{
    PID_Init(&g_imu_temp_pid, 1600.0f, 0.2f, 0, 4500, 4400, 0);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    /* This is the handle of imu task, task will be suspended until waken up*/
    imu_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    int error = BMI088_init();
    while (error)
    {
        error = BMI088_init();
        osDelay(100);
    }
    BMI088_init();
    //    while (ist8310_init())
    //    {
    //        osDelay(100);
    //    }

    BMI088_read(imu->bmi088_raw.gyro, imu->bmi088_raw.accel, &imu->bmi088_raw.temp);

    /* Initializing Quaternion */
    imu->quat[0] = 1.0f;
    imu->quat[1] = 0.0f;
    imu->quat[2] = 0.0f;
    imu->quat[3] = 0.0f;

    imu->AccelLPF = 0.0089f;
    //    HAL_SPI_Receive_DMA(&hspi1, gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    //    HAL_SPI_Transmit_DMA(&hspi1, gyro_dma_tx_buf, SPI_DMA_GYRO_LENGHT);
    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    imu_start_dma_flag = 1;
}

void IMU_Task_Process(IMU_t *imu)
{
    /* Decode DMA Buffer, Calling Decoding Functions in Hardware Driver */
    if (gyro_update_flag & (1 << IMU_NOTIFY_SHFITS))
    {
        gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
        BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, g_imu.bmi088_raw.gyro);
    }

    if (accel_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
        accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
        BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, g_imu.bmi088_raw.accel, &g_imu.bmi088_raw.time);
    }

    if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
        accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
        BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &g_imu.bmi088_raw.temp);
    }

#ifdef WITH_MAGNETOMETER
    // fusion using magnetometer
    MahonyAHRSupdate(imu->quat,
                     imu->bmi088_raw.gyro[0], imu->bmi088_raw.gyro[1], imu->bmi088_raw.gyro[2],
                     imu->bmi088_raw.accel[0], imu->bmi088_raw.accel[1], imu->bmi088_raw.accel[2],
                     imu->ist8310_raw.mag[0], imu->ist8310_raw.mag[1], imu->ist8310_raw.mag[2]);
#else
    // fusion without magnetometer
    MahonyAHRSupdateIMU(imu->quat,
                        imu->bmi088_raw.gyro[0], imu->bmi088_raw.gyro[1], imu->bmi088_raw.gyro[2],
                        imu->bmi088_raw.accel[0], imu->bmi088_raw.accel[1], imu->bmi088_raw.accel[2]);
#endif
    /* Fusion Estimation */
    // Initialize Fusion Interface with Gyro and Accel values
    const FusionVector fusion_vector_gyro = {{imu->bmi088_raw.gyro[0] * RAD_TO_DEG, imu->bmi088_raw.gyro[1] * RAD_TO_DEG, imu->bmi088_raw.gyro[2] * RAD_TO_DEG}};
    const FusionVector fusion_vector_accel = {{imu->bmi088_raw.accel[0], imu->bmi088_raw.accel[1], imu->bmi088_raw.accel[2]}};
    FusionAhrsUpdateNoMagnetometer(&g_fusion_ahrs, fusion_vector_gyro, fusion_vector_accel, IMU_TASK_PERIOD_SEC);

    const FusionEuler fusion_imu = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&g_fusion_ahrs));
    g_imu.deg_fusion.pitch = fusion_imu.angle.pitch;
    g_imu.deg_fusion.roll = fusion_imu.angle.roll;
    g_imu.deg_fusion.yaw = fusion_imu.angle.yaw;

    g_imu.rad_fusion.pitch = fusion_imu.angle.pitch * DEG_TO_RAD;
    g_imu.rad_fusion.roll = fusion_imu.angle.roll * DEG_TO_RAD;
    g_imu.rad_fusion.yaw = fusion_imu.angle.yaw * DEG_TO_RAD;

    /* Quternion to Euler */
    imu->rad.yaw = atan2f(2.0f * (imu->quat[0] * imu->quat[3] + imu->quat[1] * imu->quat[2]), 2.0f * (imu->quat[0] * imu->quat[0] + imu->quat[1] * imu->quat[1]) - 1.0f);
    imu->rad.pitch = asinf(-2.0f * (imu->quat[1] * imu->quat[3] - imu->quat[0] * imu->quat[2]));
    imu->rad.roll = atan2f(2.0f * (imu->quat[0] * imu->quat[1] + imu->quat[2] * imu->quat[3]), 2.0f * (imu->quat[0] * imu->quat[0] + imu->quat[3] * imu->quat[3]) - 1.0f);
    /* Radian to Degree*/
    imu->deg.yaw = imu->rad.yaw * RAD_TO_DEG;
    imu->deg.pitch = imu->rad.pitch * RAD_TO_DEG;
    imu->deg.roll = imu->rad.roll * RAD_TO_DEG;


    const float ins_dt = 0.001f;
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, imu->quat);
    for (uint8_t i = 0; i < 3; i++)
    {
        imu->accel_body[i] = (imu->bmi088_raw.accel[i] - gravity_b[i]) * ins_dt / (imu->AccelLPF + ins_dt) + imu->accel_body[i] * imu->AccelLPF / (imu->AccelLPF + ins_dt);
    }
    BodyFrameToEarthFrame(imu->accel_body, imu->accel_earth, imu->quat);
    		// 锟斤拷锟斤拷锟斤拷锟斤拷
		if (fabsf(imu->accel_earth[0]) < 0.02f)
		{
			imu->accel_earth[0] = 0.0f; // x锟斤拷
		}
		if (fabsf(imu->accel_earth[1]) < 0.04f)
		{
			imu->accel_earth[1] = 0.0f; // y锟斤拷
		}
		if (fabsf(imu->accel_earth[2]) < 0.04f)
		{
			imu->accel_earth[2] = 0.0f; // z锟斤拷
		}
    IMU_Task_Temp();
}

void IMU_Task_Temp()
{
    static uint8_t start_complete = 0;
    if (g_imu.bmi088_raw.temp > 40.0f)
    {
        start_complete = 1;
    }
    switch (start_complete)
    {
    case 1:
    {
        uint16_t temp_pwm = (uint16_t)PID(&g_imu_temp_pid, 40 - g_imu.bmi088_raw.temp);
        __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, temp_pwm);
        break;
    }
    case 0:
        __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, 4999);
        break;

    default:
        break;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT1_ACCEL_Pin)
    {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if (GPIO_Pin == INT1_GYRO_Pin)
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if (GPIO_Pin == DRDY_IST8310_Pin)
    {
        mag_update_flag |= 1 << IMU_DR_SHFITS;

        if (mag_update_flag &= 1 << IMU_DR_SHFITS)
        {
            mag_update_flag &= ~(1 << IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);

            // ist8310_read_mag(g_imu.ist8310_raw.mag);
        }
    }
    else if (GPIO_Pin == GPIO_PIN_0)
    {
        // wake up the task
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(imu_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

static void imu_cmd_spi_dma(void)
{
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    /* Enable Chip Selection for Gyro, and Start Receiveing Data from SPI */
    if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    /* Enable Chip Selection for Accel, and Start Receiveing Data from SPI */
    if ((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    /* Enable Chip Selection for Accel, and Start Receiveing Data from SPI */
    if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void DMA2_Stream2_IRQHandler(void)
{

    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        // gyro read over
        if (gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }

        // accel read over
        if (accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        // temperature read over
        if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}