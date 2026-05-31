#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "control.h"
#include "ble.h"
#include "battery.h"




static float init_altitude, init_temperature;


/*-----------------------------------------------------------*/

static void exampleTask( void * parameters );

/*-----------------------------------------------------------*/

static void exampleTask( void * parameters )
{
    /* Unused parameters. */
    ( void ) parameters;

    for( ; ; ) {
        /* Example Task Code */
        vTaskDelay( 100 ); /* delay 100 ticks */

        // switch led
        static uint8_t ledState = 0;
        ledState = !ledState;
        if (ledState) {
            GPIO_LED->BSRRL = GPIO_LED_PIN;
        } else {
            GPIO_LED->BSRRH = GPIO_LED_PIN;
        }

    }
}
/*-----------------------------------------------------------*/


int main() {
    RCC_ClocksTypeDef RCC_Clocks;
    /* Enable Clock Security System(CSS): this will generate an NMI exception
     when HSE clock fails *****************************************************/
    RCC_ClockSecuritySystemCmd(ENABLE);

    /*!< At this stage the microcontroller clock setting is already configured,
        this is done through SystemInit() function which is called from startup
        files before to branch to application main.
        To reconfigure the default setting of SystemInit() function,
        refer to system_stm32f4xx.c file */

    /* SysTick end of count event each 1ms */
    SystemCoreClockUpdate();                           // 更新  RCC_Clocks 系统时钟变量
    RCC_GetClocksFreq(&RCC_Clocks);                    // 获取  RCC_Clocks  系统时钟
    SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);  // 设置  SysTick  系统时钟中断为1mS

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    Key_GPIO_Init();
    LED_GPIO_Init();
    Battery_GPIO_Init();
    Battery_ADC_Init();
    Battery_Measure_Reset();

    LED_Blink(LED_TOGGLE_CMD_BLINK_FAST, 1);

    OLED_GPIO_Init();
    OLED_Init();
    GFX_Init();
    UI_Logger_Init(&logWindow, 0, 0, 127, 60);
    Window_Init();

    Init_DMA_For_USART1_RX(bleRxBuffer, sizeof(bleRxBuffer));
    Init_DMA_For_USART1_TX(bleTxBuffer);
    UART1_GPIO_Init();
    UART2_GPIO_Init();
    BLE_Init(BLE_BAUDRATE_115200);
    ExtraSensor_Init(ES_BAUDRATE_115200);
    UI_Logger_AddLine(&logWindow, "UART Init OK");

    SPI_Sensor_GPIO_Init();
    SPI_Sensor_Init();

    bmi260_dev_t bmi;
    bmi260_raw_xyz_t acc, gyr;
    bmi.sensor_id = SPI_Sensor_Register(GPIO_IMU_SPI, GPIO_IMU_SPI_CS_PIN);
    BMI260_SetDelay(Delay_ms);
    int8_t imu_ret = BMI260_Init(&bmi);

    bmp580_dev_t bmp580;
    bmp580_data_t baro_data;
    bmp580.sensor_id = SPI_Sensor_Register(GPIO_BARO_SPI, GPIO_BARO_SPI_CS_PIN);
    BMP580_SetDelay(Delay_ms);
    int8_t bmp580_ret = BMP580_Init(&bmp580);

    mmc5983ma_dev_t mmc;
    mmc5983ma_raw_data_t mag_data;
    mmc.sensor_id = SPI_Sensor_Register(GPIO_MAG_SPI, GPIO_MAG_SPI_CS_PIN);
    MMC5983MA_SetDelay(Delay_ms);
    int8_t mmc_ret = MMC5983MA_Init(&mmc);

    BMP580_ForcedMeasure(&bmp580, &baro_data);
    float sea_level_p = BMP580_AltitudeToSeaLevelPressure(
        baro_data.pressure_pa,
        baro_data.temperature_deg,
        50.0f);
    float altitude = BMP580_PressureToAltitudeWithTemp(
        baro_data.pressure_pa,
        baro_data.temperature_deg,
        sea_level_p);
    init_altitude = altitude;  // 以初始压力作为基准高度
    init_temperature = baro_data.temperature_deg;
    BMP580_SetPowerMode(&bmp580, BMP580_MODE_NORMAL);

    // MMC5983MA_SetBandwidth(&mmc, MMC_BW_00); /* 8ms, 最低噪声 0.4mG */
    // MMC5983MA_EnableAutoSR(&mmc, 1);
    // MMC5983MA_EnableContinuousMode(&mmc, MMC_CM_FREQ_100HZ, MMC_PRD_SET_100);

    Delay_ms(50);
    // Init_DMA_For_IMU_SPI2_TIM2(imu_tx_buf, imu_rx_buf);
    UI_Logger_AddLine(&logWindow, "Sensor Init OK");
    uint8_t imu_who_am_i;
    BMI260_WhoAmI(&bmi, &imu_who_am_i);
    uint8_t bmp_who_am_i;
    BMP580_WhoAmI(&bmp580, &bmp_who_am_i);
    uint8_t mag_who_am_i;
    MMC5983MA_WhoAmI(&mmc, &mag_who_am_i);
    char buf[64];
    snprintf(buf, sizeof(buf), "BMI260: %d %d 0x%02X", bmi.sensor_id, imu_ret, imu_who_am_i);
    UI_Logger_AddLine(&logWindow, buf);
    snprintf(buf, sizeof(buf), "BMP580: %d %d 0x%02X", bmp580.sensor_id, bmp580_ret, bmp_who_am_i);
    UI_Logger_AddLine(&logWindow, buf);
    snprintf(buf, sizeof(buf), "MMC5983MA: %d %d 0x%02X", mmc.sensor_id, mmc_ret, mag_who_am_i);
    UI_Logger_AddLine(&logWindow, buf);

    /* Attitude_Init 内部自行从 Flash 加载 accel + mag 校准参数 */
    sm_vec3_t accel_bias = {0}, accel_scale = {1, 1, 1};
    Attitude_Init(accel_bias, accel_scale, init_altitude);

    UI_Logger_AddLine(&logWindow, buf);

    PWM_GPIO_Init();
    PWM_TIM_Init(PWM_PERIOD, PWM_PRESCALER);
    // Init_DMA_For_PWM_TIM3(pwmDutyBuffer);


    Control_Init();
    UI_Logger_AddLine(&logWindow, "Control Init OK");

    Command_SetModeCallback(Control_SetMode);
    Command_SetThrottleCallback(Control_SetThrottle);
    Command_SetHeightCallback(Control_SetHeight);
    Command_MoveCallback(Control_Move);
    Command_SetAttitudeCallback(Control_SetAttitude);
    Command_SetArmCallback(Control_Arm);
    Command_SetDisarmCallback(Control_Disarm);
    Command_SetEStopCallback(Control_EmergencyStop);
    Command_SetTakeoffCallback(Control_Takeoff);
    Command_SetLandCallback(Control_Land);
    Command_SetHoverCallback(Control_Hover);

    Window_To(WINDOW_IMU);

    // TaskHandle_t xExampleTaskHandle = NULL;
    // ( void ) xTaskCreate( exampleTask,
    //                     "example",
    //                     configMINIMAL_STACK_SIZE,
    //                     NULL,
    //                     configMAX_PRIORITIES - 1U,
    //                     &xExampleTaskHandle );


    // /* Start the scheduler. */
    // vTaskStartScheduler();

    for (;;) {
        /* Should not reach here. */

        FPS_StartFrame();
        
        float dt = FPS_GetDeltaTime();
        Attitude_Update(dt);

        Key_Toggle_Handler();
        LED_Toggle_Handler();

        BMI260_ReadAccel(&bmi, &acc);
        BMI260_ReadGyro(&bmi, &gyr);
        memcpy(imu_rx_buf, &acc, 6);
        memcpy(imu_rx_buf + 6, &gyr, 6);
        /* ARM 小端 → imu_rx_buf 需要大端，逐对交换 */
        for (int i = 0; i < 12; i += 2) {
            uint8_t t = imu_rx_buf[i];
            imu_rx_buf[i] = imu_rx_buf[i + 1];
            imu_rx_buf[i + 1] = t;
        }

        BMP580_ReadData(&bmp580, &baro_data);
        altitude_rx = BMP580_PressureToAltitudeWithTemp(
            baro_data.pressure_pa,
            baro_data.temperature_deg,
            sea_level_p);
        temperature_rx = baro_data.temperature_deg;

        MMC5983MA_ReadMagRaw(&mmc, &mag_data);

        {
            int16_t x = (int16_t)(mag_data.x - MMC_NULL_FIELD_16BIT);
            int16_t y = (int16_t)(mag_data.y - MMC_NULL_FIELD_16BIT);
            int16_t z = (int16_t)(mag_data.z - MMC_NULL_FIELD_16BIT);

            mag_rx_buf[0] = (uint8_t)(x & 0xFF);
            mag_rx_buf[1] = (uint8_t)((x >> 8) & 0xFF);
            mag_rx_buf[2] = (uint8_t)(y & 0xFF);
            mag_rx_buf[3] = (uint8_t)((y >> 8) & 0xFF);
            mag_rx_buf[4] = (uint8_t)(z & 0xFF);
            mag_rx_buf[5] = (uint8_t)((z >> 8) & 0xFF);
        }



    
        Battery_Measure_Step();

        if (esRxStatusUart2 == ES_RX_STATE_COMPLETE) {
            uint8_t extra_sensor_data[64] = {0};
            uint16_t extra_sensor_len = ExtraSensor_ReadData(extra_sensor_data);
            if (extra_sensor_len > 0) {
                UI_Logger_AddLine(&logWindow, (char*)extra_sensor_data);
            }
        }

        /* 发送数据包 */
        {
            static uint32_t last_telemetry_time = 0;
            static uint8_t telemetry_tick = 0; // 控制发送包类型频率
            telemetry_tick++;
            
            // 每 10 帧发送一次状态包 (类型 0x01)，根据你的配置通常帧率200-1000？ 假设降到一个合适的频率 1-5Hz
            // 假设主循环约100-200Hz
            if (telemetry_tick % 20 == 0) {
                uint8_t status_buf[32] = {0};
                status_buf[0] = 0xAA;
                status_buf[1] = 0x01; // 包类型
                status_buf[2] = (uint8_t)Control_GetFlightPhase();
                status_buf[3] = (uint8_t)Control_GetMode();
                status_buf[4] = Control_IsArmed();
                status_buf[5] = Battery_GetPercentage();
                status_buf[6] = 0; // GPS 星数
                int16_t rssi = 0;
                memcpy(&status_buf[7], &rssi, 2);
                BLE_WriteData(status_buf, 32);
            }
            
            // 姿态与 PID 包 (类型 0x02)，每帧或每几帧发送一次以同步曲线
            if (telemetry_tick % 5 == 0) { // 稍微降低频率
                uint8_t att_buf[32] = {0};
                att_buf[0] = 0xAA;
                att_buf[1] = 0x02; // 包类型
                
                sm_quat_t quat;
                Attitude_GetQuat(quat);
                memcpy(&att_buf[2], &quat[0], 4);
                memcpy(&att_buf[6], &quat[1], 4);
                memcpy(&att_buf[10], &quat[2], 4);
                memcpy(&att_buf[14], &quat[3], 4);
                
                float rates[3];
                rates[0] = rateSetRoll;
                rates[1] = rateSetPitch;
                rates[2] = rateSetYaw;
                
                memcpy(&att_buf[18], &rates[0], 4);
                memcpy(&att_buf[22], &rates[1], 4);
                memcpy(&att_buf[26], &rates[2], 4);
                
                BLE_WriteData(att_buf, 32);
            }

            // GPS 包 (类型 0x03)，每 10 帧发送一次
            if (telemetry_tick % 10 == 0) {
                uint8_t gps_buf[32] = {0};
                gps_buf[0] = 0xAA;
                gps_buf[1] = 0x03;

                // 纬度 (GPS 暂未接入，填 0)
                double latitude = 0.0;
                memcpy(&gps_buf[2], &latitude, 8);

                // 经度 (GPS 暂未接入，填 0)
                double longitude = 0.0;
                memcpy(&gps_buf[10], &longitude, 8);

                // 高度 (气压计)
                float altitude;
                Attitude_GetAltitude(&altitude);
                memcpy(&gps_buf[18], &altitude, 4);

                // 水平速度 (GPS 暂未接入，填 0)
                float velocity = 0.0f;
                memcpy(&gps_buf[22], &velocity, 4);

                // Padding 已由初始化置零

                BLE_WriteData(gps_buf, 32);
            }
        }

        if (Key_PressConsume()) {
            if (Window_Current() == WINDOW_NONE) {
                Window_To(WINDOW_IMU);
            } else if (Window_Current() == WINDOW_IMU) {
                Window_To(WINDOW_CUBE);
            } else if (Window_Current() == WINDOW_CUBE) {
                Window_To(WINDOW_BATTERY);
            } else if (Window_Current() == WINDOW_BATTERY) {
                Window_To(WINDOW_PID);
            } else if (Window_Current() == WINDOW_PID) {
                Window_To(WINDOW_NONE);
            }
        }

        uint8_t buffer[12] = {0};
        uint16_t len = 0;
        if (bleRxStatusUart1 == BLE_RX_STATE_COMPLETE) {
            len = BLE_ReadData(buffer);
            uint8_t result = Command_ParseAndExecute((char*)buffer, len);
            BLE_WriteData(&result, 1);
            // BLE_WriteData(buffer, len); // Echo back received data

            
        }
        if (len > 0) {
            UI_Logger_AddLine(&logWindow, (char*)buffer);
        }

        ControlAttitude_Loop();
        // ControlMotor_Loop();

        // char pwm_status[64];
        // sprintf(pwm_status, "PWM: %d, %d, %d, %d\r\n", 
        //         TIM3->CCR1, TIM3->CCR2, TIM3->CCR3, TIM3->CCR4);
        // BLE_WriteData(pwm_status, strlen(pwm_status));


        Window_Render();

        FPS_EndFrame();
    }

}


/*-----------------------------------------------------------*/

#if ( configCHECK_FOR_STACK_OVERFLOW > 0 )

    void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                        char * pcTaskName )
    {
        /* Check pcTaskName for the name of the offending task,
         * or pxCurrentTCB if pcTaskName has itself been corrupted. */
        ( void ) xTask;
        ( void ) pcTaskName;

        // UI_Logger_AddLine(&logWindow, "Stack overflow detected!");

        taskDISABLE_INTERRUPTS();
        for( ; ; ) {
            /* Infinite loop to halt the system */
        }
    }

#endif /* #if ( configCHECK_FOR_STACK_OVERFLOW > 0 ) */
/*-----------------------------------------------------------*/
