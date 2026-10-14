#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "control.h"
#include "ble.h"
#include "battery.h"
#include "navigator.h"




static float init_altitude, init_temperature;

/* UART2 (子板 光流+激光) 接收调试计数 */
static uint32_t u2Frames = 0;
static uint32_t u2Bytes = 0;
static uint32_t u2Bursts = 0; /* IDLE 成帧次数 (看时序: b/n = 每突发字节数) */
static uint32_t u2LastSum = 0;
static uint16_t u2LastLen = 0; /* 最近一行的长度 */
static char     u2Line[80];    /* 行重组缓冲 (按 '\n' 切行, 不依赖 IDLE 成帧) */
static uint16_t u2LineLen = 0;

/**
 * @brief UART2 收到完整一行时的处理 (解析 + 限速显示到 OLED log 页)
 * @note  显示限速 500ms, 否则子板 20Hz 会把屏幕刷成一团
 */
static void Uart2_HandleLine(const char* line) {
    char dbg[TEXTLIST_MAX_CHARS + 1];

    /* 只显示子板的 [..] 消息 (初始化/错误/状态):
     * OLED 只有几行, 数据行 (20Hz) 不能上屏 — 只计数, 看 10s 心跳行即可 */
    if (line[0] == '[') {
        snprintf(dbg, sizeof(dbg), "%s", line);
        UI_Logger_AddLine(&logWindow, dbg);
    }
}

/* 磁力计重新校准入口 (0x1C): 飞行中拒绝, 避免中途作废地磁参考 */
static int8_t MagCalib_StartSafe(void) {
    if (Control_IsArmed())
        return -1;
    Attitude_StartMagCalibrate();
    return 0;
}

/** @brief 二进制遥测包 (0xAA) 开关: 保持原有数据包不变;
 *         仅磁力计调试诊断用文本行单独发送 (SendDiagText) */
#define SEND_BINARY_TELEMETRY 1

/**
 * @brief 文本诊断行 (BLE 纯 ASCII, 2Hz, ≤64B 受 BLE_TX_BUFFER_SIZE 限制)
 *
 *  C1   = 地磁参考已标定 (磁力计在修正 yaw)
 *  X/Y/Z = 磁场新息 EMA (µT, τ=0.3s, 截±99) — 静止应稳定≈0,
 *          旋转测试读稳定值 (瞬时值噪声大, 不用瞬时)
 *  Q     = 最近 1s 新息峰峰值最大分量 (µT, 噪声/稳定性指标)
 *  R     = 门控拒绝计数 (k=千)
 *  N/E   = 实测/参考磁场模长 (µT, 均应 25~65 且接近)
 *  W     = yaw (°)
 * 不用 %f: newlib-nano 未必链接 float printf, 手工拆整数小数
 */
static void SendDiagText(void) {
    uint32_t ok, rej;
    Attitude_MagStats(&ok, &rej);
    float iv[3] = {0.0f, 0.0f, 0.0f};
    Attitude_MagInnovEma(iv);
    float pp = Attitude_MagInnovPp();
    float n = Attitude_MagFieldNorm();
    float e = Attitude_MagRefNorm();
    uint8_t c = Attitude_MagRefCalibrated();
    /* 连续化 yaw: ±180 wrap 不再跳, 还跳就是真实跳变 */
    float yaw = Attitude_YawUnwrapped();
    /* M = 校准: 采集中=进度%, 有效=100, 失败=-1, 无参数=0 */
    uint8_t mst = Attitude_MagCalibStatus();
    int mv;
    if (mst == 1) {
        mv = (int)(Attitude_MagCalibrateProgress() * 100.0f + 0.5f);
        if (mv > 100) mv = 100;
    } else if (Attitude_MagCalibValid()) {
        mv = 100;
    } else {
        mv = (mst == 3) ? -1 : 0;
    }
    /* A = 对准状态 (0对准中/1已对准), m = 磁场好帧 (模长20~100才计, >10才建参考) */
    uint8_t astate;
    int acnt, amcnt;
    Attitude_AlignStats(&astate, &acnt, &amcnt);
    (void)acnt; /* 行长受限不发送, 屏幕 MAGDBG 画面有 */

    int xi = (int)iv[0], yi = (int)iv[1], zi = (int)iv[2];
    if (xi > 99) xi = 99;  if (xi < -99) xi = -99;
    if (yi > 99) yi = 99;  if (yi < -99) yi = -99;
    if (zi > 99) zi = 99;  if (zi < -99) zi = -99;

    char rbuf[6];
    if (rej >= 1000u) {
        unsigned rk = (unsigned)(rej / 1000u);
        if (rk > 999u) rk = 999u;
        snprintf(rbuf, sizeof(rbuf), "%uk", rk);
    } else {
        snprintf(rbuf, sizeof(rbuf), "%u", (unsigned)rej);
    }

    int ni = (int)(n + 0.5f);
    if (ni < 0) ni = 0;  if (ni > 999) ni = 999;
    int ei = (int)(e + 0.5f);
    if (ei < 0) ei = 0;  if (ei > 999) ei = 999;
    int qp = (int)(pp + 0.5f);
    if (qp < 0) qp = 0;  if (qp > 999) qp = 999;

    float ya = yaw < 0.0f ? -yaw : yaw;
    char ysign = (yaw < 0.0f) ? '-' : '+';
    int ydeg = (int)ya;
    int yfrac = (int)((ya - (float)ydeg) * 10.0f + 0.5f);
    if (yfrac >= 10) { ydeg++; yfrac = 0; }

    char line[64];
    int len = snprintf(line, sizeof(line),
                       "C%c A%d M%3d m%3d X%+3d Y%+3d Z%+3d Q%d R%s N%d E%d W%c%d.%d\r\n",
                       c ? '1' : '0', astate, mv, amcnt,
                       xi, yi, zi, qp, rbuf, ni, ei,
                       ysign, ydeg, yfrac);
    if (len > 0) {
        if (len >= (int)sizeof(line))
            len = (int)sizeof(line) - 1;
        BLE_WriteData((uint8_t*)line, (uint16_t)len);
    }
}


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

    /* ★ 气压计必须显式配置: BMP580_Init 只校验 chip ID 与状态位,
     * 不写任何测量寄存器。press_en (OSR_CONFIG bit6) 复位值为 0
     * → 压力测量被禁用 → 压力寄存器恒定 → 高度读数不动。
     * (OSR 是噪声/耗时的权衡, ODR 是 NORMAL 模式的连续测量速率) */
    BMP580_ConfigMeasurement(&bmp580,
                             BMP580_OSR_4X,   /* 压力过采样 (降噪声) */
                             BMP580_OSR_2X,   /* 温度过采样 */
                             BMP580_ODR_62HZ, /* 连续测量速率 */
                             1);              /* press_en = 1 使能压力 */

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
    Command_SetRollOLCallback(Control_RollOpenLoopTest);
    Command_SetMagCalibCallback(MagCalib_StartSafe);

    Window_To(WINDOW_NONE); /* 默认显示 log 页 (UART2 接收调试用) */

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

        Key_Toggle_Handler();
        LED_Toggle_Handler();

        /* 先读传感器，再更新姿态：
         * 若先 Attitude_Update 则 EKF/速率环始终用上一帧旧数据，
         * 200Hz 主循环 + 500Hz 速率环下会多约一帧 (5ms) 滞后，
         * 相位裕度不足 → 回正过程抖振、稳态抖 */
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

        /*
         * 磁力计非阻塞轮询: 原 MMC5983MA_ReadMagRaw 每帧阻塞 ~11ms
         * (SET 1ms + bw=00 测量 10ms 的 delay_ms 轮询), 主循环被拖到
         * ~90Hz; ATTITUDE_USE_MAG=1 后数据还要喂 EKF (yaw 修正),
         * 仍用非阻塞状态机分摊, 有新数据才更新 mag_rx_buf,
         * 否则保留上帧值 (~40Hz 出数)
         */
        if (MMC5983MA_ReadMagRawPoll(&mmc, &mag_data)) {
            int32_t x_centered = (int32_t)mag_data.x - (int32_t)MMC_NULL_FIELD_16BIT;
            int32_t y_centered = (int32_t)mag_data.y - (int32_t)MMC_NULL_FIELD_16BIT;
            int32_t z_centered = (int32_t)mag_data.z - (int32_t)MMC_NULL_FIELD_16BIT;

            /* 限幅，防止异常值打包溢出 (16-bit 模式范围本身就在 int16 内，正常不会越界) */
            int16_t x = (int16_t)x_centered;
            int16_t y = (int16_t)y_centered;
            int16_t z = (int16_t)z_centered;

            mag_rx_buf[0] = (uint8_t)(x & 0xFF);
            mag_rx_buf[1] = (uint8_t)((x >> 8) & 0xFF);
            mag_rx_buf[2] = (uint8_t)(y & 0xFF);
            mag_rx_buf[3] = (uint8_t)((y >> 8) & 0xFF);
            mag_rx_buf[4] = (uint8_t)(z & 0xFF);
            mag_rx_buf[5] = (uint8_t)((z >> 8) & 0xFF);
        }

        Attitude_Update(dt);



    
        Battery_Measure_Step();

        /* ── UART2 自发自收测试 (调试用, 验证完改 0) ──
         * 每秒从 PA2 发一行模拟子板格式的文本:
         *   ① 示波器/USB-TTL 接主控 PA2 应看到波形;
         *   ② PA2-PA3 短接时, log 页应出现 #n V1 Q80 D123
         *      (验证 发送→接收→解析→显示 全链路) */
#define UART2_TX_TEST 0
#if UART2_TX_TEST
        {
            static uint32_t u2TxLast = 0;
            static uint32_t u2TxCnt = 0;
            if (systemTick - u2TxLast >= 1000u) {
                u2TxLast = systemTick;
                u2TxCnt++;
                char t[48];
                int n = snprintf(t, sizeof(t),
                                 "V:1 X:%d Y:%d Q:80 dX:0 dY:0 D:123 S:0\r\n",
                                 (int)(u2TxCnt % 1000u), (int)(u2TxCnt % 100u));
                if (n > 0)
                    ExtraSensor_WriteData((uint8_t*)t, (uint16_t)n);
            }
        }
#endif

        /* ── UART2 (子板) 接收: 按行重组 ──
         * IDLE 成帧在这条链路上不可靠: 发送方字节之间有停顿就会“每1字节一帧”
         * (实测 b/f ≈ 1), 导致解析永远失败。改成按 '\n'/\r' 组行, 与成帧无关。 */
        if (esRxStatusUart2 == ES_RX_STATE_COMPLETE) {
            uint8_t extra_sensor_data[64] = {0};
            uint16_t extra_sensor_len = ExtraSensor_ReadData(extra_sensor_data);
            u2Bursts++; /* 一次 IDLE 成帧 = 一个突发 */

            for (uint16_t i = 0; i < extra_sensor_len; i++) {
                uint8_t ch = extra_sensor_data[i];
                u2Bytes++;
                if (ch == '\n' || ch == '\r') {
                    if (u2LineLen > 0) {
                        u2Line[u2LineLen] = '\0';
                        u2LastLen = u2LineLen;
                        u2LineLen = 0;
                        u2Frames++;
                        Uart2_HandleLine(u2Line); /* 解析 + 限速显示 */
                    }
                } else if (u2LineLen < sizeof(u2Line) - 1) {
                    u2Line[u2LineLen++] = (char)ch;
                } else {
                    u2LineLen = 0; /* 超长: 丢弃重来 */
                }

            }

        }

        /* 每 10s 一行心跳统计 (很安静):
         *   n=成帧次数 b=字节数 L=最近行长度 e=线路错误数 */
        if (systemTick - u2LastSum >= 10000u) {
            u2LastSum = systemTick;
            char dbg[TEXTLIST_MAX_CHARS + 1];
            snprintf(dbg, sizeof(dbg), "n%lu b%lu L%u e%u",
                     (unsigned long)u2Bursts, (unsigned long)u2Bytes,
                     (unsigned)u2LastLen, (unsigned)esUart2Err);
            UI_Logger_AddLine(&logWindow, dbg);
        }

        /* 发送数据包: 原有二进制遥测 (0xAA) 保持不变,
         * 另加磁力计调试诊断文本行 (500ms 一次, 纯 ASCII) */
        {
#if SEND_BINARY_TELEMETRY
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

            /* 速率环诊断包 (类型 0x04), 每 2 帧 (100Hz):
             * 实测陀螺速率 (deg/s, 与 0x02 的 rateSet 同量纲) + 速率环输出。
             *   rateSet(0x02) vs gx(0x04) → 内环跟踪相位;
             *   rollCmd(0x04) 阶跃 vs gx  → 执行机构延迟/上升时间 */
            if (telemetry_tick % 2 == 0) {
                uint8_t rate_buf[32] = {0};
                rate_buf[0] = 0xAA;
                rate_buf[1] = 0x04;

                sm_vec3_t g;
                Attitude_GetGyro(g);
                float gdeg[3];
                gdeg[0] = g[0] * 57.29577951f;
                gdeg[1] = g[1] * 57.29577951f;
                gdeg[2] = g[2] * 57.29577951f;
                float rollCmd = rateCmdRoll;
                memcpy(&rate_buf[2],  &gdeg[0], 4);
                memcpy(&rate_buf[6],  &gdeg[1], 4);
                memcpy(&rate_buf[10], &gdeg[2], 4);
                memcpy(&rate_buf[14], &rollCmd, 4);

                BLE_WriteData(rate_buf, 32);
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
#endif /* SEND_BINARY_TELEMETRY */

            /* 文本诊断行 (500ms 一次), 格式见 SendDiagText */
            static uint32_t last_diag_time = 0;
            if (systemTick - last_diag_time >= 500u) {
                last_diag_time = systemTick;
                SendDiagText();
            }
        }

        if (Key_PressConsume()) {
            if (Window_Current() == WINDOW_NONE) {
                Window_To(WINDOW_IMU);
            } else if (Window_Current() == WINDOW_IMU) {
                Window_To(WINDOW_MAGDBG);
            } else if (Window_Current() == WINDOW_MAGDBG) {
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
        ControlMotor_Loop(); /* 与外环同线程顺序执行, 避免 TIM4 并发撕裂反馈 */

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
