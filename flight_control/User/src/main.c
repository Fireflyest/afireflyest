#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "control.h"
#include "ble.h"
#include "battery.h"
#include "navigator.h"
#include "flow.h"
#include "gps.h"




static float init_altitude, init_temperature;

/* UART2 光流子板: 二进制帧解析在 flow.c, 这里只保留 10s 心跳统计计时 */
static uint32_t u2StatTick = 0;

/* 磁力计重新校准入口 (0x1C): 飞行中拒绝, 避免中途作废地磁参考 */
static int8_t MagCalib_StartSafe(void) {
    if (Control_IsArmed())
        return -1;
    Attitude_StartMagCalibrate();
    return 0;
}

/**
 * @brief GPS 接收波特率切换 (WINDOW_GPS 内短按)
 * @note  默认值由 GPS_BAUDRATE_DEFAULT 决定 (当前 38400)。
 *        切换后 GPS_SetBaudrate 会清空接收统计与环形缓冲,
 *        所以每次切换等 1~2 秒再读数。诊断见 GPS 窗口 RAW/OK 两行。
 */
static void Gps_BaudNext(void) {
    static const uint32_t table[] = { 38400u, 115200u, 9600u };
    const uint32_t cur = GPS_GetBaudrate();
    uint8_t next = 0u;

    for (uint8_t i = 0u; i < (uint8_t)(sizeof(table) / sizeof(table[0])); i++) {
        if (table[i] == cur) {
            next = (uint8_t)((i + 1u) % (uint8_t)(sizeof(table) / sizeof(table[0])));
            break;
        }
    }
    GPS_SetBaudrate(table[next]);
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
    Flow_Init(); /* 光流子板二进制协议解析器 */
    UART6_GPIO_Init();
    GPS_Init(GPS_BAUDRATE_DEFAULT); /* NMEA GPS/北斗 (USART6, 8N1) */
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

    Window_To(WINDOW_GPS); /* 默认显示 log 页 */

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

        /* 注: 旧的 UART2 自发自收 ASCII 测试 (发送 "V:1 X:.. Y:.." 文本行)
         * 已随按行解析一起删除 —— 二进制协议下文本行只会被当作噪声丢弃。
         * 若需链路自检, 用子板的 50Hz 二进制帧 + 下面对应的 ok/crc/lost 计数。 */

        /* ── UART2 (光流子板) 接收: 字节流交给二进制帧状态机 ──
         * ISR 只负责把字节放进 esRxBuffer (可保留 IDLE 辅助切帧),
         * 这里无条件排空缓冲并逐字节喂给解析器:
         *   - 不依赖 IDLE 成帧, 数据流粘连/分片都能重新对齐 AA 55
         *   - 解析开销 = 36 字节状态机 + CRC, 50Hz 下可忽略, 不阻塞主循环 */
        {
            uint8_t rx[ES_RX_BUFFER_SIZE];
            uint16_t n = ExtraSensor_ReadData(rx);
            if (n > 0) {
                Flow_FeedBytes(rx, n);
            }
        }
        Flow_Update(); /* 200ms 无有效帧 -> link_ok=0 */

        /* ── UART6 GPS: 排空环形缓冲并组句解析 (NMEA, 1Hz) ──
         * 逐字节状态机 + 校验和, 不用 IDLE 成帧; 解析量小不阻塞主循环 */
        GPS_Update();

        /* 每 10s 一行光流链路统计 (很安静):
         *   ok=有效帧 crc=CRC错误 lost=seq 估算丢帧 err=UART 线路错误 */
        if (systemTick - u2StatTick >= 10000u) {
            const flow_data_t *fl = Flow_Get();
            char dbg[TEXTLIST_MAX_CHARS + 1];
            u2StatTick = systemTick;
            snprintf(dbg, sizeof(dbg), "F ok%lu c%lu l%lu e%u",
                     (unsigned long)fl->rx_ok, (unsigned long)fl->rx_crc_err,
                     (unsigned long)fl->rx_lost, (unsigned)esUart2Err);
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
                status_buf[6] = GPS_Get()->sats_used; // GPS 星数
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
                const gps_data_t* gd = GPS_Get();
                gps_buf[0] = 0xAA;
                gps_buf[1] = 0x03;

                // 纬度 (度, 微度换算; 未定位时填 0)
                double latitude = (gd->pos_valid && gd->lat_udeg != 0)
                                      ? ((double)gd->lat_udeg / 1000000.0)
                                      : 0.0;
                memcpy(&gps_buf[2], &latitude, 8);

                // 经度 (度; 未定位时填 0)
                double longitude = (gd->pos_valid && gd->lon_udeg != 0)
                                       ? ((double)gd->lon_udeg / 1000000.0)
                                       : 0.0;
                memcpy(&gps_buf[10], &longitude, 8);

                // 高度 (气压计)
                float altitude;
                Attitude_GetAltitude(&altitude);
                memcpy(&gps_buf[18], &altitude, 4);

                // 水平速度 (GPS 地速 m/s)
                float velocity = gd->has_speed ? gd->speed_mps : 0.0f;
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

        /* ── 按键 ──
         * 普通: 按链表切换窗口
         *       NONE→IMU→MAGDBG→CUBE→BATTERY→PID→FLOW→GPS
         * WINDOW_FLOW: 短按 = 累积位置清零, 长按 800ms = 切到 GPS
         * WINDOW_GPS : 短按 = 切接收波特率 (115200 → 38400 → 9600)
         *              长按 800ms  = 回环自检 (先短接 PA11/PA12)
         *              长按 2500ms = 回到 log 页
         * 记录 keyHoldTick 用于区分长按档位 */
        {
            static uint32_t keyDownTick = 0;
            static uint8_t  keyWasDown  = 0;
            static uint8_t  lvl800Fired = 0;
            static uint8_t  lvl2500Fired = 0;
            uint8_t keyDown = (Key_Status() == KEY_STATE_PRESSED);
            uint8_t win = Window_Current();
            uint32_t held;

            if (keyDown && !keyWasDown) {
                keyDownTick = systemTick;
                lvl800Fired = 0;
                lvl2500Fired = 0;
            }
            held = (uint32_t)(systemTick - keyDownTick);

            if (keyDown) {
                if (win == WINDOW_GPS) {
                    if (!lvl2500Fired && held >= 2500u) {
                        lvl2500Fired = 1;
                        Window_To(WINDOW_NONE);
                    } else if (!lvl800Fired && held >= 800u) {
                        lvl800Fired = 1;
                        GPS_LoopbackTest();   /* 需先短接 PA11 <-> PA12 */
                    }
                } else if (win == WINDOW_FLOW) {
                    if (!lvl800Fired && held >= 800u) {
                        lvl800Fired = 1;
                        Window_To(WINDOW_GPS);
                    }
                }
            }
            keyWasDown = keyDown;
        }

        if (Key_PressConsume()) {
            if (Window_Current() == WINDOW_FLOW) {
                /* 光流窗口内: 短按清零累积位置 (长按已在上面的检测里切窗口) */
                Flow_ResetPosition();
            } else if (Window_Current() == WINDOW_GPS) {
                /* GPS 窗口内: 短按切换接收波特率 (115200 -> 38400 -> 9600) */
                Gps_BaudNext();
            } else if (Window_Current() == WINDOW_NONE) {
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
                Window_To(WINDOW_FLOW);
            } else {
                /* 其它窗口 -> 回到 log 页 */
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
