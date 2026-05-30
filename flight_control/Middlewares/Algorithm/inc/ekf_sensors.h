/**
 * @file ekf_sensors.h
 * @brief EKF 四旋翼无人机 — 传感器输入约定与测量模型
 *
 * ============================================================================
 *  坐标系与旋转矩阵约定
 * ============================================================================
 *
 *  机体系 (Body Frame): FRD  — X 前, Y 右, Z 下
 *  导航系 (Nav Frame):  NED  — X 北, Y 东, Z 下
 *
 *  旋转矩阵 R 的约定:
 *    R = R_earth_to_body, 即 v_body = R * v_earth
 *    若四元数 q 定义为 body→earth 则 R = R_body_to_earth(q)^T
 *    所有观测模型中 R 的含义统一为此。
 *
 * ============================================================================
 *  传感器输入总览
 * ============================================================================
 *
 *  ┌─────────────┬────────────────────────────────────────────────────────────┐
 *  │  传感器      │  输出约定                                                  │
 *  ├─────────────┼────────────────────────────────────────────────────────────┤
 *  │  加速度计    │  比力 (specific force)，机体系 FRD，单位 m/s²              │
 *  │             │  水平静止: [0, 0, -g]    (Z 轴读数为 -9.81)               │
 *  │             │  自由落体: [0, 0, 0]     (失重)                           │
 *  │             │  右倾 90° 静止: [0, -g, 0]                                │
 *  │             │  倒扣静止: [0, 0, +g]                                     │
 *  │             │  机头朝上 90° 静止: [+g, 0, 0]                            │
 *  │             │  机头朝下 90° 静止: [-g, 0, 0]                            │
 *  ├─────────────┼────────────────────────────────────────────────────────────┤
 *  │  陀螺仪      │  机体系 FRD 角速度，单位 rad/s                            │
 *  │             │  ω_x: roll rate  (右翼下沉为正)                           │
 *  │             │  ω_y: pitch rate (机头向上为正)                           │
 *  │             │  ω_z: yaw rate   (俯视顺时针为正)                         │
 *  │             │  静止时理想输出: [0, 0, 0] (实际含 bias drift)             │
 *  ├─────────────┼────────────────────────────────────────────────────────────┤
 *  │  磁力计      │  机体系 FRD 地磁向量，单位 μT                             │
 *  │             │  水平朝北时: [m_N, m_E, m_D] (取决于当地地磁场)            │
 *  │             │  水平朝东时: [m_E, -m_N, m_D]                            │
 *  │             │  初始化时需采集当地地磁参考，补偿磁偏角 (declination)        │
 *  ├─────────────┼────────────────────────────────────────────────────────────┤
 *  │  气压计      │  相对高度 h，单位 m，向上为正                              │
 *  │             │  注意: NED 系中 D 轴向下为正，使用时需取反                   │
 *  ├─────────────┼────────────────────────────────────────────────────────────┤
 *  │  GPS         │  [lat(°), lon(°), alt(m)] WGS-84                        │
 *  │             │  内部转换为 NED 局部平面: N=m, E=m, D=-alt                 │
 *  ├─────────────┼────────────────────────────────────────────────────────────┤
 *  │  光流        │  机体系水平面积分角位移 [integrated_x, integrated_y], rad   │
 *  │             │  integrated_x: 沿 X_b (前方) 积分角位移                    │
 *  │             │  integrated_y: 沿 Y_b (右侧) 积分角位移                    │
 *  │             │  乘以距地高度 h_agl 可转换为线位移                          │
 *  └─────────────┴────────────────────────────────────────────────────────────┘
 *
 * ============================================================================
 *  观测模型说明
 * ============================================================================
 *
 *  (R 为 earth→body 旋转矩阵, g = 9.81 m/s²)
 *
 *  加速度计观测模型 (静止/低速):
 *      a_measured = R * (a_world - g_world) + bias + noise
 *      其中 g_world = [0, 0, +g] (NED 系重力向量)
 *      静止时 a_world = 0, 故 a_measured = R * [0,0,-g] + bias
 *      水平时: a_measured ≈ [0, 0, -g]
 *
 *  磁力计观测模型:
 *      m_measured = R * m_earth + bias + noise
 *      m_earth = 当地 NED 地磁参考向量 (初始化时标定)
 *
 *  气压计观测模型:
 *      h_measured = -p_D + bias + noise
 *      (NED 系 D 轴向下为正，高度向上为正，故取负)
 *
 *  GPS 观测模型:
 *      位置: z = [p_N, p_E, p_D],  h(x) = [x[PN], x[PE], x[PD]]  (全部 NED)
 *      速度: z = [v_N, v_E, v_D],  h(x) = [x[VN], x[VE], x[VD]]
 *      高度: h_GPS = -p_D  (若需向上高度，从 NED D 分量取反)
 *
 * ============================================================================
 *  传感器数据有效性标记
 * ============================================================================
 *
 *  每个传感器数据包包含 timestamp 和 status 标记。
 *  EKF 仅在 status = EKF_SENSOR_VALID 且 timestamp > 上次更新时间时使用该数据。
 *  传感器之间的采样率可以不同 (异步量测更新)。
 */

#ifndef EKF_SENSORS_H
#define EKF_SENSORS_H

#include <stdint.h>
#include "ekf_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*  传感器采样率参考 (Hz)                                                      */
/* ========================================================================== */

/**
 * @name 传感器典型采样率
 * @{
 */
#define SENSOR_RATE_IMU_HZ 400    /**< IMU (陀螺仪+加速度计) 400 Hz  */
#define SENSOR_RATE_MAG_HZ 100    /**< 磁力计 100 Hz                 */
#define SENSOR_RATE_BARO_HZ 50    /**< 气压计 50 Hz                  */
#define SENSOR_RATE_GPS_HZ 10     /**< GPS 10 Hz                     */
#define SENSOR_RATE_OPTFLOW_HZ 50 /**< 光流 50 Hz                    */
/** @} */

/* ========================================================================== */
/*  传感器状态与有效性标记                                                      */
/* ========================================================================== */

/**
 * @brief 传感器数据有效性状态
 */
typedef enum {
    EKF_SENSOR_INVALID = 0, /**< 数据无效，不参与更新    */
    EKF_SENSOR_VALID = 1,   /**< 数据有效，可参与更新    */
    EKF_SENSOR_TIMEOUT = 2, /**< 数据超时，传感器可能掉线 */
} ekf_sensor_status_t;

/**
 * @brief 传感器数据包公共头部
 *
 * 所有传感器数据结构的第一个成员，用于统一时间戳和有效性管理。
 * EKF 主循环通过此头部判断是否触发量测更新。
 */
typedef struct {
    uint64_t timestamp_us;      /**< 采样时间戳 (μs, monotonic)   */
    ekf_sensor_status_t status; /**< 数据有效性标记               */
} ekf_sensor_header_t;

/* ========================================================================== */
/*  IMU 数据                                                                  */
/* ========================================================================== */

/**
 * @brief 陀螺仪原始测量数据
 *
 * 测量机体系 FRD 下的角速度。
 *
 * 角速度定义 (右手定则):
 *   ω_x: Roll rate  — 右翼下沉方向为正
 *   ω_y: Pitch rate — 机头向上方向为正
 *   ω_z: Yaw rate   — 俯视顺时针 (机头右偏) 为正
 *
 * 理想静止输出: [0, 0, 0] rad/s
 * 实际输出模型: ω_measured = ω_true + bias + noise
 *   - bias: 缓慢漂移，EKF 在线估计并补偿
 *   - noise: 高斯白噪声，由 gyro_noise 参数建模
 *
 * 使用注意:
 *   - 送入 EKF 前需减去当前 bias 估计: ω_corrected = ω_measured - b_g
 *   - 采样率通常为 400 Hz (与加速度计同步)
 */
typedef struct {
    ekf_sensor_header_t header; /**< 公共头部: 时间戳 + 有效性   */
    float omega_x;              /**< X 轴角速度 (rad/s), roll rate  */
    float omega_y;              /**< Y 轴角速度 (rad/s), pitch rate */
    float omega_z;              /**< Z 轴角速度 (rad/s), yaw rate   */
} ekf_gyro_t;

/**
 * @brief 加速度计原始测量数据
 *
 * 测量比力 (specific force)，即非引力加速度。
 *
 * 核心公式:
 *   a_measured = a_true_non_grav + gravity_reaction + bias + noise
 *
 * 各状态下的机体系输出 [a_x, a_y, a_z]:
 *
 *   ┌──────────────────────┬──────────────────────────────────────────┐
 *   │  状态                 │  输出 (m/s²)                             │
 *   ├──────────────────────┼──────────────────────────────────────────┤
 *   │  水平静止             │  [0, 0, -g]        Roll=0,   Pitch=0   │
 *   │  右倾 90° 静止        │  [0, -g, 0]        Roll=+90°           │
 *   │  左倾 90° 静止        │  [0, +g, 0]        Roll=-90°           │
 *   │  机头朝天 (竖立)      │  [-g, 0, 0]        Pitch=-90° (抬头)  │
 *   │  机头朝地 (俯冲)      │  [+g, 0, 0]        Pitch=+90° (低头)  │
 *   │  倒扣 (翻转)          │  [0, 0, +g]        Roll=180°           │
 *   │  自由落体             │  [0, 0, 0]         失重                 │
 *   │  北向加速 1m/s²       │  [+1, 0, -g]       水平加速度叠加       │
 *   └──────────────────────┴──────────────────────────────────────────┘
 *
 *   推导: f_body = R(q) · f_world, 其中 f_world = [0, 0, -g] (NED 系比力)
 *
 *   验证 (机头朝上 90°, θ=+90°):
 *     R_y(+90°) = [[0, 0, -1],[0, 1, 0],[1, 0, 0]]
 *     f_body = R_y(+90°) * [0, 0, -g]ᵀ = [+g, 0, 0]  ✓
 *
 *   关键: 静止时加速度计读数 = -重力在机体系的投影。
 *
 * EKF 姿态观测原理:
 *   加速度计在静止/低速时可提供 roll 和 pitch 的观测量，
 *   因为测量值中重力分量的方向反映了机体相对水平面的倾斜。
 *   高速机动时加速度计不可靠，仅用于修正 gyro bias。
 */
typedef struct {
    ekf_sensor_header_t header; /**< 公共头部: 时间戳 + 有效性   */
    float a_x;                  /**< X 轴加速度 (m/s²), 机头前方  */
    float a_y;                  /**< Y 轴加速度 (m/s²), 右侧      */
    float a_z;                  /**< Z 轴加速度 (m/s²), 机腹下方   */
} ekf_accel_t;

/**
 * @brief IMU 组合数据 (陀螺仪 + 加速度计同步输出)
 *
 * 多数 IMU 芯片同时输出角速度和加速度，时间戳相同。
 * 使用此结构可保证两个测量在 EKF predict 步骤中同步使用。
 *
 * 使用说明:
 *   ekf_predict() 读取 header.timestamp_us 判断时间步长。
 *   gyro 和 accel 内部的 header 字段为预留，当前不参与逻辑，
 *   调用方可选择不填充以简化赋值。
 */
typedef struct {
    ekf_sensor_header_t header; /**< 公共头部 (陀螺仪+加速度计共用) */
    ekf_gyro_t gyro;            /**< 陀螺仪数据                  */
    ekf_accel_t accel;          /**< 加速度计数据                 */
} ekf_imu_t;

/* ========================================================================== */
/*  磁力计数据                                                                */
/* ========================================================================== */

/**
 * @brief 磁力计原始测量数据
 *
 * 测量机体系 FRD 下的地磁向量。
 *
 * 各朝向下机体系输出 (m_body = R · m_earth):
 *
 *   ┌──────────────────────┬───────────────────────────────┐
 *   │  状态                 │  输出 [m_x, m_y, m_z] (μT)    │
 *   ├──────────────────────┼───────────────────────────────┤
 *   │  水平朝北             │  [ m_N,  m_E,  m_D]           │
 *   │  水平朝东             │  [ m_E, -m_N,  m_D]           │
 *   │  水平朝南             │  [-m_N, -m_E,  m_D]           │
 *   │  水平朝西             │  [-m_E,  m_N,  m_D]           │
 *   │  右倾 90°             │  [ m_D,  m_E, -m_N]           │
 *   │  机头朝天 (竖立)      │  [ m_D, -m_E,  m_N]           │
 *   └──────────────────────┴───────────────────────────────┘
 *
 *   其中 [m_N, m_E, m_D] 为当地 NED 系下的地磁参考分量。
 *   注意: m_D (垂直分量) 在中国地区约 25~45 μT，不可忽略。
 *
 * 初始化要求:
 *   1. 在已知方位 (如水平朝北) 下采集磁力计数据
 *   2. 标定硬铁/软铁误差
 *   3. 计算当地磁偏角 (declination): ψ_mag = atan2(m_E, m_N)
 *   4. 存储参考向量 m_earth = [m_N, m_E, m_D] 供 EKF 观测模型使用
 *
 * 观测模型:
 *   m_measured = R(q) * m_earth + bias + noise
 *   其中 R(q) 为当前姿态对应的旋转矩阵 (世界系 → 机体系)
 */
typedef struct {
    ekf_sensor_header_t header; /**< 公共头部: 时间戳 + 有效性   */
    float m_x;                  /**< X 轴磁场 (μT), 机头前方     */
    float m_y;                  /**< Y 轴磁场 (μT), 右侧         */
    float m_z;                  /**< Z 轴磁场 (μT), 机腹下方     */
} ekf_mag_t;

/* ========================================================================== */
/*  辅助传感器数据                                                             */
/* ========================================================================== */

/**
 * @brief 气压计测量数据
 *
 * 输出相对高度，向上为正 (注意与 NED 的 D 轴方向相反)。
 *
 * 观测模型:
 *   h_measured = -p_D + baro_bias + noise
 *
 * 使用注意:
 *   - 气压计存在缓慢漂移 (温度、天气)，bias 在 EKF 中在线估计
 *   - 初始参考高度在初始化时记录，后续输出为相对值
 *   - 受螺旋桨气流影响，建议滤波后再送入 EKF
 */
typedef struct {
    ekf_sensor_header_t header; /**< 公共头部: 时间戳 + 有效性    */
    float altitude;             /**< 相对高度 (m)，向上为正       */
    float temperature;          /**< 温度 (°C)，用于气压补偿 (可选) */
} ekf_baro_t;

/**
 * @brief GPS 测量数据
 *
 * 原始输出: WGS-84 大地坐标 [latitude, longitude, altitude]
 * EKF 使用: 转换为 NED 局部平面坐标
 *
 * 坐标转换:
 *   N (m) = (lat - lat_ref) * π/180 * R_earth        (北向)
 *   E (m) = (lon - lon_ref) * π/180 * R_earth * cos(lat_ref)  (东向)
 *   D (m) = -(alt - alt_ref)                         (地向，高度取负)
 *
 * GPS 数据标记:
 *   fix_type: 0=无定位, 2=2D, 3=3D, 4=DGPS, 5=RTK Float, 6=RTK Fix
 *   num_sats: 可见卫星数，建议 ≥ 6 时使用
 *   hdop:     水平精度因子，越小越好
 *
 * 采样率: 通常 1-10 Hz
 *
 * 注意: latitude 和 longitude 使用 double 类型以保证经纬度精度。
 *       在 STM32 上 double 为 64 位，要求 8 字节对齐。
 *       请勿对本结构体使用 #pragma pack(1)。
 */
typedef struct {
    ekf_sensor_header_t header; /**< 公共头部: 时间戳 + 有效性    */
    double latitude;            /**< 纬度 (°, WGS-84)             */
    double longitude;           /**< 经度 (°, WGS-84)             */
    float altitude_msl;         /**< 海拔高度 (m, WGS-84)         */
    float vel_north;            /**< 北向地面速度 (m/s)           */
    float vel_east;             /**< 东向地面速度 (m/s)           */
    float vel_down;             /**< 垂直速度 (m/s, 向下为正)     */
    float horiz_acc;            /**< 水平位置精度 (m, 1σ)         */
    float vert_acc;             /**< 垂直位置精度 (m, 1σ)         */
    float vel_acc;              /**< 速度精度 (m/s, 1σ)           */
    float hdop;                 /**< 水平精度因子                  */
    uint8_t fix_type;           /**< 定位类型 (0-6)               */
    uint8_t num_sats;           /**< 可见卫星数                    */
} ekf_gps_t;

/**
 * @brief 光流测量数据
 *
 * 测量机体系水平面上的积分角位移 (非速度)。
 *
 * 输出约定:
 *   velocity_x: 沿 X_b (机头前方) 的地面投影速度 (m/s)，前方运动为正
 *   velocity_y: 沿 Y_b (机翼右侧) 的地面投影速度 (m/s)，右方运动为正
 *
 * 计算方法:
 *   光流原始输出为角速度 ω (rad/s)，调用方需自行转换为线速度:
 *   velocity = ω_optflow × h_agl (距地高度, above ground level)
 *   然后填入 velocity_x / velocity_y。
 *
 * 局限:
 *   - 仅在纹理丰富的地面有效
 *   - 高速旋转时受旋转分量污染，需陀螺仪补偿 (integrated_xgyro/ygyro)
 *   - 高度过大时信噪比下降
 *   - 采样率: 通常 20-100 Hz
 */
typedef struct {
    ekf_sensor_header_t header; /**< 公共头部: 时间戳 + 有效性    */
    float velocity_x;           /**< X 方向地面投影速度 (m/s)     */
    float velocity_y;           /**< Y 方向地面投影速度 (m/s)     */
    float integrated_xgyro;     /**< 积分期间陀螺仪 X 角位移 (rad) */
    float integrated_ygyro;     /**< 积分期间陀螺仪 Y 角位移 (rad) */
    float distance_m;           /**< 距地高度 (m), 无效时为 -1    */
    uint8_t quality;            /**< 数据质量 (0-255, 越高越好)   */
} ekf_optflow_t;

/* ========================================================================== */
/*  传感器噪声参数                                                             */
/* ========================================================================== */

/**
 * @brief EKF 传感器噪声参数
 *
 * 噪声参数的离散化规则 (EKF 内部根据 Δt 计算):
 *
 *   随机游走驱动噪声 — 参数单位 [unit]²/s:
 *     Q_discrete = Q_continuous × Δt
 *     例: gyro_bias_noise (rad/s/√s) → Q_d = σ² × Δt
 *
 *   白噪声等效离散方差 — 参数单位 [unit]²/Hz:
 *     Q_discrete = Q_continuous / Δt = Q_continuous × f_s
 *     例: gyro_noise (rad/s/√Hz) → Q_d = σ² / Δt
 *
 * 典型值参考 (需根据实际传感器手册和 Allan 方差测试调整):
 *
 *   参数                典型值            单位
 *   ─────────────────────────────────────────────
 *   gyro_noise          1e-3 ~ 1e-2     rad/s/√Hz
 *   gyro_bias_noise     1e-5 ~ 1e-4     rad/s/√s   (random walk)
 *   accel_noise         1e-2 ~ 1e-1     m/s²/√Hz
 *   accel_bias_noise    1e-3 ~ 1e-2     m/s²/√s    (random walk)
 *   mag_noise           0.1 ~ 1.0       μT/√Hz
 *   baro_noise          0.1 ~ 1.0       m/√Hz
 *   baro_bias_noise     1e-2 ~ 1e-1     m/√s       (random walk)
 *   gps_pos_noise       0.5 ~ 5.0       m (1σ)
 *   gps_vel_noise       0.1 ~ 1.0       m/s (1σ)
 *   optflow_noise       0.01 ~ 0.1      (m/s)/√Hz
 */
typedef struct {
    /* 陀螺仪噪声 */
    float gyro_noise;      /**< 陀螺仪白噪声密度 (rad/s/√Hz)      */
    float gyro_bias_noise; /**< 陀螺仪 bias 随机游走 (rad/s/√s)   */

    /* 加速度计噪声 */
    float accel_noise;      /**< 加速度计白噪声密度 (m/s²/√Hz)     */
    float accel_bias_noise; /**< 加速度计 bias 随机游走 (m/s²/√s)  */

    /* 磁力计噪声 */
    float mag_noise; /**< 磁力计白噪声密度 (μT/√Hz)         */

    /* 气压计噪声 */
    float baro_noise;      /**< 气压计白噪声密度 (m/√Hz)          */
    float baro_bias_noise; /**< 气压计 bias 随机游走 (m/√s)       */

    /* GPS 噪声 */
    float gps_pos_noise; /**< GPS 位置测量噪声 (m, 1σ)          */
    float gps_vel_noise; /**< GPS 速度测量噪声 (m/s, 1σ)        */

    /* 光流噪声 */
    float optflow_noise; /**< 光流速度白噪声密度 (m/s/√Hz)      */
} ekf_noise_params_t;

/**
 * @brief 初始化默认噪声参数
 *
 * 提供一组典型的小型四旋翼无人机传感器噪声参数。
 * 实际使用时应根据传感器手册和 Allan 方差测试结果校准。
 *
 * @param[out] params 输出默认噪声参数
 */
void ekf_noise_params_init_default(ekf_noise_params_t* params);

/* ========================================================================== */
/*  GPS 坐标转换辅助                                                          */
/* ========================================================================== */

/**
 * @brief GPS 原点参考 (用于 WGS-84 → NED 转换)
 *
 * 第一次收到有效 GPS 数据时，将当前位置设为原点 (lat0, lon0, alt0)。
 * 后续 GPS 数据均相对于此原点转换为 NED 局部坐标。
 *
 * 注意: lat0 和 lon0 使用 double 类型。
 *       请勿对本结构体使用 #pragma pack(1) (8 字节对齐要求)。
 */
typedef struct {
    double lat0;         /**< 原点纬度 (°)    */
    double lon0;         /**< 原点经度 (°)    */
    float alt0;          /**< 原点海拔 (m)    */
    uint8_t initialized; /**< 是否已初始化原点 */
} ekf_gps_origin_t;

/**
 * @brief WGS-84 GPS 坐标转换为 NED 局部平面坐标
 *
 * @param[in]  origin GPS 原点参考
 * @param[in]  lat    当前纬度 (°)
 * @param[in]  lon    当前经度 (°)
 * @param[in]  alt    当前海拔 (m)
 * @param[out] ned    输出 NED 位置 (m): [N, E, D]
 *
 * 转换公式 (小角度近似):
 *   N = (lat - lat0) * π/180 * R_earth
 *   E = (lon - lon0) * π/180 * R_earth * cos(lat0 * π/180)
 *   D = -(alt - alt0)
 *
 * 其中 R_earth ≈ 6371000 m
 * 精度: 距原点 10 km 内误差 < 1 m
 */
void ekf_gps_to_ned(const ekf_gps_origin_t* origin,
                    double lat,
                    double lon,
                    float alt,
                    ekf_vec3_t* ned);

/**
 * @brief 初始化 GPS 原点
 *
 * 第一次调用时记录当前位置为原点，后续调用无效。
 *
 * @param[out] origin GPS 原点结构体
 * @param[in]  lat    原点纬度 (°)
 * @param[in]  lon    原点经度 (°)
 * @param[in]  alt    原点海拔 (m)
 */
void ekf_gps_origin_init(ekf_gps_origin_t* origin,
                         double lat,
                         double lon,
                         float alt);

/* ========================================================================== */
/*  观测向量索引 (量测更新用)                                                   */
/* ========================================================================== */

/**
 * @brief 观测向量索引 — 加速度计观测
 *
 * 加速度计提供 roll/pitch 信息，观测向量维度 = 3
 * z = [a_x, a_y, a_z], h(x) = R * [0, 0, -g] (静止近似, R 为 earth→body)
 */
typedef enum {
    EKF_OBS_ACCEL_X = 0,
    EKF_OBS_ACCEL_Y = 1,
    EKF_OBS_ACCEL_Z = 2,
    EKF_OBS_ACCEL_DIM = 3,
} ekf_obs_accel_index_t;

/**
 * @brief 观测向量索引 — 磁力计观测
 *
 * 磁力计提供 yaw 信息，观测向量维度 = 3 (或 2，忽略 Z 轴)
 * z = [m_x, m_y, m_z], h(x) = R * m_earth  (R 为 earth→body)
 */
typedef enum {
    EKF_OBS_MAG_X = 0,
    EKF_OBS_MAG_Y = 1,
    EKF_OBS_MAG_Z = 2,
    EKF_OBS_MAG_DIM = 3,
} ekf_obs_mag_index_t;

/**
 * @brief 观测向量索引 — GPS 位置观测
 *
 * z = [p_N, p_E, p_D], h(x) = [x[PN], x[PE], x[PD]]
 */
typedef enum {
    EKF_OBS_GPS_N = 0,
    EKF_OBS_GPS_E = 1,
    EKF_OBS_GPS_D = 2,
    EKF_OBS_GPS_DIM = 3,
} ekf_obs_gps_pos_index_t;

/**
 * @brief 观测向量索引 — GPS 速度观测
 *
 * z = [v_N, v_E, v_D], h(x) = [x[VN], x[VE], x[VD]]
 */
typedef enum {
    EKF_OBS_GPS_VN = 0,
    EKF_OBS_GPS_VE = 1,
    EKF_OBS_GPS_VD = 2,
    EKF_OBS_GPS_VEL_DIM = 3,
} ekf_obs_gps_vel_index_t;

/**
 * @brief 观测向量索引 — 气压计高度观测
 *
 * z = [h], h(x) = -x[PD]  (NED D轴取反得到向上高度)
 * 观测维度 = 1
 */
typedef enum {
    EKF_OBS_BARO_H = 0,
    EKF_OBS_BARO_DIM = 1,
} ekf_obs_baro_index_t;

/**
 * @brief 观测向量索引 — 光流观测
 *
 * z = [v_x_body, v_y_body]
 * h(x) = (R * [v_N, v_E, v_D]) 的 X,Y 分量  (R 为 earth→body)
 * 观测维度 = 2
 */
typedef enum {
    EKF_OBS_OPTFLOW_VX = 0,
    EKF_OBS_OPTFLOW_VY = 1,
    EKF_OBS_OPTFLOW_DIM = 2,
} ekf_obs_optflow_index_t;

/* ========================================================================== */
/*  地磁参考存储                                                               */
/* ========================================================================== */

/**
 * @brief 当地地磁参考向量 (NED 系)
 *
 * 初始化时由磁力计标定得到，用于 EKF 磁力计观测模型:
 *   h(x) = R * m_earth_ref  (R 为 earth→body)
 *
 * 同时存储磁偏角供航向校正使用:
 *   真北航向 = 磁航向 + declination
 */
typedef struct {
    ekf_vec3_t m_earth; /**< NED 系地磁参考向量 (μT)     */
    float declination;  /**< 磁偏角 (rad)，东偏为正       */
    float inclination;  /**< 磁倾角 (rad)，向下为正       */
    float total_field;  /**< 当地地磁总强度 (μT)          */
    uint8_t calibrated; /**< 是否已完成标定               */
} ekf_mag_reference_t;

#ifdef __cplusplus
}
#endif

#endif /* EKF_SENSORS_H */
