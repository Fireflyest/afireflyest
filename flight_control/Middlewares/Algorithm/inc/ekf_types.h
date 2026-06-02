/**
 * @file ekf_types.h
 * @brief EKF 四旋翼无人机 — 核心类型定义与协议约定
 *
 * ============================================================================
 *  坐标系约定
 * ============================================================================
 *
 *  世界坐标系: NED (North-East-Down)
 *      X_N -> 北 (North)
 *      Y_E -> 东 (East)
 *      Z_D -> 地 (Down)，与重力方向一致
 *
 *  机体坐标系: FRD (Front-Right-Down)
 *      X_b -> 机头前方
 *      Y_b -> 机翼右侧
 *      Z_b -> 机腹下方（与重力同向）
 *
 *  初始对齐: 水平放置、机头朝北时，机体系与 NED 世界系完全重合，
 *            姿态为单位旋转，即 q = [1, 0, 0, 0]
 *
 * ============================================================================
 *  四元数约定
 * ============================================================================
 *
 *  采用 Hamilton 约定，标量在前:
 *      q = [q_w, q_x, q_y, q_z] = [cos(θ/2), u*sin(θ/2)]
 *      旋转语义: 将世界系向量旋转到机体系
 *          v_body = q ⊗ v_world ⊗ q*
 *      单位四元数 (无旋转): [1, 0, 0, 0]
 *      共轭: q* = [q_w, -q_x, -q_y, -q_z]
 *
 * ============================================================================
 *  欧拉角约定
 * ============================================================================
 *
 *  旋转顺序: ZYX (Yaw -> Pitch -> Roll)
 *      R = Rz(ψ) · Ry(θ) · Rx(φ)
 *
 *      Yaw   (ψ): 绕 Z 轴，机头向右偏为正 (俯视顺时针)
 *      Pitch (θ): 绕 Y 轴，机头向上为正 (抬头为正)
 *      Roll  (φ): 绕 X 轴，右翼下沉为正 (右倾为正)
 *
 *  水平朝北时: roll = 0, pitch = 0, yaw = 0, q = [1, 0, 0, 0]
 *
 * ============================================================================
 *  单位约定
 * ============================================================================
 *
 *      位置         m
 *      速度         m/s
 *      加速度       m/s²
 *      角速度       rad/s
 *      角度 (欧拉角) rad
 *      磁场         μT
 *      时间步长 Δt   s
 *      重力常数 g    9.80665 m/s²
 */

#ifndef EKF_TYPES_H
#define EKF_TYPES_H

#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*  常量定义                                                                   */
/* ========================================================================== */

/**
 * @brief 标准重力加速度 (m/s²)
 *
 * NED 系下静止时，加速度计在机体系 Z 轴读数为 -EKF_GRAVITY。
 * 即自由落体时加速度计读数为 [0, 0, 0]，静止平放时为 [0, 0, -g]。
 */
#define EKF_GRAVITY         9.80665f

/**
 * @brief 圆周率常量
 */
#ifndef M_PI_F
#define M_PI_F              3.14159265f
#endif

/**
 * @brief 角度 ↔ 弧度转换
 */
#define EKF_DEG2RAD(deg)    ((deg) * (M_PI_F / 180.0f))
#define EKF_RAD2DEG(rad)    ((rad) * (180.0f / M_PI_F))

/**
 * @brief EKF 状态向量维度
 *
 * 标称状态 x 包含:
 *   [0-2]   位置 p   (NED, m)
 *   [3-5]   速度 v   (NED, m/s)
 *   [6-9]   姿态 q   (Hamilton quaternion, [w,x,y,z])
 *   [10-12] 陀螺仪 bias b_g (rad/s)
 *   [13-15] 加速度计 bias b_a (m/s²) — 可选，去掉则状态为13维
 */
#define EKF_STATE_DIM       16

/**
 * @brief 误差状态 EKF 中姿态修正向量维度
 *
 * 误差状态使用 3 维旋转向量 δθ 修正四元数，
 * 协方差矩阵实际维数 = EKF_STATE_DIM - 1 = 15
 */
#define EKF_ERROR_STATE_DIM 15

/* ========================================================================== */
/*  状态向量索引                                                               */
/* ========================================================================== */

/**
 * @brief 状态向量 x 中各分量的起始索引
 *
 * 使用示例:
 *   float pos_north = ekf->state.pos.x;
 *   float gyro_bias_x = ekf->state.gyro_bias.x;
 */
typedef enum {
    /* 位置 — NED 世界系 (m) */
    EKF_STATE_PN = 0,      /**< 位置: 北向 (m)        */
    EKF_STATE_PE = 1,      /**< 位置: 东向 (m)        */
    EKF_STATE_PD = 2,      /**< 位置: 地向 (m, 向下正) */

    /* 速度 — NED 世界系 (m/s) */
    EKF_STATE_VN = 3,      /**< 速度: 北向 (m/s)      */
    EKF_STATE_VE = 4,      /**< 速度: 东向 (m/s)      */
    EKF_STATE_VD = 5,      /**< 速度: 地向 (m/s)      */

    /* 姿态 — Hamilton 四元数, 标量在前 */
    EKF_STATE_QW = 6,      /**< 四元数标量分量 q_w    */
    EKF_STATE_QX = 7,      /**< 四元数向量分量 q_x    */
    EKF_STATE_QY = 8,      /**< 四元数向量分量 q_y    */
    EKF_STATE_QZ = 9,      /**< 四元数向量分量 q_z    */

    /* 陀螺仪加性偏置 (rad/s) */
    EKF_STATE_BGX = 10,    /**< 陀螺仪 X 轴 bias (rad/s) */
    EKF_STATE_BGY = 11,    /**< 陀螺仪 Y 轴 bias (rad/s) */
    EKF_STATE_BGZ = 12,    /**< 陀螺仪 Z 轴 bias (rad/s) */

    /* 加速度计加性偏置 (m/s²) — 可选 */
    EKF_STATE_BAX = 13,    /**< 加速度计 X 轴 bias (m/s²) */
    EKF_STATE_BAY = 14,    /**< 加速度计 Y 轴 bias (m/s²) */
    EKF_STATE_BAZ = 15,    /**< 加速度计 Z 轴 bias (m/s²) */
} ekf_state_index_t;

/* ========================================================================== */
/*  核心数据结构                                                               */
/* ========================================================================== */

/**
 * @brief 四元数 — Hamilton 约定，标量在前
 *
 * 存储布局: q = [w, x, y, z]
 *
 * 语义: 将 NED 世界系向量旋转到 FRD 机体系
 *       v_body = q ⊗ v_world ⊗ q*
 *
 * 单位四元数 (无旋转，水平朝北): q = {1, 0, 0, 0}
 * 模长约束: w² + x² + y² + z² = 1
 */
typedef struct {
    float w;    /**< 标量分量 cos(θ/2)              */
    float x;    /**< 向量分量 u_x * sin(θ/2)        */
    float y;    /**< 向量分量 u_y * sin(θ/2)        */
    float z;    /**< 向量分量 u_z * sin(θ/2)        */
} ekf_quat_t;

/**
 * @brief 欧拉角 — ZYX 旋转顺序 (ψ→θ→φ)
 *
 * 注意事项:
 *   - Yaw  (ψ): 绕 NED-Z 轴，机头右偏为正，范围 (-π, π]
 *   - Pitch (θ): 绕旋转后 Y 轴，抬头为正，范围 [-π/2, π/2]
 *   - Roll  (φ): 绕旋转后 X 轴，右倾为正，范围 (-π, π]
 *
 * 全部单位为弧度 (rad)。
 * 水平朝北时: {0, 0, 0}
 */
typedef struct {
    float roll;     /**< 横滚角 φ (rad)，右倾为正   */
    float pitch;    /**< 俯仰角 θ (rad)，抬头为正   */
    float yaw;      /**< 偏航角 ψ (rad)，右偏为正   */
} ekf_euler_t;

/**
 * @brief 3D 向量 — 通用三轴浮点量
 *
 * 用于位置、速度、角速度、磁场等三轴物理量。
 * 具体含义取决于上下文中的坐标系标注。
 */
typedef struct {
    float x;    /**< 第一轴分量                     */
    float y;    /**< 第二轴分量                     */
    float z;    /**< 第三轴分量                     */
} ekf_vec3_t;

/**
 * @brief 3×3 旋转矩阵 — 行主序存储
 *
 * R[row][col]，维度 3×3
 * 行主序 (row-major): R[0] = 第一行, R[1] = 第二行, R[2] = 第三行
 *
 * 语义: 将世界系向量旋转到机体系
 *       v_body = R * v_world
 *
 * ZYX 欧拉角对应的旋转矩阵:
 *       R = Rx(φ) · Ry(θ) · Rz(ψ)
 *
 * 正交约束: R^T * R = I, det(R) = +1
 */
typedef struct {
    float m[3][3];  /**< m[row][col] */
} ekf_mat3_t;

/**
 * @brief EKF 完整状态向量
 *
 * 维度: EKF_STATE_DIM (16)
 *
 * 初始状态假设 (水平静止、机头朝北):
 *   位置: [0, 0, 0] 或 GPS 初始值
 *   速度: [0, 0, 0]
 *   姿态: [1, 0, 0, 0]
 *   陀螺仪 bias: [0, 0, 0]
 *   加速度计 bias: [0, 0, 0]
 *
 * 初始化方法:
 *   姿态 — 静止时由加速度计估计 roll/pitch，磁力计估计 yaw
 *   bias — 静止 N 秒取均值作为初始估计
 */
typedef struct {
    ekf_vec3_t   pos;       /**< 位置 [N, E, D]   (m)        */
    ekf_vec3_t   vel;       /**< 速度 [N, E, D]   (m/s)      */
    ekf_quat_t   quat;      /**< 姿态四元数 [w,x,y,z]        */
    ekf_vec3_t   gyro_bias; /**< 陀螺仪偏置 (rad/s)          */
    ekf_vec3_t   accel_bias;/**< 加速度计偏置 (m/s²)         */
} ekf_state_t;

/**
 * @brief 误差状态向量 (用于 Error-State EKF)
 *
 * 在 EKF 中，四元数的 4 个分量存在单位模长约束（冗余参数），
 * 直接在协方差矩阵中传播 4 维四元数会导致奇异性问题。
 *
 * 解决方案: 使用 3 维误差旋转向量 δθ 表示姿态的微小扰动，
 * 协方差矩阵维度 = EKF_ERROR_STATE_DIM (15)。
 *
 * 误差状态定义:
 *   δx = [δp(3), δv(3), δθ(3), δb_g(3), δb_a(3)]
 *
 * 四元数修正:
 *   q_true ≈ q_nominal ⊗ [1, δθ/2]  (一阶近似)
 */
typedef struct {
    ekf_vec3_t delta_pos;       /**< 位置误差 (m)           */
    ekf_vec3_t delta_vel;       /**< 速度误差 (m/s)         */
    ekf_vec3_t delta_theta;     /**< 姿态误差旋转向量 (rad) */
    ekf_vec3_t delta_gyro_bias; /**< 陀螺仪 bias 误差      */
    ekf_vec3_t delta_accel_bias;/**< 加速度计 bias 误差     */
} ekf_error_state_t;

/**
 * @brief 15×15 协方差矩阵
 *
 * 对应误差状态 [δp(3), δv(3), δθ(3), δb_g(3), δb_a(3)]。
 * 行主序存储，索引 [i][j]。
 *
 */
typedef struct {
    float data[EKF_ERROR_STATE_DIM][EKF_ERROR_STATE_DIM];
} ekf_cov_t;

/* ========================================================================== */
/*  工具函数声明                                                               */
/* ========================================================================== */

/**
 * @brief 四元数归一化
 * @param[in,out] q 指向待归一化的四元数
 * @note 传播过程中数值误差可能导致模长偏离 1，需定期调用
 */
void ekf_quat_normalize(ekf_quat_t *q);

/**
 * @brief 四元数 → 旋转矩阵
 * @param[in]  q 输入四元数 (Hamilton, scalar-first)
 * @param[out] R 输出旋转矩阵, v_body = R * v_world
 */
void ekf_quat_to_rotmat(const ekf_quat_t *q, ekf_mat3_t *R);

/**
 * @brief 四元数 → 欧拉角
 * @param[in]  q   输入四元数 (Hamilton, scalar-first)
 * @param[out] euler 输出欧拉角 [roll, pitch, yaw] (rad)
 *
 * 旋转顺序: ZYX (Yaw→Pitch→Roll)
 */
void ekf_quat_to_euler(const ekf_quat_t *q, ekf_euler_t *euler);

/**
 * @brief 欧拉角 → 四元数
 * @param[in]  euler 输入欧拉角 [roll, pitch, yaw] (rad)
 * @param[out] q     输出四元数 (Hamilton, scalar-first)
 */
void ekf_euler_to_quat(const ekf_euler_t *euler, ekf_quat_t *q);

/**
 * @brief 四元数乘法 (Hamilton 积)
 * @param[in]  q1 左乘四元数
 * @param[in]  q2 右乘四元数
 * @param[out] q_out = q1 ⊗ q2
 */
void ekf_quat_mult(const ekf_quat_t *q1, const ekf_quat_t *q2,
                    ekf_quat_t *q_out);

/**
 * @brief 用误差旋转向量修正标称四元数
 * @param[in]     q_nom  标称四元数
 * @param[in]     delta  误差旋转向量 δθ (3 维, rad)
 * @param[out]    q_out  修正后的四元数
 *
 * 计算: q_out = q_nom ⊗ [1, δθ/2] (一阶近似后归一化)
 */
void ekf_quat_apply_correction(const ekf_quat_t *q_nom,
                                const ekf_vec3_t *delta,
                                ekf_quat_t *q_out);

/**
 * @brief 初始化默认状态 (水平静止、机头朝北)
 * @param[out] state 输出初始状态
 */
void ekf_state_init_default(ekf_state_t *state);

/**
 * @brief 初始化协方差矩阵为对角阵
 * @param[out] P     输出协方差矩阵
 * @param[in]  pos_std   位置初始标准差 (m)
 * @param[in]  vel_std   速度初始标准差 (m/s)
 * @param[in]  att_std   姿态初始标准差 (rad)
 * @param[in]  gbias_std 陀螺仪 bias 初始标准差 (rad/s)
 * @param[in]  abias_std 加速度计 bias 初始标准差 (m/s²)
 */
void ekf_cov_init_diagonal(ekf_cov_t *P,
                            float pos_std,
                            float vel_std,
                            float att_std,
                            float gbias_std,
                            float abias_std);

#ifdef __cplusplus
}
#endif

#endif /* EKF_TYPES_H */
