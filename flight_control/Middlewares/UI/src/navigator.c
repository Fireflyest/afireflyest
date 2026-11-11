#include "navigator.h"
#include "flow.h"
#include "gps.h"
#include <string.h>
#include <math.h>

UI_Logger logWindow;

static UI_Widget* widgets[16];
static uint8_t currentWindow;
static void UI_IMU_Draw(UI_Widget* widget);
static void UI_Cube_Draw(UI_Widget* widget);
static void UI_Pid_Draw(UI_Widget* widget);
static void UI_Accel_Calib_Draw(UI_Widget* widget);
static void UI_Gyro_Calib_Draw(UI_Widget* widget);
static void UI_Mag_Calib_Draw(UI_Widget* widget);
static void UI_MagDbg_Draw(UI_Widget* widget);
static void UI_Battery_Draw(UI_Widget* widget);
static void UI_Flow_Draw(UI_Widget* widget);
static void UI_Gps_Draw(UI_Widget* widget);

static UI_Window mpuWindow;
static UI_Window pidWindow;
static UI_Window homeWindow;
static UI_Window accelCalibWindow;
static UI_Window gyroCalibWindow;
static UI_Window magCalibWindow;
static UI_Window magDbgWindow;
static UI_Window cubeWindow;
static UI_Window batteryWindow;
static UI_Window flowWindow;
static UI_Window gpsWindow;


void Window_Init(void) {
    widgets[WINDOW_NONE] = (UI_Widget*)&logWindow;
    currentWindow = WINDOW_NONE;

    UI_Window_Init(&mpuWindow, 0, 0, 128, 64);
    mpuWindow.base.draw = UI_IMU_Draw;
    widgets[WINDOW_IMU] = (UI_Widget*)&mpuWindow;

    UI_Window_Init(&pidWindow, 0, 0, 128, 64);
    pidWindow.base.draw = UI_Pid_Draw;
    widgets[WINDOW_PID] = (UI_Widget*)&pidWindow;

    UI_Window_Init(&homeWindow, 0, 0, 128, 64);
    static UI_Label lblTitle, lblCount;
    UI_Label_Init(&lblTitle, 10, 10, "Home:");
    UI_Label_Init(&lblCount, 50, 30, "aaaaaa");
    UI_AddChild((UI_Widget*)&homeWindow, (UI_Widget*)&lblTitle);
    UI_AddChild((UI_Widget*)&homeWindow, (UI_Widget*)&lblCount);
    widgets[WINDOW_HOME] = (UI_Widget*)&homeWindow;


    UI_Window_Init(&accelCalibWindow, 0, 0, 128, 64);
    accelCalibWindow.base.draw = UI_Accel_Calib_Draw;
    widgets[WINDOW_ACCEL_CALIBRATE] = (UI_Widget*)&accelCalibWindow;

    UI_Window_Init(&gyroCalibWindow, 0, 0, 128, 64);
    gyroCalibWindow.base.draw = UI_Gyro_Calib_Draw;
    widgets[WINDOW_GYRO_CALIBRATE] = (UI_Widget*)&gyroCalibWindow;

    UI_Window_Init(&magCalibWindow, 0, 0, 128, 64);
    magCalibWindow.base.draw = UI_Mag_Calib_Draw;
    widgets[WINDOW_MAG_CALIBRATE] = (UI_Widget*)&magCalibWindow;

    UI_Window_Init(&magDbgWindow, 0, 0, 128, 64);
    magDbgWindow.base.draw = UI_MagDbg_Draw;
    widgets[WINDOW_MAGDBG] = (UI_Widget*)&magDbgWindow;

    UI_Window_Init(&cubeWindow, 0, 0, 128, 64);
    cubeWindow.base.draw = UI_Cube_Draw;
    widgets[WINDOW_CUBE] = (UI_Widget*)&cubeWindow;

    UI_Window_Init(&batteryWindow, 0, 0, 128, 64);
    batteryWindow.base.draw = UI_Battery_Draw;
    widgets[WINDOW_BATTERY] = (UI_Widget*)&batteryWindow;

    UI_Window_Init(&flowWindow, 0, 0, 128, 64);
    flowWindow.base.draw = UI_Flow_Draw;
    widgets[WINDOW_FLOW] = (UI_Widget*)&flowWindow;

    UI_Window_Init(&gpsWindow, 0, 0, 128, 64);
    gpsWindow.base.draw = UI_Gps_Draw;
    widgets[WINDOW_GPS] = (UI_Widget*)&gpsWindow;
}

uint8_t Window_Current(void) {
    return currentWindow;
}

void Window_To(uint8_t windowId) {
    if (windowId < 16 && widgets[windowId]) {
        currentWindow = windowId;
    }
}

void Window_Render(void) {
    GFX_Clear();
    UI_DrawTree(widgets[currentWindow], 0, 0);
    GFX_Update();
}


void UI_IMU_Draw(UI_Widget* widget) {
    char line[40];
    int x = widget->x + 2;
    int y = widget->y + 2;
    int lh = 10; // 行高，根据字体调整

    sm_vec3_t accel, gyro, mag;
    Attitude_GetAccel(accel);
    Attitude_GetGyro(gyro);
    Attitude_GetMag(mag);

    int16_t tempRaw = (int16_t)((imu_rx_buf[12] << 8) | imu_rx_buf[13]);
    float tempC = (float)tempRaw / 333.87f + 21.0f;

    snprintf(line, sizeof(line), "AX %6d GX %6d", (int)(accel[0] * 10), (int)(gyro[0] * 10));
    GFX_DrawString(x, y + 0 * lh, line, GFX_COLOR_WHITE);

    snprintf(line, sizeof(line), "AY %6d GY %6d", (int)(accel[1] * 10), (int)(gyro[1] * 10));
    GFX_DrawString(x, y + 1 * lh, line, GFX_COLOR_WHITE);

    snprintf(line, sizeof(line), "AZ %6d GZ %6d", (int)(accel[2] * 10), (int)(gyro[2] * 10));
    GFX_DrawString(x, y + 2 * lh, line, GFX_COLOR_WHITE);

    snprintf(line, sizeof(line), "MX %6d MY %6d", (int)(mag[0]), (int)(mag[1]));
    GFX_DrawString(x, y + 3 * lh, line, GFX_COLOR_WHITE);

    /* N = 实测磁场模长, E = 参考磁场模长 (都应 25~65µT) */
    snprintf(line, sizeof(line), "MZ %5d N%3d E%3d", (int)mag[2],
             (int)(Attitude_MagFieldNorm() + 0.5f),
             (int)(Attitude_MagRefNorm() + 0.5f));
    GFX_DrawString(x, y + 4 * lh, line, GFX_COLOR_WHITE);

    // {
    //     float at = tempC < 0.0f ? -tempC : tempC;
    //     int ti = (int)at;                       // integer part
    //     int tf = (int)(at * 10.0f) % 10;         // one decimal
    //     if (tempC < 0.0f) {
    //         snprintf(line, sizeof(line), "MZ %6d  T -%d.%dC", (int)mz, ti, tf);
    //     } else {
    //         snprintf(line, sizeof(line), "MZ %6d  T %d.%dC", (int)mz, ti, tf);
    //     }
    // }
    // GFX_DrawString(x, y + 4 * lh, line, GFX_COLOR_WHITE);


    {
        float at = temperature_rx < 0.0f ? -temperature_rx : temperature_rx;
        int t_i = (int)at;
        int t_d = (int)(at * 10.0f) % 10;

        float aa = altitude_rx < 0.0f ? -altitude_rx : altitude_rx;
        int a_i = (int)aa;
        int a_d = (int)(aa * 10.0f) % 10;
        snprintf(line, sizeof(line), "%3d.%1d %3d.%1dm", t_i, t_d, a_i, a_d);
    }
    GFX_DrawString(x, y + 5 * lh, line, GFX_COLOR_WHITE);
    GFX_DrawString(x, y + 6 * lh, " ", GFX_COLOR_WHITE);
}

void UI_Cube_Draw(UI_Widget* widget) {
    Point3D center = { 0.0f, 0.0f, -20.0f };    // closer to camera (try -20, -40, ...)
    Vector3D halfExtent = { 20.0f, 20.0f, 20.0f }; // larger half-size for clearer view

    Quaternion q;
    sm_quat_t current_quat;
    Attitude_GetQuat(current_quat);
    
    q.w = current_quat[0];
    q.x = current_quat[1];
    q.y = current_quat[2];
    q.z = current_quat[3];

    GFX3D_DrawCube(&center, &halfExtent, &q, GFX_COLOR_WHITE);

    float axisLen = 10.0f; // 轴的长度（应大于立方体半长 20.0f）
    Vector3D vX = {axisLen, 0.0f, 0.0f};
    Vector3D vY = {0.0f, axisLen, 0.0f};
    Vector3D vZ = { 0.0f, 0.0f, axisLen * 2 }; // *2 让Z轴更明显

    // 使用四元数旋转轴向量
    Math3D_QuatRotateVector(&vX, &q);
    Math3D_QuatRotateVector(&vY, &q);
    Math3D_QuatRotateVector(&vZ, &q);

    // 计算三轴末端在 3D 空间的位置
    Point3D pStart, pEnd;

    // X轴
    pStart = center;
    pEnd.x = center.x + vX.x; pEnd.y = center.y + vX.y; pEnd.z = center.z + vX.z;
    GFX3D_DrawLineStyled(&pStart, &pEnd, GFX_COLOR_WHITE, GFX_LINE_STYLE_THICK_DOT);

    // Y轴
    pStart = center;
    pEnd.x = center.x + vY.x; pEnd.y = center.y + vY.y; pEnd.z = center.z + vY.z;
    GFX3D_DrawLine(&pStart, &pEnd, GFX_COLOR_WHITE);

    // Z轴
    pStart = center;
    pEnd.x = center.x + vZ.x; pEnd.y = center.y + vZ.y; pEnd.z = center.z + vZ.z;
    GFX3D_DrawLine(&pStart, &pEnd, GFX_COLOR_WHITE);

    // show logic FPS
    char line[20];
    snprintf(line, sizeof(line), "FPS: %d", FPS_Get());
    GFX_DrawString(0, 0, line, GFX_COLOR_WHITE);

    uint8_t still;
    uint8_t accel_clib_face;
    float velocityZ;
    Attitude_IsStill(&still);
    Attitude_CalibratingFace(&accel_clib_face);
    Attitude_GetVelocityZ(&velocityZ);

    /* 状态 */
    snprintf(line, sizeof(line), "S:%d F:%d", still, accel_clib_face);
    GFX_DrawString(0, 0, line, GFX_COLOR_WHITE);

    /* 磁力计校准 */
    {
        uint8_t mag_st = Attitude_MagCalibStatus();
        float mag_pct = Attitude_MagCalibrateProgress();
        switch (mag_st) {
        case 1:
            snprintf(line, sizeof(line), "M:%3d%%", (int)(mag_pct * 100.0f));
            break;
        case 2:
            snprintf(line, sizeof(line), "M: OK");
            break;
        case 3:
            snprintf(line, sizeof(line), "M: ERR");
            break;
        default:
            /* 状态机 IDLE 不代表没校准: 重启后参数从 Flash 加载,
             * 状态机回 IDLE 但 mag_calib.is_valid=1, 应显示 OK */
            snprintf(line, sizeof(line), Attitude_MagCalibValid() ? "M: OK" : "M: --");
            break;
        }
        /* C = EKF 地磁参考已标定, 1 才代表磁力计在修正 yaw */
        char mbuf[24];
        snprintf(mbuf, sizeof(mbuf), "%s C%d", line, Attitude_MagRefCalibrated());
        GFX_DrawString(0, 10, mbuf, GFX_COLOR_WHITE);
    }

    /* 高度 */
    {
        float alt;
        Attitude_GetAltitude(&alt);
        int sign = (alt < 0.0f) ? -1 : 1;
        if (sign < 0)
            alt = -alt;
        int intPart = (int)alt;
        int fracPart = (int)((alt - (float)intPart) * 10.0f + 0.5f);
        if (fracPart >= 10) {
            intPart += 1;
            fracPart = 0;
        }
        if (sign < 0)
            intPart = -intPart;
        snprintf(line, sizeof(line), "H:%d.%d", intPart, fracPart);
        GFX_DrawString(0, 20, line, GFX_COLOR_WHITE);
    }

    /* 垂直速度 */
    {
        int sign = (velocityZ < 0.0f) ? -1 : 1;
        float av = fabsf(velocityZ);
        int vi = (int)av;
        int vf = (int)(av * 10.0f) % 10;
        if (sign < 0)
            vi = -vi;
        snprintf(line, sizeof(line), "V:%d.%d", vi, vf);
        GFX_DrawString(0, 30, line, GFX_COLOR_WHITE);
    }

    /* FPS */
    snprintf(line, sizeof(line), "FPS:%d", FPS_Get());
    GFX_DrawString(0, 40, line, GFX_COLOR_WHITE);

    /* 磁力计诊断: 新息向量 µT (X/Y/Z, 截±99) + 门控拒绝计数 (R 千位缩写)
     * 旋转测试: 转90°/180° 看分量变化, 定位轴向映射错误还是硬铁偏置 */
    {
        uint32_t mok, mrej;
        Attitude_MagStats(&mok, &mrej);
        float iv[3] = {0.0f, 0.0f, 0.0f};
        Attitude_MagInnovVec(iv);
        int x = (int)iv[0], y = (int)iv[1], z = (int)iv[2];
        if (x > 99) x = 99;  if (x < -99) x = -99;
        if (y > 99) y = 99;  if (y < -99) y = -99;
        if (z > 99) z = 99;  if (z < -99) z = -99;
        char rbuf[8];
        if (mrej >= 1000u)
            snprintf(rbuf, sizeof(rbuf), "%uk", (unsigned)(mrej / 1000u));
        else
            snprintf(rbuf, sizeof(rbuf), "%u", (unsigned)mrej);
        snprintf(line, sizeof(line), "X%+3d Y%+3d Z%+3d R%s", x, y, z, rbuf);
        GFX_DrawString(0, 50, line, GFX_COLOR_WHITE);
    }
}

static float pidErrorHistory[3][128] = {0}; // 0:roll, 1:pitch, 2:yaw
static uint16_t pidErrorLen = 0;
void UI_Pid_Draw(UI_Widget* widget) {
    UI_Cube_Draw(widget); // 先画立方体作为背景参考

    int x0 = widget->x + 2;
    int y0 = widget->y + widget->h / 2; // 中线
    int w = widget->w - 4;
    int h = widget->h - 4;

    // 获取当前误差
    sm_vec3_t gyro;
    Attitude_GetGyro(gyro);
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];
    float errors[3] = {
        (rateSetRoll - gx), 
        (rateSetPitch - gy), 
        (rateSetYaw - gz)
    };

    /*
     * 更新历史 — 每 8 帧采一点: 200Hz/8 = 25Hz 采样, 124 列 ≈ 5s 窗口。
     * 原来每帧一列只有 0.6s 窗口, 0.2~2Hz 的慢摆在屏上看不出来,
     * 25Hz 采样可分辨 ≤12Hz 的摆动 (足够覆盖姿态环/延迟振荡)
     */
    static uint8_t decim = 0;
    if (++decim >= 8) {
        decim = 0;
        if (pidErrorLen < w) {
            for (int i = 0; i < 3; i++)
                pidErrorHistory[i][pidErrorLen] = errors[i];
            pidErrorLen++;
        } else {
            // 左移一格
            for (int i = 0; i < 3; i++)
                memmove(&pidErrorHistory[i][0], &pidErrorHistory[i][1], (w - 1) * sizeof(float));
            for (int i = 0; i < 3; i++)
                pidErrorHistory[i][w - 1] = errors[i];
        }
    }

    static float globalScale = 0.1f; // 初始可视化幅度（单位与误差一致）
    float curMax = 1e-6f;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < pidErrorLen; j++)
            if (fabsf(pidErrorHistory[i][j]) > curMax)
                curMax = fabsf(pidErrorHistory[i][j]);

    // 快速增长，缓慢衰减（调整下方因子以改变响应/回落速度）
    if (curMax > globalScale) {
        globalScale = curMax;
    } else {
        globalScale *= 0.995f; // 0.995 => 0.5x 需要约 ln(0.5)/ln(0.995) 帧数
        if (globalScale < 1e-3f) globalScale = 1e-3f; // 下限保护
    }

    float scale = globalScale;

    // 画三条线（逐点画像素）
    for (int i = 0; i < 3; i++) {
        uint16_t color = GFX_COLOR_WHITE;
        int lastY = y0 - (int)(pidErrorHistory[i][0] / scale * (h / 2));
        for (int j = 1; j < pidErrorLen; j++) {
            int y = y0 - (int)(pidErrorHistory[i][j] / scale * (h / 2));
            GFX_Line_Style style = i == 0 ? GFX_LINE_STYLE_THICK_DOT :
                                  (i == 1 ? GFX_LINE_STYLE_ALTERNATE_2_1PX : GFX_LINE_STYLE_SINGLE_PIXEL);
            GFX_DrawLineStyled(x0 + j - 1, lastY, x0 + j, y, color, style);
            lastY = y;
        }
    }

    // 画左侧中线原点
    GFX_DrawPixel(x0, y0, GFX_COLOR_WHITE);
}

void UI_Accel_Calib_Draw(UI_Widget* widget) {

}

void UI_Gyro_Calib_Draw(UI_Widget* widget) {

}

void UI_Mag_Calib_Draw(UI_Widget* widget) {

}

/* 磁力计坐标轴判断画面 (纯数据):
 *  M = 实测磁场 (机体系)   H = 预测磁场 R·m_earth (机体系)
 *  D = 新息 EMA (M−H)      转动后哪个分量 M/H 对不上 = 错误轴线索
 *  验收: 各姿态转动停稳后 D 三个分量都 ≈0 */
void UI_MagDbg_Draw(UI_Widget* widget) {
    char line[40];
    int x = widget->x + 2;
    int y = widget->y + 2;
    int lh = 10;

    float m[3], p[3], d[3];
    Attitude_GetMag(m);
    Attitude_MagPredicted(p);
    Attitude_MagInnovEma(d);

    int mi[3], pi[3], di[3];
    for (int i = 0; i < 3; i++) {
        mi[i] = (int)(m[i] + (m[i] < 0 ? -0.5f : 0.5f));
        pi[i] = (int)(p[i] + (p[i] < 0 ? -0.5f : 0.5f));
        di[i] = (int)(d[i] + (d[i] < 0 ? -0.5f : 0.5f));
        if (mi[i] > 999) mi[i] = 999;  if (mi[i] < -999) mi[i] = -999;
        if (pi[i] > 999) pi[i] = 999;  if (pi[i] < -999) pi[i] = -999;
        if (di[i] > 999) di[i] = 999;  if (di[i] < -999) di[i] = -999;
    }
    snprintf(line, sizeof(line), "M%+4d%+4d%+4d", mi[0], mi[1], mi[2]);
    GFX_DrawString(x, y + 0 * lh, line, GFX_COLOR_WHITE);
    snprintf(line, sizeof(line), "H%+4d%+4d%+4d", pi[0], pi[1], pi[2]);
    GFX_DrawString(x, y + 1 * lh, line, GFX_COLOR_WHITE);
    snprintf(line, sizeof(line), "D%+4d%+4d%+4d", di[0], di[1], di[2]);
    GFX_DrawString(x, y + 2 * lh, line, GFX_COLOR_WHITE);

    uint32_t mok, mrej;
    Attitude_MagStats(&mok, &mrej);
    int n = (int)(Attitude_MagFieldNorm() + 0.5f);
    int e = (int)(Attitude_MagRefNorm() + 0.5f);
    int q = (int)(Attitude_MagInnovPp() + 0.5f);
    if (n < 0) n = 0;  if (n > 999) n = 999;
    if (e < 0) e = 0;  if (e > 999) e = 999;
    if (q < 0) q = 0;  if (q > 999) q = 999;
    snprintf(line, sizeof(line), "N%3d E%3d C%d Q%2d", n, e,
             Attitude_MagRefCalibrated(), q);
    GFX_DrawString(x, y + 3 * lh, line, GFX_COLOR_WHITE);

    char rbuf[6];
    if (mrej >= 1000u) {
        unsigned rk = (unsigned)(mrej / 1000u);
        if (rk > 999u) rk = 999u;
        snprintf(rbuf, sizeof(rbuf), "%uk", rk);
    } else {
        snprintf(rbuf, sizeof(rbuf), "%u", (unsigned)mrej);
    }
    int wdeg = (int)(Attitude_YawUnwrapped() +
                     (Attitude_YawUnwrapped() < 0 ? -0.5f : 0.5f));
    snprintf(line, sizeof(line), "R%-5s W%+5d", rbuf, wdeg);
    GFX_DrawString(x, y + 4 * lh, line, GFX_COLOR_WHITE);

    /* 对准状态: A=att_state(0对准中/1已对准) n=IMU好帧 m=磁场好帧
     * C0 时判卡点: n 不涨→在动; m 不涨→磁数据模长出界; A1 但 C0→对准走了兕底 */
    uint8_t astate;
    int acnt, amcnt;
    Attitude_AlignStats(&astate, &acnt, &amcnt);
    snprintf(line, sizeof(line), "A%d n%3d m%3d", astate, acnt, amcnt);
    GFX_DrawString(x, y + 5 * lh, line, GFX_COLOR_WHITE);
}

void UI_Battery_Draw(UI_Widget* widget) {
    /* External battery/state variables (define these elsewhere in the project) */
    float battery_voltage = Battery_GetVoltage() / 1000.0f; // Convert mV to V
    uint8_t battery_percent = Battery_GetPercentage();

    const int x = widget->x + 2;
    const int y = widget->y + 2;

    /* Battery icon geometry (right side of 128x64 display) */
    const int iconW = 28;
    const int iconH = 12;
    const int iconX = 128 - iconW - 6; /* margin from right */
    const int iconY = y;

    uint8_t pct = battery_percent;
    if (pct > 100) pct = 100;

    /* Draw battery outline */
    GFX_DrawRect(iconX, iconY, iconW, iconH, GFX_COLOR_WHITE);
    /* draw battery tip */
    GFX_FillRect(iconX + iconW, iconY + (iconH / 3), 2, iconH / 3, GFX_COLOR_WHITE);

    /* Fill level inside (leave 2px padding) */
    const int fillMaxW = iconW - 4;
    const int fillW = (fillMaxW * pct) / 100;
    if (fillW > 0) {
        GFX_FillRect(iconX + 2, iconY + 2, fillW, iconH - 4, GFX_COLOR_WHITE);
    }

    /* Draw percent and voltage text to the left of the icon */
    char buf[32];
    if (pct <= 100) {
        snprintf(buf, sizeof(buf), "%3d%%", pct);
    } else {
        snprintf(buf, sizeof(buf), "  - ");
    }
    GFX_DrawString(iconX - 34, iconY, buf, GFX_COLOR_WHITE);

    if (battery_voltage > 0.0f) {
        /* show voltage with one decimal */
        int v_i = (int)battery_voltage;
        int v_d = (int)(battery_voltage * 10.0f) % 10;
        snprintf(buf, sizeof(buf), "%d.%dV", v_i, v_d);
    } else {
        snprintf(buf, sizeof(buf), "N/A");
    }
    GFX_DrawString(iconX - 34, iconY + 10, buf, GFX_COLOR_WHITE);

    snprintf(buf, sizeof(buf), "status %d", pwr_state);
    GFX_DrawString(iconX - 34, iconY + 20, buf, GFX_COLOR_WHITE);
    
}

/* ── WINDOW_FLOW: 光流航位推算轨迹 ───────────────────────────────────
 *
 *  屏幕中心 = 累积位移原点, 虚十字为坐标轴 (浅色 = 隔像素点)。
 *  点位置 = 累积位移 (cm) × PX_PER_CM; 这是航位推算累积量, 会漂移,
 *  不是绝对位置。
 *  数据可信度用点的样式表示:
 *    OK   (valid==1, 链路正常)  -> 实心圆
 *    LOW  (valid==2) / 已出界   -> 空心圆
 *    LOST (链路超时断开)        -> 叉号
 *
 *  布局 (128x64):
 *    y+0  : 顶部状态行   Q质量 D距离cm V速度cm/s + 状态字
 *    11..43: 绘图区     中心 (64,27), 纵向约 ±8cm, 横向约 ±31cm
 *    y+44 : 帧统计      有效帧 / CRC 错 / 丢帧
 *    y+52 : 底部        累积位移 PX/PY (cm, 1 位小数)
 */

#define FLOW_PX_PER_CM      2.0f    /* 1cm = 2px, 横向可视约 ±31cm */
#define FLOW_PLOT_TOP       11
#define FLOW_PLOT_BOTTOM    43
#define FLOW_CX             (128 / 2)
#define FLOW_CY             ((FLOW_PLOT_TOP + FLOW_PLOT_BOTTOM) / 2)

static int Flow_RoundI(float v)
{
    return (int)(v + (v < 0.0f ? -0.5f : 0.5f));
}

/* 实心圆 (GFX_DrawCircle 未实现, 这里自己画) */
static void Flow_FillCircle(int cx, int cy, int r, GFX_Color color)
{
    int dx, dy;
    int r2 = r * r;
    for (dy = -r; dy <= r; dy++) {
        for (dx = -r; dx <= r; dx++) {
            if (dx * dx + dy * dy <= r2) {
                GFX_DrawPixel(cx + dx, cy + dy, color);
            }
        }
    }
}

/* 空心圆: 表示 "数据降级/已出界" */
static void Flow_DrawRing(int cx, int cy, int r, GFX_Color color)
{
    int dx, dy;
    int outer = r * r;
    int inner = (r - 1) * (r - 1);
    for (dy = -r; dy <= r; dy++) {
        for (dx = -r; dx <= r; dx++) {
            int d2 = dx * dx + dy * dy;
            if (d2 <= outer && d2 > inner) {
                GFX_DrawPixel(cx + dx, cy + dy, color);
            }
        }
    }
}

/* 有符号 cm, 1 位小数; 手工拆分整数/小数, 不用浮点 printf
 * |v| >= 100cm 时省掉小数位以控制行宽 */
static void Flow_FmtCm(char *out, int outSize, float cm)
{
    int neg = (cm < 0.0f);
    float a = neg ? -cm : cm;
    int ip, fp;

    if (a > 999.0f) {
        a = 999.0f;
    }
    ip = (int)a;

    if (a >= 100.0f) {
        snprintf(out, outSize, "%s%d", neg ? "-" : "", ip);
        return;
    }

    fp = (int)((a - (float)ip) * 10.0f + 0.5f);
    if (fp >= 10) {
        fp = 0;
        ip++;
    }
    snprintf(out, outSize, "%s%d.%d", neg ? "-" : "", ip, fp);
}

static void UI_Flow_Draw(UI_Widget* widget)
{
    char line[32];
    char s1[12], s2[12];
    const flow_data_t *f = Flow_Get();

    const int x = widget->x + 2;
    const int y = widget->y + 2;
    const int right = widget->x + widget->w - 2;

    float pxcm = Flow_GetPosX() * 100.0f;
    float pycm = Flow_GetPosY() * 100.0f;
    int sx, sy;
    int out = 0;
    const char *st;

    /* ── 状态判定 ── */
    if (!f->link_ok) {
        st = "LOST";
    } else if (f->valid == 1u) {
        st = "OK";
    } else if (f->valid == 2u) {
        st = "LOW";
    } else {
        st = "IDLE";
    }

    /* ── 顶部状态行: Q / D(cm) / V(cm/s) / 状态 ── */
    if (isfinite(f->distance_m) && f->distance_m > 0.0f) {
        int dcm = Flow_RoundI(f->distance_m * 100.0f);
        if (dcm > 999) {
            dcm = 999;
        }
        snprintf(line, sizeof(line), "Q%3u D%3d", (unsigned)f->quality, dcm);
    } else {
        /* 距离未知 / 激光失联: 位移被冻结, 用 D--- 标注 */
        snprintf(line, sizeof(line), "Q%3u D---", (unsigned)f->quality);
    }
    GFX_DrawString(x, y, line, GFX_COLOR_WHITE);

    /* 速度: v = 单帧位移 / integration_us, 单位 m/s -> 显示 cm/s */
    if (f->link_ok && f->valid == 1u && f->distance_m > 0.0f &&
        f->integration_us > 0u &&
        isfinite(f->flow_x_rad) && isfinite(f->flow_y_rad)) {
        float vx = f->flow_x_rad * f->distance_m;
        float vy = f->flow_y_rad * f->distance_m;
        float v = sqrtf(vx * vx + vy * vy) * 1e6f / (float)f->integration_us;
        int vcm = isfinite(v) ? Flow_RoundI(v * 100.0f) : 0;
        if (vcm > 999) {
            vcm = 999;
        }
        if (vcm < 0) {
            vcm = 0;
        }
        snprintf(line, sizeof(line), "V%3d", vcm);
    } else {
        snprintf(line, sizeof(line), "V  -");
    }
    GFX_DrawString(x + 56, y, line, GFX_COLOR_WHITE);

    GFX_DrawString(right - (int)strlen(st) * 6, y, st, GFX_COLOR_WHITE);

    /* ── 虚十字轴线 (屏幕中心 = 原点) ── */
    {
        int xx, yy;
        for (xx = x; xx < right; xx += 2) {
            GFX_DrawPixel(xx, FLOW_CY, GFX_COLOR_WHITE);
        }
        for (yy = FLOW_PLOT_TOP; yy <= FLOW_PLOT_BOTTOM; yy += 2) {
            GFX_DrawPixel(FLOW_CX, yy, GFX_COLOR_WHITE);
        }
    }

    /* ── 累积位置 -> 屏幕坐标, 出界钳位 ── */
    sx = FLOW_CX + Flow_RoundI(pxcm * FLOW_PX_PER_CM);
    sy = FLOW_CY + Flow_RoundI(pycm * FLOW_PX_PER_CM);

    if (sx < x)                     { sx = x;                     out = 1; }
    if (sx > right)                 { sx = right;                 out = 1; }
    if (sy < FLOW_PLOT_TOP + 2)     { sy = FLOW_PLOT_TOP + 2;     out = 1; }
    if (sy > FLOW_PLOT_BOTTOM - 2)  { sy = FLOW_PLOT_BOTTOM - 2;  out = 1; }

    /* ── 画点: 样式随数据可信度变化 ── */
    if (!f->link_ok) {
        /* 链路断开: 叉号, 提示当前点不可信 */
        GFX_DrawLine(sx - 2, sy - 2, sx + 2, sy + 2, GFX_COLOR_WHITE);
        GFX_DrawLine(sx - 2, sy + 2, sx + 2, sy - 2, GFX_COLOR_WHITE);
    } else if (out || f->valid == 2u) {
        /* 出界或质量被拒: 空心圆 */
        Flow_DrawRing(sx, sy, 3, GFX_COLOR_WHITE);
    } else {
        Flow_FillCircle(sx, sy, 2, GFX_COLOR_WHITE);
    }

    /* ── 帧统计 (调试): K=有效帧 C=CRC错 L=seq 估算丢帧 ── */
    {
        unsigned ok = f->rx_ok > 99999u ? 99999u : f->rx_ok;
        unsigned cr = f->rx_crc_err > 999u ? 999u : f->rx_crc_err;
        unsigned lo = f->rx_lost > 999u ? 999u : f->rx_lost;
        snprintf(line, sizeof(line), "K%u C%u L%u", ok, cr, lo);
        GFX_DrawString(x, y + 44, line, GFX_COLOR_WHITE);
    }

    /* ── 底部: 累积位移 (cm, 1 位小数) ── */
    Flow_FmtCm(s1, sizeof(s1), pxcm);
    Flow_FmtCm(s2, sizeof(s2), pycm);
    snprintf(line, sizeof(line), "PX %s PY %scm", s1, s2);
    GFX_DrawString(x, y + 52, line, GFX_COLOR_WHITE);
}

/* ── WINDOW_GPS: NMEA 定位信息 (USART6) ─────────────────────────────
 *
 *  8 行 × 8px 铺满 64 像素 (6x8 字体), 每行最多 21 字符:
 *    FIX 3D Q1 S 23     定位模式(3D/2D/NO) / GGA 质量 / 使用卫星数
 *    LA 22.676927       纬度 (度, 6 位小数; 无效时 --)
 *    LO 113.997752      经度
 *    AL  96.5m H0.7     海拔(MSL) / HDOP
 *    SP  0.16 CR 123    地速 m/s / 航迹向(真北 °)
 *    UTC 02:36:56
 *    DAT 2024-03-14
 *    ANTENNA OPEN       模块 TXT 消息与统计行交替显示 (2s)
 *
 *  速率只有 1Hz: 采样值可能"跳变", 属正常。
 */

/* 微度 -> "[-]d.dddddd" (手工拆整数/小数, 不用浮点 printf) */
static void Gps_FmtUdeg(char *out, int outSize, int32_t udeg)
{
    int32_t a = (udeg < 0) ? -udeg : udeg;

    snprintf(out, outSize, "%s%ld.%06ld", (udeg < 0) ? "-" : "",
             (long)(a / 1000000), (long)(a % 1000000));
}

/* 有符号 1 位小数 (alt / hdop 共用) */
static void Gps_Fmt1(char *out, int outSize, float v)
{
    int neg = (v < 0.0f);
    float a = neg ? -v : v;
    int ip, fp;

    if (a > 9999.0f) {
        a = 9999.0f;
    }
    ip = (int)a;
    fp = (int)((a - (float)ip) * 10.0f + 0.5f);
    if (fp >= 10) {
        fp = 0;
        ip++;
    }
    snprintf(out, outSize, "%s%d.%d", neg ? "-" : "", ip, fp);
}

static void UI_Gps_Draw(UI_Widget* widget)
{
    char line[24];
    char b1[16], b2[16];
    const gps_data_t *g = GPS_Get();
    const int x = widget->x + 2;
    const int y = widget->y;    /* 8 行 × 8px = 64, 正好占满 */
    const int lh = 8;

    /* ══ 无有效语句: 显示接收诊断页 ══
     *
     * 每一项都是实测计数:
     *   BAUD        当前 MCU 侧接收波特率 (短按可切换)
     *   RAW /s      ★线上收到的全部字节速率, 含帧错字节
     *               这是判断"模块以多快在发"的唯一依据
     *   OK /s       其中成功组帧(无 FE/NE/PE)的字节速率
     *   A%          OK 字节里落在 0x20~0x7E 的占比
     *   H%          OK 字节里 bit7=1 的占比
     *   I%          OK 字节里"取反后是 ASCII"的占比
     *   $ / E       收到 '$' 的个数 / 校验和错误数
     *   R           带 FE/NE/PE 的字节数
     *   BR          其中 FE 且数据=0x00 的次数 (整帧被采成低电平)
     *   O / K       硬件 ORE / 环形缓冲溢出
     *
     * 长按 800ms 回环自检 / 长按 2500ms 回 log 页 */
    if (!g->link_ok) {
        unsigned rate = g->rx_raw_bps > 99999u ? 99999u : g->rx_raw_bps;
        unsigned ok = g->rx_bps > 99999u ? 99999u : g->rx_bps;
        unsigned a = GPS_AsciiRatio();
        unsigned h = GPS_HiBitRatio();
        unsigned iv = GPS_InvRatio();
        unsigned d = g->rx_dollar > 9999u ? 9999u : g->rx_dollar;
        unsigned e = g->rx_bad_csum > 999u ? 999u : g->rx_bad_csum;
        unsigned r = g->rx_line_err > 999u ? 999u : g->rx_line_err;
        unsigned br = g->rx_break > 999u ? 999u : g->rx_break;
        unsigned o = g->rx_hw_ovr > 99u ? 99u : g->rx_hw_ovr;
        unsigned k = g->rx_overrun > 99u ? 99u : g->rx_overrun;

        snprintf(line, sizeof(line), "GPS NO DATA %u", (unsigned)GPS_GetBaudrate());
        GFX_DrawString(x, y + 0 * lh, line, GFX_COLOR_WHITE);

        /* RAW: 线上全部字节速率 (含坏字节) */
        snprintf(line, sizeof(line), "RAW %u/s", rate);
        GFX_DrawString(x, y + 1 * lh, line, GFX_COLOR_WHITE);

        /* OK: 成功组帧的字节速率 + ASCII 占比 */
        snprintf(line, sizeof(line), "OK %u/s A%u%%", ok, a);
        GFX_DrawString(x, y + 2 * lh, line, GFX_COLOR_WHITE);

        snprintf(line, sizeof(line), "H%u%% I%u%% $%u", h, iv, d);
        GFX_DrawString(x, y + 3 * lh, line, GFX_COLOR_WHITE);

        snprintf(line, sizeof(line), "E%u R%u BR%u", e, r, br);
        GFX_DrawString(x, y + 4 * lh, line, GFX_COLOR_WHITE);

        snprintf(line, sizeof(line), "O%u K%u", o, k);
        GFX_DrawString(x, y + 5 * lh, line, GFX_COLOR_WHITE);

        {
            uint8_t lb = GPS_LoopbackResult();
            const char *lbs = (lb == 1u) ? "OK" :
                              (lb == 2u) ? "NONE" :
                              (lb == 3u) ? "BAD" : "--";
            snprintf(line, sizeof(line), "LB:%s F:%02X%02X%02X%02X", lbs,
                     g->raw[4], g->raw[5], g->raw[6], g->raw[7]);
        }
        GFX_DrawString(x, y + 6 * lh, line, GFX_COLOR_WHITE);

        /* 结论: 只按实测比值分支, 不外推硬件 */
        if (rate == 0u) {
            snprintf(line, sizeof(line), "NO BYTES ON RX");
        } else if (ok > 0u && iv >= 80u && a <= 10u) {
            snprintf(line, sizeof(line), "DATA IS INVERTED");
        } else if (r > 0u && br > 0u && br >= r / 2u) {
            snprintf(line, sizeof(line), "RX PIN HELD LOW");
        } else if (a >= 80u && d > 0u) {
            snprintf(line, sizeof(line), "TEXT BUT NO CRC");
        } else if (a < 50u) {
            snprintf(line, sizeof(line), "NOT ASCII DATA");
        } else if (d == 0u) {
            snprintf(line, sizeof(line), "NO $ IN DATA");
        } else if (r > 0u || o > 0u || k > 0u) {
            snprintf(line, sizeof(line), "BYTES DROPPED");
        } else {
            snprintf(line, sizeof(line), "CHECKSUM FAIL");
        }
        GFX_DrawString(x, y + 7 * lh, line, GFX_COLOR_WHITE);
        return;
    }

    /* 行 0: 定位模式 / 质量 / 卫星数
     *   U = GGA 字段 7: 模块解算时实际用了几颗星
     *   V = 各星座 GSV 字段 3 之和: 跟踪到的可见星数
     *
     * ★ 这两个数来自不同语句, 不存在"U <= V"的必然关系, 所以不用 a/b 形式:
     *   - GGA 可能是全系统合计 (如 $GNGGA 报 23)
     *   - GSV 是分星座上报, 求和后可能少于 GGA 的值
     *   要看明细请等第 7 行轮换到星座拆分 (如 "GP8 BD14") */
    snprintf(line, sizeof(line), "FIX %s Q%u U%u V%u", GPS_FixStr(),
             (unsigned)g->fix_quality, (unsigned)g->sats_used,
             (unsigned)g->sats_in_view);
    GFX_DrawString(x, y + 0 * lh, line, GFX_COLOR_WHITE);

    /* 行 1-2: 经纬度 (度, 6 位小数; S/W 用负号表示) */
    if (g->pos_valid) {
        Gps_FmtUdeg(b1, sizeof(b1), g->lat_udeg);
        snprintf(line, sizeof(line), "LA %s", b1);
    } else {
        snprintf(line, sizeof(line), "LA --");
    }
    GFX_DrawString(x, y + 1 * lh, line, GFX_COLOR_WHITE);

    if (g->pos_valid) {
        Gps_FmtUdeg(b1, sizeof(b1), g->lon_udeg);
        snprintf(line, sizeof(line), "LO %s", b1);
    } else {
        snprintf(line, sizeof(line), "LO --");
    }
    GFX_DrawString(x, y + 2 * lh, line, GFX_COLOR_WHITE);

    /* 行 3: 海拔 (MSL) + HDOP */
    if (g->has_alt) {
        Gps_Fmt1(b1, sizeof(b1), g->alt_m);
        Gps_Fmt1(b2, sizeof(b2), g->hdop);
        snprintf(line, sizeof(line), "AL %sm H%s", b1, b2);
    } else {
        snprintf(line, sizeof(line), "AL --");
    }
    GFX_DrawString(x, y + 3 * lh, line, GFX_COLOR_WHITE);

    /* 行 4: 地速 (m/s, 2 位小数) + 航迹向 */
    {
        char sp[8], cr[6];
        if (g->has_speed) {
            int cms = (int)(g->speed_mps * 100.0f + 0.5f);
            if (cms < 0) {
                cms = 0;
            }
            if (cms > 99999) {
                cms = 99999;
            }
            snprintf(sp, sizeof(sp), "%d.%02d", cms / 100, cms % 100);
        } else {
            snprintf(sp, sizeof(sp), "--");
        }
        if (g->has_course) {
            int cd = (int)(g->course_deg + 0.5f) % 360;
            snprintf(cr, sizeof(cr), "%d", cd);
        } else {
            snprintf(cr, sizeof(cr), "--");
        }
        snprintf(line, sizeof(line), "SP %5s CR %3s", sp, cr);
    }
    GFX_DrawString(x, y + 4 * lh, line, GFX_COLOR_WHITE);

    /* 行 5: UTC + 本地时间并排
     * NMEA 的时间字段恒为 UTC, 与模块装在哪个时区无关。
     * 本地时间 = UTC + GPS_LOCAL_TZ_HOURS (中国大陆 +8)。
     * 例: 本地晚上 22:05 -> 屏幕 "UTC14:05 CST22:05" 是正常的
     *     若 CST 后的小时与你手机不一致, 才是时区/时钟问题 */
    if (g->has_time) {
        int32_t sec = (int32_t)g->hour * 3600 + g->minute * 60 + g->second;
        int32_t lsec = sec + (int32_t)GPS_LOCAL_TZ_HOURS * 3600;
        unsigned lh, lm;

        while (lsec >= 86400) {
            lsec -= 86400;
        }
        while (lsec < 0) {
            lsec += 86400;
        }
        lh = (unsigned)(lsec / 3600);
        lm = (unsigned)((lsec % 3600) / 60);

        snprintf(line, sizeof(line), "UTC%02u:%02u %s%02u:%02u",
                 (unsigned)g->hour, (unsigned)g->minute,
                 GPS_LOCAL_TZ_LABEL, lh, lm);
    } else {
        snprintf(line, sizeof(line), "UTC--:-- %s--:--", GPS_LOCAL_TZ_LABEL);
    }
    GFX_DrawString(x, y + 5 * lh, line, GFX_COLOR_WHITE);

    /* 行 6: 无日期时显示 卫星峰值 + 各类语句累计条数
     *   PK = 上电/清零以来视野内可见卫星数的峰值 (户外看这个涨不涨)
     *   R  = RMC 条数   G = GGA 条数   P = GSV 分页数
     * 判读: PK 涨到 8 以上 -> 能收到卫星信号
     *       PK 一直是 0     -> 还没跟踪到任何卫星 */
    if (g->has_date) {
        unsigned pk = g->sats_in_view_max;
        snprintf(line, sizeof(line), "DAT %04u-%02u-%02u PK%u",
                 (unsigned)g->year, (unsigned)g->month, (unsigned)g->day, pk);
    } else {
        unsigned pk = g->sats_in_view_max;
        unsigned nrmc = g->n_rmc > 999u ? 999u : g->n_rmc;
        unsigned ngga = g->n_gga > 999u ? 999u : g->n_gga;
        unsigned ngsv = g->n_gsv > 999u ? 999u : g->n_gsv;
        snprintf(line, sizeof(line), "PK%u R%u G%u P%u", pk, nrmc, ngga, ngsv);
    }
    GFX_DrawString(x, y + 6 * lh, line, GFX_COLOR_WHITE);

    /* 行 7: 每 2s 四态轮换
     *   ① L/E/O 统计 + A(定位有效)/V(无效)
     *   ② 各星座可见星数明细, 如 "GP8 BD14 GL0" —— 解释 U/V 的来源
     *   ③ 最近一条 RMC 原文前 20 字符
     *   ④ 最近一条 GGA 原文前 20 字符 */
    {
        unsigned phase = (unsigned)((systemTick / 2000u) % 4u);

        if (phase == 0u || (!g->has_last_rmc && !g->has_last_gga)) {
            unsigned l = g->rx_lines > 9999u ? 9999u : g->rx_lines;
            unsigned e = g->rx_bad_csum > 99u ? 99u : g->rx_bad_csum;
            unsigned o = g->rx_overrun > 99u ? 99u : g->rx_overrun;
            snprintf(line, sizeof(line), "L%u E%u O%u %c", l, e, o,
                     g->pos_valid ? 'A' : 'V');
        } else if (phase == 1u) {
            /* 最多 20 字符, 与 128px 宽 / 6px 字宽匹配 */
            GPS_ViewBreakdown(line, 21u);
        } else if (phase == 2u && g->has_last_rmc) {
            snprintf(line, sizeof(line), "%.20s", g->last_rmc);
        } else if (g->has_last_gga) {
            snprintf(line, sizeof(line), "%.20s", g->last_gga);
        } else {
            unsigned l = g->rx_lines > 9999u ? 9999u : g->rx_lines;
            snprintf(line, sizeof(line), "L%u", l);
        }
    }
    GFX_DrawString(x, y + 7 * lh, line, GFX_COLOR_WHITE);
}
