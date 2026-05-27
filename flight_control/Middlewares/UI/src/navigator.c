#include "navigator.h"

UI_Logger logWindow;

static UI_Widget* widgets[16];
static uint8_t currentWindow;
static void UI_IMU_Draw(UI_Widget* widget);
static void UI_Cube_Draw(UI_Widget* widget);
static void UI_Pid_Draw(UI_Widget* widget);
static void UI_Accel_Calib_Draw(UI_Widget* widget);
static void UI_Gyro_Calib_Draw(UI_Widget* widget);
static void UI_Mag_Calib_Draw(UI_Widget* widget);
static void UI_Battery_Draw(UI_Widget* widget);

static UI_Window mpuWindow;
static UI_Window pidWindow;
static UI_Window homeWindow;
static UI_Window accelCalibWindow;
static UI_Window gyroCalibWindow;
static UI_Window magCalibWindow;
static UI_Window cubeWindow;
static UI_Window batteryWindow;


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

    UI_Window_Init(&cubeWindow, 0, 0, 128, 64);
    cubeWindow.base.draw = UI_Cube_Draw;
    widgets[WINDOW_CUBE] = (UI_Widget*)&cubeWindow;

    UI_Window_Init(&batteryWindow, 0, 0, 128, 64);
    batteryWindow.base.draw = UI_Battery_Draw;
    widgets[WINDOW_BATTERY] = (UI_Widget*)&batteryWindow;
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

    snprintf(line, sizeof(line), "MX %6d MY %6d", (int)(mag[0] * 1000), (int)(mag[1] * 1000));
    GFX_DrawString(x, y + 3 * lh, line, GFX_COLOR_WHITE);

    snprintf(line, sizeof(line), "MZ %6d", (int)(mag[2] * 1000));
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
    snprintf(line, sizeof(line), "S: %d", still);
    GFX_DrawString(0, 10, line, GFX_COLOR_WHITE);
    snprintf(line, sizeof(line), "F: %d", accel_clib_face);
    GFX_DrawString(0, 20, line, GFX_COLOR_WHITE);
    snprintf(line, sizeof(line), "V: %d.%d", (int)velocityZ, (int)(fabsf(velocityZ) * 10.0f) % 10);
    GFX_DrawString(0, 40, line, GFX_COLOR_WHITE);
    {
        float alt;
        Attitude_GetAltitude(&alt);
        float a = alt;
        int sign = (a < 0.0f) ? -1 : 1;
        if (sign < 0) a = -a;

        int intPart = (int)a;
        int fracPart = (int)((a - (float)intPart) * 10.0f + 0.5f); // 一位小数并四舍五入
        if (fracPart >= 10) { intPart += 1; fracPart = 0; }
        if (sign < 0) intPart = -intPart;

        char altLine[16];
        snprintf(altLine, sizeof(altLine), "H: %d.%d", intPart, fracPart);

        GFX_DrawString(0, 30, altLine, GFX_COLOR_WHITE);    // 整数部分
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

    // 更新历史
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
