#include "fps.h"

__IO uint32_t systemTick;

static const uint32_t TARGET_FRAME_TIME = 5;   // 帧间隔ms
static uint32_t frameStartTick = 0;
static uint32_t logicCounter = 0;               // 程序循环计数
static uint32_t logicFps = 0;                   // 逻辑帧率
static uint32_t lastUpdateTick = 0;
static float dt = 0.01f;                        // 上一帧时间（秒）

void Delay_ms(__IO uint32_t nTime) {
    uint32_t start = systemTick;
    while((systemTick - start) < nTime);
}

void FPS_StartFrame(void) {
    uint32_t now = systemTick;
    if (frameStartTick != 0) {
        uint32_t elapsed = now - frameStartTick;
        dt = (float)elapsed / 1000.0f; // 上一帧时间，可在本帧逻辑中使用
    } else {
        dt = (float)TARGET_FRAME_TIME / 1000.0f;
    }
    frameStartTick = now;
}

void FPS_EndFrame(void) {
    // 1. 计算当前帧耗时
    uint32_t frameElapsed = systemTick - frameStartTick;
    
    // 2. 如果未到目标时间，则执行延迟
    if (frameElapsed < TARGET_FRAME_TIME) {
        Delay_ms(TARGET_FRAME_TIME - frameElapsed);
    }

    // 3. 统计逻辑（计算 FPS）
    logicCounter++;
    if (systemTick - lastUpdateTick >= 1000) {
        logicFps = logicCounter;
        logicCounter = 0;
        lastUpdateTick = systemTick;
    }
}

uint32_t FPS_Get(void) {
    return logicFps;
}

float FPS_GetDeltaTime(void) {
    return dt;
}

