#ifndef __GFX_H
#define __GFX_H

#include "stm32f4xx.h"
#include "oled.h"
#include "math3d.h"

#define SYNC_FPS_TO_SCREEN_REFRESH

// Colors
typedef enum {
    GFX_COLOR_BLACK = 0,
    GFX_COLOR_WHITE = 1,
    GFX_COLOR_INVERT = 2
} GFX_Color;

typedef enum {
    GFX_LINE_STYLE_SINGLE_PIXEL,    // 单像素线
    GFX_LINE_STYLE_THICK_DOT,       // 2像素粗线
    GFX_LINE_STYLE_ALTERNATE_2_1PX  // 2像素宽和1像素宽交替
} GFX_Line_Style;

// FrameBuffer
extern uint8_t GFX_Buffer[OLED_WIDTH * OLED_HEIGHT / 8];

// Initialization
void GFX_Init(void);

// Core drawing functions
void GFX_Clear(void);
uint8_t GFX_Update(void);
void GFX_DrawPixel(int x, int y, GFX_Color color);
void GFX_DrawLine(int x0, int y0, int x1, int y1, GFX_Color color);
void GFX_DrawLineStyled(int x0, int y0, int x1, int y1, GFX_Color color, GFX_Line_Style style);
void GFX_DrawRect(int x, int y, int w, int h, GFX_Color color);
void GFX_FillRect(int x, int y, int w, int h, GFX_Color color);
void GFX_DrawCircle(int x0, int y0, int r, GFX_Color color);
void GFX_DrawChar(int x, int y, char c, GFX_Color color);
void GFX_DrawString(int x, int y, const char* str, GFX_Color color);
void GFX_DrawBitmap(int x, int y, const uint8_t* bitmap, int w, int h, GFX_Color color);

// 3d
void GFX3D_DrawPixel(Point3D* p, GFX_Color color);
void GFX3D_DrawLine(Point3D* p1, Point3D* p2, GFX_Color color);
void GFX3D_DrawLineStyled(Point3D* p1, Point3D* p2, GFX_Color color, GFX_Line_Style style);
void GFX3D_DrawRect(Point3D* p, Vector3D* v, GFX_Color color);
void GFX3D_FillRect(Point3D* p, Vector3D* v, GFX_Color color);
void GFX3D_DrawCube(Point3D* center, Vector3D* v, Quaternion* q, GFX_Color color);
void GFX3D_DrawPyramid(Point3D* baseCenter, Vector3D* baseHalf, float height, Quaternion* q, GFX_Color color);
void GFX3D_DrawSphere(Point3D* center, Vector3D* v, GFX_Color color);

#endif /* __GFX_H */
