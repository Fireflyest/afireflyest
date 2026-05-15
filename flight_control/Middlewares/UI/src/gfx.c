#include "gfx.h"
#include "font.h"
#include <string.h>
#include <stdlib.h>

// Framebuffer: 128 * 64 bits = 1024 bytes
uint8_t GFX_Buffer[OLED_WIDTH * OLED_HEIGHT / 8];

static void GFX_SetPixelInt(int x, int y, GFX_Color color);
static void fillTriangle2D(int x0, int y0, int x1, int y1, int x2, int y2, GFX_Color color);
static void drawCircle2D(int cx, int cy, int r, GFX_Color color);


void GFX_Init(void) {
    GFX_Clear();
}

void GFX_Clear(void) {
    memset(GFX_Buffer, 0, sizeof(GFX_Buffer));
}

uint8_t GFX_Update(void) {
    if (hasOLED == 0) {
        return 0; // OLED not present, skip update
    }

    #ifndef SYNC_FPS_TO_SCREEN_REFRESH
    if (OLED_IsDMABusy()) {
        return 0; // Previous DMA transfer still in progress
    }
    #endif

    for (int i = 0; i < 8; i++) {
        // Calculate pointer to the start of the current page in the buffer
        uint8_t* pPageData = GFX_Buffer + (i * OLED_WIDTH);
        
        // Send this page using the hardware driver
        OLED_UpdatePage_DMA(i, pPageData);

        #ifdef SYNC_FPS_TO_SCREEN_REFRESH
        // Wait for DMA to complete before proceeding to next page
        uint16_t timeout;
        for (timeout = 0x0FFF; timeout > 0 && OLED_IsDMABusy(); timeout--);
        #endif
    }
    return 1; // Update started
}

void GFX_DrawPixel(int x, int y, GFX_Color color) {
    GFX_SetPixelInt(x, y, color);
}

void GFX_DrawLine(int x0, int y0, int x1, int y1, GFX_Color color) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    for (;;) {
        GFX_SetPixelInt(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// 线型：GFX_LINE_STYLE_THIN = 单像素，GFX_LINE_STYLE_THICK = 2像素粗，GFX_LINE_STYLE_ALTERNATE = 二一像素变化
void GFX_DrawLineStyled(int x0, int y0, int x1, int y1, GFX_Color color, GFX_Line_Style style) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    int count = 0;

    for (;;) {
        if (style == GFX_LINE_STYLE_SINGLE_PIXEL) {
            GFX_SetPixelInt(x0, y0, color);
        } else if (style == GFX_LINE_STYLE_THICK_DOT) {
            if ((x0 % 2) == 0) {
                GFX_SetPixelInt(x0, y0, color);
            }
        } else if (style == GFX_LINE_STYLE_ALTERNATE_2_1PX) {
            if ((x0 % 4) < 3) {
                GFX_SetPixelInt(x0, y0, color);
            }
        }

        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
        count++;
    }
}

void GFX_DrawRect(int x, int y, int w, int h, GFX_Color color) {
    GFX_DrawLine(x, y, x + w - 1, y, color);
    GFX_DrawLine(x + w - 1, y, x + w - 1, y + h - 1, color);
    GFX_DrawLine(x + w - 1, y + h - 1, x, y + h - 1, color);
    GFX_DrawLine(x, y + h - 1, x, y, color);
}

void GFX_FillRect(int x, int y, int w, int h, GFX_Color color) {
    for (int i = x; i < x + w; i++) {
        for (int j = y; j < y + h; j++) {
            GFX_SetPixelInt(i, j, color);
        }
    }
}

void GFX_DrawChar(int x, int y, char c, GFX_Color color) {
    if (c < ' ' || c > '~') return;
    c -= ' ';
    for (int i = 0; i < 6; i++) {
        uint8_t line = F6x8[c][i];
        for (int j = 0; j < 8; j++) {
            if (line & (1 << j)) {
                GFX_SetPixelInt(x + i, y + j, color);
            } else if (color != GFX_COLOR_INVERT) {
                GFX_SetPixelInt(x + i, y + j, GFX_COLOR_BLACK);
            }
        }
    }
}

void GFX_DrawString(int x, int y, const char* str, GFX_Color color) {
    while (*str) {
        GFX_DrawChar(x, y, *str++, color);
        x += 6;
    }
}

void GFX_DrawBitmap(int x, int y, const uint8_t* bitmap, int w, int h, GFX_Color color) {
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            int byteIndex = (i + (j / 8) * w);
            int bitIndex = j % 8;
            if (bitmap[byteIndex] & (1 << bitIndex)) {
                GFX_SetPixelInt(x + i, y + j, color);
            } else if (color != GFX_COLOR_INVERT) {
                GFX_SetPixelInt(x + i, y + j, GFX_COLOR_BLACK);
            }
        }
    }
}


void GFX3D_DrawPixel(Point3D* p, GFX_Color color) {
    Math3D_Project(p, FOCAL_LENGTH_DEFAULT, CAMERA_Z_DEFAULT);
    GFX_SetPixelInt((int)p->x, (int)p->y, color);
}

void GFX3D_DrawLine(Point3D* p1, Point3D* p2, GFX_Color color) {
    Math3D_Project(p1, FOCAL_LENGTH_DEFAULT, CAMERA_Z_DEFAULT);
    Math3D_Project(p2, FOCAL_LENGTH_DEFAULT, CAMERA_Z_DEFAULT);
    GFX_DrawLine((int)p1->x, (int)p1->y, (int)p2->x, (int)p2->y, color);
}

void GFX3D_DrawLineStyled(Point3D* p1, Point3D* p2, GFX_Color color, GFX_Line_Style style) {
    Math3D_Project(p1, FOCAL_LENGTH_DEFAULT, CAMERA_Z_DEFAULT);
    Math3D_Project(p2, FOCAL_LENGTH_DEFAULT, CAMERA_Z_DEFAULT);
    GFX_DrawLineStyled((int)p1->x, (int)p1->y, (int)p2->x, (int)p2->y, color, style);
}

void GFX3D_DrawRect(Point3D* p, Vector3D* v, GFX_Color color) {
     Point3D p0 = *p;
    Point3D p1 = { p->x + v->x, p->y,           p->z           };
    Point3D p2 = { p->x + v->x, p->y + v->y,    p->z + v->z    };
    Point3D p3 = { p->x,        p->y + v->y,    p->z + v->z    };

    // simple camera-front check: skip drawing if all corners are behind camera
    float camz = CAMERA_Z_DEFAULT;
    int behind = 0;
    behind += (p0.z + camz) <= 0.0f;
    behind += (p1.z + camz) <= 0.0f;
    behind += (p2.z + camz) <= 0.0f;
    behind += (p3.z + camz) <= 0.0f;
    if (behind == 4) return; // all behind camera

    // project to screen ints (project_to_int makes a local copy)
    Math3D_Project(&p0, FOCAL_LENGTH_DEFAULT, CAMERA_Z_DEFAULT);
    Math3D_Project(&p1, FOCAL_LENGTH_DEFAULT, CAMERA_Z_DEFAULT);
    Math3D_Project(&p2, FOCAL_LENGTH_DEFAULT, CAMERA_Z_DEFAULT);
    Math3D_Project(&p3, FOCAL_LENGTH_DEFAULT, CAMERA_Z_DEFAULT);    

    // draw edges (will clip naturally in GFX_DrawLine/GFX_SetPixelInt)
    GFX_DrawLine((int)p0.x, (int)p0.y, (int)p1.x, (int)p1.y, color);
    GFX_DrawLine((int)p1.x, (int)p1.y, (int)p2.x, (int)p2.y, color);
    GFX_DrawLine((int)p2.x, (int)p2.y, (int)p3.x, (int)p3.y, color);
    GFX_DrawLine((int)p3.x, (int)p3.y, (int)p0.x, (int)p0.y, color);
}

void GFX3D_FillRect(Point3D* p, Vector3D* v, GFX_Color color) {
    Point3D p0 = *p;
    Point3D p1 = { p->x + v->x, p->y,           p->z           };
    Point3D p2 = { p->x + v->x, p->y + v->y,    p->z + v->z    };
    Point3D p3 = { p->x,        p->y + v->y,    p->z + v->z    };

    // cull if fully behind camera
    float camz = CAMERA_Z_DEFAULT;
    int behind = 0;
    behind += (p0.z + camz) <= 0.0f;
    behind += (p1.z + camz) <= 0.0f;
    behind += (p2.z + camz) <= 0.0f;
    behind += (p3.z + camz) <= 0.0f;
    if (behind == 4) return;

    // project copies to 2D
    Point3D t0 = p0, t1 = p1, t2 = p2, t3 = p3;
    Math3D_Project(&t0, FOCAL_LENGTH_DEFAULT, camz);
    Math3D_Project(&t1, FOCAL_LENGTH_DEFAULT, camz);
    Math3D_Project(&t2, FOCAL_LENGTH_DEFAULT, camz);
    Math3D_Project(&t3, FOCAL_LENGTH_DEFAULT, camz);

    // fill as two triangles
    fillTriangle2D((int)t0.x, (int)t0.y, (int)t1.x, (int)t1.y, (int)t2.x, (int)t2.y, color);
    fillTriangle2D((int)t0.x, (int)t0.y, (int)t2.x, (int)t2.y, (int)t3.x, (int)t3.y, color);
}

void GFX3D_DrawCube(Point3D* center, Vector3D* v, Quaternion* q, GFX_Color color) {
    Point3D corners[8];
    float cx = center->x, cy = center->y, cz = center->z;
    float hx = v->x, hy = v->y, hz = v->z;

    // build rotation matrix once (if q provided convert q->matrix else identity)
    Matrix4x4 R;
    if (q) {
        // convert quaternion to matrix quickly (avoid recomputing trig)
        // expand quaternion->matrix inline to save function call overhead
        float qw = q->w, qx = q->x, qy = q->y, qz = q->z;
        float qx2 = qx + qx, qy2 = qy + qy, qz2 = qz + qz;
        float xx = qx * qx2, yy = qy * qy2, zz = qz * qz2;
        float xy = qx * qy2, xz = qx * qz2, yz = qy * qz2;
        float wx = qw * qx2, wy = qw * qy2, wz = qw * qz2;

        R.m[0][0] = 1.0f - (yy + zz); R.m[0][1] = xy - wz;           R.m[0][2] = xz + wy;           R.m[0][3] = 0.0f;
        R.m[1][0] = xy + wz;           R.m[1][1] = 1.0f - (xx + zz); R.m[1][2] = yz - wx;           R.m[1][3] = 0.0f;
        R.m[2][0] = xz - wy;           R.m[2][1] = yz + wx;           R.m[2][2] = 1.0f - (xx + yy); R.m[2][3] = 0.0f;
        R.m[3][0] = 0.0f;              R.m[3][1] = 0.0f;              R.m[3][2] = 0.0f;              R.m[3][3] = 1.0f;
    } else {
        // identity
        R.m[0][0] = 1; R.m[0][1] = 0; R.m[0][2] = 0; R.m[0][3] = 0;
        R.m[1][0] = 0; R.m[1][1] = 1; R.m[1][2] = 0; R.m[1][3] = 0;
        R.m[2][0] = 0; R.m[2][1] = 0; R.m[2][2] = 1; R.m[2][3] = 0;
        R.m[3][0] = 0; R.m[3][1] = 0; R.m[3][2] = 0; R.m[3][3] = 1;
    }

    // rotate three basis half-vectors using matrix (3 * 3x3 mul)
    Point3D rex = { R.m[0][0]*hx + R.m[0][1]*0.0f + R.m[0][2]*0.0f,
                    R.m[1][0]*hx + R.m[1][1]*0.0f + R.m[1][2]*0.0f,
                    R.m[2][0]*hx + R.m[2][1]*0.0f + R.m[2][2]*0.0f };
    Point3D rey = { R.m[0][0]*0.0f + R.m[0][1]*hy + R.m[0][2]*0.0f,
                    R.m[1][0]*0.0f + R.m[1][1]*hy + R.m[1][2]*0.0f,
                    R.m[2][0]*0.0f + R.m[2][1]*hy + R.m[2][2]*0.0f };
    Point3D rez = { R.m[0][0]*0.0f + R.m[0][1]*0.0f + R.m[0][2]*hz,
                    R.m[1][0]*0.0f + R.m[1][1]*0.0f + R.m[1][2]*hz,
                    R.m[2][0]*0.0f + R.m[2][1]*0.0f + R.m[2][2]*hz };

    // explicit signs ordering matching edges
    const int signs[8][3] = {
        {-1,-1,-1}, {1,-1,-1}, {1,1,-1}, {-1,1,-1},
        {-1,-1,1},  {1,-1,1},  {1,1,1},  {-1,1,1}
    };

    // projection constants
    const float focal = FOCAL_LENGTH_DEFAULT;
    const float camz = CAMERA_Z_DEFAULT;
    const float half_w = (float)OLED_WIDTH * 0.5f;
    const float half_h = (float)OLED_HEIGHT * 0.5f;

    // build & project corners (inline projection to avoid extra calls)
    int px[8], py[8];
    for (int i = 0; i < 8; ++i) {
        int sx = signs[i][0], sy = signs[i][1], sz = signs[i][2];
        float x = cx + sx * rex.x + sy * rey.x + sz * rez.x;
        float y = cy + sx * rex.y + sy * rey.y + sz * rez.y;
        float z = cz + sx * rex.z + sy * rey.z + sz * rez.z;

        float denom = z + camz;
        if (denom <= 1e-6f) {
            // put off-screen if behind/too close
            px[i] = -999; py[i] = -999;
            continue;
        }
        float inv = 1.0f / denom; // replace with fast reciprocal if desired
        float scale = focal * inv;
        px[i] = (int)(x * scale + half_w + 0.5f);
        py[i] = (int)(y * scale + half_h + 0.5f);
    }

    // draw edges (indices fixed)
    const int edges[12][2] = {
        {0,1},{1,2},{2,3},{3,0},
        {4,5},{5,6},{6,7},{7,4},
        {0,4},{1,5},{2,6},{3,7}
    };
    for (int e = 0; e < 12; ++e) {
        int a = edges[e][0], b = edges[e][1];
        // skip lines with off-screen endpoints
        if (px[a] == -999 || px[b] == -999) continue;
        GFX_DrawLine(px[a], py[a], px[b], py[b], color);
    }
}


void GFX3D_DrawPyramid(Point3D* baseCenter, Vector3D* baseHalf, float height, Quaternion* q, GFX_Color color) {
    // 基底四角局部坐标（z = 0）
    Vector3D b0 = { baseHalf->x,  baseHalf->y, 0.0f };
    Vector3D b1 = {-baseHalf->x,  baseHalf->y, 0.0f };
    Vector3D b2 = {-baseHalf->x, -baseHalf->y, 0.0f };
    Vector3D b3 = { baseHalf->x, -baseHalf->y, 0.0f };
    Vector3D apex = { 0.0f, 0.0f, height };

    // 旋转到世界/视图空间（按惯例使用现有旋转函数）
    Math3D_QuatRotateVector(&b0, q);
    Math3D_QuatRotateVector(&b1, q);
    Math3D_QuatRotateVector(&b2, q);
    Math3D_QuatRotateVector(&b3, q);
    Math3D_QuatRotateVector(&apex, q);

    // 平移到中心位置
    Point3D p0 = *baseCenter; p0.x += b0.x; p0.y += b0.y; p0.z += b0.z;
    Point3D p1 = *baseCenter; p1.x += b1.x; p1.y += b1.y; p1.z += b1.z;
    Point3D p2 = *baseCenter; p2.x += b2.x; p2.y += b2.y; p2.z += b2.z;
    Point3D p3 = *baseCenter; p3.x += b3.x; p3.y += b3.y; p3.z += b3.z;
    Point3D pap = *baseCenter; pap.x += apex.x; pap.y += apex.y; pap.z += apex.z;

    // 绘制基底四边
    GFX3D_DrawLine(&p0, &p1, color);
    GFX3D_DrawLine(&p1, &p2, color);
    GFX3D_DrawLine(&p2, &p3, color);
    GFX3D_DrawLine(&p3, &p0, color);

    // 绘制四条侧棱
    GFX3D_DrawLine(&p0, &pap, color);
    GFX3D_DrawLine(&p1, &pap, color);
    GFX3D_DrawLine(&p2, &pap, color);
    GFX3D_DrawLine(&p3, &pap, color);
}

void GFX3D_DrawSphere(Point3D* center, Vector3D* v, GFX_Color color) {
    // v->x = radius
    Point3D c = *center;
    float camz = CAMERA_Z_DEFAULT;
    // simple cull if behind camera
    if ((c.z + camz) <= 0.0f) return;

    Point3D tc = c;
    Math3D_Project(&tc, FOCAL_LENGTH_DEFAULT, camz);
    float denom = center->z + camz;
    if (denom == 0.0f) denom = 1e-6f;
    float scale = FOCAL_LENGTH_DEFAULT / denom;
    int r = (int)(v->x * scale + 0.5f);
    if (r <= 0) return;
    drawCircle2D((int)tc.x, (int)tc.y, r, color);
}


static void GFX_SetPixelInt(int x, int y, GFX_Color color) {
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    
    uint16_t idx = x + (y / 8) * OLED_WIDTH;
    uint8_t bit = 1 << (y % 8);
    
    if (color == GFX_COLOR_WHITE) {
        GFX_Buffer[idx] |= bit;
    } else if (color == GFX_COLOR_BLACK) {
        GFX_Buffer[idx] &= ~bit;
    } else {
        GFX_Buffer[idx] ^= bit;
    }
}

/* helper: fill triangle in 2D using barycentric coordinates */
static void fillTriangle2D(int x0, int y0, int x1, int y1, int x2, int y2, GFX_Color color) {
    int minx = x0 < x1 ? (x0 < x2 ? x0 : x2) : (x1 < x2 ? x1 : x2);
    int maxx = x0 > x1 ? (x0 > x2 ? x0 : x2) : (x1 > x2 ? x1 : x2);
    int miny = y0 < y1 ? (y0 < y2 ? y0 : y2) : (y1 < y2 ? y1 : y2);
    int maxy = y0 > y1 ? (y0 > y2 ? y0 : y2) : (y1 > y2 ? y1 : y2);

    if (minx < 0) minx = 0;
    if (miny < 0) miny = 0;
    if (maxx >= OLED_WIDTH)  maxx = OLED_WIDTH - 1;
    if (maxy >= OLED_HEIGHT) maxy = OLED_HEIGHT - 1;

    float denom = (float)((y1 - y2)*(x0 - x2) + (x2 - x1)*(y0 - y2));
    if (denom == 0.0f) return; // degenerate

    for (int y = miny; y <= maxy; ++y) {
        for (int x = minx; x <= maxx; ++x) {
            float a = ((y1 - y2)*(x - x2) + (x2 - x1)*(y - y2)) / denom;
            float b = ((y2 - y0)*(x - x2) + (x0 - x2)*(y - y2)) / denom;
            float c = 1.0f - a - b;
            if (a >= 0.0f && b >= 0.0f && c >= 0.0f) {
                GFX_DrawPixel(x, y, color);
            }
        }
    }
}

/* helper: draw circle (midpoint) */
static void drawCircle2D(int cx, int cy, int r, GFX_Color color) {
    if (r <= 0) return;
    int x = r, y = 0;
    int err = 0;
    while (x >= y) {
        GFX_SetPixelInt(cx + x, cy + y, color);
        GFX_SetPixelInt(cx + y, cy + x, color);
        GFX_SetPixelInt(cx - y, cy + x, color);
        GFX_SetPixelInt(cx - x, cy + y, color);
        GFX_SetPixelInt(cx - x, cy - y, color);
        GFX_SetPixelInt(cx - y, cy - x, color);
        GFX_SetPixelInt(cx + y, cy - x, color);
        GFX_SetPixelInt(cx + x, cy - y, color);
        y++;
        if (err <= 0) {
            err += 2*y + 1;
        }
        if (err > 0) {
            x--;
            err -= 2*x + 1;
        }
    }
}