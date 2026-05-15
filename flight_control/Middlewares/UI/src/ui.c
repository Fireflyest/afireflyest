#include "ui.h"
#include <stddef.h>
#include <string.h>

UI_Widget* g_focusWidget = NULL;

void UI_Init(void) {
    g_focusWidget = NULL;
}

void UI_AddChild(UI_Widget* parent, UI_Widget* child) {
    child->parent = parent;
    child->next = NULL;
    
    if (parent->child == NULL) {
        parent->child = child;
    } else {
        UI_Widget* p = parent->child;
        while (p->next != NULL) {
            p = p->next;
        }
        p->next = child;
    }
}

// Global offset for rendering
static int g_uiOffsetX = 0;
static int g_uiOffsetY = 0;

void UI_DrawWidget(UI_Widget* widget) {
    if (widget->draw) {
        widget->draw(widget);
    }
}

void UI_DrawTree_Recursive(UI_Widget* root, int x, int y) {
    if (!root->isVisible) return;
    
    int oldX = g_uiOffsetX;
    int oldY = g_uiOffsetY;
    
    g_uiOffsetX = x + root->x;
    g_uiOffsetY = y + root->y;
    
    UI_DrawWidget(root);
    
    UI_Widget* child = root->child;
    while (child) {
        UI_DrawTree_Recursive(child, g_uiOffsetX, g_uiOffsetY);
        child = child->next;
    }
    
    g_uiOffsetX = oldX;
    g_uiOffsetY = oldY;
}

void UI_DrawTree(UI_Widget* root, int absX, int absY) {
    UI_DrawTree_Recursive(root, absX, absY);
}

void UI_SetFocus(UI_Widget* widget) {
    g_focusWidget = widget;
}

void UI_ProcessKey(uint8_t key) {
    if (g_focusWidget && g_focusWidget->onKey) {
        g_focusWidget->onKey(g_focusWidget, key);
    }
}

// --- Widget Implementations ---

void UI_Window_Draw(UI_Widget* w) {
    GFX_DrawRect(g_uiOffsetX - 1, g_uiOffsetY - 1, w->w + 2, w->h + 2, GFX_COLOR_WHITE);
}

void UI_Window_Init(UI_Window* window, int x, int y, int w, int h) {
    window->base.x = x;
    window->base.y = y;
    window->base.w = w;
    window->base.h = h;
    window->base.parent = NULL;
    window->base.child = NULL;
    window->base.next = NULL;
    window->base.draw = UI_Window_Draw;
    window->base.onKey = NULL;
    window->base.isVisible = 1;
    window->base.isFocusable = 0;
}

void UI_Label_Draw(UI_Widget* w) {
    UI_Label* label = (UI_Label*)w;
    GFX_DrawString(g_uiOffsetX, g_uiOffsetY, label->text, GFX_COLOR_WHITE);
}

void UI_Label_Init(UI_Label* label, int x, int y, const char* text) {
    label->base.x = x;
    label->base.y = y;
    label->base.w = 0; // Auto width?
    label->base.h = 8;
    label->base.parent = NULL;
    label->base.child = NULL;
    label->base.next = NULL;
    label->base.draw = UI_Label_Draw;
    label->base.onKey = NULL;
    label->base.isVisible = 1;
    label->base.isFocusable = 0;
    label->text = text;
}

void UI_Button_Draw(UI_Widget* w) {
    UI_Button* btn = (UI_Button*)w;
    GFX_Color color = (w == g_focusWidget) ? GFX_COLOR_INVERT : GFX_COLOR_WHITE;
    
    GFX_DrawRect(g_uiOffsetX, g_uiOffsetY, w->w, w->h, GFX_COLOR_WHITE);
    GFX_DrawString(g_uiOffsetX + 2, g_uiOffsetY + 2, btn->text, color);
}

void UI_Button_OnKey(UI_Widget* w, uint8_t key) {
    UI_Button* btn = (UI_Button*)w;
    if (key == 1 && btn->onClick) { // Assume key 1 is Enter/Click
        btn->onClick();
    }
}

void UI_Button_Init(UI_Button* button, int x, int y, int w, int h, const char* text, void (*onClick)(void)) {
    button->base.x = x;
    button->base.y = y;
    button->base.w = w;
    button->base.h = h;
    button->base.parent = NULL;
    button->base.child = NULL;
    button->base.next = NULL;
    button->base.draw = UI_Button_Draw;
    button->base.onKey = UI_Button_OnKey;
    button->base.isVisible = 1;
    button->base.isFocusable = 1;
    button->text = text;
    button->onClick = onClick;
}


void UI_Logger_AnimStep(UI_Logger* logger) {
    if (logger->animLineIndex >= logger->lineCount) return;

    if (logger->animWaitFrame > 0) {
        logger->animWaitFrame--;
        return;
    }

    int len = strlen(logger->buffer[logger->animLineIndex]);
    if (logger->animCharIndex < len) {
        logger->animCharIndex += 3;
        if (logger->animCharIndex > len) logger->animCharIndex = len;
    } else {
        logger->animLineIndex++;
        logger->animCharIndex = 0;
        logger->animWaitFrame = 5;
    }
}

static void UI_Logger_Draw(UI_Widget* widget) {
    UI_Logger* logger = (UI_Logger*)widget;

    UI_Logger_AnimStep(logger);

    int startX = widget->x;
    int startY = widget->y;
    int lineHeight = 9;

    if (logger->lineCount == 0) return;

    // draw fully shown lines
    for (int i = 0; i < logger->animLineIndex && i < logger->maxVisibleLines; i++) {
        GFX_DrawString(startX, startY + (i * lineHeight), logger->buffer[i], GFX_COLOR_WHITE);
    }

    static uint16_t ui_blinkCounter = 0;
    ui_blinkCounter++;
    // draw current animating line (partial) with cursor
    if (logger->animLineIndex < logger->lineCount && logger->animLineIndex < logger->maxVisibleLines) {
        int i = logger->animLineIndex;
        int fullLen = strlen(logger->buffer[i]);
        int showLen = logger->animCharIndex;
        if (showLen > fullLen) showLen = fullLen;

        if (showLen > 0) {
            char temp[TEXTLIST_MAX_CHARS + 1] = {0};
            strncpy(temp, logger->buffer[i], showLen);
            temp[showLen] = '\0';
            GFX_DrawString(startX, startY + (i * lineHeight), temp, GFX_COLOR_WHITE);
        }

        // draw blinking vertical cursor (no character concat)
        if ((ui_blinkCounter & 0x10) == 0) {
            int cursorX = startX + showLen * 6 + 1;
            int y1 = startY + (i * lineHeight) - 1;
            int y2 = y1 + lineHeight;
            GFX_DrawLine(cursorX, y1, cursorX, y2, GFX_COLOR_WHITE);
        }

    } else if (logger->animLineIndex >= logger->lineCount) {
        // animation finished: draw all lines and cursor at end of last line
        for (int i = 0; i < logger->lineCount && i < logger->maxVisibleLines; i++) {
            GFX_DrawString(startX, startY + (i * lineHeight), logger->buffer[i], GFX_COLOR_WHITE);
        }
        int last = logger->lineCount - 1;
        int fullLen = strlen(logger->buffer[last]);
        if ((ui_blinkCounter & 0x10) == 0) {
            int cursorX = startX + fullLen * 6 + 1;
            int y1 = startY + (last * lineHeight) - 1;
            int y2 = y1 + lineHeight;
            GFX_DrawLine(cursorX, y1, cursorX, y2, GFX_COLOR_WHITE);
        }
    }
}

void UI_Logger_Init(UI_Logger* logger, int x, int y, int w, int h) {
    logger->base.x = x;
    logger->base.y = y;
    logger->base.w = w;
    logger->base.h = h;
    logger->base.parent = NULL;
    logger->base.child = NULL;
    logger->base.next = NULL;
    logger->base.isVisible = 1;
    logger->base.isFocusable = 0;
    logger->base.draw = UI_Logger_Draw;
    logger->base.onKey = NULL;

    logger->lineCount = 0;
    logger->maxVisibleLines = (h - 4) / 8; 
    if (logger->maxVisibleLines > TEXTLIST_MAX_LINES) {
        logger->maxVisibleLines = TEXTLIST_MAX_LINES;
    }
    
    memset(logger->buffer, 0, sizeof(logger->buffer));
}

void UI_Logger_AddLine(UI_Logger* logger, const char* text) {
    if (logger->lineCount < logger->maxVisibleLines) {
        strncpy(logger->buffer[logger->lineCount], text, TEXTLIST_MAX_CHARS - 1);
        logger->buffer[logger->lineCount][TEXTLIST_MAX_CHARS - 1] = '\0';
        logger->lineCount++;
    } else {
        for (int i = 0; i < logger->maxVisibleLines - 1; i++) {
            strcpy(logger->buffer[i], logger->buffer[i+1]);
        }
        
        strncpy(logger->buffer[logger->maxVisibleLines - 1], text, TEXTLIST_MAX_CHARS - 1);
        logger->buffer[logger->maxVisibleLines - 1][TEXTLIST_MAX_CHARS - 1] = '\0';
    }
}

void UI_Logger_Clear(UI_Logger* logger) {
    logger->lineCount = 0;
    logger->animCharIndex = 0;
    logger->animLineIndex = 0;
    logger->animWaitFrame = 0;
    memset(logger->buffer, 0, sizeof(logger->buffer));
}


