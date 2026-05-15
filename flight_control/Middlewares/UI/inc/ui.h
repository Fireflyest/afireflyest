#ifndef __UI_H
#define __UI_H

#include "gfx.h"

#define TEXTLIST_MAX_LINES 10
#define TEXTLIST_MAX_CHARS 20



typedef struct UI_Widget UI_Widget;

struct UI_Widget {
    int x, y, w, h;           // Relative coordinates
    UI_Widget* parent;
    UI_Widget* child;
    UI_Widget* next;
    
    // Virtual functions
    void (*draw)(UI_Widget*);
    void (*onKey)(UI_Widget*, uint8_t key);
    
    uint8_t isVisible;
    uint8_t isFocusable;
};



// Global Focus
extern UI_Widget* g_focusWidget;


// Standard Widgets
typedef struct {
    UI_Widget base;
    const char* text;
} UI_Label;

typedef struct {
    UI_Widget base;
    const char* text;
    void (*onClick)(void);
} UI_Button;

typedef struct {
    UI_Widget base;
    char buffer[TEXTLIST_MAX_LINES][TEXTLIST_MAX_CHARS];
    int lineCount;
    int maxVisibleLines;
    int animCharIndex;
    int animLineIndex;
    int animWaitFrame;
} UI_Logger;

typedef struct {
    UI_Widget base;
} UI_Window;

// Core functions
void UI_Init(void);
void UI_AddChild(UI_Widget* parent, UI_Widget* child);
void UI_DrawTree(UI_Widget* root, int absX, int absY);
void UI_ProcessKey(uint8_t key);
void UI_SetFocus(UI_Widget* widget);

// Widget Constructors
void UI_Window_Init(UI_Window* window, int x, int y, int w, int h);
void UI_Label_Init(UI_Label* label, int x, int y, const char* text);
void UI_Button_Init(UI_Button* button, int x, int y, int w, int h, const char* text, void (*onClick)(void));

void UI_Logger_Init(UI_Logger* logger, int x, int y, int w, int h);
void UI_Logger_AddLine(UI_Logger* logger, const char* text);
void UI_Logger_Clear(UI_Logger* logger);

#endif /* __UI_H */
