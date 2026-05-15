

#include "key.h"
#include <stddef.h>

static __IO uint8_t keyStatus = KEY_STATE_RELEASED;
static __IO uint8_t lastRawStatus = KEY_STATE_RELEASED;
static __IO uint8_t keyPressCount = 0;
static __IO uint8_t keyTiming = 0;
static void (*keyOnPress)(void) = NULL;
static void (*keyOnRelease)(void) = NULL;

uint8_t Key_Status() {
    return keyStatus;
}

uint8_t Key_PressConsume() {
    if (keyPressCount > 0) {
        keyPressCount--;
        return 1;
    }
    return 0;
}

void Key_Callback(void (*onPress)(void), void (*onRelease)(void)) {
    keyOnPress = onPress;
    keyOnRelease = onRelease;
}

void Key_Toggle_Handler(void) {
    uint8_t currentRawStatus = !(GPIO_KEY->IDR & GPIO_KEY_PIN);

    if (currentRawStatus == lastRawStatus) {
        if (keyTiming < KEY_DEBOUNCE_TIME && ++keyTiming == KEY_DEBOUNCE_TIME) {
            if (keyStatus != currentRawStatus) {
                if (keyStatus == KEY_STATE_RELEASED) {
                    keyPressCount++;
                    if (keyOnPress != NULL) {
                        keyOnPress();
                    }
                } else if (keyOnRelease != NULL) {
                    keyOnRelease();
                }
            }
            keyStatus = currentRawStatus;
        }
    } else {
        keyTiming = 0;
        lastRawStatus = currentRawStatus;
    }
}