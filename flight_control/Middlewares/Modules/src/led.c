# include "led.h"

static __IO uint8_t ledToggleCmd = LED_TOGGLE_CMD_ALL_OFF;
static __IO uint8_t ledToggleCount = 0;

void LED_On(void) {
    ledToggleCmd = LED_TOGGLE_CMD_ALL_ON;
    ledToggleCount = sizeof(ledToggleCmd) * 8;
}

void LED_Off(void) {
    ledToggleCmd = LED_TOGGLE_CMD_ALL_OFF;
    ledToggleCount = sizeof(ledToggleCmd) * 8;
}

void LED_Blink(uint8_t toggleCmd, uint8_t times) {
    ledToggleCmd = toggleCmd;
    ledToggleCount = sizeof(ledToggleCmd) * 8 * times;
}

void LED_Toggle_Handler(void) {
    static uint8_t ledTiming = 0;
    if (ledToggleCount > 0) {
        ledTiming++;
        if (ledTiming >= LED_TOGGLE_INTERVAL) {
            ledTiming = 0;

            if (ledToggleCmd & 0x01) {
              GPIO_LED->BSRRL = GPIO_LED_PIN;
            } else {
              GPIO_LED->BSRRH = GPIO_LED_PIN;
            }

            uint8_t lastBit = ledToggleCmd & 0x01;
            ledToggleCmd = (ledToggleCmd >> 1) | (lastBit << 7);

            ledToggleCount--;
        }
    } else {
        ledTiming = LED_TOGGLE_INTERVAL; 
    }
}