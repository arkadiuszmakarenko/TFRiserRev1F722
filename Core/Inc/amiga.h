#ifndef __AMIGA_INCLUDED__
#define __AMIGA_INCLUDED__

#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include "main.h"

#define KEY_PRESSED_MAX 6
typedef struct {
	int lctrl;
	int lshift;
	int lalt;
	int lgui;
	int rctrl;
	int rshift;
	int ralt;
	int rgui;
	uint8_t keys[KEY_PRESSED_MAX];
} keyboard_code_t;

typedef enum {
	NO_LED = 0,
	LED_CAPS_LOCK_ON,
	LED_NUM_LOCK_ON,
	LED_SCROLL_LOCK_ON,
	LED_CAPS_LOCK_OFF,
	LED_NUM_LOCK_OFF,
	LED_SCROLL_LOCK_OFF,
	LED_RESET_BLINK,
} led_status_t;

typedef enum {
	NUM_LOCK_LED = (1 << 0),
	CAPS_LOCK_LED = (1 << 1),
	SCROLL_LOCK_LED = (1 << 2),
} keyboard_led_t;

void amikb_process_irq();
void amikb_init();
extern void amikb_startup(void);
extern led_status_t amikb_process(keyboard_code_t *data);
extern void amikb_notify(const char *notify);
extern void amikb_gpio_init(void);
extern void amikb_ready(int isready);
extern bool amikb_reset_check(void);
extern void amikb_reset(void);

#endif
