#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** Configure all board GPIO groups (does not start FreeRTOS tasks). */
void hexnet_board_gpio_init(void);

/** Start Run LED blink task (call early after gpio init). */
void hexnet_board_start_status_led(void);

#ifdef __cplusplus
}
#endif
