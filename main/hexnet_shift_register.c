#include "hexnet_shift_register.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hexnet_bluetooth.h"
#include "hexnet_canbus.h"
#include "hexnet_io_map.h"

static void shift_register_write(uint16_t value)
{
    for (int i = 15; i >= 0; i--) {
        gpio_set_level(HEXNET_IO_SR_CLK_GPIO, 0);
        gpio_set_level(HEXNET_IO_SR_DATA_GPIO, (value >> i) & 0x01);
        gpio_set_level(HEXNET_IO_SR_CLK_GPIO, 1);
    }
    gpio_set_level(HEXNET_IO_SR_STB_GPIO, 0);
    gpio_set_level(HEXNET_IO_SR_STB_GPIO, 1);
}

void hexnet_shift_register_task(void *arg)
{
    (void)arg;
    while (1) {
        /* Tek kaynak: set_outputs() (BLE + platform ayni) */
        shift_register_write(get_outputs());
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
