#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"

#define HEXNET_HTTP_MIN_HEAP_BYTES 65536

bool hexnet_http_heap_ok(uint32_t min_free_bytes);
bool hexnet_http_lock(TickType_t timeout_ticks);
void hexnet_http_unlock(void);
