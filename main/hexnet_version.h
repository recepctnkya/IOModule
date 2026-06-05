#pragma once

/** Geliştirme hattı (klasör adı ile uyumlu). */
#define HEXNET_IO_DEV_LINE "v0-50"

/**
 * Panel / OTA katalog etiketi. VERSION dosyası ve CMake project(VERSION) ile uyumlu tutun.
 * CMake: project(... VERSION "0.50.0") → çalışan imajda esp_app_desc.version = "0.50.0"
 */
#define HEXNET_IO_RELEASE "v0.50"

/** Telemetri device.fw — boş dönmez (OTA sonrası çalışan partition sürümü veya HEXNET_IO_RELEASE). */
const char *hexnet_firmware_version_string(void);
