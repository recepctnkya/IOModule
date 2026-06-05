#include "hexnet_version.h"

#include "esp_app_format.h"
#include "esp_ota_ops.h"

const char *hexnet_firmware_version_string(void)
{
    const esp_app_desc_t *app = esp_ota_get_app_description();
    if (app != NULL) {
        const char *v = app->version;
        if (v != NULL && v[0] != '\0') {
            return v;
        }
    }
    return HEXNET_IO_RELEASE;
}
