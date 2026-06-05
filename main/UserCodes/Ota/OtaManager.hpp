#pragma once

#include <cstdint>

#include "ota_manager.h"

#ifdef __cplusplus

enum class OtaState : uint8_t {
    Idle,
    Requested,
    Downloading,
    Writing,
    Verifying,
    SetBoot,
    Rebooting,
    Success,
    Failed,
};

class OtaManager {
public:
    static OtaManager &instance();

    void init();
    bool requestStart(const char *url, const char *version, bool force, const char *request_id);
    void getStatus(
        char *state_out,
        size_t state_out_len,
        int *progress_out,
        char *target_version_out,
        size_t target_version_out_len,
        char *last_error_out,
        size_t last_error_out_len) const;

private:
    OtaManager() = default;

    void setState(OtaState state);
    void setProgress(int pct);
    void setLastError(const char *msg);
    void runOtaTask(void *arg);
    bool performOta();

    static const char *stateToString(OtaState state);

    OtaState state_{OtaState::Idle};
    int progress_{0};
    char target_version_[128]{};
    char last_error_[160]{};
    char pending_url_[1024]{};
    char pending_version_[128]{};
    char pending_request_id_[48]{};
    bool pending_force_{false};
    bool ota_busy_{false};
};

#endif
