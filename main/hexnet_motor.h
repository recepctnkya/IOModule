#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void hexnet_motor_forward(void);
void hexnet_motor_backward(void);
void hexnet_motor_stop(void);
void hexnet_motor_control_task(void *arg);

#ifdef __cplusplus
}
#endif
