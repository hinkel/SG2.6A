#pragma once

bool Snr_init(void);
bool hcsr04_get_distancePWM(volatile int32_t *distance);
bool MaxBotix_get_distancePWM(volatile int32_t *distance);
bool DaddyW_get_i2c_distance(int32_t *distance);
