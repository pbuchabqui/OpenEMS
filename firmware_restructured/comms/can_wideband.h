#ifndef TWAI_LAMBDA_H
#define TWAI_LAMBDA_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*twai_lambda_callback_t)(float lambda, uint32_t timestamp_ms, void *ctx);

esp_err_t twai_lambda_init(void);
void twai_lambda_deinit(void);
bool twai_lambda_get_latest(float *out_lambda, uint32_t *out_age_ms);
esp_err_t twai_lambda_register_callback(twai_lambda_callback_t cb, void *ctx);
void twai_lambda_unregister_callback(void);

#ifdef __cplusplus
}
#endif

#endif // TWAI_LAMBDA_H
