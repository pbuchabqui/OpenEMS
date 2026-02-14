#include "engine_control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>

static const char* TAG = "S3_MAIN";

void app_main(void) {
    ESP_LOGI(TAG, "Starting ECU S3 Pro-Spec Engine Control");
    
    // Initialize engine control system
    esp_err_t err = engine_control_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Engine control init failed: %s", esp_err_to_name(err));
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    ESP_LOGI(TAG, "Engine control system initialized successfully");
    
    // Main loop - just keep the system running
    while (1) {
        // System is controlled by FreeRTOS tasks
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Log system status periodically
        engine_params_t params = {0};
        if (engine_control_get_engine_parameters(&params) == ESP_OK) {
            engine_perf_stats_t perf = {0};
            (void)engine_control_get_perf_stats(&perf);
            engine_injection_diag_t inj = {0};
            bool have_inj = (engine_control_get_injection_diag(&inj) == ESP_OK);
            ESP_LOGI(TAG, "System running - RPM: %" PRIu32 ", Load: %" PRIu32 " kPa, Limp: %s",
                     params.rpm,
                     params.load / 10,
                     params.is_limp_mode ? "YES" : "NO");
            ESP_LOGI(TAG,
                     "Perf planner(us) p95=%" PRIu32 " p99=%" PRIu32 " max=%" PRIu32 " miss=%" PRIu32 " | exec(us) p95=%" PRIu32 " p99=%" PRIu32 " max=%" PRIu32 " miss=%" PRIu32 " | q_ovr=%" PRIu32 " q_peak=%" PRIu32 " n=%" PRIu32,
                     perf.planner_p95_us, perf.planner_p99_us, perf.planner_max_us, perf.planner_deadline_miss,
                     perf.executor_p95_us, perf.executor_p99_us, perf.executor_max_us, perf.executor_deadline_miss,
                     perf.queue_overruns, perf.queue_depth_peak, perf.sample_count);
            if (have_inj) {
                ESP_LOGI(TAG,
                         "EOIT diag: target=%.1fdeg fallback=%.1fdeg normal=%.2f boundary=%.2f map=%s sync=%s SOI1=%.1f d1=%uus",
                         inj.eoit_target_deg,
                         inj.eoit_fallback_target_deg,
                         inj.normal_used,
                         inj.boundary,
                         inj.map_mode_enabled ? "ON" : "OFF",
                         inj.sync_acquired ? "FULL" : "PART",
                         inj.soi_deg[0],
                         (unsigned)inj.delay_us[0]);
            }
        } else {
            ESP_LOGW(TAG, "System running - engine parameters unavailable");
        }
    }
}
