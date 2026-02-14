#ifndef SENSOR_PROCESSING_H
#define SENSOR_PROCESSING_H

#include "esp_err.h"
#include "esp_adc/adc_continuous.h"
#include "driver/gpio.h"

// Sensor channels
typedef enum {
    SENSOR_MAP = 0,      // Manifold Absolute Pressure
    SENSOR_TPS,          // Throttle Position Sensor
    SENSOR_CLT,          // Coolant Temperature
    SENSOR_IAT,          // Intake Air Temperature
    SENSOR_O2,           // Oxygen Sensor
    SENSOR_VBAT,         // Battery Voltage
    SENSOR_SPARE,        // Spare analog input
    SENSOR_COUNT
} sensor_channel_t;

// Sensor data structure
typedef struct {
    // Dados brutos do ADC
    uint16_t raw_adc[SENSOR_COUNT];           // Valores brutos 0-4095
    
    // Dados filtrados
    uint16_t map_kpa10;            // Pressão em kPa * 10
    uint16_t tps_percent;          // Posição da borboleta %
    int16_t clt_c;                 // Temperatura do líquido °C
    int16_t iat_c;                 // Temperatura do ar °C
    uint16_t o2_mv;                // Sonda O2 em mV
    uint16_t vbat_dv;              // Bateria em deci-Volts
    uint16_t spare_mv;             // Spare input em mV
    
    // Dados processados
    uint32_t engine_load;          // Carga do motor %
    uint16_t barometric_pressure;  // Pressão barométrica kPa
    bool tps_changed;              // Flag de mudança de posição
    
    // Estatísticas
    uint32_t sample_count;         // Contador de amostras
    uint32_t error_count;          // Contador de erros
} sensor_data_t;

// Sensor configuration
typedef struct {
    // Configuração ADC
    adc_atten_t attenuation;       // Atenuação do ADC
    adc_bitwidth_t width;          // Resolução
    uint32_t sample_rate_hz;       // Taxa de amostragem
    
    // Configuração filtros
    float map_filter_alpha;        // Alfa do filtro MAP
    float tps_filter_alpha;        // Alfa do filtro TPS
    float temp_filter_alpha;       // Alfa do filtro temperatura
    
    // Configuração sincronização
    bool map_sync_enabled;         // Sincronização com motor
    uint32_t map_sync_angle;       // Ângulo de sincronização
} sensor_config_t;

// Function prototypes
esp_err_t sensor_init(void);
esp_err_t sensor_deinit(void);
esp_err_t sensor_start(void);
esp_err_t sensor_stop(void);
esp_err_t sensor_get_data(sensor_data_t *data);
esp_err_t sensor_get_data_fast(sensor_data_t *data);
esp_err_t sensor_set_config(const sensor_config_t *config);
esp_err_t sensor_get_config(sensor_config_t *config);
esp_err_t sensor_calibrate(sensor_channel_t channel, uint16_t raw_value, float engineering_value);

#endif // SENSOR_PROCESSING_H
