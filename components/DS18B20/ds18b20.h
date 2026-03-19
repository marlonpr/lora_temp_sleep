#pragma once
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t pin;
} ds18b20_t;


esp_err_t ds18b20_init(ds18b20_t *sensor, gpio_num_t pin);
esp_err_t ds18b20_read_temperature(ds18b20_t *sensor, float *temperature);
esp_err_t ds18b20_read_temperature_int(ds18b20_t *sensor, int16_t *temperature);




#define DS18B20_CMD_CONVERT_T     0x44
#define DS18B20_CMD_READ_SCRATCH  0xBE
#define DS18B20_CMD_SKIP_ROM      0xCC

typedef enum {
    DS18B20_STATE_IDLE,
    DS18B20_STATE_WAIT_CONVERSION
} ds18b20_state_t;

typedef struct {
    ds18b20_t *sensor;
    ds18b20_state_t state;
    TickType_t conversion_start_tick;
    int16_t temperature;
    bool valid;
} ds18b20_nonblocking_t;



esp_err_t ds18b20_start_conversion(ds18b20_t *sensor);


esp_err_t ds18b20_read_temperature_nonblocking(ds18b20_t *sensor, int16_t *temp);



typedef enum {
    DS18B20_RES_9BIT  = 9,
    DS18B20_RES_10BIT = 10,
    DS18B20_RES_11BIT = 11,
    DS18B20_RES_12BIT = 12
} ds18b20_resolution_t;

esp_err_t ds18b20_set_resolution(ds18b20_t *sensor,
                                 ds18b20_resolution_t res);



#ifdef __cplusplus
}
#endif
