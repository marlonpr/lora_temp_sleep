#include "ds18b20.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

#define DS18B20_CMD_CONVERT_T     0x44
#define DS18B20_CMD_READ_SCRATCH  0xBE
#define DS18B20_CMD_SKIP_ROM      0xCC

static const char *TAG = "DS18B20";

static void ds18b20_write_bit(ds18b20_t *dev, int bit) {
    gpio_set_direction(dev->pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dev->pin, 0);
    esp_rom_delay_us(bit ? 6 : 60);
    gpio_set_level(dev->pin, 1);
    if (bit) esp_rom_delay_us(64);
    else esp_rom_delay_us(10);
}

static int ds18b20_read_bit(ds18b20_t *dev) {
    int bit;
    gpio_set_direction(dev->pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dev->pin, 0);
    esp_rom_delay_us(6);
    gpio_set_level(dev->pin, 1);
    gpio_set_direction(dev->pin, GPIO_MODE_INPUT);
    esp_rom_delay_us(9);
    bit = gpio_get_level(dev->pin);
    esp_rom_delay_us(55);
    return bit;
}

static void ds18b20_write_byte(ds18b20_t *dev, uint8_t data) {
    for (int i = 0; i < 8; i++) {
        ds18b20_write_bit(dev, data & 1);
        data >>= 1;
    }
}

static uint8_t ds18b20_read_byte(ds18b20_t *dev) {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        data >>= 1;
        if (ds18b20_read_bit(dev)) data |= 0x80;
    }
    return data;
}

static bool ds18b20_reset(ds18b20_t *dev) {
    gpio_set_direction(dev->pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dev->pin, 0);
    esp_rom_delay_us(480);
    gpio_set_level(dev->pin, 1);
    gpio_set_direction(dev->pin, GPIO_MODE_INPUT);
    esp_rom_delay_us(70);
    int presence = !gpio_get_level(dev->pin);
    esp_rom_delay_us(410);
    return presence;
}

esp_err_t ds18b20_init(ds18b20_t *sensor, gpio_num_t pin) {
    if (!sensor) return ESP_ERR_INVALID_ARG;
    sensor->pin = pin;
    gpio_reset_pin(pin);
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY); // external 4.7kΩ recommended
    return ds18b20_reset(sensor) ? ESP_OK : ESP_FAIL;
}

esp_err_t ds18b20_read_temperature(ds18b20_t *sensor, float *temperature) {
    if (!sensor || !temperature) return ESP_ERR_INVALID_ARG;

    if (!ds18b20_reset(sensor)) {
        ESP_LOGE(TAG, "Sensor not found");
        return ESP_FAIL;
    }

    ds18b20_write_byte(sensor, DS18B20_CMD_SKIP_ROM);
    ds18b20_write_byte(sensor, DS18B20_CMD_CONVERT_T);
    vTaskDelay(pdMS_TO_TICKS(750)); // wait for conversion

    if (!ds18b20_reset(sensor)) return ESP_FAIL;
    ds18b20_write_byte(sensor, DS18B20_CMD_SKIP_ROM);
    ds18b20_write_byte(sensor, DS18B20_CMD_READ_SCRATCH);

    uint8_t lsb = ds18b20_read_byte(sensor);
    uint8_t msb = ds18b20_read_byte(sensor);
    int16_t raw_temp = (msb << 8) | lsb;

    *temperature = (float)raw_temp / 16.0f;
    return ESP_OK;
}

esp_err_t ds18b20_read_temperature_int(ds18b20_t *sensor, int16_t *temperature) {
    if (!sensor || !temperature) return ESP_ERR_INVALID_ARG;

    if (!ds18b20_reset(sensor)) {
        ESP_LOGE("DS18B20", "Sensor not found");
        return ESP_FAIL;
    }

    ds18b20_write_byte(sensor, DS18B20_CMD_SKIP_ROM);
    ds18b20_write_byte(sensor, DS18B20_CMD_CONVERT_T);
    vTaskDelay(pdMS_TO_TICKS(750)); // wait conversion

    if (!ds18b20_reset(sensor)) return ESP_FAIL;
    ds18b20_write_byte(sensor, DS18B20_CMD_SKIP_ROM);
    ds18b20_write_byte(sensor, DS18B20_CMD_READ_SCRATCH);

    uint8_t lsb = ds18b20_read_byte(sensor);
    uint8_t msb = ds18b20_read_byte(sensor);
    int16_t raw_temp = (msb << 8) | lsb;

    *temperature = raw_temp >> 4;  // Divide by 16, truncate fractional part

    return ESP_OK;
}










// Start conversion (non-blocking)
esp_err_t ds18b20_start_conversion(ds18b20_t *sensor) {
    if (!ds18b20_reset(sensor)) return ESP_FAIL;
    ds18b20_write_byte(sensor, DS18B20_CMD_SKIP_ROM);
    ds18b20_write_byte(sensor, DS18B20_CMD_CONVERT_T);
    return ESP_OK;
}

// Read temperature after conversion
esp_err_t ds18b20_read_temperature_nonblocking(ds18b20_t *sensor, int16_t *temp) {
    if (!ds18b20_reset(sensor)) return ESP_FAIL;

    ds18b20_write_byte(sensor, DS18B20_CMD_SKIP_ROM);
    ds18b20_write_byte(sensor, DS18B20_CMD_READ_SCRATCH);

    uint8_t lsb = ds18b20_read_byte(sensor);
    uint8_t msb = ds18b20_read_byte(sensor);
    //*temp = ((msb << 8) | lsb) >> 4;
	*temp = ((int16_t)((msb << 8) | lsb)) >> 4;
    return ESP_OK;
}


esp_err_t ds18b20_set_resolution(ds18b20_t *sensor,
                                 ds18b20_resolution_t res)
{
    uint8_t config;

    switch(res)
    {
        case DS18B20_RES_9BIT:  config = 0x1F; break;
        case DS18B20_RES_10BIT: config = 0x3F; break;
        case DS18B20_RES_11BIT: config = 0x5F; break;
        case DS18B20_RES_12BIT: config = 0x7F; break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    // Reset + presence detect
    if (!ds18b20_reset(sensor))
        return ESP_FAIL;

    // Skip ROM (single device bus)
    ds18b20_write_byte(sensor, DS18B20_CMD_SKIP_ROM);

    // Write Scratchpad command
    ds18b20_write_byte(sensor, 0x4E);

    // TH register (unused)
    ds18b20_write_byte(sensor, 0x4B);

    // TL register (unused)
    ds18b20_write_byte(sensor, 0x46);

    // Configuration register
    ds18b20_write_byte(sensor, config);

    return ESP_OK;
}