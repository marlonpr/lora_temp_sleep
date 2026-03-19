#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "ds18b20.h"
#include "lora.h"
#include <stdint.h>
#include <stdbool.h>

#include "esp_sleep.h"

#define TAG "LORA_TX_NODE"

// ---------------- CONFIG ----------------
#define DEVICE_ID       0       // 0 = master
#define FREQ_HZ         433000000
#define TX_INTERVAL_MS  5000
#define DS18B20_GPIO    GPIO_NUM_3
#define WDT_TIMEOUT_MS 10000 // 10 s watchdog
// ----------------------------------------

//RTC_DATA_ATTR uint8_t wake_stage = 0;
//RTC_DATA_ATTR int16_t last_temp = 0;
RTC_DATA_ATTR uint32_t tx_counter = 0;

static ds18b20_t sensor;
static volatile int16_t current_temp = 0;
static volatile bool temp_valid = false;
//static uint32_t tx_counter = 0;

// ---------------- WDT INIT ----------------
static void wdt_init(void)
{
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT_MS
    };

    static bool wdt_initialized = false;
    if(!wdt_initialized) {
        esp_err_t err = esp_task_wdt_init(&wdt_config);
        if(err == ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "WDT already initialized");
        } else {
            ESP_ERROR_CHECK(err);
        }
        wdt_initialized = true;
    }

    // Add main task (NULL = current task)
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
}

// ---------------- DS18B20 NON-BLOCKING TASK ----------------
typedef enum {
    TEMP_STATE_START_CONVERSION,
    TEMP_STATE_WAIT_CONVERSION,
    TEMP_STATE_READ
} temp_state_t;




static ds18b20_nonblocking_t nb_sensor;







// Example task for one sensor (can extend for multiple)
void temp_task(void *arg) {
    ds18b20_nonblocking_t *nb = (ds18b20_nonblocking_t *)arg;
    nb->state = DS18B20_STATE_IDLE;

    while (1) {
        esp_task_wdt_reset(); // feed WDT

        switch (nb->state) {
            case DS18B20_STATE_IDLE:
                if (ds18b20_start_conversion(nb->sensor) == ESP_OK) {
                    nb->conversion_start_tick = xTaskGetTickCount();
                    nb->state = DS18B20_STATE_WAIT_CONVERSION;
                } else {
                    nb->valid = false;
                    vTaskDelay(pdMS_TO_TICKS(1000)); // retry if sensor missing
                }
                break;

		case DS18B20_STATE_WAIT_CONVERSION:
		    if ((xTaskGetTickCount() - nb->conversion_start_tick) * portTICK_PERIOD_MS >= 750) {
		        if (ds18b20_read_temperature_nonblocking(nb->sensor, &nb->temperature) == ESP_OK) {
		            nb->valid = true;
		            current_temp = nb->temperature;   // <--- update global
		            temp_valid = true;
		        } else {
		            nb->valid = false;
		            temp_valid = false;
		            ESP_LOGW("DS18B20", "Failed read");
		        }
		        nb->state = DS18B20_STATE_IDLE;
		        vTaskDelay(pdMS_TO_TICKS(5000)); // optional 5s between readings
		    } else {
		        vTaskDelay(pdMS_TO_TICKS(50)); // short sleep to avoid blocking WDT
		    }
		    break;
        }
    }
}

// ---------------- LORA TX ----------------
static void send_packet(void)
{
    int16_t temp = current_temp;

    uint8_t buf[8] = {
        0xAA,
        DEVICE_ID,
        (temp >> 8) & 0xFF,
        temp & 0xFF,
        (tx_counter >> 24) & 0xFF,
        (tx_counter >> 16) & 0xFF,
        (tx_counter >> 8) & 0xFF,
        tx_counter & 0xFF
    };

    tx_counter++;

    // Wake radio to standby
    lora_idle();
    vTaskDelay(pdMS_TO_TICKS(1));

    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x80);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x80);
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);

    for(int i = 0; i < sizeof(buf); i++)
        lora_write_reg(REG_FIFO, buf[i]);

    lora_write_reg(REG_PAYLOAD_LENGTH, sizeof(buf));
    //lora_write_reg(REG_PA_CONFIG, 0x8F);

    // START TX
    lora_write_reg(REG_OP_MODE,
                   MODE_LONG_RANGE_MODE | MODE_TX);

    int64_t tx_start = esp_timer_get_time();

    while (1)
    {
        uint8_t irq = lora_read_reg(REG_IRQ_FLAGS);

        if (irq & IRQ_TX_DONE)
        {
            lora_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE);

            // ⭐ IMPORTANT: put radio into LOW POWER
            lora_write_reg(REG_OP_MODE,
                           MODE_LONG_RANGE_MODE | MODE_SLEEP);

            break;
        }

        if (esp_timer_get_time() - tx_start > 2000000)
        {
            ESP_LOGW(TAG, "TX timeout");

            lora_write_reg(REG_OP_MODE,
                           MODE_LONG_RANGE_MODE | MODE_SLEEP);

            lora_write_reg(REG_IRQ_FLAGS, 0xFF);
            return;
        }

        //esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    ESP_LOGI(TAG, "TX temp=%dC", temp);
}

static void lora_task(void *arg)
{
    while(1)
    {
        if(temp_valid) {
		    send_packet();
		} else {
		    ESP_LOGW(TAG, "Temperature not valid yet, skipping TX");
		}
        esp_task_wdt_reset(); // feed WDT each packet
        vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL_MS));
    }
}



//RTC_DATA_ATTR bool first_boot = true;
//RTC_DATA_ATTR int16_t last_temp = 0;
//RTC_DATA_ATTR uint32_t tx_counter = 0;




RTC_DATA_ATTR uint8_t wake_stage = 0;
RTC_DATA_ATTR int16_t last_temp = 0;

#define DS18B20_CONV_TIME_US   200000ULL        // 200 ms conversion 200000ULL
#define TX_INTERVAL_US         (10ULL * 1000000ULL)  // 30 seconds 30ULL

RTC_DATA_ATTR bool sensor_configured = false;

static ds18b20_t sensor;

//RTC_DATA_ATTR static bool lora_configured = false;

bool lora_is_alive(void)
{
    return (lora_read_reg(REG_VERSION) == 0x12);
}


void app_main(void)
{
    ds18b20_init(&sensor, DS18B20_GPIO);

    // ALWAYS reattach SPI after deep sleep
    lora_hw_init();

    //---------------------------------
    // Verify radio
    //---------------------------------
    if (lora_read_reg(REG_VERSION) != 0x12)
    {
        ESP_LOGW(TAG, "Reconfiguring LoRa");

        lora_configure();
        lora_set_frequency(FREQ_HZ);
        lora_sleep();
    }

    //---------------------------------
    // STAGE 0 — start conversion
    //---------------------------------
    if (wake_stage == 0)
    {
        ds18b20_start_conversion(&sensor);

        wake_stage = 1;

        esp_sleep_enable_timer_wakeup(200000ULL);
        esp_deep_sleep_start();

    }else
    //---------------------------------
    // STAGE 1 — transmit
    //---------------------------------	
	{
	    if (ds18b20_read_temperature_nonblocking(
	            &sensor, &last_temp) == ESP_OK)
	    {
	        lora_wake_fast();
	
	        current_temp = last_temp;
	        send_packet();
	
	        lora_sleep();
	    }
	
	    wake_stage = 0;
	
	    // ⭐ NEW
	    lora_shutdown();
	
	    esp_sleep_enable_timer_wakeup(TX_INTERVAL_US);
	    esp_deep_sleep_start();
	}
}



//==================== working v2 ==============================================
/*

RTC_DATA_ATTR uint8_t wake_stage = 0;
RTC_DATA_ATTR int16_t last_temp = 0;
#define DS18B20_CONV_TIME_US   200000ULL        // 200 ms conversion 200000ULL
#define TX_INTERVAL_US         (5ULL * 1000000ULL)  // 30 seconds 30ULL
RTC_DATA_ATTR bool sensor_configured = false;

static ds18b20_t sensor;


void app_main(void)
{
    ESP_LOGI(TAG, "Wake stage = %d", wake_stage);

    // Init sensor GPIO each boot
    if (ds18b20_init(&sensor, DS18B20_GPIO) != ESP_OK) {
        ESP_LOGE(TAG, "DS18B20 not detected");
    }

    // -------------------------------------------------
    // Configure resolution ONLY once (stored in RTC RAM)
    // -------------------------------------------------
    if (!sensor_configured)
    {
        ESP_LOGI(TAG, "Setting DS18B20 to 10-bit resolution");

        ds18b20_set_resolution(&sensor, DS18B20_RES_10BIT);

        sensor_configured = true;
    }

    // =================================================
    // STAGE 0 — START TEMPERATURE CONVERSION
    // =================================================
    if (wake_stage == 0)
    {
        ds18b20_start_conversion(&sensor);

        wake_stage = 1;

        // 10-bit conversion ≈187 ms
        //esp_sleep_enable_timer_wakeup(200000ULL); // 200 ms  DS18B20_CONV_TIME_US

		esp_sleep_enable_timer_wakeup(DS18B20_CONV_TIME_US);

        esp_deep_sleep_start();
    }

    // =================================================
    // STAGE 1 — READ + TRANSMIT
    // =================================================
    else
    {
        if (ds18b20_read_temperature_nonblocking(&sensor,
                                                 &last_temp) == ESP_OK)
        {
            if (lora_init() == ESP_OK)
            {
                lora_set_frequency(FREQ_HZ);

                current_temp = last_temp;

                // TX power
                lora_write_reg(REG_PA_CONFIG, 0x8F);

                send_packet();

                lora_sleep();   // lowest radio power
            }
        }
        else
        {
            ESP_LOGW(TAG, "Temperature read failed");
        }

        wake_stage = 0;

        // Sleep until next measurement interval
        //esp_sleep_enable_timer_wakeup(30ULL * 1000000ULL); // 30 s TX_INTERVAL_US

		esp_sleep_enable_timer_wakeup(TX_INTERVAL_US);

        esp_deep_sleep_start();
    }
}

*/






//==================== working v1 ==============================================
/*
void app_main(void)
{
    ESP_LOGI(TAG, "Wake stage = %d", wake_stage);

    ds18b20_init(&sensor, DS18B20_GPIO);

    if (wake_stage == 0)
    {
        // ---- START TEMP CONVERSION ----
        ds18b20_start_conversion(&sensor);

        wake_stage = 1;

        // sleep during 750 ms conversion
        //esp_sleep_enable_timer_wakeup(800000); // 800 ms
		esp_sleep_enable_timer_wakeup(30ULL * 1000000ULL);
        esp_deep_sleep_start();
    }
    else
    {
        // ---- READ TEMPERATURE ----
        if(ds18b20_read_temperature_int(&sensor, &last_temp) == ESP_OK)
        {
            if(lora_init() == ESP_OK)
            {
                lora_set_frequency(FREQ_HZ);
                current_temp = last_temp;

				lora_write_reg(REG_PA_CONFIG, 0x8F);

                send_packet();
                lora_idle();
            }
        }

        wake_stage = 0;

		lora_sleep();   // SX1278 sleep mode

        // sleep until next transmission
        esp_sleep_enable_timer_wakeup(30ULL * 1000000ULL);
        esp_deep_sleep_start();
    }
}

*/





























/*
void app_main(void)
{
    ESP_LOGI(TAG, "Wake");

    // minimal init
    lora_init();
    ds18b20_init(&sensor, GPIO_NUM_3);   // <-- your pin

    // ================= FIRST BOOT =================
    if(first_boot)
    {
        ESP_LOGI(TAG, "First boot: start conversion");

        ds18b20_start_conversion(&sensor);

        first_boot = false;

        esp_sleep_enable_timer_wakeup(5000000);
        esp_deep_sleep_start();
    }

    // ================= NORMAL WAKE =================
    int16_t temp;

    if(ds18b20_read_temperature_nonblocking(&sensor, &temp) == ESP_OK)
    {
        last_temp = temp;
    }

    current_temp = last_temp;

    send_packet();

    // start next conversion before sleep
    ds18b20_start_conversion(&sensor);

    ESP_LOGI(TAG, "Sleep");

    esp_sleep_enable_timer_wakeup(5000000);
    esp_deep_sleep_start();
}

*/



/*

void app_main(void)
{
    ESP_LOGI(TAG, "Wake stage = %d", wake_stage);

    ds18b20_init(&sensor, DS18B20_GPIO);

    if (wake_stage == 0)
    {
        // ---- START TEMP CONVERSION ----
        ds18b20_start_conversion(&sensor);

        wake_stage = 1;

        // sleep during 750 ms conversion
        //esp_sleep_enable_timer_wakeup(800000); // 800 ms
		esp_sleep_enable_timer_wakeup(30ULL * 1000000ULL);
        esp_deep_sleep_start();
    }
    else
    {
        // ---- READ TEMPERATURE ----
        if(ds18b20_read_temperature_int(&sensor, &last_temp) == ESP_OK)
        {
            if(lora_init() == ESP_OK)
            {
                lora_set_frequency(FREQ_HZ);
                current_temp = last_temp;

				lora_write_reg(REG_PA_CONFIG, 0x8F);

                send_packet();
                lora_idle();
            }
        }

        wake_stage = 0;

		lora_sleep();   // SX1278 sleep mode

        // sleep until next transmission
        esp_sleep_enable_timer_wakeup(30ULL * 1000000ULL);
        esp_deep_sleep_start();
    }
}
*/


/*
// ---------------- MAIN ----------------
void app_main(void)
{
    ESP_LOGI(TAG, "Starting isolated TX node...");

    // init WDT
    wdt_init(); // only initializes once and adds main task

    // init DS18B20
    ds18b20_init(&sensor, DS18B20_GPIO);

	nb_sensor.sensor = &sensor;
	nb_sensor.state = DS18B20_STATE_IDLE;
	nb_sensor.valid = false;
	nb_sensor.temperature = 0;


    // create temp task
    TaskHandle_t tempHandle = NULL;
	xTaskCreatePinnedToCore(temp_task, "TempTask", 2048, &nb_sensor, 2, &tempHandle, 0);
    // init LoRa
    if(lora_init() != ESP_OK) {
        ESP_LOGE(TAG, "LoRa init failed");
        return;
    }
    lora_set_frequency(FREQ_HZ);
    ESP_LOGI(TAG, "Device %d started on %.1f MHz", DEVICE_ID, FREQ_HZ / 1e6);

    // create LoRa TX task
    TaskHandle_t loraHandle = NULL;
    xTaskCreatePinnedToCore(lora_task, "LoraTask", 4096, NULL, 1, &loraHandle, 0);

    // add tasks to WDT **after creation**
    if(tempHandle) ESP_ERROR_CHECK(esp_task_wdt_add(tempHandle));
    if(loraHandle) ESP_ERROR_CHECK(esp_task_wdt_add(loraHandle));

    // main loop feeds WDT
    while(1) {
        esp_task_wdt_reset();  // feed main task WDT
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
*/
