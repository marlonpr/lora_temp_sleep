#include "lora.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


static const char *TAG = "LORA";
static spi_device_handle_t spi;

void lora_write_reg(uint8_t reg, uint8_t val) {
    uint8_t out[2] = { (reg | 0x80), val };
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8*2;               // 16 bits
    t.tx_buffer = out;            // use tx_buffer (no SPI_TRANS_USE_TXDATA)
    esp_err_t err = spi_device_transmit(spi, &t);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SPI write reg 0x%02X failed", reg);
    }
}

uint8_t lora_read_reg(uint8_t reg) {
    uint8_t out[2] = { reg & 0x7F, 0x00 };
    uint8_t in[2] = {0,0};
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8*2;               // 16 bits
    t.tx_buffer = out;
    t.rx_buffer = in;
    esp_err_t err = spi_device_transmit(spi, &t);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SPI read reg 0x%02X failed", reg);
        return 0;
    }
    return in[1];
}


static void lora_reset(void) {
    gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// --- API ---
esp_err_t lora_init(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num = LORA_MISO,
        .mosi_io_num = LORA_MOSI,
        .sclk_io_num = LORA_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1*1000*1000, // safer for initial test
        .mode = 0,
        .spics_io_num = LORA_CS,
        .queue_size = 1
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    gpio_reset_pin(LORA_RST);
    lora_reset();

    uint8_t version = lora_read_reg(REG_VERSION);
    if (version != 0x12) {
        ESP_LOGE(TAG, "SX1278 not found (version=0x%02X)", version);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "SX1278 found (version=0x%02X)", version);

    // Sleep then standby
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

    // Modem config: BW125, SF7, CRC on
    lora_write_reg(REG_MODEM_CONFIG_1, 0x72);
    lora_write_reg(REG_MODEM_CONFIG_2, 0x74);
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
    lora_write_reg(REG_PA_CONFIG, 0x8F);

	// FIFO base addresses — use a TX base common for SX127x
	lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x80);
	lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);
	lora_write_reg(REG_FIFO_ADDR_PTR, 0x80);
	
	ESP_LOGI(TAG, "FIFO_TX=0x%02X PTR=0x%02X", lora_read_reg(REG_FIFO_TX_BASE_ADDR), lora_read_reg(REG_FIFO_ADDR_PTR));



    return ESP_OK;
}

void lora_set_frequency(long frequency) {
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    lora_write_reg(REG_FRF_MSB, (frf >> 16) & 0xFF);
    lora_write_reg(REG_FRF_MID, (frf >> 8) & 0xFF);
    lora_write_reg(REG_FRF_LSB, (frf >> 0) & 0xFF);
}

void lora_send_packet(const uint8_t *data, int len) {
    // Standby
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Set TX base and pointer
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x80);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x80);

    // Clear IRQs
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);

    // Write payload
    for (int i = 0; i < len; i++) {
        lora_write_reg(REG_FIFO, data[i]);
    }
    lora_write_reg(REG_PAYLOAD_LENGTH, len);

    // Ensure PA config (PA_BOOST) if your module needs it
    lora_write_reg(REG_PA_CONFIG, 0x8F);

    // Start TX
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    // Poll for TX_DONE with timeout
    int timeout = 200; // ms
    while (!(lora_read_reg(REG_IRQ_FLAGS) & 0x08) && timeout--) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    uint8_t flags = lora_read_reg(REG_IRQ_FLAGS);
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
}


void lora_enable_rx(void) {
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0);
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

int lora_receive_packet(uint8_t *buf, int maxlen) {
    int len = lora_read_reg(REG_RX_NB_BYTES);
    if (len > 0) {
        if (len > maxlen) len = maxlen;
        uint8_t fifo_addr = lora_read_reg(REG_FIFO_RX_CURRENT);
        lora_write_reg(REG_FIFO_ADDR_PTR, fifo_addr);
        for (int i = 0; i < len; i++) {
            buf[i] = lora_read_reg(REG_FIFO);
        }
        lora_write_reg(REG_IRQ_FLAGS, 0xFF);
        return len;
    }
    return 0;
}

void lora_sleep(void)
{
    uint8_t op = lora_read_reg(REG_OP_MODE);
    lora_write_reg(REG_OP_MODE, (op & 0xF8) | MODE_SLEEP);
}

void lora_idle(void)
{
    uint8_t op = lora_read_reg(REG_OP_MODE);
    lora_write_reg(REG_OP_MODE, (op & 0xF8) | MODE_STDBY);
}






void lora_hw_init(void)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = LORA_MISO,
        .mosi_io_num = LORA_MOSI,
        .sclk_io_num = LORA_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8000000, // FAST
        .mode = 0,
        .spics_io_num = LORA_CS,
        .queue_size = 1
    };

    spi_bus_add_device(SPI2_HOST, &devcfg, &spi);

    gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
}


esp_err_t lora_configure(void)
{
    lora_reset();

    uint8_t version = lora_read_reg(REG_VERSION);
    if (version != 0x12)
        return ESP_FAIL;

    lora_write_reg(REG_OP_MODE,
                   MODE_LONG_RANGE_MODE | MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(5));

    lora_write_reg(REG_OP_MODE,
                   MODE_LONG_RANGE_MODE | MODE_STDBY);

    lora_write_reg(REG_MODEM_CONFIG_1, 0x72);
    lora_write_reg(REG_MODEM_CONFIG_2, 0x74);
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
    lora_write_reg(REG_PA_CONFIG, 0x8F);

    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x80);
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);

    return ESP_OK;
}

void lora_wake_fast(void)
{
    // go directly to standby
    lora_write_reg(REG_OP_MODE,
        MODE_LONG_RANGE_MODE | MODE_STDBY);

    esp_rom_delay_us(500); // 0.5 ms settle
}

void lora_shutdown(void)
{
    if (spi == NULL)
        return;

    // Put radio in sleep mode first
    lora_sleep();

    // small settle time
    esp_rom_delay_us(500);

    // detach SPI device
    spi_bus_remove_device(spi);
    spi = NULL;

    // free SPI bus
    spi_bus_free(SPI2_HOST);
    
    
	gpio_set_direction(LORA_MOSI, GPIO_MODE_OUTPUT);
	gpio_set_direction(LORA_SCK, GPIO_MODE_OUTPUT);
	gpio_set_direction(LORA_CS, GPIO_MODE_OUTPUT);
	
	gpio_set_level(LORA_MOSI, 0);
	gpio_set_level(LORA_SCK, 0);
	gpio_set_level(LORA_CS, 1);    
    

    ESP_LOGI(TAG, "LoRa SPI released");
}