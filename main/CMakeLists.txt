idf_component_register(
	SRCS "main.c"
	INCLUDE_DIRS "."
	REQUIRES DS18B20 lora02 esp_timer nvs_flash
)