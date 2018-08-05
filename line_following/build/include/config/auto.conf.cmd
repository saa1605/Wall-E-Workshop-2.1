deps_config := \
	/Users/akshay/esp/esp-idf/components/app_trace/Kconfig \
	/Users/akshay/esp/esp-idf/components/aws_iot/Kconfig \
	/Users/akshay/esp/esp-idf/components/bt/Kconfig \
	/Users/akshay/esp/esp-idf/components/driver/Kconfig \
	/Users/akshay/esp/esp-idf/components/esp32/Kconfig \
	/Users/akshay/esp/esp-idf/components/esp_adc_cal/Kconfig \
	/Users/akshay/esp/esp-idf/components/esp_http_client/Kconfig \
	/Users/akshay/esp/esp-idf/components/ethernet/Kconfig \
	/Users/akshay/esp/esp-idf/components/fatfs/Kconfig \
	/Users/akshay/esp/esp-idf/components/freertos/Kconfig \
	/Users/akshay/esp/esp-idf/components/heap/Kconfig \
	/Users/akshay/esp/esp-idf/components/libsodium/Kconfig \
	/Users/akshay/esp/esp-idf/components/log/Kconfig \
	/Users/akshay/esp/esp-idf/components/lwip/Kconfig \
	/Users/akshay/esp/esp-idf/components/mbedtls/Kconfig \
	/Users/akshay/esp/esp-idf/components/mdns/Kconfig \
	/Users/akshay/esp/esp-idf/components/openssl/Kconfig \
	/Users/akshay/esp/esp-idf/components/pthread/Kconfig \
	/Users/akshay/esp/esp-idf/components/spi_flash/Kconfig \
	/Users/akshay/esp/esp-idf/components/spiffs/Kconfig \
	/Users/akshay/esp/esp-idf/components/tcpip_adapter/Kconfig \
	/Users/akshay/esp/esp-idf/components/vfs/Kconfig \
	/Users/akshay/esp/esp-idf/components/wear_levelling/Kconfig \
	/Users/akshay/esp/esp-idf/Kconfig.compiler \
	/Users/akshay/esp/esp-idf/components/bootloader/Kconfig.projbuild \
	/Users/akshay/esp/esp-idf/components/esptool_py/Kconfig.projbuild \
	/Users/akshay/esp/esp-idf/components/partition_table/Kconfig.projbuild \
	/Users/akshay/esp/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
