deps_config := \
	/Users/saaketAgashe/esp/esp-idf/components/app_trace/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/aws_iot/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/bt/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/driver/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/esp32/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/esp_adc_cal/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/esp_http_client/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/ethernet/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/fatfs/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/freertos/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/heap/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/http_server/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/libsodium/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/log/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/lwip/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/mbedtls/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/mdns/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/openssl/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/pthread/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/spi_flash/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/spiffs/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/tcpip_adapter/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/vfs/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/components/wear_levelling/Kconfig \
	/Users/saaketAgashe/esp/esp-idf/Kconfig.compiler \
	/Users/saaketAgashe/esp/esp-idf/components/bootloader/Kconfig.projbuild \
	/Users/saaketAgashe/esp/esp-idf/components/esptool_py/Kconfig.projbuild \
	/Users/saaketAgashe/esp/Wall-E-Workshop-2.1/self_balancing_line_following_with_server_tuning/main/Kconfig.projbuild \
	/Users/saaketAgashe/esp/esp-idf/components/partition_table/Kconfig.projbuild \
	/Users/saaketAgashe/esp/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
