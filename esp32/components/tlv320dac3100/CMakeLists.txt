idf_component_register(SRCS "tlv320dac3100.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c)
target_compile_definitions(${COMPONENT_LIB} PRIVATE CONFIG_HAS_TLV320DAC_I2C=1)
