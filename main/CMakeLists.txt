idf_component_register(SRCS "camera.c" "bsp_i2c.c" "main.c" "attitude_qmi8658.c" "lcd_ST7789.c" "io_pca9557.c"
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS ""
                       REQUIRES esp_lcd)
