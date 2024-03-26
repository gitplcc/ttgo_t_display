idf_component_register(
    SRCS "ttgo-t-display.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES button esp_lvgl_port
    PRIV_REQUIRES esp_lcd spiffs
)
