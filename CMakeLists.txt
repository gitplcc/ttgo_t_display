idf_component_register(
    SRCS "ttgo_t_display.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver button lvgl
    PRIV_REQUIRES esp_lcd
)
