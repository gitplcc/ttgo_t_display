#include <sdkconfig.h>

#include <bsp/ttgo-t-display.h>
#include <bsp_err_check.h>

#include <esp_lvgl_port.h>

#include <esp_lcd_panel_vendor.h>
#include <driver/ledc.h>
#include "driver/spi_master.h"
#include <esp_log.h>

// Bit number used to represent command and parameter
#define BSP_LCD_CMD_BITS (8)
#define BSP_LCD_PARAM_BITS (8)
#define BSP_LCD_LEDC_CH CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH

#define BSP_LCD_MIRROR_X (false)
#define BSP_LCD_MIRROR_Y (false)
#define BSP_LCD_SWAP_XY  (false)

#define BSP_LCD_OFFSET_X (51)
#define BSP_LCD_OFFSET_Y (40)

#define CMD_INVON        (0x21)

static const char *TAG = "TTGO-T-DISPLAY";

static const button_config_t bsp_button_config[BSP_BUTTON_NUM] =
{
    {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config =
        {
            .gpio_num = BSP_BUTTON_GPIO0,
            .active_level = 0,
        }
    },
    {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config =
        {
            .gpio_num = BSP_BUTTON_GPIO35,
            .active_level = 0,
        }
    },
    {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config =
        {
            .gpio_num = GPIO_NUM_37,
            .active_level = 0,
        }
    },
};

static lv_disp_t *disp;
static lv_indev_t *disp_indev = NULL;

static esp_err_t bsp_display_brightness_init(void)
{
    // Setup LEDC peripheral for PWM backlight control
    const ledc_channel_config_t LCD_backlight_channel = {
        .gpio_num = BSP_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = BSP_LCD_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 0,
        .hpoint = 0};
    const ledc_timer_config_t LCD_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = 1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK};

    ESP_LOGD(TAG, "BSP display brightness: initializing...");
    BSP_ERROR_CHECK_RETURN_ERR(ledc_timer_config(&LCD_backlight_timer));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_channel_config(&LCD_backlight_channel));
    ESP_LOGD(TAG, "BSP display brightness: initialized.");

    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent > 100)
        brightness_percent = 100;
    if (brightness_percent < 0)
        brightness_percent = 0;

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness_percent);
    uint32_t duty_cycle = (1023 * brightness_percent) / 100; // LEDC resolution set to 10bits, thus: 100% = 1023
    BSP_ERROR_CHECK_RETURN_ERR(ledc_set_duty(LEDC_LOW_SPEED_MODE, BSP_LCD_LEDC_CH, duty_cycle));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_update_duty(LEDC_LOW_SPEED_MODE, BSP_LCD_LEDC_CH));
    return ESP_OK;
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;
    assert(config != NULL && config->max_transfer_sz > 0);

    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");

    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_LCD_SCLK,
        .mosi_io_num = BSP_LCD_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = config->max_transfer_sz,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    /* Not explicitely set fields assumed to be 0 */
    const esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = BSP_LCD_CS,
        .dc_gpio_num = BSP_LCD_DC,
        .spi_mode = 0,
        .pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = BSP_LCD_CMD_BITS,
        .lcd_param_bits = BSP_LCD_PARAM_BITS,
        .flags = {
            .sio_mode = 1,         /* R/W through a single data line (MOSI) */
        }
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, ret_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .rgb_ele_order = BSP_LCD_ELE_ORDER,
        .data_endian = BSP_DATA_ENDIAN,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(*ret_io, &panel_config, ret_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(*ret_panel);
    esp_lcd_panel_init(*ret_panel);
    ESP_GOTO_ON_ERROR(esp_lcd_panel_io_tx_param(*ret_io, CMD_INVON, NULL, 0), err, TAG, "Could not set inverted mode");
    esp_lcd_panel_swap_xy(*ret_panel, BSP_LCD_SWAP_XY);
    esp_lcd_panel_mirror(*ret_panel, BSP_LCD_MIRROR_X, BSP_LCD_MIRROR_Y);
    esp_lcd_panel_set_gap(*ret_panel, BSP_LCD_OFFSET_X, BSP_LCD_OFFSET_Y);

    return ret;

err:
    if (*ret_panel)
    {
        esp_lcd_panel_del(*ret_panel);
    }
    if (*ret_io)
    {
        esp_lcd_panel_io_del(*ret_io);
    }
    spi_bus_free(BSP_LCD_SPI_NUM);
    return ret;
}

static lv_disp_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg)
{
    ESP_LOGV(TAG, "Initializing display LCD");

    assert(cfg != NULL);

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;
    const bsp_display_config_t bsp_disp_cfg = {
        .max_transfer_sz = (BSP_LCD_H_RES * CONFIG_BSP_LCD_DRAW_BUF_HEIGHT) * sizeof(uint16_t),
    };
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&bsp_disp_cfg, &panel_handle, &io_handle));

    esp_lcd_panel_disp_on_off(panel_handle, true);

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = cfg->buffer_size,
        .double_buffer = cfg->double_buffer,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
        /* NOTE: Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = BSP_LCD_SWAP_XY,
            .mirror_x = BSP_LCD_MIRROR_X,
            .mirror_y = BSP_LCD_MIRROR_Y,
        },
        .flags = {
            .buff_dma = cfg->flags.buff_dma, .buff_spiram = cfg->flags.buff_spiram,
            .swap_bytes = false, // CONFIG_LV_COLOR_16_SWAP 1
        }};

    return lvgl_port_add_disp(&disp_cfg);
}

static lv_indev_t *bsp_display_indev_init(lv_disp_t *disp)
{
    assert(disp);

    const lvgl_port_nav_btns_cfg_t btns = {
        .disp = disp,
        .button_prev = &bsp_button_config[BSP_BUTTON_DUMMY],
        .button_next = &bsp_button_config[BSP_BUTTON_NEXT],
        .button_enter = &bsp_button_config[BSP_BUTTON_ENTER]
    };

    /* Add buttons input (for selected screen) */
    return lvgl_port_add_navigation_buttons(&btns);
}

lv_disp_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_H_RES * CONFIG_BSP_LCD_DRAW_BUF_HEIGHT,
#if CONFIG_BSP_LCD_DRAW_BUF_DOUBLE
        .double_buffer = true,
#else
        .double_buffer = false,
#endif
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
        }
    };
    return bsp_display_start_with_config(&cfg);
}

lv_disp_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);

    ESP_LOGD(TAG, "Starting display...");

    ESP_LOGD(TAG, "Initializing LVGL port component...");
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));
    ESP_LOGD(TAG, "LVGL port component initialized.");

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);

    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);

    ESP_LOGD(TAG, "Display started.");

    return disp;
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}

esp_err_t bsp_iot_button_create(button_handle_t btn_array[], int *btn_cnt, int btn_array_size)
{
    return ESP_ERR_NOT_SUPPORTED;
}
