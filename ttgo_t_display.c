/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>

#include <sdkconfig.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_timer.h"

#include "driver/spi_master.h"
#include "driver/ledc.h"

#include "lvgl.h"

#include <bsp_err_check.h>
#include <bsp/ttgo_t_display.h>
#include <bsp/display.h>

#define BSP_LCD_CMD_BITS (8)
#define BSP_LCD_PARAM_BITS (8)
#define BSP_LCD_LEDC_CH (CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH)

/*******************************************************************************
 * Private function prototypes
*******************************************************************************/

static esp_err_t bsp_lvgl_init(esp_lcd_panel_handle_t panel_handle, lv_disp_t **disp_ret);
static void bsp_lvgl_increase_tick(void *arg);
static void bsp_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
static void bsp_lvgl_set_px_cb(
    lv_disp_drv_t *disp_drv, uint8_t *buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
    lv_color_t color, lv_opa_t opa
);
static void bsp_lvgl_rounder(lv_disp_drv_t *disp_drv, lv_area_t *area);
static void bsp_lvgl_port_update_cb(lv_disp_drv_t *drv);

static const char *TAG = "TTGO-T-DISPLAY";

/*******************************************************************************
 * Public function implementations
*******************************************************************************/

bool bsp_button_get(const bsp_button_t btn)
{
    return !(bool)gpio_get_level(btn);
}

esp_err_t bsp_button_init(const bsp_button_t btn)
{
    const gpio_config_t btn_io_config = {
        .pin_bit_mask = BIT64(btn),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    ESP_RETURN_ON_ERROR(gpio_config(&btn_io_config), TAG, "Button initialization failed");

    return ESP_OK;
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

static esp_err_t bsp_display_brightness_init(void)
{
    // Setup LEDC peripheral for PWM backlight control
    const ledc_timer_config_t LCD_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = 1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK};
    const ledc_channel_config_t LCD_backlight_channel = {
        .gpio_num = BSP_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = BSP_LCD_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = false};

    ESP_RETURN_ON_ERROR(ledc_timer_config(&LCD_backlight_timer), TAG, "LCD panel backlight PWM timer initialization failed");

    ESP_RETURN_ON_ERROR(ledc_channel_config(&LCD_backlight_channel), TAG, "LCD panel backlight PWM channel initialization failed");

    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent > 100)
    {
        brightness_percent = 100;
    }
    if (brightness_percent < 0)
    {
        brightness_percent = 0;
    }

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness_percent);
    // LEDC resolution set to 10bits, thus: 100% = 1023
    uint32_t duty_cycle = (1023 * brightness_percent) / 100;
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, BSP_LCD_LEDC_CH, duty_cycle), TAG, "LCD panel backlight PWM duty setting failed");

    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, BSP_LCD_LEDC_CH), TAG, "LCD panel backlight PWM timer duty update failed");

    return ESP_OK;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return NULL;
}

static esp_err_t bsp_display_lcd_init(esp_lcd_panel_io_handle_t *ret_io, esp_lcd_panel_handle_t *ret_panel)
{
    ESP_RETURN_ON_FALSE(ret_io != NULL && ret_panel != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    *ret_io = NULL;
    *ret_panel = NULL;
    esp_err_t ret = ESP_FAIL;

    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_LCD_SPI_CLK,
        .mosi_io_num = BSP_LCD_SPI_MOSI,
        .miso_io_num = BSP_LCD_SPI_MISO,
        .quadwp_io_num = -1,                                          // Quad SPI LCD driver is not yet supported
        .quadhd_io_num = -1,                                          // Quad SPI LCD driver is not yet supported
        .max_transfer_sz = BSP_LCD_DRAW_BUFF_SIZE * sizeof(uint16_t), // transfer 80 lines of pixels (assume pixel is RGB565) at most in one SPI transaction
    };
    ESP_GOTO_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), err, TAG, "LCD panel SPI bus initializaton failed");

    ESP_LOGD(TAG, "Attach the LCD to the SPI bus");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_DC,
        .cs_gpio_num = BSP_LCD_SPI_CS,
        .pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = BSP_LCD_CMD_BITS,
        .lcd_param_bits = BSP_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, ret_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Create LCD panel handle for ST7789V");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .rgb_endian = BSP_LCD_RGB_ENDIAN,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL_RGB565,
        .flags.reset_active_high = false};
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(*ret_io, &panel_config, ret_panel), err, TAG, "Panel driver installation failed");

    ESP_LOGD(TAG, "Reset LCD panel");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(*ret_panel), err, TAG, "Panel reset failed");

    ESP_LOGD(TAG, "Initialize LCD panel");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(*ret_panel), err, TAG, "Panel initialization failed");

    ESP_LOGD(TAG, "Switch on LCD panel");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_disp_on_off(*ret_panel, true), err, TAG, "Switch on panel failed");

    return ESP_OK;

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
    //     /* Add LCD screen */
    //     ESP_LOGD(TAG, "Add LCD screen");
    //     const lvgl_port_display_cfg_t disp_cfg = {
    //         .io_handle = io_handle,
    //         .panel_handle = panel_handle,
    //         .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
    //         .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
    //         .hres = BSP_LCD_H_RES,
    //         .vres = BSP_LCD_V_RES,
    //         .monochrome = false,
    //         /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
    //         .rotation = {
    //             .swap_xy = false,
    // #ifdef CONFIG_BSP_LCD_ILI9341
    //             .mirror_x = true,
    // #else
    //             .mirror_x = false,
    // #endif
    //             .mirror_y = false,
    //         },
    //         .flags = {
    //             .buff_dma = true,
    //         }
    //     };

    //     return lvgl_port_add_disp(&disp_cfg);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    // return lvgl_port_lock(timeout_ms);
    return true;
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    // ESP_RETURN_ON_FALSE(ret_io && ret_panel && (config->max_transfer_sz > 0), ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(ret_io && ret_panel, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    *ret_panel = NULL;
    *ret_io = NULL;
    esp_err_t ret = ESP_FAIL;

    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");

    ESP_LOGD(TAG, "Initialize LCD panel SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_LCD_SPI_CLK,
        .mosi_io_num = BSP_LCD_SPI_MOSI,
        .miso_io_num = BSP_LCD_SPI_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = (BSP_LCD_BITS_PER_PIXEL_RGB565 * BSP_LCD_DRAW_BUFF_SIZE) / 8 + 1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install LCD panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_DC,
        .cs_gpio_num = BSP_LCD_SPI_CS,
        .spi_mode = 0,
        .pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = BSP_LCD_CMD_BITS,
        .lcd_param_bits = BSP_LCD_PARAM_BITS,
        .trans_queue_depth = 10,
        .flags = {
            .dc_low_on_data = false,
            .octal_mode = false,
            .sio_mode = true,
            .lsb_first = false,
            .cs_high_active = false}};
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, ret_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .rgb_endian = BSP_LCD_RGB_ENDIAN,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL_RGB565,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(*ret_io, &panel_config, ret_panel), err, TAG, "Panel driver installation failed");

    ESP_LOGD(TAG, "Reset LCD panel");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(*ret_panel), err, TAG, "Panel reset failed");

    ESP_LOGD(TAG, "Initialize LCD panel");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(*ret_panel), err, TAG, "Panel initialization failed");

    return ESP_OK;

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

void bsp_display_rotate(lv_disp_t *disp, lv_disp_rot_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

lv_disp_t *bsp_display_start(void)
{
    bsp_display_config_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_H_RES * CONFIG_BSP_LCD_DRAW_BUF_HEIGHT,
#if CONFIG_BSP_LCD_DRAW_BUF_DOUBLE
        .double_buffer = 1,
#else
        .double_buffer = 0,
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
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);

    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);

    return disp;
}

esp_err_t bsp_usbpwr_init(void)
{
    const gpio_config_t led_io_config = {
        .pin_bit_mask = BSP_USBPWR_EN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&led_io_config), TAG, "USB power interface enable initialization failed");

    return ESP_OK;
}

esp_err_t bsp_usbpwr_set(const bool on)
{
    ESP_RETURN_ON_ERROR(gpio_set_level(BSP_USBPWR_EN, (uint32_t)on), TAG, "USB power interface enable setting failed");

    return ESP_OK;
}

void bsp_display_unlock(void){
    // lvgl_port_unlock();
}

/*******************************************************************************
 * Private LVGL port related functions
 *******************************************************************************/

static esp_err_t bsp_lvgl_init(esp_lcd_panel_handle_t panel_handle, lv_disp_t **disp_ret)
{
    const size_t BSP_LVGL_DRAWBUF_SIZE = BSP_LCD_H_RES * (BSP_LCD_V_RES / 10 + 1);
    const size_t BSP_LVGL_DRAWBUF_BYTES = BSP_LVGL_DRAWBUF_SIZE * sizeof(lv_color_t);

    ESP_LOGI(TAG, "Initialize LVGL support");

    ESP_RETURN_ON_FALSE(panel_handle && disp_ret, ESP_ERR_INVALID_ARG, TAG, "Parameters panel_handle and disp_ret can't be NULL");

    *disp_ret = NULL;
    esp_err_t ret = ESP_FAIL;

    ESP_LOGD(TAG, "Initialize LVGL library");
    lv_init();

    ESP_LOGD(TAG, "Allocating draw buffers used by LVGL");
    lv_color_t *buf2 = NULL;
    lv_color_t *buf1 = malloc(BSP_LVGL_DRAWBUF_BYTES);
    ESP_GOTO_ON_FALSE(buf1, ESP_ERR_NO_MEM, err, TAG, "Could not allocate %ul bytes for buf1", BSP_LVGL_DRAWBUF_BYTES);

    buf2 = malloc(BSP_LVGL_DRAWBUF_BYTES);
    ESP_GOTO_ON_FALSE(buf1, ESP_ERR_NO_MEM, err, TAG, "Could not allocate %ul bytes for buf1", BSP_LVGL_DRAWBUF_BYTES);

    ESP_LOGD(TAG, "initialize draw buffers used by LVGL");
    static lv_disp_draw_buf_t disp_buf;
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, BSP_LVGL_DRAWBUF_SIZE);

    ESP_LOGD(TAG, "Register display driver to LVGL");
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = BSP_LCD_H_RES;
    disp_drv.ver_res = BSP_LCD_V_RES;
    disp_drv.flush_cb = bsp_lvgl_flush_cb;
    disp_drv.drv_update_cb = bsp_lvgl_port_update_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    // disp_drv.offset_x = ;
    // disp_drv.offset_y = ;
    //disp_drv.rotated = LV_DISP_ROT_NONE;
    //disp_drv.rounder_cb = bsp_lvgl_rounder;
    //disp_drv.set_px_cb = bsp_lvgl_set_px_cb;
    //disp_drv.monitor_cb = ;
    //disp_drv.clean_dcache_cb = ;
    //disp_drv.render_start_cb = ;
    *disp_ret = lv_disp_drv_register(&disp_drv);
    ESP_RETURN_ON_FALSE(*disp_ret, ESP_FAIL, TAG, "LVGL Display driver registration failed");

    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    ESP_LOGD(TAG, "Install LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &bsp_lvgl_increase_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_GOTO_ON_ERROR(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer), err, TAG, "Failed to create timer for LVGL tick");
    ESP_GOTO_ON_ERROR(esp_timer_start_periodic(lvgl_tick_timer, BSP_LVGL_TICK_PERIOD_MS * 1000), err, TAG, "Failed to start timer for LVGL tick");

    return ESP_OK;

err:
    if (lvgl_tick_timer)
    {
        esp_timer_stop(lvgl_tick_timer);
        esp_timer_delete(lvgl_tick_timer);
    }
    if (buf1)
        free(buf1);
    if (buf2)
        free(buf2);
    return ret;
}

static void bsp_lvgl_increase_tick(void *arg)
{
    // Tell LVGL how many milliseconds has elapsed
    lv_tick_inc(BSP_LVGL_TICK_PERIOD_MS);
}

static bool bsp_lvgl_notify_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void bsp_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void bsp_lvgl_set_px_cb(lv_disp_drv_t *disp_drv, uint8_t *buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
                                   lv_color_t color, lv_opa_t opa)
{
    uint16_t byte_index = x + ((y >> 3) * buf_w);
    uint8_t bit_index = y & 0x7;

    if ((color.full == 0) && (LV_OPA_TRANSP != opa))
    {
        buf[byte_index] |= (1 << bit_index);
    }
    else
    {
        buf[byte_index] &= ~(1 << bit_index);
    }
}

static void bsp_lvgl_rounder(lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    area->y1 = area->y1 & (~0x7);
    area->y2 = area->y2 | 0x7;
}

static void bsp_lvgl_port_update_cb(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    switch(drv->rotated)
    {
        case LV_DISP_ROT_NONE:
            esp_lcd_panel_swap_xy(panel_handle, false);
            esp_lcd_panel_mirror(panel_handle, true, false);
            break;

        case LV_DISP_ROT_90:
            esp_lcd_panel_swap_xy(panel_handle, true);
            esp_lcd_panel_mirror(panel_handle, true, true);
            break;

        case LV_DISP_ROT_180:
            esp_lcd_panel_swap_xy(panel_handle, false);
            esp_lcd_panel_mirror(panel_handle, false, true);
            break;

        case LV_DISP_ROT_270:
            esp_lcd_panel_swap_xy(panel_handle, true);
            esp_lcd_panel_mirror(panel_handle, false, false);
            break;
        }
}
