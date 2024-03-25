/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * @file
 * @brief ESP BSP: Lilygo TTGO-T-DISPLAY
 */

#pragma once

#include <sdkconfig.h>

#include <driver/gpio.h>
#include <driver/sdmmc_host.h>
#include <lvgl.h>

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

#define BSP_CAPS_DISPLAY 1
#define BSP_CAPS_TOUCH 0
#define BSP_CAPS_BUTTONS 1
#define BSP_CAPS_AUDIO 0
#define BSP_CAPS_AUDIO_SPEAKER 0
#define BSP_CAPS_AUDIO_MIC 0
#define BSP_CAPS_SDCARD 0
#define BSP_CAPS_IMU 0

/**************************************************************************************************
 * TTGO-T-DISPLAY pinout
 **************************************************************************************************/

/* USB interface power control */
#define BSP_USBPWR_EN       (GPIO_NUM_14)

/* +5V bus USB interface ADC */
#define BSP_BAT_ADC         (GPIO_NUM_34)

/* Display */
#define BSP_LCD_SPI_MOSI    (GPIO_NUM_19)
#define BSP_LCD_SPI_MISO    (-1)
#define BSP_LCD_SPI_CLK     (GPIO_NUM_18)
#define BSP_LCD_SPI_CS      (GPIO_NUM_5)
#define BSP_LCD_DC          (GPIO_NUM_16)
#define BSP_LCD_RST         (GPIO_NUM_23)
#define BSP_LCD_BACKLIGHT   (GPIO_NUM_4)

/* Button */
typedef enum
{
    BSP_BUTTON_S1 = GPIO_NUM_0,
    BSP_BUTTON_S2 = GPIO_NUM_35
} bsp_button_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BSP display configuration structure
 *
 */
typedef struct
{
    lvgl_port_cfg_t lvgl_port_cfg; /*!< LVGL port configuration */
    uint32_t buffer_size;          /*!< Size of the buffer for the screen in pixels */
    bool double_buffer;            /*!< True, if should be allocated two buffers */
    struct
    {
        unsigned int buff_dma : 1;    /*!< Allocated LVGL buffer will be DMA capable */
        unsigned int buff_spiram : 1; /*!< Allocated LVGL buffer will be in PSRAM */
    } flags;
} bsp_display_cfg_t;

/**************************************************************************************************
 *
 * USB interface power control
 *
 **************************************************************************************************/

/**
 * @brief Set BSP_USBPWR_EN's GPIOs as output push-pull
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_usbpwr_init(void);

/**
 * @brief Turn USB interface power on/off
 *
 * @param on Switch LED on/off
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_usbpwr_set(const bool on);

/**************************************************************************************************
 *
 * Power supply voptage probing
 *
 **************************************************************************************************/

/**
 * @brief Set BAT_ADC GPIO as analog
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
//esp_err_t bsp_bat_adc_init(void);

/**
 * @brief Turn USB interface power on/off
 *
 * @param on Switch LED on/off
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
//esp_err_t bsp_batadc_get(const bool on);

/**************************************************************************************************
 *
 * Buttons
 *
 **************************************************************************************************/

/**
 * @brief Set button's GPIOs as inputs
 *
 * @param[in] btn Button to be initialized
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_button_init(const bsp_button_t btn);

/**
 * @brief Get button's state
 *
 * @param[in] btn Button to read
 * @return true  Button pressed
 * @return false Button released
 */
bool bsp_button_get(const bsp_button_t btn);

/**************************************************************************************************
 *
 * LCD interface
 *
 * TTGO-T-DISPLAY is shipped with 1.14inch ST7789 display controller. It features 16-bit colors and
 * 135x240 resolution.
 *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling any LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 *
 * Display's backlight must be enabled explicitly by calling bsp_display_backlight_on()
 **************************************************************************************************/

// it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
#define BSP_LCD_DRAW_BUFF_SIZE (BSP_LCD_H_RES * (BSP_LCD_V_RES / 10 + 1))
#define BSP_LCD_DRAW_BUFF_DOUBLE    (1)

#define BSP_LVGL_TICK_PERIOD_MS     (1)

/**
 * @brief Intialize USB interface power enable control when not connected to an USB bus
 *
 * This function initializes GPIO attached to TTGO-T-DISPLAY PWR_EN SPI as a push-pull output.
 *
 */
esp_err_t bsp_usbpwr_en_init(void);

/**
 * @brief Enable/disable USB interface power when not connected to an USB bus
 *
 * Control USB interface powering through PWR_EN line. USB interface is
 * always on when connected to an USB powered bus despite this setting.
 *
 * @param[in] on USB interface power status when not connected to an USB powered bus.
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG GPIO number error
 *
 */
esp_err_t bsp_usbpwr_en_set(const bool on);

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 *
 * @return Pointer to LVGL display or NULL when error occured
 */
lv_disp_t *bsp_display_start(void);

/**
 * @brief Get pointer to input device (touch, buttons, ...)
 *
 * @note The LVGL input device is initialized in bsp_display_start() function.
 *
 * @return Pointer to LVGL input device or NULL when not initialized
 */
lv_indev_t *bsp_display_get_input_dev(void);

/**
 * @brief Take LVGL mutex
 *
 * @param timeout_ms Timeout in [ms]. 0 will block indefinitely.
 * @return true  Mutex was taken
 * @return false Mutex was NOT taken
 */
bool bsp_display_lock(uint32_t timeout_ms);

/**
 * @brief Give LVGL mutex
 *
 */
void bsp_display_unlock(void);

/**
 * @brief Set display's brightness
 *
 * Brightness is controlled with PWM signal to a pin controling backlight.
 *
 * @param[in] brightness_percent Brightness in [%]
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   Parameter error
 */
esp_err_t bsp_display_brightness_set(int brightness_percent);

/**
 * @brief Turn on display backlight
 *
 * Display must be already initialized by calling bsp_display_start()
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   Parameter error
 */
esp_err_t bsp_display_backlight_on(void);

/**
 * @brief Turn off display backlight
 *
 * Display must be already initialized by calling bsp_display_start()
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   Parameter error
 */
esp_err_t bsp_display_backlight_off(void);

/**
 * @brief Rotate screen
 *
 * Display must be already initialized by calling bsp_display_start()
 *
 * @param[in] disp Pointer to LVGL display
 * @param[in] rotation Angle of the display rotation
 */
void bsp_display_rotate(lv_disp_t *disp, lv_disp_rot_t rotation);

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
 *
 * @return Pointer to LVGL display or NULL when error occurred
 */lv_disp_t *bsp_display_start(void);

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
 *
 * @param cfg display configuration
 *
 * @return Pointer to LVGL display or NULL when error occurred
 */
lv_disp_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
