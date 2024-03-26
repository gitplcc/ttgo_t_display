/**
 * Copyright 2024 Pedro Luis Castedo Cepeda
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <sdkconfig.h>

#include <bsp/display.h>

#include <lvgl.h>

#include <esp_lvgl_port.h>

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

#define BSP_CAPS_DISPLAY 1
#define BSP_CAPS_TOUCH 0
#define BSP_CAPS_BUTTONS 0
#define BSP_CAPS_AUDIO 0
#define BSP_CAPS_AUDIO_SPEAKER 0
#define BSP_CAPS_AUDIO_MIC 0
#define BSP_CAPS_SDCARD 0
#define BSP_CAPS_IMU 0

/**************************************************************************************************
 *  ESP-BOX pinout
 **************************************************************************************************/
/* I2C */
#define BSP_I2C_SDA (GPIO_NUM_21)
#define BSP_I2C_SCL (GPIO_NUM_22)

/* Display */
#define BSP_LCD_BACKLIGHT (GPIO_NUM_4)
#define BSP_LCD_CS (GPIO_NUM_5)
#define BSP_LCD_DC (GPIO_NUM_16)
#define BSP_LCD_SCLK (GPIO_NUM_18)
#define BSP_LCD_MOSI (GPIO_NUM_19)
#define BSP_LCD_RST (GPIO_NUM_23)

/* Buttons */
#define BSP_BUTTON_GPIO0 (GPIO_NUM_0)
#define BSP_BUTTON_GPIO35 (GPIO_NUM_35)

typedef enum
{
    BSP_BUTTON_CONFIG = 0,
    BSP_BUTTON_MUTE,
    BSP_BUTTON_NUM
} bsp_button_t;

#ifdef __cplusplus
extern "C"
{
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

    /**********************************************************************************************
     *
     * LCD interface
     *
     * TTGO-T-DISPLAY is shipped with 1.14inch ST7789V display controller.
     * It features 16-bit colors, 320x240 resolution and capacitive touch controller.
     *
     * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL
     * mutex by calling bsp_display_lock() before calling and LVGL API (lv_...) and then give the
     * mutex with bsp_display_unlock().
     *
     * Display's backlight must be enabled explicitly by calling bsp_display_backlight_on()
     **********************************************************************************************/
    #define BSP_LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
    #define BSP_LCD_SPI_NUM (SPI3_HOST)

    /**
     * @brief Initialize display
     *
     * This function initializes SPI, display controller and starts LVGL handling task.
     * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
     *
     * @return Pointer to LVGL display or NULL when error occurred
     */
    lv_disp_t *bsp_display_start(void);

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
     * @brief Initialize all buttons
     *
     * Returned button handlers must be used with espressif/button component API
     *
     * @param[out] btn_array      Output button array
     * @param[out] btn_cnt        Number of button handlers saved to btn_array, can be NULL
     * @param[in]  btn_array_size Size of output button array. Must be at least BSP_BUTTON_NUM
     * @return
     *     - ESP_OK               All buttons initialized
     *     - ESP_ERR_INVALID_ARG  btn_array is too small or NULL
     *     - ESP_FAIL             Underlaying iot_button_create failed
     */
    esp_err_t bsp_iot_button_create(button_handle_t btn_array[], int *btn_cnt, int btn_array_size);

#ifdef __cplusplus
}
#endif