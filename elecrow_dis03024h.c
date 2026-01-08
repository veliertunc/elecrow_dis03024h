/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstddef>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_vfs_fat.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/ledc.h"

#include "bsp/elecrow_dis03024h.h"
#include "bsp_err_check.h"
#include "button_gpio.h"
#include "button_adc.h"

// Display
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "bsp/display.h"
#include "esp_lcd_ili9341.h"

// Touch
#include "bsp/touch.h"
#include "esp_lcd_touch_xpt2046.h"

// Speaker
#include "driver/dac_continuous.h"

static const char *TAG = "DIS03024H";

#define BSP_DISPLAY_ROTATION_SWAP_XY    (0)
#define BSP_DISPLAY_ROTATION_MIRROR_X   (0)
#define BSP_DISPLAY_ROTATION_MIRROR_Y   (1)

#define BSP_TOUCH_ROTATION_SWAP_XY      (0)
#define BSP_TOUCH_ROTATION_MIRROR_X     (0)
#define BSP_TOUCH_ROTATION_MIRROR_Y     (1)

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *disp;
static lv_indev_t *disp_indev = NULL;
#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

static bool display_spi_initialized = false;
static esp_lcd_touch_handle_t tp = NULL;

static i2c_master_bus_handle_t i2c_handle = NULL;
static bool i2c_initialized = false;

static bool sdcard_spi_initialized = false;
static sdmmc_card_t *bsp_sdcard = NULL;    // Global uSD card handler

typedef enum {
    BSP_BUTTON_TYPE_GPIO,
    BSP_BUTTON_TYPE_ADC
} bsp_button_type_t;

typedef struct {
    bsp_button_type_t type;
    union {
        button_gpio_config_t gpio;
        button_adc_config_t  adc;
    } cfg;
} bsp_button_config_t;

static const bsp_button_config_t bsp_button_config[BSP_BUTTONS_NUM] = {
#if CONFIG_BSP_BUTTONS_NUM > 0
#if CONFIG_BSP_BUTTON_1_TYPE_GPIO
    {
        .type = BSP_BUTTON_TYPE_GPIO,
        .cfg.gpio = {
            .gpio_num = BSP_BUTTON_1_IO,
            .active_level = BSP_BUTTON_1_LEVEL,
        }
    },
#elif CONFIG_BSP_BUTTON_1_TYPE_ADC
    {
        .type = BSP_BUTTON_TYPE_ADC,
        .cfg.adc = {
            .adc_channel = CONFIG_BSP_BUTTON_1_ADC_CHANNEL,
            .button_index = BSP_BUTTON_1,
            .min = (CONFIG_BSP_BUTTON_1_ADC_VALUE - 100),
            .max = (CONFIG_BSP_BUTTON_1_ADC_VALUE + 100)
        }
    },
#endif // CONFIG_BSP_BUTTON_1_TYPE_x
#endif // CONFIG_BSP_BUTTONS_NUM >= 0

#if CONFIG_BSP_BUTTONS_NUM > 1
#if CONFIG_BSP_BUTTON_2_TYPE_GPIO
    {
        .type = BSP_BUTTON_TYPE_GPIO,
        .cfg.gpio = {
            .gpio_num = BSP_BUTTON_2_IO,
            .active_level = BSP_BUTTON_2_LEVEL,
        }
    },
#elif CONFIG_BSP_BUTTON_2_TYPE_ADC
    {
        .type = BSP_BUTTON_TYPE_ADC,
        .cfg.adc = {
            .adc_channel = CONFIG_BSP_BUTTON_2_ADC_CHANNEL,
            .button_index = BSP_BUTTON_2,
            .min = (CONFIG_BSP_BUTTON_2_ADC_VALUE - 100),
            .max = (CONFIG_BSP_BUTTON_2_ADC_VALUE + 100)
        }
    },
#endif // CONFIG_BSP_BUTTON_2_TYPE_x
#endif // CONFIG_BSP_BUTTONS_NUM >= 1
};


esp_err_t bsp_i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized) {
        return ESP_OK;
    }

    const i2c_master_bus_config_t i2c_config = {
        .i2c_port = BSP_I2C_NUM,
        .scl_io_num = BSP_I2C_SCL,
        .sda_io_num = BSP_I2C_SDA,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_new_master_bus(&i2c_config, &i2c_handle));

    i2c_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    if (i2c_initialized) {
        // Delete the I2C master bus
        BSP_ERROR_CHECK_RETURN_ERR(i2c_del_master_bus(i2c_handle));
        i2c_handle = NULL;
        i2c_initialized = false;
    }
    return ESP_OK;
}

esp_err_t bsp_feature_enable(bsp_feature_t feature, bool enable)
{
    esp_err_t err = ESP_OK;

    /* Initialize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());

    switch (feature) {
    case BSP_FEATURE_LCD:
        // backlight is controlled by GPIO
        if (enable) {
            err = bsp_display_backlight_on();
        } else {
            err = bsp_display_backlight_off();
        }
        break;
    case BSP_FEATURE_TOUCH:
    case BSP_FEATURE_SD:
    case BSP_FEATURE_SPEAKER:
        break;
    }
    return err;
}

static esp_err_t bsp_spi_init(uint32_t max_transfer_sz)
{
    bool spi_initialized = display_spi_initialized && sdcard_spi_initialized;
    /* SPI was initialized before */
    if (spi_initialized) {
        return ESP_OK;
    }

    ESP_LOGD(TAG, "Initialize SPI bus");
    if (!display_spi_initialized)
    {
        const spi_bus_config_t display_buscfg = {
            .sclk_io_num     = BSP_LCD_PCLK,
            .mosi_io_num     = BSP_LCD_MOSI,
            .miso_io_num     = BSP_LCD_MISO,
            .quadwp_io_num   = GPIO_NUM_NC,
            .quadhd_io_num   = GPIO_NUM_NC,
            .max_transfer_sz = max_transfer_sz,
        };
        ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_NUM, &display_buscfg, SPI_DMA_CH_AUTO), TAG, "Display SPI init failed");

        display_spi_initialized = true;
    }

    if (!sdcard_spi_initialized)
    {
        const spi_bus_config_t sdcard_buscfg = {
            .sclk_io_num     = BSP_SD_SPI_SCK,
            .mosi_io_num     = BSP_SD_SPI_MOSI,
            .miso_io_num     = BSP_SD_SPI_MISO,
            .quadwp_io_num   = GPIO_NUM_NC,
            .quadhd_io_num   = GPIO_NUM_NC,
            .max_transfer_sz = max_transfer_sz,
        };
        ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_SDSPI_HOST, &sdcard_buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

        sdcard_spi_initialized = true;
    }

    return ESP_OK;
}

esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path       = CONFIG_BSP_SPIFFS_MOUNT_POINT,
        .partition_label = CONFIG_BSP_SPIFFS_PARTITION_LABEL,
        .max_files       = CONFIG_BSP_SPIFFS_MAX_FILES,
#ifdef CONFIG_BSP_SPIFFS_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
    };

    esp_err_t ret_val = esp_vfs_spiffs_register(&conf);

    BSP_ERROR_CHECK_RETURN_ERR(ret_val);

    size_t total = 0, used = 0;
    ret_val = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret_val;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister(CONFIG_BSP_SPIFFS_PARTITION_LABEL);
}

sdmmc_card_t *bsp_sdcard_get_handle(void)
{
    return bsp_sdcard;
}

void bsp_sdcard_get_sdmmc_host(const int slot, sdmmc_host_t *config)
{
    assert(config);
    memset(config, 0, sizeof(sdmmc_host_t));
    ESP_LOGE(TAG, "SD card MMC mode is not supported by HW (Shared SPI)!");
}

void bsp_sdcard_get_sdspi_host(const int slot, sdmmc_host_t *config)
{
    assert(config);

    sdmmc_host_t host_config = SDSPI_HOST_DEFAULT();
    host_config.slot = slot;

    memcpy(config, &host_config, sizeof(sdmmc_host_t));
}

void bsp_sdcard_sdmmc_get_slot(const int slot, sdmmc_slot_config_t *config)
{
    assert(config);
    memset(config, 0, sizeof(sdmmc_slot_config_t));
    ESP_LOGE(TAG, "SD card MMC mode is not supported by HW (Shared SPI)!");
}

void bsp_sdcard_sdspi_get_slot(const spi_host_device_t spi_host, sdspi_device_config_t *config)
{
    assert(config);
    memset(config, 0, sizeof(sdspi_device_config_t));

    config->gpio_cs   = BSP_SD_SPI_CS;
    config->gpio_cd   = SDSPI_SLOT_NO_CD;
    config->gpio_wp   = SDSPI_SLOT_NO_WP;
    config->gpio_int  = GPIO_NUM_NC;
    config->host_id = spi_host;
}

esp_err_t bsp_sdcard_sdmmc_mount(bsp_sdcard_cfg_t *cfg)
{
    ESP_LOGE(TAG, "SD card MMC mode is not supported by HW (Shared SPI)!");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t bsp_sdcard_sdspi_mount(bsp_sdcard_cfg_t *cfg)
{
    sdmmc_host_t sdhost = {0};
    sdspi_device_config_t sdslot = {0};
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    assert(cfg);

    ESP_RETURN_ON_ERROR(bsp_spi_init((BSP_LCD_H_RES * BSP_LCD_V_RES) * sizeof(uint16_t)), TAG, "");

    if (!cfg->mount) {
        cfg->mount = &mount_config;
    }

    if (!cfg->host) {
        bsp_sdcard_get_sdspi_host(SDMMC_HOST_SLOT_0, &sdhost);
        cfg->host = &sdhost;
    }

    if (!cfg->slot.sdspi) {
        bsp_sdcard_sdspi_get_slot(BSP_SDSPI_HOST, &sdslot);
        cfg->slot.sdspi = &sdslot;
    }

#if !CONFIG_FATFS_LONG_FILENAMES
    ESP_LOGW(TAG, "Warning: Long filenames on SD card are disabled in menuconfig!");
#endif

    return esp_vfs_fat_sdspi_mount(BSP_SD_MOUNT_POINT, cfg->host, cfg->slot.sdspi, cfg->mount, &bsp_sdcard);
}

esp_err_t bsp_sdcard_mount(void)
{
    bsp_sdcard_cfg_t cfg = {0};
    return bsp_sdcard_sdspi_mount(&cfg);
}

esp_err_t bsp_sdcard_unmount(void)
{
    esp_err_t ret = ESP_OK;

    ret |= esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
    bsp_sdcard = NULL;

    if (sdcard_spi_initialized) {
        ret |= spi_bus_free(BSP_SDSPI_HOST);
        sdcard_spi_initialized = false;
    }

    return ret;
}

esp_err_t bsp_speaker_new(const bsp_speaker_config_t *cfg, dac_continuous_handle_t *ret_handle)
{
    if (cfg == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    #ifdef CONFIG_BSP_SPEAKER_DAC_WRITE_ASYNC
    if (cfg->cbs == NULL || cbs->que == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    #endif

    dac_continuous_config_t cont_cfg = {
        .chan_mask = DAC_CHANNEL_MASK_CH1, // Our Speaker is connected to GPIO26
        .desc_num = cfg->desc_num,
        .buf_size = cfg->buf_size,
        .freq_hz = cfg->sample_rate,
        .offset = cfg->offset,
        .clk_src = DAC_DIGI_CLK_SRC_APLL,   // Using APLL as clock source to get a wider frequency range
        /* Assume the data in buffer is 'A B C D E F'
         * DAC_CHANNEL_MODE_SIMUL:
         *      - channel 0: A B C D E F
         *      - channel 1: A B C D E F
         * DAC_CHANNEL_MODE_ALTER:
         *      - channel 0: A C E
         *      - channel 1: B D F
         */
        .chan_mode = DAC_CHANNEL_MODE_SIMUL,
    };
    BSP_ERROR_CHECK_RETURN_ERR(dac_continuous_new_channels(&cont_cfg, ret_handle));
    
    #ifdef CONFIG_BSP_SPEAKER_DAC_WRITE_ASYNC
    BSP_ERROR_CHECK_RETURN_ERR(dac_continuous_register_event_callback(*ret_handle, cfg->cbs, *(cfg->que)));
    #endif // CONFIG_BSP_SPEAKER_DAC_WRITE_ASYNC

    BSP_ERROR_CHECK_RETURN_ERR(dac_continuous_enable(*ret_handle));
    
    ESP_LOGI(TAG, "Speaker initialized, DAC DMA is ready");

    return ESP_OK;
}

#define LCD_CMD_BITS    (8)
#define LCD_PARAM_BITS  (8)
#define LCD_LEDC_CH     (CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH)

esp_err_t bsp_display_brightness_init(void)
{
    // Setup LEDC peripheral for PWM backlight control
    const ledc_channel_config_t LCD_backlight_channel = {
        .gpio_num = BSP_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LCD_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 0,
        .hpoint = 0
    };
    const ledc_timer_config_t LCD_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = 1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    BSP_ERROR_CHECK_RETURN_ERR(ledc_timer_config(&LCD_backlight_timer));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_channel_config(&LCD_backlight_channel));

    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent > 100) {
        brightness_percent = 100;
    }
    if (brightness_percent < 0) {
        brightness_percent = 0;
    }

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness_percent);
    uint32_t duty_cycle = (1023 * brightness_percent) / 100; // LEDC resolution set to 10bits, thus: 100% = 1023
    BSP_ERROR_CHECK_RETURN_ERR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, duty_cycle));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH));

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

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel,
                          esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;
    assert(config != NULL && config->max_transfer_sz > 0);

    BSP_ERROR_CHECK_RETURN_ERR(bsp_feature_enable(BSP_FEATURE_LCD, true));

    /* Initialize SPI */
    ESP_RETURN_ON_ERROR(bsp_spi_init(config->max_transfer_sz), TAG, "");

    ESP_LOGI(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num       = BSP_LCD_DC,
        .cs_gpio_num       = BSP_LCD_CS,
        .pclk_hz           = BSP_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits      = LCD_CMD_BITS,
        .lcd_param_bits    = LCD_PARAM_BITS,
        .spi_mode          = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, ret_io), err, TAG,
                      "New panel IO failed");

    ESP_LOGI(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,  // Shared with Touch reset
        .rgb_ele_order  = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
    };

    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_ili9341(*ret_io, &panel_config, ret_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(*ret_panel);
    esp_lcd_panel_init(*ret_panel);
    esp_lcd_panel_disp_on_off(*ret_panel, true);

    return ret;

err:
    if (*ret_panel) {
        esp_lcd_panel_del(*ret_panel);
    }
    if (*ret_io) {
        esp_lcd_panel_io_del(*ret_io);
    }
    spi_bus_free(BSP_LCD_SPI_NUM);
    display_spi_initialized = false;
    return ret;
}

esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    /* Initialize touch */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max        = BSP_LCD_H_RES,
        .y_max        = BSP_LCD_V_RES,
        .rst_gpio_num = BSP_LCD_RST,  // Shared with LCD reset
        .int_gpio_num = BSP_TOUCH_INT,
        .flags = {
            .swap_xy = BSP_TOUCH_ROTATION_SWAP_XY,
            .mirror_x = BSP_TOUCH_ROTATION_MIRROR_X,
            .mirror_y = BSP_TOUCH_ROTATION_MIRROR_Y,
        },
    };

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(BSP_TOUCH_CS);
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &tp_io_config, &tp_io_handle),
                        TAG, "");
    return esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, ret_touch);
}

esp_err_t bsp_iot_button_create(button_handle_t btn_array[], int *btn_cnt, int btn_array_size)
{
    esp_err_t ret = ESP_OK;
    if ((btn_array_size < BSP_BUTTONS_NUM) || (btn_array == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (btn_cnt) {
        *btn_cnt = 0;
    }

    const button_config_t btn_config = {0};
    for (int i = 0; i < BSP_BUTTONS_NUM; i++) {
        if (bsp_button_config[i].type == BSP_BUTTON_TYPE_GPIO) {
            ret |= iot_button_new_gpio_device(&btn_config, &bsp_button_config[i].cfg.gpio, &btn_array[i]);
        } else if (bsp_button_config[i].type == BSP_BUTTON_TYPE_ADC) {
            ret |= iot_button_new_adc_device(&btn_config, &bsp_button_config[i].cfg.adc, &btn_array[i]);
        } else {
            ESP_LOGW(TAG, "Unsupported button type!");
        }

        if (btn_cnt) {
            (*btn_cnt)++;
        }
    }
    return ret;
}

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;
    const bsp_display_config_t bsp_disp_cfg = {
        .max_transfer_sz = BSP_LCD_DRAW_BUFF_SIZE * sizeof(uint16_t),
    };

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&bsp_disp_cfg, &panel_handle, &io_handle));

    esp_lcd_panel_disp_on_off(panel_handle, true);

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle     = io_handle,
        .panel_handle  = panel_handle,
        .buffer_size   = cfg->buffer_size,
        .double_buffer = cfg->double_buffer,
        .hres          = BSP_LCD_H_RES,
        .vres          = BSP_LCD_V_RES,
        .monochrome    = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = BSP_DISPLAY_ROTATION_SWAP_XY,
            .mirror_x = BSP_DISPLAY_ROTATION_MIRROR_X,
            .mirror_y = BSP_DISPLAY_ROTATION_MIRROR_Y,
        },
        .flags = {
            .buff_dma    = cfg->flags.buff_dma,
            .buff_spiram = cfg->flags.buff_spiram,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = (BSP_LCD_BIGENDIAN ? true : false),
#endif
        }
    };

    return lvgl_port_add_disp(&disp_cfg);
}

static lv_indev_t *bsp_display_indev_init(lv_display_t *disp)
{

    BSP_ERROR_CHECK_RETURN_NULL(bsp_touch_new(NULL, &tp));
    assert(tp);
    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp   = disp,
        .handle = tp,
    };

    return lvgl_port_add_touch(&touch_cfg);
}

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size   = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags         = {
            .buff_dma    = true,
            .buff_spiram = false,
        }
    };

    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);

    // Initialize keypad input device
    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);

    // Turn on backlight
    bsp_display_backlight_on();

    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev;
}

void bsp_display_rotate(lv_display_t *disp, lv_display_rotation_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}
#endif  // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)