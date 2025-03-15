//
// Created by Administrator on 2025/1/15.
//

#include <cstdio>
#include "screen/lcd_drive/st7789_driver.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include <lvgl.h>
#include <esp_rom_gpio.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

void init_power_io() {
    // 拉高48脚
    esp_rom_gpio_pad_pullup_only(PIN_POWER_ON);
// 定义 GPIO 配置结构体
    gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << PIN_POWER_ON,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
    };

    // 应用配置
    gpio_config(&io_conf);

    // 设置 GPIO3 为高电平
    gpio_set_level((gpio_num_t)PIN_POWER_ON, 1);
}


// 主程序
void app_main(void) {
    init_power_io();
    lcd_init(); // 初始化 LCD
    lcd_set_backlight(255); // 设置背光亮度为 50%
    app_lvgl();

    lv_obj_t *label = lv_obj_create(lv_scr_act());
    lv_label_set_text(label, "Hello World!");
    lv_obj_set_style_text_color(label, lv_color_hex(0xFF0000), LV_PART_MAIN);
    lv_obj_center(label);
    static int x = 0;

    while (1) {
        lv_task_handler();
        esp_task_wdt_reset();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

#ifdef __cplusplus
}
#endif
