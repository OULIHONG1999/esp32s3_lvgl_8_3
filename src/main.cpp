//
// Created by Administrator on 2025/1/15.
//

#include <cstdio>
#include "screen/lcd_drive/st7789_driver.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif


// 主程序
void app_main(void) {
    lcd_init(); // 初始化 LCD
    lcd_set_backlight(255); // 设置背光亮度为 50%
    app_lvgl();

    lv_obj_t *label = lv_obj_create(lv_scr_act());
    lv_label_set_text(label, "Hello World!");
    lv_obj_set_style_text_color(label, lv_color_hex(0xFF0000), LV_PART_MAIN);
    lv_obj_center(label);
    static int x = 0;

    while (1) {
        if (x++ % 10) {
            lv_label_set_text_fmt(label, "tcont %d", x);
            printf("x %d\n",x);
        }
        lv_task_handler();
        esp_task_wdt_reset();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

#ifdef __cplusplus
}
#endif
