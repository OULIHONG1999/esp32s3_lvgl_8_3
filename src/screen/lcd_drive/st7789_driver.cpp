#include <cstdio>
#include "st7789_driver.h"

#include <esp_task_wdt.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "driver/ledc.h" // 包含 LEDC 头文件
#include "lvgl.h"

#undef NULL
#define NULL nullptr

#define DELAY_US(x)    (esp_rom_delay_us(x))
#define LCD_MODULE_CMD_1

void lcd_backlight_init();


// 全局变量
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;

// LCD 初始化命令序列
typedef struct {
    uint8_t cmd;
    uint8_t data[14];
    uint8_t len;
} lcd_cmd_t;


lcd_cmd_t lcd_init_cmds_170x320[] = {
    {0x11, {0}, 0 | 0x80},
    {0x3A, {0X05}, 1},
    {0xB2, {0X0B, 0X0B, 0X00, 0X33, 0X33}, 5},
    {0xB7, {0X75}, 1},
    {0xBB, {0X28}, 1},
    {0xC0, {0X2C}, 1},
    {0xC2, {0X01}, 1},
    {0xC3, {0X1F}, 1},
    {0xC6, {0X13}, 1},
    {0xD0, {0XA7}, 1},
    {0xD0, {0XA4, 0XA1}, 2},
    {0xD6, {0XA1}, 1},
    {0xE0, {0XF0, 0X05, 0X0A, 0X06, 0X06, 0X03, 0X2B, 0X32, 0X43, 0X36, 0X11, 0X10, 0X2B, 0X32}, 14},
    {0xE1, {0XF0, 0X08, 0X0C, 0X0B, 0X09, 0X24, 0X2B, 0X22, 0X43, 0X38, 0X15, 0X16, 0X2F, 0X37}, 14},
    {ST7789_MADCTL, {TFT_MAD_RGB}, 1}, // 内存访问控制模式设置，BGR模式
    {ST7789_DISPON, {0}, 0 | 0x80}, // 开启显示
};


// 初始化背光引脚
void lcd_rd_init() {
    // 配置 GPIO 为输出模式
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_LCD_RD), // 设置引脚掩码
        .mode = GPIO_MODE_OUTPUT, // 设置为输出模式
        .pull_up_en = GPIO_PULLUP_ENABLE, // 禁用上拉
        // .pull_up_en = GPIO_PULLUP_DISABLE, // 禁用上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 禁用下拉
        .intr_type = GPIO_INTR_DISABLE, // 禁用中断
    };
    gpio_config(&io_conf); // 应用配置

    // 设置背光引脚为高电平（开启背光）
    gpio_set_level((gpio_num_t) PIN_LCD_RD, 1);
}

// 初始化 LCD 屏幕
void lcd_init() {
    lcd_rd_init();
    // 初始化背光
    lcd_backlight_init();
    lcd_set_backlight(255);


    // 配置 I80 总线
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .dc_gpio_num = PIN_LCD_DC,
        .wr_gpio_num = PIN_LCD_WR,
        .clk_src = LCD_CLK_SRC_PLL160M,
        .data_gpio_nums = {
            PIN_LCD_D0,
            PIN_LCD_D1,
            PIN_LCD_D2,
            PIN_LCD_D3,
            PIN_LCD_D4,
            PIN_LCD_D5,
            PIN_LCD_D6,
            PIN_LCD_D7,
        },
        .bus_width = 8,
        .max_transfer_bytes = LCD_BUF_SIZE,
    };
    esp_lcd_new_i80_bus(&bus_config, &i80_bus);

    // 配置 LCD 面板 IO
    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = PIN_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 20,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

    // 配置 LCD 面板
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_LCD_RES,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
    };
    esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle);

    // 初始化 LCD 面板
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_invert_color(panel_handle, true);
    // 设置屏幕方向
    esp_lcd_panel_swap_xy(panel_handle, true);
    esp_lcd_panel_mirror(panel_handle, false, true);
    esp_lcd_panel_set_gap(panel_handle, 0, 35);


    for (int i = 0; i < (sizeof(lcd_init_cmds_170x320) / sizeof(lcd_cmd_t)); i++) {
        esp_lcd_panel_io_tx_param(io_handle, lcd_init_cmds_170x320[i].cmd, lcd_init_cmds_170x320[i].data,
                                  lcd_init_cmds_170x320[i].len & 0x7f);
        if (lcd_init_cmds_170x320[i].len & 0x80) {
            DELAY_US(120);
        }
    }
}


// 初始化背光引脚
void lcd_backlight_init() {
    // 配置 GPIO 为输出模式
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_LCD_BL), // 设置引脚掩码
        .mode = GPIO_MODE_OUTPUT, // 设置为输出模式
        .pull_up_en = GPIO_PULLUP_DISABLE, // 禁用上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 禁用下拉
        .intr_type = GPIO_INTR_DISABLE, // 禁用中断
    };
    gpio_config(&io_conf); // 应用配置

    // 设置背光引脚为高电平（开启背光）
    gpio_set_level((gpio_num_t) PIN_LCD_BL, 1);
}


// 设置背光亮度（PWM 控制）
void lcd_set_backlight(uint8_t brightness) {
    // 配置 PWM
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT, // 8 位分辨率（0-255）
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000, // PWM 频率 5 kHz
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);

    // 配置 PWM 通道
    ledc_channel_config_t channel_conf = {
        .gpio_num = PIN_LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = brightness, // 设置占空比
        .hpoint = 0,
    };
    ledc_channel_config(&channel_conf);

    // 更新 PWM 占空比
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// 绘制矩形区域
void lcd_draw_bitmap(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const uint16_t *color_data) {
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2, y2, color_data);
}


static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv; // contains callback functions
static lv_color_t *lv_disp_buf;

void keyboard_read(struct _lv_indev_drv_t *indev_drv, lv_indev_data_t *data);

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1 + 35;
    int offsetx2 = area->x2 + 35;
    int offsety1 = area->y1 - 35;
    int offsety2 = area->y2 - 35;

    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    // lv_disp_flush_ready(drv);
}

// 定义一个符合 LVGL 打印函数签名的函数
void my_print(const char *buf) {
    // 调用 Arduino 的 Serial.printf 函数
    printf("%s", buf);
}


void app_lvgl(void) {
    lv_log_register_print_cb(my_print);

    lv_init();

    lv_disp_buf = (lv_color_t *) heap_caps_malloc(LVGL_LCD_BUF_SIZE * sizeof(lv_color_t),
                                                  MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

    // static lv_color_t *lv_disp_buf = (lv_color_t *)ps_malloc(LVGL_LCD_BUF_SIZE * sizeof(lv_color_t)); /*A buffer for 10 rows*/
    lv_disp_draw_buf_init(&disp_buf, lv_disp_buf, NULL, LVGL_LCD_BUF_SIZE);

    /*Initialize the display*/
    lv_disp_drv_init(&disp_drv);

    /*Change the following line to your display resolution*/
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    disp_drv.sw_rotate = 1;
    // disp_drv.rotated = LV_DISP_ROT_270;
    lv_disp_drv_register(&disp_drv);


    lv_group_t *g = lv_group_create();
    lv_group_set_default(g);

    static lv_indev_drv_t indev_drv_2;
    lv_indev_drv_init(&indev_drv_2); /*Basic initialization*/
    indev_drv_2.type = LV_INDEV_TYPE_KEYPAD;
    indev_drv_2.read_cb = keyboard_read;
    lv_indev_t *kb_indev = lv_indev_drv_register(&indev_drv_2);
    lv_indev_set_group(kb_indev, g);
}


static char buf[32];

uint32_t keycode_to_ctrl_key() {
    return 0;
}

static void event_handler(lv_timer_t *t) {
    uint32_t ctrl_key = keycode_to_ctrl_key();
    //    printf("ctrl_key %X\n", ctrl_key);
    const size_t len = strlen(buf);
    if (len < 32 - 1) {
        buf[len] = (char) ctrl_key;
        buf[len + 1] = '\0';
    }
}

void keyboard_read(struct _lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
    (void) indev_drv; /*Unused*/

    static bool dummy_read = false;
    const size_t len = strlen(buf);

    /*Send a release manually*/
    if (dummy_read) {
        dummy_read = false;
        data->state = LV_INDEV_STATE_RELEASED;
        data->continue_reading = len > 0;
    }
    /*Send the pressed character*/
    else if (len > 0) {
        dummy_read = true;
        data->state = LV_INDEV_STATE_PRESSED;
        data->key = buf[0];
        memmove(buf, buf + 1, len);
        data->continue_reading = true;
    }
}
