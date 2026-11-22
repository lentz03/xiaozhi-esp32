#include "custom_lcd_display.h"
#include <esp_lcd_panel_io.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <stdio.h>
#include <vector>
#include <cstring>
#include <string>
#include <nvs.h>
#include "board.h"
#include "config.h"
#include "esp_lvgl_port.h"
#include "settings.h"
#include "lvgl_theme.h"
#include "assets/lang_config.h"

extern "C" {
LV_FONT_DECLARE(font_inkdeck_serif_34);
LV_FONT_DECLARE(font_inkdeck_serif_italic_28);
LV_FONT_DECLARE(font_inkdeck_sans_17);
}

#define TAG "CustomEpdDisplay"

#define BYTES_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565))
#define BUFF_SIZE (EXAMPLE_LCD_WIDTH * EXAMPLE_LCD_HEIGHT * BYTES_PER_PIXEL)

void CustomEpdDisplay::lvgl_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* color_p) {
    assert(disp != NULL);
    CustomEpdDisplay* driver = (CustomEpdDisplay*)lv_display_get_user_data(disp);
    uint16_t* buffer = (uint16_t*)color_p;
    driver->EPD_Clear();
    /* Inkdeck: LVGL draws a 480 x 800 portrait screen; turn it onto the 800 x 480 panel. */
    for (int y = area->y1; y <= area->y2; y++) {
        for (int x = area->x1; x <= area->x2; x++) {
            uint8_t color = (*buffer < 0x7fff) ? DRIVER_COLOR_BLACK : DRIVER_COLOR_WHITE;
            if (driver->rot_ == 0) driver->EPD_DrawColorPixel(driver->Width - 1 - y, x, color);
            else                   driver->EPD_DrawColorPixel(y, driver->Height - 1 - x, color);
            buffer++;
        }
    }
    driver->EPD_DisplayPart();
    lv_disp_flush_ready(disp);
}

CustomEpdDisplay::CustomEpdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                                   int width, int height, int offset_x, int offset_y, bool mirror_x,
                                   bool mirror_y, bool swap_xy, custom_epd_spi_t _epd_spi_data)
    : LcdDisplay(panel_io, panel, width, height),
      epd_spi_data(_epd_spi_data),
      Width(EXAMPLE_LCD_WIDTH),
      Height(EXAMPLE_LCD_HEIGHT) {
    nvs_handle_t nvs;
    int32_t rot = 0;
    if (nvs_open("inkdeck", NVS_READONLY, &nvs) == ESP_OK) {   /* same setting as the Inkdeck launcher */
        nvs_get_i32(nvs, "rot", &rot);
        nvs_close(nvs);
    }
    rot_ = rot & 1;
    ESP_LOGI(TAG, "Initialize SPI");
    spi_port_init();
    spi_gpio_init();

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    port_cfg.task_priority = 2;
    port_cfg.timer_period_ms = 50;
    lvgl_port_init(&port_cfg);
    lvgl_port_lock(0);

    buffer = (uint8_t*)heap_caps_malloc(epd_spi_data.buffer_len, MALLOC_CAP_SPIRAM);
    assert(buffer);
    display_ = lv_display_create(width, height); /* 以水平和垂直分辨率（像素）进行基本初始化 */
    lv_display_set_flush_cb(display_, lvgl_flush_cb);
    lv_display_set_user_data(display_, this);

    uint8_t* buffer_1 = NULL;
    buffer_1 = (uint8_t*)heap_caps_malloc(BUFF_SIZE, MALLOC_CAP_SPIRAM);
    assert(buffer_1);
    lv_display_set_buffers(display_, buffer_1, NULL, BUFF_SIZE, LV_DISPLAY_RENDER_MODE_FULL);

    ESP_LOGI(TAG, "EPD init");
    EPD_Init();
    ESP_LOGI(TAG, "EPD Clear");
    EPD_Clear();
    EPD_Display();

    lvgl_port_unlock();
    if (display_ == nullptr) {
        ESP_LOGE(TAG, "Failed to add display");
        return;
    }

    ESP_LOGI(TAG, "ui start");

    SetupUI();
}

CustomEpdDisplay::~CustomEpdDisplay() {}

void CustomEpdDisplay::spi_gpio_init() {
    int rst = epd_spi_data.rst;
    int cs = epd_spi_data.cs;
    int dc = epd_spi_data.dc;
    int busy = epd_spi_data.busy;

    gpio_config_t gpio_conf = {};
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = (0x1ULL << rst) | (0x1ULL << dc) | (0x1ULL << cs);
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&gpio_conf));

    gpio_conf.mode = GPIO_MODE_INPUT;
    gpio_conf.pin_bit_mask = (0x1ULL << busy);
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&gpio_conf));

    set_rst_1();
}

void CustomEpdDisplay::spi_port_init() {
    int mosi = epd_spi_data.mosi;
    int scl = epd_spi_data.scl;
    int spi_host = epd_spi_data.spi_host;
    esp_err_t ret;
    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num = -1;
    buscfg.mosi_io_num = mosi;
    buscfg.sclk_io_num = scl;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4096;

    spi_device_interface_config_t devcfg = {};
    devcfg.spics_io_num = -1;
    devcfg.clock_speed_hz = 20 * 1000 * 1000;  // Clock out at 10 MHz
    devcfg.mode = 0;                           // SPI mode 0
    devcfg.queue_size = 7;  // We want to be able to queue 7 transactions at a time

    ret = spi_bus_initialize((spi_host_device_t)spi_host, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device((spi_host_device_t)spi_host, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

void CustomEpdDisplay::read_busy() {
    int busy = epd_spi_data.busy;
    while (gpio_get_level((gpio_num_t)busy) == 1) {
        vTaskDelay(pdMS_TO_TICKS(5));  // LOW: idle, HIGH: busy
    }
}

void CustomEpdDisplay::SPI_SendByte(uint8_t data) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &data;
    ret = spi_device_polling_transmit(spi, &t);  // Transmit!
    assert(ret == ESP_OK);                       // Should have had no issues.
}

void CustomEpdDisplay::EPD_SendData(uint8_t data) {
    set_dc_1();
    set_cs_0();
    SPI_SendByte(data);
    set_cs_1();
}

void CustomEpdDisplay::EPD_SendCommand(uint8_t command) {
    set_dc_0();
    set_cs_0();
    SPI_SendByte(command);
    set_cs_1();
}

void CustomEpdDisplay::writeBytes(uint8_t* buffer, int len) {
    set_dc_1();
    set_cs_0();

    const int MAX_SPI_TRANSFER = 4096;

    int remaining = len;
    int offset = 0;

    while (remaining > 0) {
        int chunk_size = (remaining > MAX_SPI_TRANSFER) ? MAX_SPI_TRANSFER : remaining;

        esp_err_t ret;
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = 8 * chunk_size;
        t.tx_buffer = buffer + offset;
        ret = spi_device_polling_transmit(spi, &t);  // Transmit!

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI transmit failed at offset %d, chunk %d: %s", offset, chunk_size,
                     esp_err_to_name(ret));
            break;
        }

        remaining -= chunk_size;
        offset += chunk_size;
    }

    set_cs_1();
}

void CustomEpdDisplay::writeBytes(const uint8_t* buffer, int len) {
    set_dc_1();
    set_cs_0();

    const int MAX_SPI_TRANSFER = 4096;

    int remaining = len;
    int offset = 0;

    while (remaining > 0) {
        int chunk_size = (remaining > MAX_SPI_TRANSFER) ? MAX_SPI_TRANSFER : remaining;

        esp_err_t ret;
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = 8 * chunk_size;
        t.tx_buffer = buffer + offset;
        ret = spi_device_polling_transmit(spi, &t);  // Transmit!

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI transmit failed at offset %d, chunk %d: %s", offset, chunk_size,
                     esp_err_to_name(ret));
            break;
        }

        remaining -= chunk_size;
        offset += chunk_size;
    }

    set_cs_1();
}

void CustomEpdDisplay::EPD_SetWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend,
                                      uint16_t Yend) {
    EPD_SendCommand(0x44);  // SET_RAM_X_ADDRESS_START_END_POSITION
    EPD_SendData((Xstart * 8) & 0xFF);
    EPD_SendData(((Xstart * 8) >> 8) & 0xFF);
    EPD_SendData((Xend * 8) & 0xFF);
    EPD_SendData(((Xend * 8) >> 8) & 0xFF);

    EPD_SendCommand(0x45);  // SET_RAM_Y_ADDRESS_START_END_POSITION
    EPD_SendData(Yend & 0xFF);
    EPD_SendData((Yend >> 8) & 0xFF);
    EPD_SendData(Ystart & 0xFF);
    EPD_SendData((Ystart >> 8) & 0xFF);
}

void CustomEpdDisplay::EPD_SetCursor(uint16_t Xstart, uint16_t Ystart) {
    EPD_SendCommand(0x4E);
    EPD_SendData((Xstart * 8) & 0xFF);
    EPD_SendData(((Xstart * 8) >> 8) & 0xFF);

    EPD_SendCommand(0x4F);
    EPD_SendData(Ystart & 0xFF);
    EPD_SendData((Ystart >> 8) & 0xFF);
}

void CustomEpdDisplay::EPD_TurnOnDisplay() {
    EPD_SendCommand(0x22);
    EPD_SendData(0xF7);
    EPD_SendCommand(0x20);
    read_busy();
}

void CustomEpdDisplay::EPD_TurnOnDisplayPart() {
    EPD_SendCommand(0x22);
    EPD_SendData(0xFF);
    EPD_SendCommand(0x20);
    read_busy();
}

void CustomEpdDisplay::EPD_Init() {
    set_rst_1();
    vTaskDelay(pdMS_TO_TICKS(50));
    set_rst_0();
    vTaskDelay(pdMS_TO_TICKS(2));
    set_rst_1();
    vTaskDelay(pdMS_TO_TICKS(50));

    read_busy();
    EPD_SendCommand(0x12);  // SWRESET
    read_busy();

    EPD_SendCommand(0x18);
    EPD_SendData(0x80);

    EPD_SendCommand(0x0C);  // Driver output control
    EPD_SendData(0xAE);
    EPD_SendData(0xC7);
    EPD_SendData(0xC3);
    EPD_SendData(0xC0);
    EPD_SendData(0x80);

    EPD_SendCommand(0x01);  // Driver output control
    EPD_SendData((Height - 1) % 256);
    EPD_SendData((Height - 1) / 256);
    EPD_SendData(0x02);

    EPD_SendCommand(0x3C);  // BorderWavefrom
    EPD_SendData(0x01);

    EPD_SendCommand(0x11);  // data entry mode
    EPD_SendData(0x01);

    EPD_SendCommand(0x44);  // set Ram-X address start/end position
    EPD_SendData(0x00);
    EPD_SendData(0x00);
    EPD_SendData((Width - 1) % 256);
    EPD_SendData((Width - 1) / 256);

    EPD_SendCommand(0x45);  // set Ram-Y address start/end position
    EPD_SendData((Height - 1) % 256);
    EPD_SendData((Height - 1) / 256);
    EPD_SendData(0x00);
    EPD_SendData(0x00);

    EPD_SendCommand(0x4E);  // set RAM x address count to 0;
    EPD_SendData(0x00);
    EPD_SendData(0x00);
    EPD_SendCommand(0x4F);  // set RAM y address count to 0X199;
    EPD_SendData(0x00);
    EPD_SendData(0x00);

    read_busy();
}

void CustomEpdDisplay::EPD_Clear() {
    int buffer_len = epd_spi_data.buffer_len;
    memset(buffer, 0xff, buffer_len);
}

void CustomEpdDisplay::EPD_Display() {
    int buffer_len = epd_spi_data.buffer_len;
    EPD_SendCommand(0x24);
    assert(buffer);
    writeBytes(buffer, buffer_len);
    EPD_TurnOnDisplay();
}

void CustomEpdDisplay::EPD_DisplayPartBaseImage() {
    int buffer_len = epd_spi_data.buffer_len;
    EPD_SendCommand(0x24);
    assert(buffer);
    writeBytes(buffer, buffer_len);
    EPD_SendCommand(0x26);
    writeBytes(buffer, buffer_len);
    EPD_TurnOnDisplay();
}

void CustomEpdDisplay::EPD_Init_Partial() {
    set_rst_1();
    vTaskDelay(pdMS_TO_TICKS(50));
    set_rst_0();
    vTaskDelay(pdMS_TO_TICKS(2));
    set_rst_1();
    vTaskDelay(pdMS_TO_TICKS(50));

    read_busy();

    EPD_SendCommand(0x18);
    EPD_SendData(0x80);

    EPD_SendCommand(0x3C);
    EPD_SendData(0x80);

    EPD_SetWindows(0, Width - 1, Height - 1, 0);

    EPD_SetCursor(0, Height - 1);

    read_busy();
}

void CustomEpdDisplay::EPD_DisplayPart() {
    EPD_SendCommand(0x24);
    assert(buffer);
    writeBytes(buffer, 48000);
    EPD_TurnOnDisplayPart();
}

void CustomEpdDisplay::EPD_Sleep() {
    EPD_SendCommand(0x10);  // enter deep sleep
    EPD_SendData(0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    set_rst_0();
    set_cs_0();
    set_dc_0();
}

void CustomEpdDisplay::EPD_DrawColorPixel(uint16_t x, uint16_t y, uint8_t color) {
    if (x >= Width || y >= Height) {
        ESP_LOGE("EPD", "Out of bounds pixel: (%d,%d)", x, y);
        return;
    }

    uint16_t index = y * Width / 8 + (x >> 3);
    uint8_t bit = 7 - (x & 0x07);
    if (color == DRIVER_COLOR_WHITE) {
        buffer[index] |= (0x01 << bit);
    } else {
        buffer[index] &= ~(0x01 << bit);
    }
}

/* ------------------------------------------------------------ Inkdeck "Letter" chat screen */
static void ink_plain(lv_obj_t* o) {
    lv_obj_remove_style_all(o);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
}
static void ink_black(lv_obj_t* o) {
    lv_obj_set_style_bg_color(o, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
}
static void ink_show(lv_obj_t* o, bool on) {
    if (o == nullptr) return;
    if (on) lv_obj_remove_flag(o, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
}

void CustomEpdDisplay::SetupUI() {
    if (letter_ != nullptr) return;                     /* already built */
    LcdDisplay::SetupUI();
    DisplayLockGuard lock(this);
    auto lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    const lv_font_t* text_font = lvgl_theme->text_font()->font();
    serif_ = font_inkdeck_serif_34;
    serif_.fallback = text_font;
    ink_show(emoji_box_, false);
    ink_show(bottom_bar_, false);

    lv_obj_t* screen = lv_screen_active();
    const int W = LV_HOR_RES, H = LV_VER_RES, M = 28, top = 64, footer_y = H - 118;

    letter_ = lv_obj_create(screen);
    lv_obj_remove_style_all(letter_);
    lv_obj_set_pos(letter_, M, top);
    lv_obj_set_size(letter_, W - 2 * M, footer_y - top - 12);
    lv_obj_set_flex_flow(letter_, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(letter_, 8, 0);
    lv_obj_set_scrollbar_mode(letter_, LV_SCROLLBAR_MODE_OFF);

    auto label = [this](const lv_font_t* f) {
        lv_obj_t* l = lv_label_create(letter_);
        lv_obj_set_width(l, LV_PCT(100));
        lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_font(l, f, 0);
        lv_obj_set_style_text_color(l, lv_color_black(), 0);
        lv_label_set_text(l, "");
        return l;
    };
    me_tag_ = label(&font_inkdeck_sans_17);
    lv_obj_set_style_text_letter_space(me_tag_, 3, 0);
    lv_label_set_text(me_tag_, "ME");
    me_text_ = label(text_font);
    rule_ = lv_obj_create(letter_);
    ink_plain(rule_);
    ink_black(rule_);
    lv_obj_set_size(rule_, 32, 2);
    lv_obj_set_style_margin_top(rule_, 14, 0);
    lv_obj_set_style_margin_bottom(rule_, 14, 0);
    ai_tag_ = label(&font_inkdeck_sans_17);
    lv_obj_set_style_text_letter_space(ai_tag_, 3, 0);
    lv_label_set_text(ai_tag_, "AI");
    ai_text_ = label(&serif_);
    lv_obj_set_style_text_line_space(ai_text_, 4, 0);

    /* footer: hairline, state dot + word, sound wave, hint */
    lv_obj_t* line = lv_obj_create(screen);
    ink_plain(line);
    ink_black(line);
    lv_obj_set_pos(line, M, footer_y);
    lv_obj_set_size(line, W - 2 * M, 1);

    state_dot_ = lv_obj_create(screen);
    ink_plain(state_dot_);
    ink_black(state_dot_);
    lv_obj_set_style_radius(state_dot_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_color(state_dot_, lv_color_black(), 0);
    lv_obj_set_style_border_width(state_dot_, 2, 0);
    lv_obj_set_pos(state_dot_, M, footer_y + 34);
    lv_obj_set_size(state_dot_, 16, 16);

    state_label_ = lv_label_create(screen);
    lv_obj_set_style_text_font(state_label_, &font_inkdeck_serif_italic_28, 0);
    lv_obj_set_style_text_color(state_label_, lv_color_black(), 0);
    lv_obj_set_pos(state_label_, M + 28, footer_y + 22);
    lv_label_set_text(state_label_, "Ready");

    wave_ = lv_obj_create(screen);
    ink_plain(wave_);
    lv_obj_set_pos(wave_, W - M - 170, footer_y + 26);
    lv_obj_set_size(wave_, 170, 32);
    static const uint8_t heights[18] = { 10, 18, 26, 14, 30, 22, 12, 28, 20, 32, 16, 24, 30, 12, 22, 18, 26, 10 };
    for (int i = 0; i < 18; i++) {
        lv_obj_t* bar = lv_obj_create(wave_);
        ink_plain(bar);
        ink_black(bar);
        lv_obj_set_style_radius(bar, 2, 0);
        lv_obj_set_pos(bar, i * 9 + 2, (32 - heights[i]) / 2);
        lv_obj_set_size(bar, 6, heights[i]);
    }
    ink_show(wave_, false);

    lv_obj_t* hint = lv_label_create(screen);
    lv_obj_set_style_text_font(hint, &font_inkdeck_sans_17, 0);
    lv_obj_set_style_text_color(hint, lv_color_black(), 0);
    lv_obj_set_pos(hint, M, H - 42);
    lv_label_set_text(hint, "Tap BOOT to talk \xc2\xb7 say \xe2\x80\x9cHi ESP\xe2\x80\x9d to wake");

    RenderLetter();
}

/* Called with the display locked. */
void CustomEpdDisplay::RenderLetter() {
    if (letter_ == nullptr) return;
    const bool have_me = !last_user_.empty();
    ink_show(me_tag_, have_me);
    ink_show(me_text_, have_me);
    ink_show(rule_, have_me);
    if (have_me) lv_label_set_text(me_text_, last_user_.c_str());
    ink_show(ai_tag_, !reply_.empty() && !system_msg_);
    if (!reply_.empty()) lv_label_set_text(ai_text_, reply_.c_str());
    else if (!have_me) lv_label_set_text(ai_text_, "Say \xe2\x80\x9cHi ESP\xe2\x80\x9d, or tap BOOT to talk.");
    else lv_label_set_text(ai_text_, "");
    lv_obj_update_layout(letter_);
    lv_obj_scroll_to_view_recursive(ai_text_, LV_ANIM_OFF);
}

void CustomEpdDisplay::SetChatMessage(const char* role, const char* content) {
    if (letter_ == nullptr) {
        LcdDisplay::SetChatMessage(role, content);
        return;
    }
    if (content == nullptr || content[0] == 0) return;
    DisplayLockGuard lock(this);
    if (strcmp(role, "user") == 0) {
        last_user_ = content;
        reply_.clear();
        system_msg_ = false;
    } else if (strcmp(role, "assistant") == 0) {
        if (system_msg_) { reply_.clear(); system_msg_ = false; }
        if (!reply_.empty()) reply_ += " ";
        reply_ += content;
        if (reply_.size() > 420) {                      /* keep the end of a long answer on screen */
            size_t cut = reply_.size() - 420;
            while (cut < reply_.size() && (reply_[cut] & 0xC0) == 0x80) cut++;   /* UTF-8 boundary */
            reply_ = "\xe2\x80\xa6" + reply_.substr(cut);
        }
    } else {                                            /* system: activation code, errors */
        last_user_.clear();
        reply_ = content;
        system_msg_ = true;
    }
    RenderLetter();
}

void CustomEpdDisplay::ClearChatMessages() {
    LcdDisplay::ClearChatMessages();
    DisplayLockGuard lock(this);
    last_user_.clear();
    reply_.clear();
    system_msg_ = false;
    ink_show(bottom_bar_, false);
    RenderLetter();
}

void CustomEpdDisplay::SetStatus(const char* status) {
    LcdDisplay::SetStatus(status);
    UpdateFooter(status);
}

void CustomEpdDisplay::UpdateFooter(const char* status) {
    if (state_label_ == nullptr || status == nullptr) return;
    const char* word = nullptr;
    bool active = false, speaking = false;
    if (strcmp(status, Lang::Strings::LISTENING) == 0) { word = "Listening"; active = true; }
    else if (strcmp(status, Lang::Strings::SPEAKING) == 0) { word = "Speaking"; active = speaking = true; }
    else if (strcmp(status, Lang::Strings::STANDBY) == 0) word = "Ready";
    else if (strcmp(status, Lang::Strings::CONNECTING) == 0) word = "Connecting\xe2\x80\xa6";
    if (word == nullptr) return;                        /* other messages stay in the top bar */
    DisplayLockGuard lock(this);
    lv_label_set_text(state_label_, word);
    lv_obj_set_style_bg_opa(state_dot_, active ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
    ink_show(wave_, speaking);
}

void CustomEpdDisplay::SetTheme(Theme* theme) {
    LcdDisplay::SetTheme(theme);
    if (letter_ == nullptr) return;
    DisplayLockGuard lock(this);
    auto lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    const lv_font_t* text_font = lvgl_theme->text_font()->font();
    serif_.fallback = text_font;                        /* fonts can arrive later from the assets */
    lv_obj_set_style_text_font(me_text_, text_font, 0);
    lv_obj_set_style_text_font(ai_text_, &serif_, 0);
    ink_show(emoji_box_, false);
    ink_show(bottom_bar_, false);
}
