#include "ui.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "agent/agent_runtime.h"
#include "app/app_config.h"
#include "app/app_state.h"
#include "bsp/esp-bsp.h"
#include "control/control.h"
#include "network/device_auth_store.h"
#include "esp_log.h"
#include "perception/camera_app.h"
#include "perception/face_service.h"
#include "lvgl.h"
#include "perception/object_service.h"
#include "protocol/robot_protocol.h"
#include "audio/voice_player.h"
#include "network/wifi_mgr.h"
#include "diagnostics/startup_selftest.h"

LV_FONT_DECLARE(td_font_16);
LV_FONT_DECLARE(td_font_24);

static const char *TAG = "ui";

#define TD_COLOR_BG       0x000000u
#define TD_COLOR_PANEL    0x0D1C15u
#define TD_COLOR_GREEN    0x63E69Au
#define TD_COLOR_TEXT     0xE8F7EEu
#define TD_COLOR_MUTED    0x8DA99Au
#define TD_COLOR_BLUE     0x66B8FFu
#define TD_COLOR_AMBER    0xF4C85Bu
#define TD_COLOR_RED      0xFF6B61u

#define TD_CAMERA_PREVIEW_TOP 34
#define TD_CAMERA_STATUS_TOP (TD_CAMERA_PREVIEW_TOP + TD_CAMERA_PREVIEW_H + 3)
#define TD_CAMERA_STATUS_ROW_HEIGHT 22
_Static_assert(TD_CAMERA_STATUS_TOP + 2 * TD_CAMERA_STATUS_ROW_HEIGHT <= 397,
               "camera status rows must fit above self-test");

static lv_obj_t *s_camera_viewport;
static lv_obj_t *s_camera_placeholder;
static lv_obj_t *s_camera_canvas;
static lv_obj_t *s_track_speed;
static lv_obj_t *s_mode_label;
static lv_obj_t *s_voice_button;
static lv_obj_t *s_selftest_label;
static lv_obj_t *s_net_label;
static lv_obj_t *s_url_label;
static lv_obj_t *s_camera_label;
static lv_obj_t *s_uart_label;
static lv_obj_t *s_line_label;
static lv_obj_t *s_target_label;
static lv_obj_t *s_fps_label;
static lv_obj_t *s_sensor_label;
static lv_obj_t *s_face_label;
static lv_obj_t *s_face_panel;
static lv_obj_t *s_face_status_label;
static lv_obj_t *s_face_count_label;
static lv_obj_t *s_face_table_label;
static lv_obj_t *s_face_box;
static lv_obj_t *s_face_camera_slot;
static lv_obj_t *s_object_list;
static lv_obj_t *s_object_boxes[TD_MAX_OBJECTS];
static lv_obj_t *s_c30d_panel;
static lv_obj_t *s_c30d_summary_label;
static lv_obj_t *s_c30d_history_label;
static lv_obj_t *s_network_panel;
static lv_obj_t *s_network_summary_label;
static lv_obj_t *s_ai_panel;
static lv_obj_t *s_ai_mode_label;
static lv_obj_t *s_ai_lease_label;
static lv_obj_t *s_ai_plan_label;
static lv_obj_t *s_ai_blocked_label;
static lv_obj_t *s_ai_actions_label;
static lv_obj_t *s_ai_user_label;
static lv_obj_t *s_ai_reply_label;
static lv_obj_t *s_ai_tool_label;
static lv_obj_t *s_ai_action_label;
static lv_obj_t *s_ai_error_label;
static lv_obj_t *s_ai_takeover_button;
static uint16_t s_face_last_count = UINT16_MAX;
static td_c30d_sample_t s_ui_c30d_history[TD_C30D_HISTORY_CAPACITY];
static char s_ui_network_text[768];
static char s_ui_summary[1024];
static char s_ui_history[1280];
static char s_ui_face_table[1024];

static int scaled(int value)
{
    return value * TD_CAMERA_PREVIEW_SCALE / 256;
}

static lv_obj_t *make_panel(lv_obj_t *parent,
                            int x,
                            int y,
                            int width,
                            int height,
                            uint32_t color,
                            int radius)
{
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_set_pos(panel, x, y);
    lv_obj_set_size(panel, width, height);
    lv_obj_set_style_bg_color(panel, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_border_color(panel, lv_color_hex(0x254438), 0);
    lv_obj_set_style_radius(panel, radius, 0);
    lv_obj_set_style_pad_all(panel, 0, 0);
    lv_obj_set_scrollable(panel, false);
    return panel;
}

static lv_obj_t *make_label(lv_obj_t *parent,
                            const char *text,
                            int x,
                            int y,
                            const lv_font_t *font,
                            uint32_t color)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
    return label;
}

static lv_obj_t *make_status_label(lv_obj_t *parent,
                                   const char *text,
                                   int x,
                                   int y,
                                   int width,
                                   uint32_t color)
{
    lv_obj_t *label =
        make_label(parent, text, x, y, &td_font_16, color);
    lv_obj_set_width(label, width);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    return label;
}

static lv_obj_t *make_button(lv_obj_t *parent,
                             const char *text,
                             int x,
                             int y,
                             int width,
                             int height,
                             uint32_t color,
                             lv_event_cb_t callback,
                             intptr_t data)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, width, height);
    lv_obj_set_style_bg_color(button, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(button, 1, 0);
    lv_obj_set_style_border_color(button, lv_color_hex(0x3D6655), 0);
    lv_obj_set_style_radius(button, 9, 0);
    lv_obj_set_style_shadow_width(button, 0, 0);
    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, &td_font_16, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0xF0FAF4), 0);
    lv_obj_center(label);
    if (callback != NULL) {
        lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED,
                            (void *)data);
    }
    return button;
}

/* Each movement captures its selected command percentage when it starts. */
static uint8_t selected_track_speed(void)
{
    unsigned speed = TD_USER_TRACK_MIN_PERCENT +
                     lv_dropdown_get_selected(s_track_speed) * TD_USER_TRACK_STEP_PERCENT;
    return (uint8_t)(speed > TD_USER_TRACK_MAX_PERCENT ? TD_USER_TRACK_MAX_PERCENT : speed);
}

static void manual_event_cb(lv_event_t *event)
{
    intptr_t action = (intptr_t)lv_event_get_user_data(event);
    lv_event_code_t code = lv_event_get_code(event);

    if (code == LV_EVENT_PRESSED) {
        int speed = selected_track_speed();
        switch (action) {
        case 1:
            control_manual(speed, speed, TD_MANUAL_MAX_HOLD_MS);
            break;
        case 2:
            control_manual(-speed * 35 / 42, -speed * 35 / 42, TD_MANUAL_MAX_HOLD_MS);
            break;
        case 3:
            control_manual(-speed * 28 / 42, speed * 34 / 42, TD_MANUAL_MAX_HOLD_MS);
            break;
        case 4:
            control_manual(speed * 34 / 42, -speed * 28 / 42, TD_MANUAL_MAX_HOLD_MS);
            break;
        default:
            control_stop();
            break;
        }
    } else if (code == LV_EVENT_RELEASED ||
               code == LV_EVENT_PRESS_LOST) {
        control_stop();
    }
}

static lv_obj_t *make_hold_button(lv_obj_t *parent,
                                  const char *text,
                                  lv_coord_t x, lv_coord_t y,
                                  lv_coord_t width, lv_coord_t height,
                                  uint32_t color, intptr_t action)
{
    lv_obj_t *button = make_button(parent, text, x, y, width, height,
                                   color, NULL, action);
    lv_obj_add_event_cb(button, manual_event_cb, LV_EVENT_PRESSED,
                        (void *)action);
    lv_obj_add_event_cb(button, manual_event_cb, LV_EVENT_RELEASED,
                        (void *)action);
    lv_obj_add_event_cb(button, manual_event_cb, LV_EVENT_PRESS_LOST,
                        (void *)action);
    return button;
}

static void start_event_cb(lv_event_t *event)
{
    (void)event;
    if (control_start_line_speed(selected_track_speed())) {
        voice_play_async("line_follow");
    }
}

static void stop_event_cb(lv_event_t *event)
{
    (void)event;
    control_halt();
}

static void estop_event_cb(lv_event_t *event)
{
    (void)event;
    control_estop();
    voice_play_async("obstacle");
}

static void clear_event_cb(lv_event_t *event)
{
    (void)event;
    control_clear_estop();
}

static void inspect_event_cb(lv_event_t *event)
{
    (void)event;
    control_start_inspect();
    voice_play_async("inspect");
}

static void voice_test_event_cb(lv_event_t *event)
{
    (void)event;
    voice_play_async("say_test");
}

static void pan_left_event_cb(lv_event_t *event)
{
    (void)event;
    td_status_t status;
    app_state_snapshot(&status);
    uint16_t target = status.pan_deg > 30u ? status.pan_deg - 30u : 0u;
    control_actuator(target, RP_PUSH_STOP);
}

static void pan_right_event_cb(lv_event_t *event)
{
    (void)event;
    td_status_t status;
    app_state_snapshot(&status);
    uint16_t target = status.pan_deg < RP_PAN_MAX_DEG - 30u ?
        status.pan_deg + 30u : RP_PAN_MAX_DEG;
    control_actuator(target, RP_PUSH_STOP);
}

static void face_place_canvas(bool registration_view)
{
    if (s_camera_canvas == NULL) {
        return;
    }
    if (registration_view && s_face_camera_slot != NULL) {
        lv_obj_set_parent(s_camera_canvas, s_face_camera_slot);
        lv_image_set_pivot(s_camera_canvas, TD_CAM_VIEW_WIDTH / 2, TD_CAM_VIEW_HEIGHT / 2);
        lv_image_set_scale(s_camera_canvas, 171);
        lv_obj_set_pos(s_camera_canvas, -80, -60);
    } else if (s_camera_viewport != NULL) {
        lv_obj_set_parent(s_camera_canvas, s_camera_viewport);
        lv_image_set_pivot(s_camera_canvas, 0, 0);
        lv_image_set_scale(s_camera_canvas, TD_CAMERA_PREVIEW_SCALE);
        lv_obj_set_pos(s_camera_canvas, 0, 0);
    }
    lv_obj_set_size(s_camera_canvas, TD_CAM_VIEW_WIDTH, TD_CAM_VIEW_HEIGHT);
    lv_obj_set_scrollable(s_camera_canvas, false);
}

static void face_open_event_cb(lv_event_t *event)
{
    (void)event;
    if (s_face_panel == NULL) {
        return;
    }
    if (s_c30d_panel != NULL) {
        lv_obj_set_hidden(s_c30d_panel, true);
    }
    if (s_ai_panel != NULL) {
        lv_obj_set_hidden(s_ai_panel, true);
    }
    face_place_canvas(true);
    lv_obj_set_hidden(s_face_panel, false);
    lv_obj_move_foreground(s_face_panel);
}

static void face_close_event_cb(lv_event_t *event)
{
    (void)event;
    if (s_face_panel != NULL) {
        lv_obj_set_hidden(s_face_panel, true);
    }
    face_place_canvas(false);
    for (int i = 0; i < TD_MAX_OBJECTS; ++i) {
        if (s_object_boxes[i] != NULL) {
            lv_obj_move_foreground(s_object_boxes[i]);
        }
    }
    lv_obj_move_foreground(s_object_list);
    lv_obj_move_foreground(s_face_box);
}

static void face_enroll_event_cb(lv_event_t *event)
{
    (void)event;
    if (face_service_request_enroll() == ESP_OK) {
        voice_play_async("face_found");
    }
}

static void face_delete_event_cb(lv_event_t *event)
{
    (void)event;
    (void)face_service_delete_last();
}

static void face_clear_event_cb(lv_event_t *event)
{
    (void)event;
    (void)face_service_clear_all();
}

static void c30d_open_event_cb(lv_event_t *event)
{
    (void)event;
    if (s_c30d_panel == NULL) {
        return;
    }
    if (s_face_panel != NULL) {
        lv_obj_set_hidden(s_face_panel, true);
    }
    if (s_ai_panel != NULL) {
        lv_obj_set_hidden(s_ai_panel, true);
    }
    face_place_canvas(false);
    lv_obj_set_hidden(s_c30d_panel, false);
    lv_obj_move_foreground(s_c30d_panel);
}

static void c30d_close_event_cb(lv_event_t *event)
{
    (void)event;
    if (s_c30d_panel != NULL) {
        lv_obj_set_hidden(s_c30d_panel, true);
    }
}

static void network_open_event_cb(lv_event_t *event)
{
    (void)event;
    if (s_network_panel == NULL) {
        return;
    }
    if (s_face_panel != NULL) {
        lv_obj_set_hidden(s_face_panel, true);
    }
    if (s_c30d_panel != NULL) {
        lv_obj_set_hidden(s_c30d_panel, true);
    }
    if (s_ai_panel != NULL) {
        lv_obj_set_hidden(s_ai_panel, true);
    }
    face_place_canvas(false);
    lv_obj_set_hidden(s_network_panel, false);
    lv_obj_move_foreground(s_network_panel);
}

static void network_close_event_cb(lv_event_t *event)
{
    (void)event;
    if (s_network_panel != NULL) {
        lv_obj_set_hidden(s_network_panel, true);
    }
}

static void network_station_event_cb(lv_event_t *event)
{
    (void)event;
    (void)wifi_mgr_set_mode(TD_WIFI_MODE_STATION);
}

static void network_ap_event_cb(lv_event_t *event)
{
    (void)event;
    (void)wifi_mgr_set_mode(TD_WIFI_MODE_ACCESS_POINT);
}

static void network_setup_event_cb(lv_event_t *event)
{
    (void)event;
    (void)wifi_mgr_start_setup_portal();
}

static void network_forget_event_cb(lv_event_t *event)
{
    (void)event;
    (void)wifi_mgr_forget_station();
}

static uint32_t object_group_color(uint8_t category)
{
    if (category == 0u) {
        return TD_COLOR_RED;
    }
    if (category >= 1u && category <= 8u) {
        return TD_COLOR_BLUE;
    }
    if (category >= 14u && category <= 23u) {
        return TD_COLOR_AMBER;
    }
    if (category == 58u) {
        return TD_COLOR_GREEN;
    }
    return 0x55D9CFu;
}

static void create_detection_overlay(void)
{
    for (int i = 0; i < TD_MAX_OBJECTS; ++i) {
        lv_obj_t *box = lv_obj_create(s_camera_viewport);
        lv_obj_set_size(box, 30, 24);
        lv_obj_set_style_bg_opa(box, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(box, 2, 0);
        lv_obj_set_style_border_color(box, lv_color_hex(TD_COLOR_BLUE), 0);
        lv_obj_set_style_radius(box, 3, 0);
        lv_obj_set_style_pad_all(box, 0, 0);
        lv_obj_set_hidden(box, true);
        s_object_boxes[i] = box;
    }

    s_object_list = make_label(
        s_camera_viewport, "物体识别", 8, 38,
        &td_font_16, 0xFFFFFF);
    lv_obj_set_width(s_object_list, 188);
    lv_label_set_long_mode(s_object_list, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_bg_color(s_object_list,
                              lv_color_hex(TD_COLOR_PANEL), 0);
    lv_obj_set_style_bg_opa(s_object_list, LV_OPA_80, 0);
    lv_obj_set_style_pad_all(s_object_list, 4, 0);
    lv_obj_set_hidden(s_object_list, true);

    s_face_box = lv_obj_create(s_camera_viewport);
    lv_obj_set_size(s_face_box, 30, 30);
    lv_obj_set_style_bg_opa(s_face_box, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_face_box, 2, 0);
    lv_obj_set_style_border_color(s_face_box, lv_color_hex(TD_COLOR_BLUE), 0);
    lv_obj_set_style_radius(s_face_box, 4, 0);
    lv_obj_set_style_pad_all(s_face_box, 0, 0);
    lv_obj_set_hidden(s_face_box, true);
}

static void create_face_panel(lv_obj_t *screen)
{
    s_face_panel = make_panel(screen, 10, 10, 780, 460,
                              TD_COLOR_PANEL, 14);
    lv_obj_set_style_bg_opa(s_face_panel, LV_OPA_COVER, 0);
    lv_obj_set_hidden(s_face_panel, true);

    make_label(s_face_panel, "人脸登记表", 20, 14,
               &td_font_24, TD_COLOR_GREEN);
    s_face_count_label = make_status_label(
        s_face_panel, "登记人数：0", 300, 19, 220, TD_COLOR_BLUE);
    lv_obj_set_style_text_align(s_face_count_label,
                                LV_TEXT_ALIGN_CENTER, 0);
    make_button(s_face_panel, "返回", 650, 12, 110, 38,
                0x233D32, face_close_event_cb, 0);

    s_face_camera_slot = make_panel(
        s_face_panel, 20, 62, 320, 240, 0x000000, 10);
    s_face_status_label = make_status_label(
        s_face_panel, "人脸功能启动中", 20, 310, 320, 0xD3E5DA);

    make_button(s_face_panel, "拍摄登记", 20, 350, 120, 38,
                0x17452E, face_enroll_event_cb, 0);
    make_button(s_face_panel, "删除最后", 152, 350, 120, 38,
                0x26362E, face_delete_event_cb, 0);
    make_button(s_face_panel, "清空人脸库", 284, 350, 120, 38,
                0x5A1C18, face_clear_event_cb, 0);

    s_face_table_label = make_label(
        s_face_panel,
        "编号      人员          状态\n"
        "----------------------------------------\n"
        "暂无登记人员",
        360, 66, &td_font_16, TD_COLOR_TEXT);
    lv_obj_set_size(s_face_table_label, 400, 270);
    lv_label_set_long_mode(s_face_table_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_line_space(s_face_table_label, 7, 0);

    make_label(s_face_panel,
               "实时画面位于左侧；特征仅保存在本机，不保存人脸照片。",
               20, 414, &td_font_16, TD_COLOR_MUTED);
}

static void create_c30d_panel(lv_obj_t *screen)
{
    s_c30d_panel = make_panel(screen, 10, 10, 780, 460,
                              TD_COLOR_PANEL, 14);
    lv_obj_set_style_bg_opa(s_c30d_panel, LV_OPA_COVER, 0);
    lv_obj_set_hidden(s_c30d_panel, true);

    make_label(s_c30d_panel, "C30D 数据监控", 20, 14,
               &td_font_24, TD_COLOR_GREEN);
    make_button(s_c30d_panel, "返回", 650, 12, 110, 38,
                0x233D32, c30d_close_event_cb, 0);

    s_c30d_summary_label = make_label(
        s_c30d_panel,
        "USB：等待连接\n"
        "数据：未收到\n"
        "C30D固件：未知\n\n"
        "接收帧：0\n"
        "CRC错误：0\n"
        "长度错误：0\n\n"
        "电压：--\n"
        "距离：--\n"
        "航向：--\n"
        "速度：--",
        20, 62, &td_font_16, TD_COLOR_TEXT);
    lv_obj_set_size(s_c30d_summary_label, 360, 340);
    lv_obj_set_style_text_line_space(s_c30d_summary_label, 8, 0);

    s_c30d_history_label = make_label(
        s_c30d_panel,
        "最近记录\n"
        "等待 C30D 数据",
        400, 62, &td_font_16, TD_COLOR_TEXT);
    lv_obj_set_size(s_c30d_history_label, 360, 340);
    lv_label_set_long_mode(s_c30d_history_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_line_space(s_c30d_history_label, 6, 0);

    make_label(s_c30d_panel,
               "记录保存在内存，最近 60 秒；可通过 /api/c30d.csv 下载。",
               20, 414, &td_font_16, TD_COLOR_MUTED);
}

static void create_network_panel(lv_obj_t *screen)
{
    s_network_panel = make_panel(screen, 10, 10, 780, 460,
                                 TD_COLOR_PANEL, 14);
    lv_obj_set_style_bg_opa(s_network_panel, LV_OPA_COVER, 0);
    lv_obj_set_hidden(s_network_panel, true);

    make_label(s_network_panel, "网络设置", 20, 14,
               &td_font_24, TD_COLOR_GREEN);
    make_button(s_network_panel, "返回", 650, 12, 110, 38,
                0x233D32, network_close_event_cb, 0);

    s_network_summary_label = make_label(
        s_network_panel,
        "网络状态读取中",
        20, 64, &td_font_16, TD_COLOR_TEXT);
    lv_obj_set_size(s_network_summary_label, 740, 218);
    lv_label_set_long_mode(s_network_summary_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_line_space(s_network_summary_label, 6, 0);

    make_button(s_network_panel, "作为终端联网", 20, 290, 170, 44,
                0x17452E, network_station_event_cb, 0);
    make_button(s_network_panel, "保留热点模式", 205, 290, 170, 44,
                0x26362E, network_ap_event_cb, 0);
    make_button(s_network_panel, "打开配置热点", 390, 290, 170, 44,
                0x4A3A12, network_setup_event_cb, 0);
    make_button(s_network_panel, "清除已保存网络", 575, 290, 170, 44,
                0x5A1C18, network_forget_event_cb, 0);

    make_label(
        s_network_panel,
        "路由器名称和密码请在浏览器中填写：连接配置热点\n"
        "Goumang-S31-Setup 后，打开上方显示的网址。\n\n"
        "正常情况下 S31 是路由器终端，不常驻热点。\n"
        "终端连接失败 20 秒后会自动进入配置热点。",
        20, 350, &td_font_16, TD_COLOR_MUTED);
}

static void ai_close_event_cb(lv_event_t *event)
{
    (void)event;
    if (s_ai_panel != NULL) {
        lv_obj_set_hidden(s_ai_panel, true);
    }
}

static void ai_open_event_cb(lv_event_t *event)
{
    (void)event;
    if (s_ai_panel == NULL) {
        return;
    }
    if (s_face_panel != NULL) {
        lv_obj_set_hidden(s_face_panel, true);
    }
    if (s_c30d_panel != NULL) {
        lv_obj_set_hidden(s_c30d_panel, true);
    }
    if (s_network_panel != NULL) {
        lv_obj_set_hidden(s_network_panel, true);
    }
    face_place_canvas(false);
    lv_obj_set_hidden(s_ai_panel, false);
    lv_obj_move_foreground(s_ai_panel);
}

static void ui_ai_update_labels(const agent_state_t *state)
{
    if (s_ai_panel == NULL || state == NULL) {
        return;
    }

    lv_label_set_text_fmt(s_ai_mode_label, "MODE: %s",
                          state->takeover ? "TAKEOVER" : "ASSIST");
    lv_obj_set_style_text_color(
        s_ai_mode_label,
        lv_color_hex(state->takeover ? TD_COLOR_AMBER : TD_COLOR_BLUE), 0);

    if (state->lease_remaining_ms >= 1000u) {
        lv_label_set_text_fmt(
            s_ai_lease_label, "LEASE: %lu s",
            (unsigned long)((state->lease_remaining_ms + 999u) / 1000u));
    } else if (state->lease_remaining_ms > 0u) {
        lv_label_set_text(s_ai_lease_label, "LEASE: <1 s");
    } else {
        lv_label_set_text(s_ai_lease_label, "LEASE: --");
    }

    if (state->plan_total > 0u) {
        lv_label_set_text_fmt(s_ai_plan_label, "PLAN: %u/%u %s",
                              (unsigned int)state->plan_done,
                              (unsigned int)state->plan_total,
                              state->plan_running ? "RUN" : "IDLE");
    } else {
        lv_label_set_text(s_ai_plan_label, "PLAN: -");
    }
    lv_label_set_text_fmt(s_ai_blocked_label, "BLOCKED: %lu",
                          (unsigned long)state->blocked_actions);
    lv_label_set_text_fmt(s_ai_actions_label, "ACTIONS: %lu",
                          (unsigned long)state->actions_total);

    lv_label_set_text(s_ai_user_label,
                      state->last_user[0] != '\0' ? state->last_user : "-");
    lv_label_set_text(s_ai_reply_label,
                      state->last_reply[0] != '\0' ? state->last_reply : "-");
    lv_label_set_text_fmt(s_ai_tool_label, "TOOL: %s",
                          state->last_tool[0] != '\0' ?
                          state->last_tool : "-");
    lv_label_set_text_fmt(s_ai_action_label, "ACTION: %s",
                          state->last_action[0] != '\0' ?
                          state->last_action : "-");

    if (state->plan_error[0] != '\0') {
        lv_label_set_text_fmt(s_ai_error_label, "PLAN ERROR: %s",
                              state->plan_error);
        lv_obj_set_style_text_color(s_ai_error_label,
                                    lv_color_hex(TD_COLOR_RED), 0);
    } else {
        lv_label_set_text(s_ai_error_label, "PLAN ERROR: -");
        lv_obj_set_style_text_color(s_ai_error_label,
                                    lv_color_hex(TD_COLOR_MUTED), 0);
    }

    lv_label_set_text(lv_obj_get_child(s_ai_takeover_button, 0),
                      state->takeover ? "REVOKE TAKEOVER" : "TAKEOVER 60s");
    lv_obj_set_style_bg_color(
        s_ai_takeover_button,
        lv_color_hex(state->takeover ? 0x5A1C18 : 0x17452E), 0);
}

static void ui_ai_refresh_now(void)
{
    agent_state_t state;
    agent_runtime_state_snapshot(&state);
    ui_ai_update_labels(&state);
}

/* Confirmation dialog callbacks.  LVGL event callbacks run with the display
 * lock held, so they may update widgets directly. */
static void ai_takeover_enable_event_cb(lv_event_t *event)
{
    lv_obj_t *mbox = (lv_obj_t *)lv_event_get_user_data(event);
    if (mbox != NULL) {
        lv_msgbox_close(mbox);
    }
    (void)agent_runtime_set_takeover(true, 60);
    ui_ai_refresh_now();
}

static void ai_takeover_revoke_event_cb(lv_event_t *event)
{
    lv_obj_t *mbox = (lv_obj_t *)lv_event_get_user_data(event);
    if (mbox != NULL) {
        lv_msgbox_close(mbox);
    }
    (void)agent_runtime_set_takeover(false, 0);
    agent_runtime_revoke("screen");
    ui_ai_refresh_now();
}

static void ai_msgbox_cancel_event_cb(lv_event_t *event)
{
    lv_obj_t *mbox = (lv_obj_t *)lv_event_get_user_data(event);
    if (mbox != NULL) {
        lv_msgbox_close(mbox);
    }
}

static void ai_takeover_event_cb(lv_event_t *event)
{
    (void)event;
    bool takeover = agent_runtime_mode() == AGENT_MODE_TAKEOVER;
    lv_obj_t *mbox = lv_msgbox_create(NULL);
    if (mbox == NULL) {
        return;
    }
    lv_obj_set_style_text_font(mbox, &td_font_16, 0);
    lv_msgbox_add_title(mbox, "AI TAKEOVER");
    if (takeover) {
        lv_msgbox_add_text(mbox, "Revoke takeover and return to ASSIST?");
        lv_obj_t *ok = lv_msgbox_add_footer_button(mbox, "REVOKE");
        if (ok != NULL) {
            lv_obj_add_event_cb(ok, ai_takeover_revoke_event_cb,
                                LV_EVENT_CLICKED, mbox);
        }
    } else {
        lv_msgbox_add_text(
            mbox, "Enable takeover for 60s? The AI may move the robot.");
        lv_obj_t *ok = lv_msgbox_add_footer_button(mbox, "ENABLE");
        if (ok != NULL) {
            lv_obj_add_event_cb(ok, ai_takeover_enable_event_cb,
                                LV_EVENT_CLICKED, mbox);
        }
    }
    lv_obj_t *cancel = lv_msgbox_add_footer_button(mbox, "CANCEL");
    if (cancel != NULL) {
        lv_obj_add_event_cb(cancel, ai_msgbox_cancel_event_cb,
                            LV_EVENT_CLICKED, mbox);
    }
}

static void create_ai_panel(lv_obj_t *screen)
{
    s_ai_panel = make_panel(screen, 10, 10, 780, 460,
                            TD_COLOR_PANEL, 14);
    lv_obj_set_style_bg_opa(s_ai_panel, LV_OPA_COVER, 0);
    lv_obj_set_hidden(s_ai_panel, true);

    make_label(s_ai_panel, "AI AGENT", 20, 14, &td_font_24, TD_COLOR_GREEN);
    make_button(s_ai_panel, "BACK", 650, 12, 110, 38,
                0x233D32, ai_close_event_cb, 0);

    s_ai_mode_label = make_status_label(
        s_ai_panel, "MODE: ASSIST", 20, 58, 190, TD_COLOR_BLUE);
    s_ai_lease_label = make_status_label(
        s_ai_panel, "LEASE: --", 216, 58, 150, TD_COLOR_AMBER);
    s_ai_plan_label = make_status_label(
        s_ai_panel, "PLAN: -", 372, 58, 160, TD_COLOR_TEXT);
    s_ai_blocked_label = make_status_label(
        s_ai_panel, "BLOCKED: 0", 538, 58, 120, TD_COLOR_MUTED);
    s_ai_actions_label = make_status_label(
        s_ai_panel, "ACTIONS: 0", 660, 58, 100, TD_COLOR_MUTED);

    /* The reply can be long, so both texts live in one vertically
     * scrollable, wrapping container. */
    lv_obj_t *conversation = lv_obj_create(s_ai_panel);
    lv_obj_set_pos(conversation, 20, 88);
    lv_obj_set_size(conversation, 740, 196);
    lv_obj_set_style_bg_color(conversation, lv_color_hex(0x07120C), 0);
    lv_obj_set_style_bg_opa(conversation, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(conversation, 1, 0);
    lv_obj_set_style_border_color(conversation, lv_color_hex(0x254438), 0);
    lv_obj_set_style_radius(conversation, 8, 0);
    lv_obj_set_style_pad_all(conversation, 8, 0);
    lv_obj_add_flag(conversation, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(conversation, LV_DIR_VER);
    lv_obj_set_flex_flow(conversation, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(conversation, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(conversation, 4, 0);

    make_label(conversation, "LAST Q", 0, 0, &td_font_16, TD_COLOR_MUTED);
    s_ai_user_label = make_label(conversation, "-", 0, 0,
                                 &td_font_16, TD_COLOR_TEXT);
    lv_obj_set_width(s_ai_user_label, 716);
    lv_label_set_long_mode(s_ai_user_label, LV_LABEL_LONG_WRAP);

    make_label(conversation, "LAST A", 0, 0, &td_font_16, TD_COLOR_GREEN);
    s_ai_reply_label = make_label(conversation, "-", 0, 0,
                                  &td_font_16, TD_COLOR_TEXT);
    lv_obj_set_width(s_ai_reply_label, 716);
    lv_label_set_long_mode(s_ai_reply_label, LV_LABEL_LONG_WRAP);

    s_ai_tool_label = make_status_label(
        s_ai_panel, "TOOL: -", 20, 296, 360, TD_COLOR_TEXT);
    s_ai_action_label = make_status_label(
        s_ai_panel, "ACTION: -", 400, 296, 360, TD_COLOR_TEXT);
    s_ai_error_label = make_status_label(
        s_ai_panel, "PLAN ERROR: -", 20, 322, 740, TD_COLOR_MUTED);

    s_ai_takeover_button = make_button(
        s_ai_panel, "TAKEOVER 60s", 20, 352, 240, 48,
        0x17452E, ai_takeover_event_cb, 0);
    lv_obj_t *hint = make_label(
        s_ai_panel,
        "Human-only enable, confirmed in a dialog.\n"
        "The lease expires automatically.",
        280, 360, &td_font_16, TD_COLOR_MUTED);
    lv_obj_set_width(hint, 460);
    lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
}

esp_err_t ui_start(void)
{
    bsp_display_lock(0);
    lv_obj_t *screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(TD_COLOR_BG), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_set_scrollable(screen, false);

    lv_obj_t *header = make_panel(screen, 0, 0, 800, 46,
                                  0x07110Cu, 0);
    lv_obj_set_style_border_width(header, 0, 0);
    make_label(header, "句芒", 16, 7, &td_font_24, TD_COLOR_GREEN);
    make_label(header, "移动巡检机器人 · ESP32-S31 本地视觉控制台",
               116, 14, &td_font_16, TD_COLOR_MUTED);

    lv_obj_t *camera_panel = make_panel(
        screen, 10, 52, 520, 420, TD_COLOR_PANEL, 14);
    make_label(camera_panel, "DVP 摄像头 / 640×480", 14, 7,
               &td_font_16, 0xA8C4B5);
    s_face_label = make_status_label(
        camera_panel, "人脸：启动中", 220, 7, 146, TD_COLOR_BLUE);
    s_camera_label = make_status_label(
        camera_panel, "摄像头未连接", 370, 7, 136, TD_COLOR_AMBER);
    lv_obj_set_style_text_align(s_camera_label, LV_TEXT_ALIGN_RIGHT, 0);

    s_camera_viewport = make_panel(
        camera_panel, (520 - TD_CAMERA_PREVIEW_W) / 2,
        TD_CAMERA_PREVIEW_TOP, TD_CAMERA_PREVIEW_W,
        TD_CAMERA_PREVIEW_H, 0x000000, 0);
    lv_obj_set_style_border_width(s_camera_viewport, 0, 0);
    s_camera_placeholder = make_label(
        s_camera_viewport,
        "等待摄像头画面",
        0, 0, &td_font_16, 0xA8C4B5);
    lv_obj_set_width(s_camera_placeholder, TD_CAMERA_PREVIEW_W);
    lv_obj_set_style_text_align(s_camera_placeholder,
                                LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_camera_placeholder, LV_ALIGN_CENTER, 0, 0);
    create_detection_overlay();

    s_line_label = make_status_label(
        camera_panel, "巡线：未识别  偏差 --", 12, TD_CAMERA_STATUS_TOP,
        250, 0xD3E5DA);
    s_fps_label = make_status_label(
        camera_panel, "画面：-- 帧/秒", 270, TD_CAMERA_STATUS_TOP,
        238, 0xD3E5DA);
    s_target_label = make_status_label(
        camera_panel, "目标：模型加载中", 12,
        TD_CAMERA_STATUS_TOP + TD_CAMERA_STATUS_ROW_HEIGHT, 250, 0xD3E5DA);
    s_sensor_label = make_status_label(
        camera_panel, "距离：--  电池：--", 270,
        TD_CAMERA_STATUS_TOP + TD_CAMERA_STATUS_ROW_HEIGHT, 238, 0xD3E5DA);

    s_selftest_label = make_status_label(camera_panel,
        "上电自检：等待模块就绪", 12, 397, 496, TD_COLOR_MUTED);
    lv_obj_set_style_bg_color(s_selftest_label, lv_color_hex(TD_COLOR_PANEL), 0);
    lv_obj_set_style_bg_opa(s_selftest_label, LV_OPA_COVER, 0);

    lv_obj_t *side = make_panel(screen, 540, 52, 250, 420,
                                TD_COLOR_PANEL, 14);
    make_label(side, "控制台", 12, 4, &td_font_24, TD_COLOR_TEXT);
    make_button(side, "AI", 182, 2, 58, 30,
                0x1B3A4A, ai_open_event_cb, 0);
    s_mode_label = make_label(side, "待机", 12, 32,
                              &td_font_16, TD_COLOR_AMBER);
    lv_obj_set_width(s_mode_label, 100);
    s_net_label = make_label(side, "热点启动中", 112, 32,
                             &td_font_16, TD_COLOR_AMBER);
    lv_obj_set_width(s_net_label, 126);
    s_url_label = make_label(side, TD_AP_URL, 12, 56,
                             &td_font_16, TD_COLOR_BLUE);

    make_button(side, "开始巡线", 12, 82, 108, 34,
                0x17452E, start_event_cb, 0);
    make_button(side, "停车", 128, 82, 108, 34,
                0x26362E, stop_event_cb, 0);
    make_button(side, "检测", 12, 122, 70, 34,
                0x4A3A12, inspect_event_cb, 0);
    make_button(side, "解除", 88, 122, 70, 34,
                0x233D32, clear_event_cb, 0);
    make_button(side, "人脸登记", 164, 122, 72, 34,
                0x1B4733, face_open_event_cb, 0);

    make_label(side, "履带速度", 12, 161, &td_font_16, TD_COLOR_MUTED);
    s_track_speed = lv_dropdown_create(side);
    lv_dropdown_set_options_static(s_track_speed,
        "15%\n20%\n25%\n30%\n35%\n40%\n45%\n50%\n55%\n"
        "60%\n65%\n70%\n75%\n80%\n85%\n90%\n95%\n100%");
    lv_dropdown_set_selected(s_track_speed,
        (TD_LINE_MODULE_SPEED - TD_USER_TRACK_MIN_PERCENT) / TD_USER_TRACK_STEP_PERCENT);
    lv_obj_set_pos(s_track_speed, 112, 157);
    lv_obj_set_size(s_track_speed, 124, 26);
    lv_obj_set_style_pad_ver(s_track_speed, 3, 0);
    lv_obj_set_style_text_font(s_track_speed, &td_font_16, 0);
    // LV_SYMBOL_DOWN is provided by LVGL's icon font.
    lv_obj_set_style_text_font(s_track_speed, &lv_font_montserrat_14, LV_PART_INDICATOR);
    lv_obj_t *speed_list = lv_dropdown_get_list(s_track_speed);
    lv_obj_set_style_text_font(speed_list, &td_font_16, 0);

    make_hold_button(side, "前进", 82, 185, 86, 32,
                     0x173125, 1);
    make_hold_button(side, "左转", 12, 222, 70, 32,
                     0x173125, 3);
    make_hold_button(side, "停止", 88, 222, 70, 32,
                     0x303C35, 0);
    make_hold_button(side, "右转", 164, 222, 72, 32,
                     0x173125, 4);
    make_hold_button(side, "后退", 82, 260, 86, 32,
                     0x173125, 2);

    make_label(side, "云台", 12, 302,
               &td_font_16, TD_COLOR_MUTED);
    make_button(side, "云台-", 12, 294, 86, 32,
                0x1B3027, pan_left_event_cb, 0);
    make_button(side, "云台+", 152, 294, 86, 32,
                0x1B3027, pan_right_event_cb, 0);

    s_uart_label = make_label(side, "C30D：离线", 12, 340,
                              &td_font_16, TD_COLOR_RED);
    lv_obj_set_width(s_uart_label, 104);
    make_button(side, "C30D数据", 120, 332, 118, 30,
                0x1B3027, c30d_open_event_cb, 0);
    make_button(side, "急停", 12, 362, 108, 36,
                0x5A1C18, estop_event_cb, 0);
    s_voice_button = make_button(side, "语音测试", 128, 362, 108, 36,
                0x1B3027, voice_test_event_cb, 0);
    make_button(side, "网络设置", 12, 398, 108, 20,
                0x1B3027, network_open_event_cb, 0);
    make_label(side, "USB-A → C30D", 128, 401,
               &td_font_16, TD_COLOR_MUTED);

    create_face_panel(screen);
    create_c30d_panel(screen);
    create_network_panel(screen);
    create_ai_panel(screen);
    bsp_display_unlock();
    ESP_LOGI(TAG, "800x480 touch console ready");
    return ESP_OK;
}

void ui_attach_camera(lv_obj_t *canvas)
{
    if (canvas == NULL) {
        return;
    }
    bsp_display_lock(0);
    s_camera_canvas = canvas;
    lv_obj_set_hidden(canvas, true);
    bool registration_open =
        s_face_panel != NULL && !lv_obj_is_hidden(s_face_panel);
    face_place_canvas(registration_open);
    for (int i = 0; i < TD_MAX_OBJECTS; ++i) {
        lv_obj_move_foreground(s_object_boxes[i]);
    }
    lv_obj_move_foreground(s_object_list);
    lv_obj_move_foreground(s_face_box);
    app_state_lock();
    app_state_mutable()->display_ready = true;
    app_state_mutable()->touch_ready = true;
    app_state_unlock();
    bsp_display_unlock();
}

void ui_detach_camera(void)
{
    bsp_display_lock(0);
    s_camera_canvas = NULL;
    if (s_camera_placeholder != NULL) {
        lv_obj_set_hidden(s_camera_placeholder, false);
    }
    bsp_display_unlock();
}

static const char *mode_name_zh(td_mode_t mode)
{
    switch (mode) {
    case TD_MODE_LINE:
        return "巡线";
    case TD_MODE_MANUAL:
        return "手动";
    case TD_MODE_ESTOP:
        return "急停";
    case TD_MODE_INSPECT:
        return "检测";
    case TD_MODE_IDLE:
    default:
        return "待机";
    }
}

static const char *line_source_name(uint8_t source)
{
    switch (source) {
    case 1u:
        return "CCD";
    case 2u:
        return "四路";
    case 3u:
        return "CCD+四路";
    case 4u:
        return "丢线";
    case 5u:
        return "轨迹倒放";
    case 6u:
        return "避障";
    default:
        return "待机";
    }
}

void ui_refresh(void)
{
    td_status_t status;
    td_c30d_meta_t c30d_meta;
    td_face_status_t face;
    td_object_status_t objects;
    td_wifi_status_t network;
    agent_state_t agent_state;
    size_t c30d_history_count;
    app_state_snapshot(&status);
    app_state_build_c30d_meta(&status, &c30d_meta);
    app_state_lock();
    c30d_history_count = td_c30d_monitor_snapshot(
        s_ui_c30d_history, TD_C30D_HISTORY_CAPACITY);
    app_state_unlock();
    face_service_snapshot(&face);
    object_service_snapshot(&objects);
    wifi_mgr_get_status(&network);
    agent_runtime_state_snapshot(&agent_state);

    bsp_display_lock(0);
    voice_status_t voice;
    voice_get_status(&voice);
    startup_report_t selftest;
    startup_selftest_snapshot(&selftest);
    if (selftest.complete) {
        if (selftest.audio_error != ESP_OK) {
            lv_label_set_text(s_selftest_label, "自检完成：语音播报失败，检查日志");
        } else {
            lv_label_set_text_fmt(s_selftest_label, "自检完成：%u 项异常，%u 项需关注",
                selftest.checks.errors, selftest.checks.attention);
        }
        lv_obj_set_style_text_color(s_selftest_label,
            lv_color_hex(selftest.checks.attention || selftest.audio_error != ESP_OK ?
                         TD_COLOR_AMBER : TD_COLOR_GREEN), 0);
    } else if (selftest.announcing && selftest.current < ST_ITEM_COUNT) {
        lv_label_set_text_fmt(s_selftest_label, "上电自检：%s %s",
            st_module_name(selftest.current),
            st_result_name(selftest.checks.results[selftest.current]));
    }
    lv_label_set_text(lv_obj_get_child(s_voice_button, 0),
        !voice.ready ? "语音未就绪" : voice.playing ? "播放中" :
        voice.last_error != ESP_OK ? "播报失败" : "语音测试");
    lv_label_set_text(s_mode_label, mode_name_zh(status.mode));
    lv_obj_set_style_text_color(
        s_mode_label,
        lv_color_hex(status.mode == TD_MODE_ESTOP ? TD_COLOR_RED :
                     status.mode == TD_MODE_IDLE ? 0xA8C4B5 :
                     TD_COLOR_GREEN), 0);
    lv_label_set_text(
        s_net_label,
        status.sta_connected ? "终端在线" :
        status.setup_ap ? "配置热点" :
        status.ap_started ? "热点在线" : "联网中");
    lv_obj_set_style_text_color(
        s_net_label,
        lv_color_hex(status.sta_connected ? TD_COLOR_GREEN :
                     status.ap_started ? TD_COLOR_AMBER :
                     TD_COLOR_MUTED), 0);
    lv_label_set_text(s_url_label, status.url);
    bool camera_live = camera_app_has_fresh_frame();
    if (s_camera_canvas != NULL) {
        lv_obj_set_hidden(s_camera_canvas, !camera_live);
    }
    lv_obj_set_hidden(s_camera_placeholder, camera_live);
    lv_label_set_text(s_camera_label,
                      camera_live ? "摄像头实时" : "等待摄像头画面");
    lv_obj_set_style_text_color(
        s_camera_label, lv_color_hex(camera_live ? TD_COLOR_GREEN : TD_COLOR_AMBER), 0);

    lv_label_set_text_fmt(
        s_line_label, "巡线：%s  偏差 %d",
        line_source_name(status.line_state.source),
        status.line_state.line_error);
    if (!objects.ready) {
        lv_label_set_text(s_target_label, "目标：模型加载中");
    } else if (objects.count > 0u) {
        lv_label_set_text_fmt(s_target_label, "目标：%s %u%%",
                              objects.items[0].name,
                              objects.items[0].confidence);
    } else {
        lv_label_set_text(s_target_label, "目标：未识别");
    }
    if (camera_live) {
        lv_label_set_text_fmt(s_fps_label, "画面：%u.%u 帧/秒",
                              status.vision_fps_x10 / 10u, status.vision_fps_x10 % 10u);
    } else {
        lv_label_set_text(s_fps_label, "画面：-- 帧/秒");
    }
    if (status.sensor.distance_cm == 0xFFFFu) {
        lv_label_set_text_fmt(s_sensor_label, "距离：--  电池：%u mV",
                              status.sensor.battery_mv);
    } else {
        if (status.sensor.distance_cm == 0u || status.sensor.distance_cm == UINT16_MAX) {
            lv_label_set_text_fmt(s_sensor_label, "距离：--  电池：%u mV",
                                  status.sensor.battery_mv);
        } else {
            lv_label_set_text_fmt(s_sensor_label, "距离：%u cm  电池：%u mV",
                                  status.sensor.distance_cm, status.sensor.battery_mv);
        }
    }
    lv_label_set_text(
        s_uart_label,
        status.stm32_online ? "C30D:在线" :
        status.stm32_usb_connected ? "C30D:静默" :
        "C30D:离线");
    lv_obj_set_style_text_color(
        s_uart_label,
        lv_color_hex(status.stm32_online ? TD_COLOR_GREEN :
                     status.stm32_usb_connected ? TD_COLOR_AMBER :
                     TD_COLOR_RED), 0);

    if (s_ai_panel != NULL && !lv_obj_is_hidden(s_ai_panel)) {
        ui_ai_update_labels(&agent_state);
    }

    if (s_network_summary_label != NULL) {
        snprintf(
            s_ui_network_text, sizeof(s_ui_network_text),
            "网页控制码：%s\n"
            "热点密码：%s\n"
            "首选模式：%s\n"
            "当前状态：%s\n"
            "IP：%s\n"
            "网址：%s\n"
            "已保存 SSID：%s\n"
            "配置热点：%s",
            device_auth_control_code(),
            device_auth_ap_password(),
            network.preferred_mode == TD_WIFI_MODE_ACCESS_POINT ?
                "热点" : "路由器终端",
            network.station_connected ? "终端已联网" :
            network.access_point_active ?
                (network.setup_access_point ? "配置热点已开启" :
                 "手动热点已开启") : "正在连接",
            network.ip[0] != '\0' ? network.ip : "--",
            network.url[0] != '\0' ? network.url : "--",
            network.ssid[0] != '\0' ? network.ssid : "未配置",
            TD_AP_SETUP_SSID);
        lv_label_set_text(s_network_summary_label, s_ui_network_text);
    }

    if (s_c30d_summary_label != NULL) {
        char rx_age[24];
        char distance[24];
        char yaw[32];
        uint8_t lf_bits = status.sensor.push_top_limit & 0x0Fu;
        if (c30d_meta.last_rx_age_ms == UINT32_MAX) {
            strlcpy(rx_age, "--", sizeof(rx_age));
        } else {
            snprintf(rx_age, sizeof(rx_age), "%lu ms",
                     (unsigned long)c30d_meta.last_rx_age_ms);
        }
        if (c30d_meta.distance_cm == 0xFFFFu) {
            strlcpy(distance, "--", sizeof(distance));
        } else {
            snprintf(distance, sizeof(distance), "%u cm",
                     (unsigned int)c30d_meta.distance_cm);
        }
        int yaw_abs = abs((int)c30d_meta.yaw_cdeg);
        snprintf(yaw, sizeof(yaw), "%c%d.%02d°",
                 c30d_meta.yaw_cdeg < 0 ? '-' : '+',
                 yaw_abs / 100, yaw_abs % 100);
        snprintf(
            s_ui_summary, sizeof(s_ui_summary),
            "USB：%s\n"
            "数据：%s  延迟 %s\n"
            "C30D固件：%u.%u\n\n"
            "接收帧：%lu\n"
            "底盘/传感/安全：%lu/%lu/%lu\n"
            "CRC/长度错误：%lu/%lu\n"
            "最后命令：0x%02X  状态/故障：%u/%u\n\n"
            "电压：%u.%03u V\n"
            "距离：%s\n"
            "航向：%s\n"
            "左右速度：%d / %d mm/s\n"
            "融合源：%s  CCD中心/置信度：%u/%u\n"
            "线误差：%d  倒放进度：%u%%  四路：%u%u%u%u\n"
            "陀螺Z：%d  加速度Z：%d mg",
            c30d_meta.usb_connected ? "已连接" : "未连接",
            c30d_meta.online ? "在线" : "离线", rx_age,
            c30d_meta.fw_major, c30d_meta.fw_minor,
            (unsigned long)c30d_meta.rx_frames,
            (unsigned long)c30d_meta.chassis_frames,
            (unsigned long)c30d_meta.sensor_frames,
            (unsigned long)c30d_meta.safety_frames,
            (unsigned long)c30d_meta.crc_errors,
            (unsigned long)c30d_meta.length_errors,
            c30d_meta.last_cmd, c30d_meta.robot_state,
            c30d_meta.fault_flags,
            c30d_meta.battery_mv / 1000u,
            c30d_meta.battery_mv % 1000u, distance, yaw,
            c30d_meta.left_speed_mm_s, c30d_meta.right_speed_mm_s,
            line_source_name(c30d_meta.line_source),
            c30d_meta.ccd_center, c30d_meta.ccd_confidence,
            c30d_meta.line_error, c30d_meta.rewind_progress,
            (lf_bits >> 3) & 1u,
            (lf_bits >> 2) & 1u,
            (lf_bits >> 1) & 1u,
            lf_bits & 1u,
            c30d_meta.gyro_z_ddps, c30d_meta.accel_z_mg);
        lv_label_set_text(s_c30d_summary_label, s_ui_summary);

        int written = snprintf(
            s_ui_history, sizeof(s_ui_history),
            "最近记录（内存 %u 秒）\n",
            TD_C30D_HISTORY_CAPACITY);
        if (c30d_history_count == 0u) {
            snprintf(s_ui_history + written,
                     sizeof(s_ui_history) - (size_t)written,
                     "等待 C30D 数据");
        } else {
            size_t shown = c30d_history_count > 10u ?
                10u : c30d_history_count;
            size_t start = c30d_history_count - shown;
            for (size_t i = start;
                 i < c30d_history_count && written > 0; ++i) {
                const td_c30d_sample_t *sample = &s_ui_c30d_history[i];
                int sample_yaw_abs = abs((int)sample->yaw_cdeg);
                int added = snprintf(
                    s_ui_history + written,
                    sizeof(s_ui_history) - (size_t)written,
                    "%c L%+d R%+d Y%c%d.%02d D%u B%u.%02uV\n",
                    sample->online ? 'O' : '-',
                    sample->left_speed_mm_s,
                    sample->right_speed_mm_s,
                    sample->yaw_cdeg < 0 ? '-' : '+',
                    sample_yaw_abs / 100,
                    sample_yaw_abs % 100,
                    (unsigned int)sample->distance_cm,
                    sample->battery_mv / 1000u,
                    sample->battery_mv % 1000u);
                if (added < 0 ||
                    (size_t)added >= sizeof(s_ui_history) -
                    (size_t)written) {
                    break;
                }
                written += added;
            }
        }
        lv_label_set_text(s_c30d_history_label, s_ui_history);
    }

    if (!face.ready) {
        lv_label_set_text(s_face_label, "人脸：启动中");
        lv_obj_set_style_text_color(
            s_face_label, lv_color_hex(TD_COLOR_AMBER), 0);
    } else if (face.face_found && face.recognized) {
        lv_label_set_text_fmt(s_face_label, "人脸：人员 %03u",
                              face.face_id);
        lv_obj_set_style_text_color(
            s_face_label, lv_color_hex(TD_COLOR_GREEN), 0);
    } else if (face.face_found) {
        lv_label_set_text(s_face_label, "人脸：陌生人");
        lv_obj_set_style_text_color(
            s_face_label, lv_color_hex(TD_COLOR_BLUE), 0);
    } else if (!status.camera_ready) {
        lv_label_set_text(s_face_label, "人脸：等待摄像头");
        lv_obj_set_style_text_color(
            s_face_label, lv_color_hex(TD_COLOR_RED), 0);
    } else {
        lv_label_set_text(s_face_label, "人脸：未检测");
        lv_obj_set_style_text_color(
            s_face_label, lv_color_hex(TD_COLOR_MUTED), 0);
    }

    if (camera_live && face.ready && face.face_found && face.width > 0u &&
        face.height > 0u) {
        lv_obj_set_pos(s_face_box, scaled(face.x), scaled(face.y));
        lv_obj_set_size(s_face_box, scaled(face.width), scaled(face.height));
        lv_obj_set_hidden(s_face_box, false);
    } else {
        lv_obj_set_hidden(s_face_box, true);
    }

    if (camera_live && objects.ready && objects.count > 0u) {
        char text[360];
        uint8_t shown = objects.count > 4u ? 4u : objects.count;
        int written = snprintf(text, sizeof(text), "物体识别");
        for (uint8_t i = 0; i < shown && written > 0; ++i) {
            int added = snprintf(
                text + written, sizeof(text) - (size_t)written,
                "\n%s %u%%",
                objects.items[i].name, objects.items[i].confidence);
            if (added < 0 ||
                (size_t)added >= sizeof(text) - (size_t)written) {
                break;
            }
            written += added;
        }
        lv_label_set_text(s_object_list, text);
        lv_obj_set_hidden(s_object_list, false);
    } else {
        lv_obj_set_hidden(s_object_list, true);
    }

    for (uint8_t i = 0; i < TD_MAX_OBJECTS; ++i) {
        lv_obj_t *box = s_object_boxes[i];
        if (camera_live && objects.ready && i < objects.count &&
            objects.items[i].width > 0u &&
            objects.items[i].height > 0u) {
            lv_obj_set_pos(box, scaled(objects.items[i].x),
                           scaled(objects.items[i].y));
            lv_obj_set_size(box, scaled(objects.items[i].width),
                            scaled(objects.items[i].height));
            lv_obj_set_style_border_color(
                box, lv_color_hex(
                    object_group_color(objects.items[i].category)), 0);
            lv_obj_set_hidden(box, false);
        } else {
            lv_obj_set_hidden(box, true);
        }
    }

    lv_label_set_text(
        s_face_status_label,
        status.camera_ready ? face.message :
        "摄像头未连接，请检查 FFC 排线和摄像头模组");
    lv_label_set_text_fmt(s_face_count_label, "登记人数：%u",
                          face.registered_count);
    if (face.registered_count != s_face_last_count) {
        int written = snprintf(
            s_ui_face_table, sizeof(s_ui_face_table),
            "编号      人员          状态\n"
            "----------------------------------------\n");
        if (face.registered_count == 0u) {
            written += snprintf(s_ui_face_table + written,
                                sizeof(s_ui_face_table) - (size_t)written,
                                "暂无登记人员");
        } else {
            uint16_t rows = face.registered_count > 10u ?
                10u : face.registered_count;
            for (uint16_t id = 1; id <= rows && written > 0; ++id) {
                written += snprintf(
                    s_ui_face_table + written,
                    sizeof(s_ui_face_table) - (size_t)written,
                    "%03u       人员 %03u      已登记\n", id, id);
            }
            if (face.registered_count > rows && written > 0) {
                    snprintf(s_ui_face_table + written,
                             sizeof(s_ui_face_table) - (size_t)written,
                         "还有 %u 人未显示",
                         face.registered_count - rows);
            }
        }
        lv_label_set_text(s_face_table_label, s_ui_face_table);
        s_face_last_count = face.registered_count;
    }

    bsp_display_unlock();
}
