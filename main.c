/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
/********************************************************************************
 **********                           INCLUDE FILES                   ***********
*********************************************************************************/
#include "project.h"
#include "lib_mkb.h"
#include "lib_seg_display.h"
#include "lib_lcd1602.h"
#include "lib_adc.h"
#include "lib_acc_gyr.h"
#include "lib_magnetometer.h"
#include "lib_barometer.h"
#include "lib_rfid.h"
#include "lib_common_types.h"
#include "lib_buzzer.h"
#include "lib_servo.h"
#include <stdio.h>

#define LOG_LEVEL LOG_LEVEL_DBG
#define TAG "MAIN"
#include "log_dbg.h"


/********************************************************************************
 **********                        PRIVATE DEFINITIONS                ***********
 *********************************************************************************/
#define ADC_LCD_UPDATE_INTERVAL_MS  (200u)
#define RFID_LCD_UPDATE_INTERVAL_MS (100u)
#define LOOP_DELAY_MS               (10u)
#define ADC_LCD_UPDATE_TICKS        (ADC_LCD_UPDATE_INTERVAL_MS / LOOP_DELAY_MS)
#define RFID_LCD_UPDATE_TICKS       (RFID_LCD_UPDATE_INTERVAL_MS / LOOP_DELAY_MS)
#define RFID_NO_TAG_READS           (500u / RFID_LCD_UPDATE_INTERVAL_MS)
#define SENSOR_SHOW_DELAY_TICKS     (1200u / LOOP_DELAY_MS)
#define LDR_DARK_THRESHOLD          (400)
#define TILT_THRESHOLD_MG           (600)

typedef enum {
    STATE_IDLE,
    STATE_SHADOW_MODE,
    STATE_KINETIC_KEY,
    STATE_OPENED
}safe_state_t;

/********************************************************************************
 **********                         PRIVATE VARIABLES                 ***********
*********************************************************************************/
static char g_lcd_line0[17] = {'\0'};
static char g_lcd_line1[17] = {'\0'};
static uint8_t  g_key3_live_adc_mode = 0u;
static uint16_t g_key3_delay_tick    = 0u;
static uint8_t  g_key4_live_acc_mode = 0u;
static uint16_t g_key4_delay_tick    = 0u;
static uint8_t  g_key5_live_gyr_mode = 0u;
static uint16_t g_key5_delay_tick    = 0u;
static uint8_t  g_key6_live_mag_mode = 0u;
static uint16_t g_key6_delay_tick    = 0u;
static uint8_t  g_key7_live_baro_mode  = 0u;
static uint8_t  g_key9_live_rfid_mode = 0u;
static uint8_t  g_key12_live_reed_mode = 0u;
static uint8_t  g_key9_no_tag_reads = 0u;
static uint8_t  g_key9_has_tag = 0u;

static int16_t  g_last_imu_temp = 0;
static int16_t  g_last_mag_temp = 0;

static const char g_correct_password[] = "1234";
static char g_entered_password[5] = "";
static uint8_t g_password_index = 0;
static uint8_t g_wrong_attempts = 0;
static uint8_t g_is_locked_by_attempts = 0;

static safe_state_t g_safe_state = STATE_IDLE;
static uint8_t g_kinetic_step = 0;

/********************************************************************************
 **********                        PRIVATE FUNCTIONS                  ***********
*********************************************************************************/
static void app_lcd_show(const char *line0, const char *line1)
{
    snprintf(g_lcd_line0, sizeof(g_lcd_line0), "%-16s", line0);
    snprintf(g_lcd_line1, sizeof(g_lcd_line1), "%-16s", line1);

    lib_lcd1602_clear();
    lib_lcd1602_write_str(0u, 0u, g_lcd_line0);
    lib_lcd1602_write_str(0u, 1u, g_lcd_line1);
}

static void app_show_default_lcd_splash(void)
{
    app_lcd_show("     UzhNU", "   HACKATHON!");
}

static void app_show_adc_live_on_lcd(int16_t adc0, int16_t adc1, int16_t adc2)
{
    char line0[17];
    char line1[17];

    /* Keep all ADC values on one LCD screen in live mode. */
    snprintf(line0, sizeof(line0), "0:%+5d 1:%+5d", adc0, adc1);
    snprintf(line1, sizeof(line1), "2:%+5d", adc2);

    app_lcd_show(line0, line1);
}

static void app_show_key_splash(const char *line0, const char *line1)
{
    app_lcd_show(line0, line1);
}

static void app_key_0_action()
{
    app_show_key_splash("0: RGB Pattern", "R G B RGB loop");
    LOG_I(TAG, "KEY 0: RGB LED pattern start");

    LED_RED_Write(1u);
    LED_GREEN_Write(0u);
    LED_BLUE_Write(0u);
    CyDelay(180u);

    LED_RED_Write(0u);
    LED_GREEN_Write(1u);
    LED_BLUE_Write(0u);
    CyDelay(180u);

    LED_RED_Write(0u);
    LED_GREEN_Write(0u);
    LED_BLUE_Write(1u);
    CyDelay(180u);

    LED_RED_Write(1u);
    LED_GREEN_Write(1u);
    LED_BLUE_Write(0u);
    CyDelay(180u);

    LED_RED_Write(1u);
    LED_GREEN_Write(0u);
    LED_BLUE_Write(1u);
    CyDelay(180u);

    LED_RED_Write(0u);
    LED_GREEN_Write(1u);
    LED_BLUE_Write(1u);
    CyDelay(180u);

    LED_RED_Write(1u);
    LED_GREEN_Write(1u);
    LED_BLUE_Write(1u);
    CyDelay(180u);

    LED_RED_Write(1u);
    LED_GREEN_Write(1u);
    LED_BLUE_Write(1u);
    LOG_I(TAG, "KEY 0: RGB LED pattern done");
    app_show_default_lcd_splash();
}

static void app_key_1_action()
{
    app_show_key_splash("1: Servo Sweep", "0->90->180->90");
    LOG_I(TAG, "KEY 1: Servo pattern start");
    lib_servo_set_angle(0u);
    CyDelay(400u);
    lib_servo_set_angle(90u);
    CyDelay(400u);
    lib_servo_set_angle(180u);
    CyDelay(400u);
    lib_servo_set_angle(90u);
    LOG_I(TAG, "KEY 1: Servo pattern done");
}

static void app_key_2_action()
{
    app_show_key_splash("2: Buzzer", "Melody playback");
    LOG_I(TAG, "KEY 2: Play buzzer melody");
    lib_buzzer_play_melody();
}

static void app_key_3_action()
{
    app_show_key_splash("3: ADC Live", "Live display");
    g_key3_delay_tick    = 0u;
    g_key3_live_adc_mode = 1u;
    LOG_I(TAG, "KEY 3: Live ADC LCD mode ON");
}

static void app_show_acc_live_on_lcd(int32_t ax, int32_t ay, int32_t az)
{
    char line0[17];
    char line1[17];

    /* Display values divided by 1000 (mg -> g) as sign + integer.fraction */
    snprintf(line0, sizeof(line0), "X:%+5ld Y:%+5ld",
             ax / 1000L, ay / 1000L);
    snprintf(line1, sizeof(line1), "Z:%+5ld  [g]",
             az / 1000L);

    app_lcd_show(line0, line1);
}

static void app_show_sensor_temp_on_lcd(const char *label, int16_t temp)
{
    char line0[17];
    char line1[17];
    int t_int  = (int)(temp / 10);
    int t_frac = (int)((temp < 0 ? -temp : temp) % 10);

    snprintf(line0, sizeof(line0), "%-16s", label);
    snprintf(line1, sizeof(line1), "Temp: %+d.%d C", t_int, t_frac);

    app_lcd_show(line0, line1);
}

static void app_key_4_action()
{
    LOG_I(TAG, "KEY 4 TEMP: IMU=%d C", (int)g_last_imu_temp);
    app_show_sensor_temp_on_lcd("4: Accelerometer", g_last_imu_temp);

    g_key4_delay_tick    = 0u;
    g_key4_live_acc_mode = 1u;
}

static void app_show_gyr_live_on_lcd(int32_t gx, int32_t gy, int32_t gz)
{
    char line0[17];
    char line1[17];

    snprintf(line0, sizeof(line0), "X:%+5ld Y:%+5ld", gx / 1000L, gy / 1000L);
    snprintf(line1, sizeof(line1), "Z:%+5ld [dps]", gz / 1000L);

    app_lcd_show(line0, line1);
}

static void app_show_mag_live_on_lcd(int32_t mx, int32_t my, int32_t mz)
{
    char line0[17];
    char line1[17];

    snprintf(line0, sizeof(line0), "X:%+5ld Y:%+5ld", mx, my);
    snprintf(line1, sizeof(line1), "Z:%+5ld [mG]", mz);

    app_lcd_show(line0, line1);
}

static void app_show_reed_btn_on_lcd(uint8_t reed_raw, uint8_t btn_raw)
{
    char line0[17];
    char line1[17];
    snprintf(line0, sizeof(line0), "REED: %s", reed_raw == 0u ? "Closed" : "Open");
    snprintf(line1, sizeof(line1), "BTN:  %s", btn_raw == 0u ? "Pressed" : "Released");
    app_lcd_show(line0, line1);
}

static void app_key_5_action()
{
    LOG_I(TAG, "KEY 5 TEMP: IMU=%d C", (int)g_last_imu_temp);
    app_show_sensor_temp_on_lcd("5: Gyroscope", g_last_imu_temp);

    g_key5_delay_tick    = 0u;
    g_key5_live_gyr_mode = 1u;
}

static void app_key_6_action()
{
    LOG_I(TAG, "KEY 6 TEMP: Mag=%d C", (int)g_last_mag_temp);
    app_show_sensor_temp_on_lcd("6: Magnetometer", g_last_mag_temp);

    g_key6_delay_tick    = 0u;
    g_key6_live_mag_mode = 1u;
}

static void app_show_baro_live_on_lcd(lib_barometer_data_t baro)
{
    char line0[17];
    char line1[17];
    int t_int  = (int)(baro.temperature / 10);
    int t_frac = (int)((baro.temperature < 0 ? -baro.temperature : baro.temperature) % 10);
    snprintf(line0, sizeof(line0), "P:%7lu Pa", (unsigned long)baro.pressure);
    snprintf(line1, sizeof(line1), "T: %+d.%d C", t_int, t_frac);
    app_lcd_show(line0, line1);
}

static void app_key_7_action()
{
    app_show_key_splash("7: Barometer", "Live P+T");
    g_key7_live_baro_mode = 1u;
    LOG_I(TAG, "KEY 7: Live barometer mode ON");
}

static void app_key_8_action(uint8_t digits[8])
{
    LOG_I(TAG, "KEY 8: Countdown timer start");
    app_show_key_splash("8: Countdown", "99999999 -> 0");

    /* Step 1009 (not divisible by 10) so every digit visibly changes each update. */
    uint32_t value = 99999999u;

    do
    {
        digits[0] = (uint8_t)((value / 10000000u) % 10u);
        digits[1] = (uint8_t)((value / 1000000u)  % 10u);
        digits[2] = (uint8_t)((value / 100000u)   % 10u);
        digits[3] = (uint8_t)((value / 10000u)    % 10u);
        digits[4] = (uint8_t)((value / 1000u)     % 10u);
        digits[5] = (uint8_t)((value / 100u)      % 10u);
        digits[6] = (uint8_t)((value / 10u)       % 10u);
        digits[7] = (uint8_t)(value               % 10u);
        lib_seg_display_update(digits);

        if (value < 1009u) { break; }
        value -= 1009u;
    } while (1);

    /* Show zero */
    for (uint8_t i = 0u; i < 8u; i++) { digits[i] = i; }
    lib_seg_display_update(digits);

    LOG_I(TAG, "KEY 8: Countdown done");
    app_show_default_lcd_splash();
}

static void app_key_9_action()
{
    g_safe_state = STATE_IDLE;
    app_show_key_splash("SAFE: RFID SCAN", "Wait for tag...");
    g_key9_live_rfid_mode = 1u;
    LOG_I(TAG, "Safe logic reset to IDLE, waiting for RFID");
}

static void app_key_star_action(void)
{
    if (g_safe_state == STATE_OPENED){
        app_show_key_splash("ACCESS GRANTED", "OPENING...");
        lib_servo_set_angle(180u);
        RELAY_Write(1u);
        FET_Write(1u);
        lib_buzzer_play_melody();
        
        CyDelay(3000);
        
        RELAY_Write(0u);
        FET_Write(0u);
        
        lib_servo_set_angle(SERVO_MID_ANGLE_DEG);
        g_safe_state = STATE_IDLE;
        app_show_default_lcd_splash();
    }else{
        app_show_key_splash("LOCKED!", "Follow stages!");
    }
}

static void app_key_hash_action(void)
{
    uint8_t reed_raw = REED_SW_Read();
    uint8_t btn_raw  = BTN_SW_Read();

    g_key12_live_reed_mode = 1u;
    app_show_reed_btn_on_lcd(reed_raw, btn_raw);
    LOG_I(TAG, "KEY 12/#: REED_SW raw=%u BTN_SW raw=%u", reed_raw, btn_raw);
}

static void app_deactivate_all_live_modes(void)
{
    g_key3_live_adc_mode   = 0u;
    g_key4_live_acc_mode   = 0u;
    g_key5_live_gyr_mode   = 0u;
    g_key6_live_mag_mode   = 0u;
    g_key7_live_baro_mode  = 0u;
    g_key9_live_rfid_mode  = 0u;
    g_key9_no_tag_reads    = 0u;
    g_key9_has_tag         = 0u;
    g_key12_live_reed_mode = 0u;
}

static void app_handle_password_input(lib_mkb_key_t key)
{
    if (g_is_locked_by_attempts) {
        app_lcd_show("SYSTEM LOCKED!", "SCAN RFID TAG");
        return;
    }

    if (key >= LIB_MKB_KEY_0 && key <= LIB_MKB_KEY_9) {
        char pressed_char = '0' + (key - LIB_MKB_KEY_0);
        
        if (g_password_index < 4) {
            g_entered_password[g_password_index++] = pressed_char;
            g_entered_password[g_password_index] = '\0';
            
            char buf[17];
            snprintf(buf, sizeof(buf), "Code: %s", g_entered_password);
            app_lcd_show("ENTER PASSWORD", buf);
            lib_buzzer_play_melody();
        }

        if (g_password_index == 4) {
            CyDelay(500);
            if (strcmp(g_entered_password, g_correct_password) == 0) {
                g_wrong_attempts = 0;
                LED_GREEN_Write(1u);
                app_lcd_show("ACCESS GRANTED", "TURN THE KNOB");
                g_safe_state = STATE_OPENED;
            } else {
                g_wrong_attempts++;
                g_password_index = 0;
                memset(g_entered_password, 0, sizeof(g_entered_password));
                LED_RED_Write(1u);
                
                if (g_wrong_attempts >= 3) {
                    g_is_locked_by_attempts = 1;
                    app_lcd_show("WRONG CODE!", "RFID REQUIRED");
                } else {
                    char err[17];
                    snprintf(err, sizeof(err), "Try: %d/3", g_wrong_attempts);
                    app_lcd_show("ACCESS DENIED", err);
                    CyDelay(1000);
                    LED_RED_Write(0u);
                    app_show_default_lcd_splash();
                }
            }
        }
    }
}

static void app_execute_key_action(lib_mkb_key_t key, uint8_t digits[8])
{
    app_deactivate_all_live_modes();
    switch (key)
    {
        case LIB_MKB_KEY_0:
            app_key_0_action();
            break;
        case LIB_MKB_KEY_1:
            app_key_1_action();
            break;
        case LIB_MKB_KEY_2:
            app_key_2_action();
            break;
        case LIB_MKB_KEY_3:
            app_key_3_action();
            break;
        case LIB_MKB_KEY_4:
            app_key_4_action();
            break;
        case LIB_MKB_KEY_5:
            app_key_5_action();
            break;
        case LIB_MKB_KEY_6:
            app_key_6_action();
            break;
        case LIB_MKB_KEY_7:
            app_key_7_action();
            break;
        case LIB_MKB_KEY_8:
            app_key_8_action(digits);
            break;
        case LIB_MKB_KEY_9:
            app_key_9_action();
            break;
        case LIB_MKB_KEY_STAR:
            app_key_star_action();
            break;
        case LIB_MKB_KEY_HASH:
            app_key_hash_action();
            break;
        default:
            break;
    }
}

static void app_handle_key_release(lib_mkb_key_t key)
{
    if (key == LIB_MKB_KEY_1)
    {
        app_show_default_lcd_splash();
        LOG_I(TAG, "KEY 1: Released, splash restored");
    }
    else if (key == LIB_MKB_KEY_2)
    {
        app_show_default_lcd_splash();
        LOG_I(TAG, "KEY 2: Released, splash restored");
    }
}



static void app_process_keyboard(uint8_t digits[8])
{
    uint8_t keys[4][3];
    lib_mkb_result_t mkb_result = lib_mkb_read(keys);

    if (mkb_result.status != LIB_MKB_STATE_CHANGED)
    {
        return;
    }

    LOG_I("MKB", "Key=%d State=%d", mkb_result.key_code, mkb_result.key_state);

    if (mkb_result.key_state == LIB_MKB_KEY_PRESSED)
    {
        app_handle_password_input(mkb_result.key_code);
        app_execute_key_action(mkb_result.key_code, digits);
    }
    else if (mkb_result.key_state == LIB_MKB_KEY_RELEASED)
    {
        app_handle_key_release(mkb_result.key_code);
    }
}

static void app_process_adc_live_mode(void)
{
    if (g_key3_live_adc_mode == 1u)
    {
        g_key3_delay_tick++;
        if (g_key3_delay_tick >= SENSOR_SHOW_DELAY_TICKS)
        {
            int16_t adc0 = lib_adc_get(ADC_CH_POT_2_0);
            int16_t adc1 = lib_adc_get(ADC_CH_POT_2_1);
            int16_t adc2 = lib_adc_get(ADC_CH_EXT_2_5);
            app_show_adc_live_on_lcd(adc0, adc1, adc2);
            g_key3_live_adc_mode = 2u;
        }
    }
    else if (g_key3_live_adc_mode == 2u)
    {
        static uint16_t adc_tick = 0u;

        if (adc_tick == 0u)
        {
            int16_t adc0 = lib_adc_get(ADC_CH_POT_2_0);
            int16_t adc1 = lib_adc_get(ADC_CH_POT_2_1);
            int16_t adc2 = lib_adc_get(ADC_CH_EXT_2_5);
            app_show_adc_live_on_lcd(adc0, adc1, adc2);
        }

        adc_tick++;
        if (adc_tick >= ADC_LCD_UPDATE_TICKS)
        {
            adc_tick = 0u;
        }
    }
}

static void app_process_rfid_live_mode(void)
{
    if (g_key9_live_rfid_mode != 0u)
    {
        static uint16_t rfid_tick = 0u;

        if (rfid_tick == 0u)
        {
            uint8_t uid[RFID_UID_LEN];
            ret_code_t result = lib_rfid_scan(uid);

            if (result == RET_CODE_OK)
            {
                char line0[17];
                char line1[17];

                snprintf(line0, sizeof(line0), "%02X %02X %02X %02X %02X",
                         uid[0], uid[1], uid[2], uid[3], uid[4]);
                snprintf(line1, sizeof(line1), "TAG FOUND!");

                app_lcd_show(line0, line1);
                g_key9_no_tag_reads = 0u;
                g_key9_has_tag = 1u;
                g_is_locked_by_attempts = 0;
                g_wrong_attempts = 0;

                LOG_I(TAG, "KEY 9: TAG %02X %02X %02X %02X %02X",
                      uid[0], uid[1], uid[2], uid[3], uid[4]);
            }
            else
            {
                if (g_key9_no_tag_reads < 255u)
                {
                    g_key9_no_tag_reads++;
                }

                if (g_key9_has_tag == 0u)
                {
                    if (g_key9_no_tag_reads >= RFID_NO_TAG_READS)
                    {
                        app_show_key_splash("9: RFID Scan", "No tag found");
                    }
                }
                else if (g_key9_no_tag_reads >= RFID_NO_TAG_READS)
                {
                    g_key9_has_tag = 0u;
                    app_show_key_splash("9: RFID Scan", "No tag found");
                }
            }
        }

        rfid_tick++;
        if (rfid_tick >= RFID_LCD_UPDATE_TICKS)
        {
            rfid_tick = 0u;
        }
    }
}

static void app_process_baro_live_mode(void)
{
    if (g_key7_live_baro_mode != 0u)
    {
        static uint16_t baro_tick = 0u;

        if (baro_tick == 0u)
        {
            lib_barometer_data_t baro = lib_barometer_get();
            app_show_baro_live_on_lcd(baro);
            LOG_I(TAG, "KEY 7: Baro P=%lu Pa T=%d C",
                  (unsigned long)baro.pressure, (int)baro.temperature);
        }

        baro_tick++;
        if (baro_tick >= ADC_LCD_UPDATE_TICKS)
        {
            baro_tick = 0u;
        }
    }
}

static void app_process_reed_live_mode(void)
{
    if (g_key12_live_reed_mode != 0u)
    {
        static uint16_t reed_tick = 0u;

        if (reed_tick == 0u)
        {
            uint8_t reed_raw = REED_SW_Read();
            uint8_t btn_raw  = BTN_SW_Read();
            app_show_reed_btn_on_lcd(reed_raw, btn_raw);
            LOG_I(TAG, "KEY 12/#: REED_SW raw=%u BTN_SW raw=%u", reed_raw, btn_raw);
        }

        reed_tick++;
        if (reed_tick >= ADC_LCD_UPDATE_TICKS)
        {
            reed_tick = 0u;
        }
    }
}

static void app_process_acc_live_mode(const lib_acc_gyr_data_t *acc_gyr_data)
{
    if (g_key4_live_acc_mode == 1u)
    {
        g_key4_delay_tick++;
        if (g_key4_delay_tick >= SENSOR_SHOW_DELAY_TICKS)
        {
            LOG_I(TAG, "KEY 4: Acc X=%ld Y=%ld Z=%ld",
                  acc_gyr_data->acc.x, acc_gyr_data->acc.y, acc_gyr_data->acc.z);
            app_show_acc_live_on_lcd(acc_gyr_data->acc.x, acc_gyr_data->acc.y, acc_gyr_data->acc.z);
            g_key4_live_acc_mode = 2u;
        }
    }
    else if (g_key4_live_acc_mode == 2u)
    {
        static uint16_t acc_live_tick = 0u;
        acc_live_tick++;
        if (acc_live_tick >= ADC_LCD_UPDATE_TICKS)
        {
            acc_live_tick = 0u;
            LOG_I(TAG, "KEY 4: Acc X=%ld Y=%ld Z=%ld",
                  acc_gyr_data->acc.x, acc_gyr_data->acc.y, acc_gyr_data->acc.z);
            app_show_acc_live_on_lcd(acc_gyr_data->acc.x, acc_gyr_data->acc.y, acc_gyr_data->acc.z);
        }
    }
}

static void app_process_gyr_live_mode(const lib_acc_gyr_data_t *acc_gyr_data)
{
    if (g_key5_live_gyr_mode == 1u)
    {
        g_key5_delay_tick++;
        if (g_key5_delay_tick >= SENSOR_SHOW_DELAY_TICKS)
        {
            LOG_I(TAG, "KEY 5: Gyr X=%ld Y=%ld Z=%ld",
                  acc_gyr_data->gyr.x, acc_gyr_data->gyr.y, acc_gyr_data->gyr.z);
            app_show_gyr_live_on_lcd(acc_gyr_data->gyr.x, acc_gyr_data->gyr.y, acc_gyr_data->gyr.z);
            g_key5_live_gyr_mode = 2u;
        }
    }
    else if (g_key5_live_gyr_mode == 2u)
    {
        static uint16_t gyr_live_tick = 0u;
        gyr_live_tick++;
        if (gyr_live_tick >= ADC_LCD_UPDATE_TICKS)
        {
            gyr_live_tick = 0u;
            LOG_I(TAG, "KEY 5: Gyr X=%ld Y=%ld Z=%ld",
                  acc_gyr_data->gyr.x, acc_gyr_data->gyr.y, acc_gyr_data->gyr.z);
            app_show_gyr_live_on_lcd(acc_gyr_data->gyr.x, acc_gyr_data->gyr.y, acc_gyr_data->gyr.z);
        }
    }
}

static void app_process_mag_live_mode(const lib_magnetometer_data_t *magnetometer_data)
{
    if (g_key6_live_mag_mode == 1u)
    {
        g_key6_delay_tick++;
        if (g_key6_delay_tick >= SENSOR_SHOW_DELAY_TICKS)
        {
            LOG_I(TAG, "KEY 6: Mag X=%ld Y=%ld Z=%ld",
                  magnetometer_data->mag.x, magnetometer_data->mag.y, magnetometer_data->mag.z);
            app_show_mag_live_on_lcd(magnetometer_data->mag.x, magnetometer_data->mag.y, magnetometer_data->mag.z);
            g_key6_live_mag_mode = 2u;
        }
    }
    else if (g_key6_live_mag_mode == 2u)
    {
        static uint16_t mag_live_tick = 0u;
        mag_live_tick++;
        if (mag_live_tick >= ADC_LCD_UPDATE_TICKS)
        {
            mag_live_tick = 0u;
            LOG_I(TAG, "KEY 6: Mag X=%ld Y=%ld Z=%ld",
                  magnetometer_data->mag.x, magnetometer_data->mag.y, magnetometer_data->mag.z);
            app_show_mag_live_on_lcd(magnetometer_data->mag.x, magnetometer_data->mag.y, magnetometer_data->mag.z);
        }
    }
}

static void app_update_sensor_temperature_cache(const lib_acc_gyr_data_t *acc_gyr_data,
                                                const lib_magnetometer_data_t *magnetometer_data)
{
    if (acc_gyr_data->is_new.tmp)
    {
        g_last_imu_temp = acc_gyr_data->temperature;
    }
    if (magnetometer_data->is_new)
    {
        g_last_mag_temp = magnetometer_data->temperature;
    }
}

static void app_process_runtime_modes(const lib_acc_gyr_data_t *acc_gyr_data,
                                      const lib_magnetometer_data_t *magnetometer_data)
{
    app_process_adc_live_mode();
    app_process_rfid_live_mode();
    app_process_baro_live_mode();
    app_process_reed_live_mode();
    app_process_acc_live_mode(acc_gyr_data);
    app_process_gyr_live_mode(acc_gyr_data);
    app_process_mag_live_mode(magnetometer_data);
}

static void app_process_safe_logic(const lib_acc_gyr_data_t *imu)
{
    // 1. Акселерометр: сигналізація при трясці [cite: 291, 326]
    if (abs(imu->acc.x) > 1500 || abs(imu->acc.y) > 1500) {
        LED_RED_Write(1u);
        lib_buzzer_play_melody(); // Вмикаємо звук [cite: 304]
        app_lcd_show("ALARM!", "VIBRATION!");
    }

    // 2. Датчик світла (LDR): якщо сейф закритий, а всередині стало світло [cite: 20, 292]
    int16_t light = lib_adc_get(ADC_CH_POT_2_0); // Зчитуємо датчик світла [cite: 29]
    if (light < LDR_DARK_THRESHOLD && g_safe_state == STATE_IDLE) {
        LED_RED_Write(1u);
        app_lcd_show("ALARM!", "LIGHT DETECTED");
    }

    // 3. Поворот ручки (Потенціометр) після пароля [cite: 300, 312]
    if (g_safe_state == STATE_OPENED) {
        int16_t knob = lib_adc_get(ADC_CH_POT_2_1); // Зчитуємо ручку [cite: 29]
        if (knob > 2000) { // Якщо ручку повернули достатньо
            RELAY_Write(1u); // Відкриваємо соленоїд [cite: 24, 306]
            FET_Write(1u);
            app_lcd_show("ACCESS GRANTED", "SAFE OPEN");
        } else {
            app_lcd_show("CODE OK", "TURN THE KNOB");
        }
    }

    // 4. Reed Switch: закриття соленоїда при зачиненні дверей [cite: 22, 326]
    if (REED_SW_Read() == 0u) { // Магніт піднесено (двері зачинено)
        RELAY_Write(0u); // Закриваємо замок
        FET_Write(0u);
        if (g_safe_state == STATE_OPENED) {
            g_safe_state = STATE_IDLE;
            LED_GREEN_Write(0u);
            app_show_default_lcd_splash();
        }
    }
}

static void app_init_libraries(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    dbg_log_init();

    SPIM_Start();
    lib_rfid_init();
    lib_mkb_init();

    lib_seg_display_init();

    I2C_Start();
    lib_lcd1602_init();    
    app_show_default_lcd_splash();

    lib_adc_init();

    lib_servo_init();

    lib_acc_gyr_init();
    lib_magnetometer_init();
    lib_barometer_init();
    lib_buzzer_init();

    lib_servo_set_angle(SERVO_MID_ANGLE_DEG);
    LOG_I("MAIN", "All libraries initialized");
}


/********************************************************************************
 **********                             MAIN                          ***********
*********************************************************************************/
int main(void)
{
    app_init_libraries();

/**************             7Seg Display set default data         ***************/
    uint8_t digits[8];
    for (int i = 0; i < 8; i++)
    {
        digits[i] = i;
    }
    lib_seg_display_update(digits);



/**************                     Turn Off Leds                  ***************/
    LED_RED_Write(0u);
    LED_BLUE_Write(0u);
    LED_GREEN_Write(0u);


    for(;;)
    {   
        lib_acc_gyr_data_t acc_gyr_data = lib_acc_gyr_get();
        lib_magnetometer_data_t magnetometer_data = lib_magnetometer_get();

        app_process_safe_logic(&acc_gyr_data);

        app_update_sensor_temperature_cache(&acc_gyr_data, &magnetometer_data);
        app_process_keyboard(digits);
        app_process_runtime_modes(&acc_gyr_data, &magnetometer_data);
      
        CyDelay(LOOP_DELAY_MS);
    }
}

/* [] END OF FILE */
