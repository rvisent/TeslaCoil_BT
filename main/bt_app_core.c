/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

// modified by Roberto Visentin (2020) to output upsampled sigma-delta stream
//  with I2S to drive musical Tesla Coil
// Apache license 2.0

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include "freertos/xtensa_api.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bt_app_core.h"
#include "driver/i2s.h"
#include "freertos/ringbuf.h"
#include "driver/adc.h" // RV (pot)
#include "i2c_rtc_clk.h"    // RV: I2C_WRITEREG
#include "i2c_apll.h"       // RV: I2C_WRITEREG params

static void bt_app_task_handler(void *arg);
static bool bt_app_send_msg(bt_app_msg_t *msg);
static void bt_app_work_dispatched(bt_app_msg_t *msg);

void ets_write_char_uart(char c);

#define ENCDATA_BUFSIZE 8192
#define TRIANGLE_TEST 0     // if 1, send triangular wave at 10.76 Hz (4096 samples at 44.1 kHz)
#define SEQUENCE_TEST 0     // if 1, send increasing one bit count 0-16
#define DUMP_BLOCK 0        // if 1, dumps block #1000 to serial

static size_t sigma_delta_encode(uint8_t *data, size_t item_size);
static void adjust_pll();
static uint8_t *pEncodedData = NULL;
static long prev_audio_sample = 0;
static long frac_time_Q14 = 0;
//extern long fOut;
extern union APLL_DATA apll_data;
static union APLL_DATA apll_data_dyn;     // modified in real-time
extern long fMusic;
extern uint8_t s_volume;
static uint8_t outBuf[16];  // partial output sequence container
static uint8_t outIdx = 0;
static long z1, z2;

static xQueueHandle s_bt_app_task_queue = NULL;
static xTaskHandle s_bt_app_task_handle = NULL;
static xTaskHandle s_bt_i2s_task_handle = NULL;
static RingbufHandle_t s_ringbuf_i2s = NULL;;

bool bt_app_work_dispatch(bt_app_cb_t p_cback, uint16_t event, void *p_params, int param_len, bt_app_copy_cb_t p_copy_cback)
{
    ESP_LOGD(BT_APP_CORE_TAG, "%s event 0x%x, param len %d", __func__, event, param_len);

    bt_app_msg_t msg;
    memset(&msg, 0, sizeof(bt_app_msg_t));

    msg.sig = BT_APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = p_cback;

    if (param_len == 0) {
        return bt_app_send_msg(&msg);
    } else if (p_params && param_len > 0) {
        if ((msg.param = malloc(param_len)) != NULL) {
            memcpy(msg.param, p_params, param_len);
            /* check if caller has provided a copy callback to do the deep copy */
            if (p_copy_cback) {
                p_copy_cback(&msg, msg.param, p_params);
            }
            return bt_app_send_msg(&msg);
        }
    }

    return false;
}

static bool bt_app_send_msg(bt_app_msg_t *msg)
{
    if (msg == NULL) {
        return false;
    }

    if (xQueueSend(s_bt_app_task_queue, msg, 10 / portTICK_RATE_MS) != pdTRUE) {
        ESP_LOGE(BT_APP_CORE_TAG, "%s xQueue send failed", __func__);
        return false;
    }
    return true;
}

static void bt_app_work_dispatched(bt_app_msg_t *msg)
{
    if (msg->cb) {
        msg->cb(msg->event, msg->param);
    }
}

static void bt_app_task_handler(void *arg)
{
    bt_app_msg_t msg;
    for (;;) {
        if (pdTRUE == xQueueReceive(s_bt_app_task_queue, &msg, (portTickType)portMAX_DELAY)) {
            ESP_LOGD(BT_APP_CORE_TAG, "%s, sig 0x%x, 0x%x", __func__, msg.sig, msg.event);
            switch (msg.sig) {
            case BT_APP_SIG_WORK_DISPATCH:
                bt_app_work_dispatched(&msg);
                break;
            default:
                ESP_LOGW(BT_APP_CORE_TAG, "%s, unhandled sig: %d", __func__, msg.sig);
                break;
            } // switch (msg.sig)

            if (msg.param) {
                free(msg.param);
            }
        }
    }
}

void bt_app_task_start_up(void)
{
    s_bt_app_task_queue = xQueueCreate(10, sizeof(bt_app_msg_t));
    xTaskCreatePinnedToCore(bt_app_task_handler, "BtAppT", 3072, NULL, configMAX_PRIORITIES - 3, &s_bt_app_task_handle,0);
    return;
}

void bt_app_task_shut_down(void)
{
    if (s_bt_app_task_handle) {
        vTaskDelete(s_bt_app_task_handle);
        s_bt_app_task_handle = NULL;
    }
    if (s_bt_app_task_queue) {
        vQueueDelete(s_bt_app_task_queue);
        s_bt_app_task_queue = NULL;
    }
}

static void bt_i2s_task_handler(void *arg)
{
    uint8_t *data = NULL;
    size_t item_size = 0;
    size_t bytes_written = 0;

#if DUMP_BLOCK
    static int delay=1000;
#endif

#if TRIANGLE_TEST
    int i;
    long d;
    uint8_t *p;
    size_t encoded_size;
    int cnt = 0;
    data = malloc(4096);    // ok, not freed...
    for (i=0; i<1024; i++) {
        // triangle +/-16384
        if (i<=256) d = i*64;
        else if (i<=768) d = 32768-i*64;
        else d = -65536+i*64;
        p = data+i*4;
        *p = *(p+2) = (uint8_t)(d>>8);
        *(p+1)=*(p+3) = (uint8_t)d;
    }
    item_size = 4096;

    ESP_LOGI(BT_APP_CORE_TAG, "entering loop");
    for (;;) {
        encoded_size = sigma_delta_encode(data, item_size);
        if (i2s_write(I2S_NUM_1, pEncodedData, encoded_size, &bytes_written, portMAX_DELAY) != ESP_OK)
            ESP_LOGE(BT_APP_CORE_TAG, "i2s_write error");
        else {
            if (cnt++ > 100) {
                cnt = 0;
                ESP_LOGI(BT_APP_CORE_TAG, "looping");
            }
        }
    }

#elif SEQUENCE_TEST

    int i;
    long d;
    uint8_t *p;
    size_t encoded_size;
    int cnt = 0;
    pEncodedData[0] = 0; pEncodedData[1] = 0x80; pEncodedData[2] = 0; pEncodedData[3] = 0;
    pEncodedData[4] = 0; pEncodedData[5] = 0xa8; pEncodedData[6] = 0; pEncodedData[7] = 0xa0;
    pEncodedData[8] = 0x80; pEncodedData[9] = 0xaa; pEncodedData[10] = 0; pEncodedData[11] = 0xaa;
    pEncodedData[12] = 0xa8; pEncodedData[13] = 0xaa; pEncodedData[14] = 0xa0; pEncodedData[15] = 0xaa;
    pEncodedData[16] = 0xff; pEncodedData[17] = 0xff; pEncodedData[18] = 0xaa; pEncodedData[19] = 0xaa;
    encoded_size = 20;

    ESP_LOGI(BT_APP_CORE_TAG, "entering loop");
    for (;;) {
        if (i2s_write(I2S_NUM_1, pEncodedData, encoded_size, &bytes_written, portMAX_DELAY) != ESP_OK)
            ESP_LOGE(BT_APP_CORE_TAG, "i2s_write error");
        else {
            if (cnt++ > 10000) {
                cnt = 0;
                ESP_LOGI(BT_APP_CORE_TAG, "looping");
            }
        }
    }


#else
    for (;;) {
        data = (uint8_t *)xRingbufferReceive(s_ringbuf_i2s, &item_size, (portTickType)portMAX_DELAY);
        if (item_size != 0) {
            size_t encoded_size;
#if DUMP_BLOCK

            if (delay-- == 0) {
                // dump block to serial
                int k;
                // insert markers
                ets_write_char_uart('\r');
                ets_write_char_uart('\n');
                ets_write_char_uart(0);
                ets_write_char_uart(0);
                for (k=0; k<item_size; k++)
                    ets_write_char_uart(data[k]);
                ets_write_char_uart(0);
                ets_write_char_uart(0);
                ets_write_char_uart('\r');
                ets_write_char_uart('\n');
            }
#endif
            encoded_size = sigma_delta_encode(data, item_size);
            i2s_write(I2S_NUM_1, pEncodedData, encoded_size, &bytes_written, portMAX_DELAY);
            vRingbufferReturnItem(s_ringbuf_i2s,(void *)data);
        }
    }
#endif
}

void bt_i2s_task_start_up(void)
{
    s_ringbuf_i2s = xRingbufferCreate(8 * 1024, RINGBUF_TYPE_BYTEBUF);
    if(s_ringbuf_i2s == NULL){
        return;
    }

    // RV: allocate memory for sigma-delta encoded data
    pEncodedData = (uint8_t *)malloc(ENCDATA_BUFSIZE);
    if (!pEncodedData) {
        ESP_LOGE(BT_APP_CORE_TAG, "Malloc error");
        vRingbufferDelete(s_ringbuf_i2s);
        s_ringbuf_i2s = NULL;
        return;
    }

    // RV: initialize ADC, GPIO35, range 0-2.2 V
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_6);

    outIdx = 0;
    frac_time_Q14 = 0;
    z1 = z2 = 0;
    apll_data_dyn = apll_data;

    // @@@@@@ RV26apr2020 stack from 1024 to 2048, discover why it did stack overflow
    // RV10mag2020 PinnedToCore(...,1)
    xTaskCreatePinnedToCore(bt_i2s_task_handler, "BtI2ST", /*1024*/2048, NULL, configMAX_PRIORITIES - 3, &s_bt_i2s_task_handle,1);
    return;
}

void bt_i2s_task_shut_down(void)
{
    if (s_bt_i2s_task_handle) {
        vTaskDelete(s_bt_i2s_task_handle);
        s_bt_i2s_task_handle = NULL;
    }

    if (s_ringbuf_i2s) {
        vRingbufferDelete(s_ringbuf_i2s);
        s_ringbuf_i2s = NULL;
    }

    if (pEncodedData) {
        free(pEncodedData);
        pEncodedData = NULL;
    }
}

size_t write_ringbuf(const uint8_t *data, size_t size)
{
#if TRIANGLE_TEST | SEQUENCE_TEST
    return size;
#endif
    BaseType_t done = xRingbufferSend(s_ringbuf_i2s, (void *)data, size, (portTickType)portMAX_DELAY);
    if(done){
        return size;
    } else {
        return 0;
    }
}


size_t sigma_delta_encode(uint8_t *data, size_t item_size)
{
    int i,j;
    long audio_sample, audio_resampled;
    long fStep_Q14;
    float fTesla;
    long d;

    adjust_pll();
    fTesla = apll_data_dyn.group.fOut_real/2;
    fStep_Q14 = (long)((fMusic<<14)/fTesla+.5);

    // pick one stereo sample at a time    
    for (i=j=0; i<item_size; i+=4) {
        // build one mono sample, use swap to pass from big endian notation
        //int16_t data_swapped;
        //swab(data+i, &data_swapped, 2);
        //audio_sample = data_swapped;
        //swab(data+i+2, &data_swapped, 2);
        //audio_sample += data_swapped;

        // no... it seems data isn't big endian...
        audio_sample = *(int16_t *)(data+i);
        audio_sample += *(int16_t *)(data+i+2);

        // apply volume and saturation to +/-65536 (we have added two channels)
        // volume is 0..127, allow multiplication by 8
        audio_sample = (audio_sample*s_volume) >> 4;
        if (audio_sample < -65536)
            audio_sample = -65536;
        else if (audio_sample > 65535)
            audio_sample = 65535;

        // interpolate to samples @fOut
        // frac_time is Q14
        while (frac_time_Q14 < 16384) {
            audio_resampled = ((prev_audio_sample<<14)+(audio_sample-prev_audio_sample)*frac_time_Q14) >> 14;
            frac_time_Q14 += fStep_Q14;

            // encode current sample
            // we map audio sample to output range 0.5-1.0 for the Tesla
            // now audio is Q16 (Q15 native in range +/-1.0, one bit added by average of stereo channels without division)
            // we interpret as Q18, so range becomes +/-0.25, and add Q18(0.75)=196608;
            audio_resampled += 196608L;
            d = audio_resampled - (2*z1 - z2);
            // saturate at +/-10
            if (d > 2621440L)
                d = 2621440L;
            else if (d < -2621440L)
                d = -2621440L;
            
            // set output bit
            outBuf[outIdx++] = (d > 0) ? 1:0;

            // update z^-2, z^-1 = out-d
            z2 = z1;
            z1 = ((d > 0) ? 262144L:0) - d;

            // if we have a full buffer, we can fill an I2S stereo sample (32 bits)
            if (outIdx == 16) {
                uint8_t *p = pEncodedData+j;
                // all bits in even positions are zero
                *p++ = (outBuf[12]?0x80:0) | (outBuf[13]?0x20:0) | (outBuf[14]?0x08:0) | (outBuf[15]?0x02:0);
                *p++ = (outBuf[8]?0x80:0) | (outBuf[9]?0x20:0) | (outBuf[10]?0x08:0) | (outBuf[11]?0x02:0);
                *p++ = (outBuf[4]?0x80:0) | (outBuf[5]?0x20:0) | (outBuf[6]?0x08:0) | (outBuf[7]?0x02:0);
                *p   = (outBuf[0]?0x80:0) | (outBuf[1]?0x20:0) | (outBuf[2]?0x08:0) | (outBuf[3]?0x02:0);
                outIdx = 0;
                j += 4;
                if (j > ENCDATA_BUFSIZE-4) {
                    ESP_LOGE(BT_APP_CORE_TAG, "pEncodedData overflow, item_size=%u, i=%d", item_size, i);
                    j = ENCDATA_BUFSIZE-4;
                }
            }
        }

        // copy current sample to previous and adjust frac_time_Q14, removing one sample time
        prev_audio_sample = audio_sample;
        frac_time_Q14 -= 16384;
    }

    return j;
}


void adjust_pll()
{
    union APLL_DATA apll;

    // get potentiometer value
    int pot = adc1_get_raw(ADC1_CHANNEL_7);
    // 1 digit in sdm0 weighs 40000000/65536/14/16 = 2.72 Hz
    // we want to move +/-10 kHz, so with 12-bit adc we have 10000/2048=4.88 Hz/digit
    // approx we want 2*adc in sdm0
    int delta_tune = (pot-2048)*2;
    apll.group.sdm = apll_data.group.sdm + delta_tune;
    apll.group.o_div = apll_data.group.o_div;

    // check change at least 100 (about 50 Hz)
    if (abs(apll.group.sdm-apll_data_dyn.group.sdm) > 100)
    {
        // apply without interruptions
        portDISABLE_INTERRUPTS();
        if (apll.sep.sdm0 != apll_data_dyn.sep.sdm0)
            I2C_WRITEREG_MASK_RTC(I2C_APLL, I2C_APLL_DSDM0, apll.sep.sdm0);
        if (apll.sep.sdm1 != apll_data_dyn.sep.sdm1)
            I2C_WRITEREG_MASK_RTC(I2C_APLL, I2C_APLL_DSDM1, apll.sep.sdm1);
        if (apll.sep.sdm2 != apll_data_dyn.sep.sdm2)
            I2C_WRITEREG_MASK_RTC(I2C_APLL, I2C_APLL_DSDM2, apll.sep.sdm2);
        taskENABLE_INTERRUPTS();

        // compute frequency (assume N=2, a=0, b=1, M=8)
        apll.sep.fOut_real = 40000000.0*(apll.sep.sdm2 + apll.sep.sdm1/256.0 + apll.sep.sdm0/65536.0 + 4)/(2*(apll.sep.o_div+2))/(8*2);

        apll_data_dyn = apll;
        ESP_LOGI(BT_APP_CORE_TAG, "fDelta = %d, f = %f", delta_tune, apll.group.fOut_real);
    }

}
