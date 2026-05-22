#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_continuous.h"
#include "dsps_fft2r.h"
#include "driver/gpio.h"

#define MOTOR_LEFT_PIN   GPIO_NUM_25
#define MOTOR_FRONT_PIN  GPIO_NUM_26
#define MOTOR_RIGHT_PIN  GPIO_NUM_27

#define MOTOR_VIBRATE_MS 800 


#define SAMPLE_RATE     20000
    #define FFT_SIZE        1024

#define MIC_LEFT   ADC_CHANNEL_6   // GPIO34
#define MIC_FRONT  ADC_CHANNEL_7   // GPIO35
#define MIC_RIGHT  ADC_CHANNEL_4   // GPIO32

#define SIREN_LOW_HZ   500.0f
#define SIREN_HIGH_HZ  4000.0f
#define SIREN_THRESHOLD 70000.0f  
#define HISTORY_SIZE 15
                        
#define NUM_CHANNELS 3

#define MIN_SIREN_SWEEP 200.0f

#define CAL_LEFT   1.0f
#define CAL_FRONT  1.0f  
#define CAL_RIGHT  1.0f

typedef struct {
    float freq_history[HISTORY_SIZE];
    int history_index;
} channel_history_t;

static channel_history_t hist_left  = {0};
static channel_history_t hist_front = {0};
static channel_history_t hist_right = {0};
static bool left_motor_running = false;
static bool front_motor_running = false;
static bool right_motor_running = false;



static adc_continuous_handle_t adc_handle;

static float fft_left[FFT_SIZE * 2];
static float fft_front[FFT_SIZE * 2];
static float fft_right[FFT_SIZE * 2];
static uint8_t raw_buf[FFT_SIZE * NUM_CHANNELS * 4];
static volatile bool buffer_ready = false;

static void motors_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_LEFT_PIN) |
                        (1ULL << MOTOR_FRONT_PIN) |
                        (1ULL << MOTOR_RIGHT_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // ensure all motors off at boot
    gpio_set_level(MOTOR_LEFT_PIN,  0);
    gpio_set_level(MOTOR_FRONT_PIN, 0);
    gpio_set_level(MOTOR_RIGHT_PIN, 0);
}

// Call motor for a set duration then cut it
static void trigger_motor(gpio_num_t pin) {
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(MOTOR_VIBRATE_MS));
    gpio_set_level(pin, 0);
}

// FreeRTOS task so motor delay doesn't block the FFT loop
static void motor_task(void *arg) {
    gpio_num_t pin = (gpio_num_t)(intptr_t)arg;
    trigger_motor(pin);
    
    // NEW: Clear flag so this motor can be triggered again later
    if (pin == MOTOR_LEFT_PIN) left_motor_running = false;
    else if (pin == MOTOR_FRONT_PIN) front_motor_running = false;
    else if (pin == MOTOR_RIGHT_PIN) right_motor_running = false;
    
    vTaskDelete(NULL);  
}
static void fire_motor(gpio_num_t pin) {
    // NEW: Check if this motor task is already active. If yes, exit early.
    if (pin == MOTOR_LEFT_PIN && left_motor_running) return;
    if (pin == MOTOR_FRONT_PIN && front_motor_running) return;
    if (pin == MOTOR_RIGHT_PIN && right_motor_running) return;

    // NEW: Set active flag before spawning the task
    if (pin == MOTOR_LEFT_PIN) left_motor_running = true;
    else if (pin == MOTOR_FRONT_PIN) front_motor_running = true;
    else if (pin == MOTOR_RIGHT_PIN) right_motor_running = true;

    // Increased stack slightly to 2048 to prevent overflows during printfs
    xTaskCreate(motor_task, "motor_task", 2048, (void *)(intptr_t)pin, 5, NULL);
}


bool is_in_siren_range(float freq, float magnitude) {
    return (freq >= SIREN_LOW_HZ && freq <= SIREN_HIGH_HZ) 
           && (magnitude > SIREN_THRESHOLD);
}

bool detect_siren(float dominant_freq, float magnitude, channel_history_t *hist) {
    // Only log if in range and loud enough
    if (!is_in_siren_range(dominant_freq, magnitude)) {
        hist->freq_history[hist->history_index] = 0;
    } else {
        hist->freq_history[hist->history_index] = dominant_freq;
    }
    hist->history_index = (hist->history_index + 1) % HISTORY_SIZE;

    // Count how many recent frames were in siren range
    int count = 0;
    float min_f = SIREN_HIGH_HZ;
    float max_f = SIREN_LOW_HZ;
    for (int i = 0; i < HISTORY_SIZE; i++) {
        if (hist->freq_history[i] > 0){
            count++;
            if (hist->freq_history[i] < min_f) min_f = hist->freq_history[i];
            if (hist->freq_history[i] > max_f) max_f = hist->freq_history[i];
        }
    }

    bool has_enough_frames = (count >= 12);
    bool is_shifting = ((max_f - min_f) >= MIN_SIREN_SWEEP);

    return has_enough_frames && is_shifting;
}

int dominant_bin_with_mag(float *fft_data, float *out_magnitude) {
    float max_mag = 0;
    int max_bin = 0;
    for (int i = 1; i < FFT_SIZE / 2; i++) {
        float re = fft_data[2 * i];
        float im = fft_data[2 * i + 1];
        float mag = re * re + im * im;
        if (mag > max_mag) {
            max_mag = mag;
            max_bin = i;
        }
    }
    *out_magnitude = max_mag;
    return max_bin;
}



static bool IRAM_ATTR adc_conv_done(adc_continuous_handle_t handle,
                                     const adc_continuous_evt_data_t *edata,
                                     void *user_data) {
    buffer_ready = true;
    return false;
}




void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(100));
    // Init ADC
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = FFT_SIZE * NUM_CHANNELS * 4 * 2,
        .conv_frame_size    = FFT_SIZE * NUM_CHANNELS * 4,
    };
    adc_continuous_new_handle(&adc_config, &adc_handle);
    motors_init();           //

   

    // Configure channels
     adc_continuous_config_t chan_cfg = {
        .pattern_num = NUM_CHANNELS,
        .sample_freq_hz = SAMPLE_RATE * NUM_CHANNELS,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    adc_digi_pattern_config_t patterns[NUM_CHANNELS] = {
        { .atten = ADC_ATTEN_DB_12, .channel = MIC_LEFT,  .unit = ADC_UNIT_1, .bit_width = ADC_BITWIDTH_12 },
        { .atten = ADC_ATTEN_DB_12, .channel = MIC_FRONT, .unit = ADC_UNIT_1, .bit_width = ADC_BITWIDTH_12 },
        { .atten = ADC_ATTEN_DB_12, .channel = MIC_RIGHT, .unit = ADC_UNIT_1, .bit_width = ADC_BITWIDTH_12 },
    };
    chan_cfg.adc_pattern = patterns;

    // move config to reg
    adc_continuous_config(adc_handle, &chan_cfg);


    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_conv_done,
    };
    adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL);
    adc_continuous_start(adc_handle);


    
    uint32_t bytes_read = 0;
    dsps_fft2r_init_fc32(NULL, FFT_SIZE);

    while (1) {
        

        if(buffer_ready){
            buffer_ready = false;
            esp_err_t ret = adc_continuous_read(adc_handle, raw_buf, sizeof(raw_buf), &bytes_read, 0);
            if(ret == ESP_OK){
                memset(fft_left, 0, sizeof(fft_left));
                memset(fft_front, 0, sizeof(fft_front));
                memset(fft_right, 0, sizeof(fft_right));
                int li = 0, fi = 0, ri = 0;
                int num_results = bytes_read / sizeof(adc_digi_output_data_t);


                
                for (int i = 0; i < num_results; i++) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t *)&raw_buf[i * sizeof(adc_digi_output_data_t)];
                    
                    if (p->type1.channel == MIC_LEFT && li < FFT_SIZE) {
                        fft_left[li * 2]     = p->type1.data;
                        fft_left[li * 2 + 1] = 0;
                        li++;
                    } else if (p->type1.channel == MIC_FRONT && fi < FFT_SIZE) {
                        fft_front[fi * 2]     = p->type1.data;
                        fft_front[fi * 2 + 1] = 0;
                        fi++;
                    } else if (p->type1.channel == MIC_RIGHT && ri < FFT_SIZE) {
                        fft_right[ri * 2]     = p->type1.data;
                        fft_right[ri * 2 + 1] = 0;
                        ri++;
                    }
                }

                dsps_fft2r_fc32(fft_left,  FFT_SIZE);
                dsps_fft2r_fc32(fft_front, FFT_SIZE);
                dsps_fft2r_fc32(fft_right, FFT_SIZE);


                dsps_bit_rev_fc32(fft_left,  FFT_SIZE);
                dsps_bit_rev_fc32(fft_front, FFT_SIZE);
                dsps_bit_rev_fc32(fft_right, FFT_SIZE);

                float mag_left, mag_front, mag_right;

                int dom_left  = dominant_bin_with_mag(fft_left,  &mag_left);
                int dom_front = dominant_bin_with_mag(fft_front, &mag_front);
                int dom_right = dominant_bin_with_mag(fft_right, &mag_right);

                float freq_left  = dom_left  * (20000.0f / FFT_SIZE);
                float freq_front = dom_front * (20000.0f / FFT_SIZE);
                float freq_right = dom_right * (20000.0f / FFT_SIZE);

                mag_left *= CAL_LEFT;
                mag_right *= CAL_RIGHT;
                mag_front *= CAL_FRONT;


                printf("Left: %.1f Hz (%.0f) | Front: %.1f Hz (%.0f) | Right: %.1f Hz (%.0f)\n",
                    freq_left, mag_left, freq_front, mag_front, freq_right, mag_right);

            
                bool siren_left  = detect_siren(freq_left,  mag_left,  &hist_left);
                bool siren_front = detect_siren(freq_front, mag_front, &hist_front);
                bool siren_right = detect_siren(freq_right, mag_right, &hist_right);


                if (siren_left || siren_front || siren_right) {
                    printf("SIREN DETECTED! | ");
                    
                    // direction based on which mic is loudest
                    if (mag_left > mag_front && mag_left > mag_right) {
                        printf("Direction: LEFT\n");
                        fire_motor(MOTOR_LEFT_PIN);
                    } else if (mag_front > mag_left && mag_front > mag_right) {
                        printf("Direction: FRONT\n");
                        fire_motor(MOTOR_FRONT_PIN);
                    } else if (mag_right > mag_left && mag_right > mag_front) {
                        printf("Direction: RIGHT\n");
                        fire_motor(MOTOR_RIGHT_PIN);
                    } else {
                        printf("Direction: UNKNOWN\n");
                    }
                }

            }


            

        }

            vTaskDelay(pdMS_TO_TICKS(10));
    }
}