#include "flow3r_bsp_max98091.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#include "flow3r_bsp_max98091_hwregs.h"

#define TIMEOUT_MS 1000

static const char *TAG = "flow3r-bsp-max98091";

static uint8_t max98091_read(const uint8_t reg) {
    const uint8_t tx[] = { reg };
    uint8_t rx[1];

    i2c_master_write_read_device(
        I2C_NUM_0, 0x10, tx, sizeof(tx), rx, sizeof(rx), TIMEOUT_MS / portTICK_PERIOD_MS);
    return rx[0];
}

static esp_err_t max98091_write(const uint8_t reg, const uint8_t data) {
    const uint8_t tx[] = { reg, data };
    return i2c_master_write_to_device(I2C_NUM_0, 0x10, tx, sizeof(tx), TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t max98091_check(const uint8_t reg, const uint8_t data) {
    esp_err_t ret = max98091_write(reg, data);
    if (ret != ESP_OK) {
        return ret;
    }

    switch (reg) {
        // Do not attempt to read back registers that are write-only.
        case MAX98091_SOFTWARE_RESET:
        case MAX98091_DAI_INTERFACE:
        case MAX98091_DAC_PATH:
        case MAX98091_LINE_TO_ADC:
            return ESP_OK;
    }
    uint8_t readback = max98091_read(reg);
    if (readback != data) {
        ESP_LOGE(TAG, "Write of %02X failed: wanted %02x, got %02x", reg, data,
                 readback);
    }
    return ESP_OK;
}

void flow3r_bsp_max98091_register_poke(uint8_t reg, uint8_t data) {
    max98091_check(reg, data);
}

typedef struct {
    uint8_t raw_value;
    float volume_dB;
} volume_step_t;

static const uint8_t speaker_map_len = 40;
static const volume_step_t speaker_map[] = {
    { 0x3F, +14 }, { 0x3E, +13.5 }, { 0x3D, +13 }, { 0x3C, +12.5 },
    { 0x3B, +12 }, { 0x3A, +11.5 }, { 0x39, +11 }, { 0x38, +10.5 },
    { 0x37, +10 }, { 0x36, +9.5 },  { 0x35, +9 },  { 0x34, +8 },
    { 0x33, +7 },  { 0x32, +6 },    { 0x31, +5 },  { 0x30, +4 },
    { 0x2F, +3 },  { 0x2E, +2 },    { 0x2D, +1 },  { 0x2C, +0 },
    { 0x2B, -1 },  { 0x2A, -2 },    { 0x29, -3 },  { 0x28, -4 },
    { 0x27, -5 },  { 0x26, -6 },    { 0x25, -8 },  { 0x24, -10 },
    { 0x23, -12 }, { 0x22, -14 },   { 0x21, -17 }, { 0x20, -20 },
    { 0x1F, -23 }, { 0x1E, -26 },   { 0x1D, -29 }, { 0x1C, -32 },
    { 0x1B, -36 }, { 0x1A, -40 },   { 0x19, -44 }, { 0x18, -48 }
};

static const volume_step_t *find_speaker_volume(float vol_dB) {
    uint8_t map_index = speaker_map_len - 1;
    for (; map_index; map_index--) {
        if (speaker_map[map_index].volume_dB >= vol_dB)
            return &speaker_map[map_index];
    }
    return &speaker_map[0];
}

static const uint8_t headphones_map_len = 32;
static const volume_step_t headphones_map[] = {
    { 0x1F, +3 },  { 0x1E, +2.5 }, { 0x1D, +2 },  { 0x1C, +1.5 }, { 0x1B, +1 },
    { 0x1A, +0 },  { 0x19, -1 },   { 0x18, -2 },  { 0x17, -3 },   { 0x16, -4 },
    { 0x15, -5 },  { 0x14, -7 },   { 0x13, -9 },  { 0x12, -11 },  { 0x11, -13 },
    { 0x10, -15 }, { 0x0F, -17 },  { 0x0E, -19 }, { 0x0D, -22 },  { 0x0C, -25 },
    { 0x0B, -28 }, { 0x0A, -31 },  { 0x09, -34 }, { 0x08, -37 },  { 0x07, -40 },
    { 0x06, -43 }, { 0x06, -47 },  { 0x04, -51 }, { 0x03, -55 },  { 0x02, -59 },
    { 0x01, -63 }, { 0x00, -67 }
};

static const volume_step_t *find_headphones_volume(float vol_dB) {
    uint8_t map_index = headphones_map_len - 1;
    for (; map_index; map_index--) {
        if (headphones_map[map_index].volume_dB >= vol_dB)
            return &headphones_map[map_index];
    }
    return &headphones_map[0];
}

float flow3r_bsp_max98091_headphones_set_volume(bool mute, float dB) {
    const volume_step_t *step = find_headphones_volume(dB);
    ESP_LOGI(TAG, "Setting headphones volume: %d/%02x", mute, step->raw_value);
    max98091_check(MAX98091_LEFT_HP_VOLUME,
                   MAX98091_BOOL(LEFT_HP_VOLUME_HPLM, mute) |
                       MAX98091_BITS(LEFT_HP_VOLUME_HPVOLL, step->raw_value));
    max98091_check(MAX98091_RIGHT_HP_VOLUME,
                   MAX98091_BOOL(RIGHT_HP_VOLUME_HPRM, mute) |
                       MAX98091_BITS(RIGHT_HP_VOLUME_HPVOLR, step->raw_value));
    return step->volume_dB;
}

float flow3r_bsp_max98091_speaker_set_volume(bool mute, float dB) {
    const volume_step_t *step = find_speaker_volume(dB);
    ESP_LOGI(TAG, "Setting speakers volume: %d/%02x", mute, step->raw_value);
    max98091_check(MAX98091_LEFT_SPK_VOLUME,
                   MAX98091_BOOL(LEFT_SPK_VOLUME_SPLM, mute) |
                       MAX98091_BITS(LEFT_SPK_VOLUME_SPVOLL, step->raw_value));
    max98091_check(MAX98091_RIGHT_SPK_VOLUME,
                   MAX98091_BOOL(RIGHT_SPK_VOLUME_SPRM, mute) |
                       MAX98091_BITS(RIGHT_SPK_VOLUME_SPVOLR, step->raw_value));
    return step->volume_dB;
}
void flow3r_bsp_max98091_init(void) {
#define POKE(n, v) ESP_ERROR_CHECK(max98091_check(MAX98091_##n, v))
    ESP_LOGI(TAG, "Codec initializing...");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    POKE(SOFTWARE_RESET, 0x80);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    POKE(DEVICE_SHUTDOWN, 0);
    // pclk = mclk / 1
    POKE(SYSTEM_CLOCK, MAX98091_BITS(SYSTEM_CLOCK_PSCLK, 1));
    // music, dc filter in record and playback
    POKE(FILTER_CONFIGURATION, (1 << 7) | (1 << 6) | (1 << 5) | (1 << 2));
    // Sets up DAI for left-justified slave mode operation.
    POKE(DAI_INTERFACE, 1 << 2);
    // Sets up the DAC to speaker path
    POKE(DAC_PATH, 1 << 5);

    // Somehow this was needed to get an input signal to the ADC, even though
    // all other registers should be taken care of later. Don't know why.
    // Sets up the line in to adc path
    POKE(LINE_TO_ADC, 1 << 6);

    // SDOUT, SDIN enabled
    POKE(IO_CONFIGURATION, (1 << 1) | (1 << 0));
    // bandgap bias
    POKE(BIAS_CONTROL, 1 << 0);
    // high performane mode
    POKE(DAC_CONTROL, 1 << 0);

    // enable micbias, line input amps, ADCs
    POKE(INPUT_ENABLE, (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0));
    // IN3 SE -> Line A, IN4 SE -> Line B
    POKE(LINE_INPUT_CONFIG, (1 << 3) | (1 << 2));
    // 64x oversampling, dithering, high performance ADC
    POKE(ADC_CONTROL, (1 << 1) | (1 << 0));

    POKE(DIGITAL_MIC_ENABLE, 0);
    // IN5/IN6 to MIC1
    POKE(INPUT_MODE, (1 << 0));

    max98091_check(MAX98091_DIGITAL_MIC_ENABLE,
                   MAX98091_BITS(DIGITAL_MIC_ENABLE_DMICCLK, 3) |
                       MAX98091_BOOL(DIGITAL_MIC_ENABLE_DIGMIC1L, false) |
                       MAX98091_BOOL(DIGITAL_MIC_ENABLE_DIGMIC1R, false));
    max98091_check(MAX98091_LEFT_ADC_MIXER, 0);
    max98091_check(MAX98091_RIGHT_ADC_MIXER, 0);

    // output enable: enable dacs
    POKE(OUTPUT_ENABLE, (1 << 1) | (1 << 0));
    // power up
    POKE(DEVICE_SHUTDOWN, 1 << 7);
    // enable outputs, dacs
    POKE(OUTPUT_ENABLE,
         (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 1) | (1 << 0));
    // disable all digital filters except for dc blocking
    POKE(DSP_FILTER_ENABLE, 0x0);

    // TODO(q3k): mute this
    ESP_LOGI(
        TAG,
        "Codec initilialied! Don't worry about the readback errors above.");
#undef POKE
}
