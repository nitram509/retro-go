// REF: https://wiki.odroid.com/odroid_go/odroid_go

// Target definition
#define RG_TARGET_NAME             "FLOW3R"

// Storage and Settings
// TODO, MK:
#define RG_STORAGE_DRIVER           2                   // 0 = Host, 1 = SDSPI, 2 = SDMMC, 3 = USB, 4 = Flash
#define RG_STORAGE_HOST             SDMMC_HOST_SLOT_1   // Used by driver 1 and 2
#define RG_STORAGE_SPEED            SDMMC_FREQ_DEFAULT  // Used by driver 1 and 2
#define RG_STORAGE_ROOT             "/sd"               // Storage mount point

// Audio
// TODO, MK: enable only, when sound driver ported
#define RG_AUDIO_USE_INT_DAC        0   // 0 = Disable, 1 = GPIO25, 2 = GPIO26, 3 = Both
#define RG_AUDIO_USE_EXT_DAC        0   // 0 = Disable, 1 = Enable

// Video
// TODO, MK:
#define RG_SCREEN_DRIVER            0   // 0 = ILI9341
#define RG_SCREEN_HOST              SPI2_HOST
#define RG_SCREEN_SPEED             SPI_MASTER_FREQ_80M
#define RG_SCREEN_TYPE              23
#define RG_SCREEN_WIDTH             240
#define RG_SCREEN_HEIGHT            240
#define RG_SCREEN_ROTATE            0
#define RG_SCREEN_MARGIN_TOP        0
#define RG_SCREEN_MARGIN_BOTTOM     0
#define RG_SCREEN_MARGIN_LEFT       0
#define RG_SCREEN_MARGIN_RIGHT      0

// Input
// TODO, MK:
#define RG_GAMEPAD_DRIVER           1   // 1 = ODROID-GO, 2 = Serial, 3 = I2C, 4 = AW9523, 5 = ESPLAY-S3, 6 = SDL2
#define RG_GAMEPAD_HAS_MENU_BTN     0
#define RG_GAMEPAD_HAS_OPTION_BTN   0
// Note: Depending on the driver, the button map can be a bitmask, an index, or a GPIO.
// Refer to rg_input.h to see all available RG_KEY_*
#define RG_GAMEPAD_MAP {\
    {RG_KEY_SELECT, RG_GPIO_GAMEPAD_SELECT},\
    {RG_KEY_START,  RG_GPIO_GAMEPAD_START},\
    {RG_KEY_A,      RG_GPIO_GAMEPAD_A},\
    {RG_KEY_B,      RG_GPIO_GAMEPAD_B},\
}

// Battery
// TODO, MK:
#define RG_BATTERY_ADC_CHANNEL      ADC1_CHANNEL_0
#define RG_BATTERY_CALC_PERCENT(raw) (((raw) * 2.f - 3500.f) / (4200.f - 3500.f) * 100.f)
#define RG_BATTERY_CALC_VOLTAGE(raw) ((raw) * 2.f * 0.001f)

// Status LED
// TODO, MK:
#define RG_GPIO_LED                 GPIO_NUM_2

// I2C BUS
// #define RG_GPIO_I2C_SDA             GPIO_NUM_15
// #define RG_GPIO_I2C_SCL             GPIO_NUM_4

// Built-in gamepad
// TODO, MK:
#define RG_GPIO_GAMEPAD_X           ADC1_CHANNEL_6
#define RG_GPIO_GAMEPAD_Y           ADC1_CHANNEL_7
#define RG_GPIO_GAMEPAD_SELECT      GPIO_NUM_27
#define RG_GPIO_GAMEPAD_START       GPIO_NUM_39
#define RG_GPIO_GAMEPAD_A           GPIO_NUM_32
#define RG_GPIO_GAMEPAD_B           GPIO_NUM_33
#define RG_GPIO_GAMEPAD_MENU        GPIO_NUM_13
#define RG_GPIO_GAMEPAD_OPTION      GPIO_NUM_0

// SPI Display
#define RG_GPIO_LCD_MISO            GPIO_NUM_NC
#define RG_GPIO_LCD_MOSI            42
#define RG_GPIO_LCD_CLK             41
#define RG_GPIO_LCD_CS              40
#define RG_GPIO_LCD_DC              38
#define RG_GPIO_LCD_BCKL            46

// SPI SD Card
// TODO, MK:
#define RG_GPIO_SDSPI_CMD          GPIO_NUM_48
#define RG_GPIO_SDSPI_CLK          GPIO_NUM_47
#define RG_GPIO_SDSPI_D0           GPIO_NUM_21
// #define RG_GPIO_SDSPI_MISO          0 // GPIO_NUM_19
// #define RG_GPIO_SDSPI_MOSI          0 // GPIO_NUM_23
// #define RG_GPIO_SDSPI_CLK           0 // GPIO_NUM_18
// #define RG_GPIO_SDSPI_CS            0 // GPIO_NUM_22

// External I2S DAC
// TODO, MK:
#define RG_GPIO_SND_I2S_BCK         0 // GPIO_NUM_4
#define RG_GPIO_SND_I2S_WS          0 // GPIO_NUM_12
#define RG_GPIO_SND_I2S_DATA        0 // GPIO_NUM_15
// #define RG_GPIO_SND_AMP_ENABLE      GPIO_NUM_NC
