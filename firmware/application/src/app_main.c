#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"

#include "app_timer.h"
#include "app_usbd.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_rng.h"
#include "nrf_power.h"
#include "nrf_pwr_mgmt.h"
#include "nrfx_nfct.h"
#include "nrfx_power.h"
#include "nrf_drv_lpcomp.h"

#define NRF_LOG_MODULE_NAME app_main
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

#include "app_cmd.h"
#include "ble_main.h"
#include "bsp_delay.h"
#include "bsp_time.h"
#include "dataframe.h"
#include "fds_util.h"
#include "hex_utils.h"
#include "rfid_main.h"
#include "tag_emulation.h"
#include "usb_main.h"
#include "rgb_marquee.h"


// Defining soft timers
APP_TIMER_DEF(m_button_check_timer); // Timer for button debounce
uint8_t m_btn_click_record = 0x00;

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
    // /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void app_timers_init(void) {
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing power management.
 */
static void rng_drv_and_srand_init(void) {
    ret_code_t err_code;
    uint8_t available;
    uint32_t rand_int;

    // First initialize the official rng management driver api
    err_code = nrf_drv_rng_init(NULL);
    APP_ERROR_CHECK(err_code);

    // Wait for the random number generator to generate enough random numbers to put in the queue
    do {
        nrf_drv_rng_bytes_available(&available);
    } while (available < 4);

    // Note that here we are forcing the address of a uint32_t value to be converted to a uint8_t address
    // to get the pointer to the first byte of uint32
    err_code = nrf_drv_rng_rand(((uint8_t *)(&rand_int)), 4);
    APP_ERROR_CHECK(err_code);

    // Finally initialize the srand seeds in the c standard library
    srand(rand_int);
}

/**@brief Initialize GPIO matrix library
 */
static void gpio_te_init(void) {
    // Initialize GPIOTE
    uint32_t err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Button Matrix Events
 */
static void button_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    device_mode_t mode = get_device_mode();
    // Temporarily allow only the analog card mode to respond to button operations
    if (mode == DEVICE_MODE_TAG) {
        static nrf_drv_gpiote_pin_t pin_static;                                  // Use static internal variables to store the GPIO where the current event occurred
        pin_static = pin;                                                        // Cache the button that currently triggers the event into an internal variable
        app_timer_start(m_button_check_timer, APP_TIMER_TICKS(50), &pin_static); // Start timer anti-shake
    }
}

/** @brief Button anti-shake timer
 * @param None
 * @return None
 */
static void timer_button_event_handle(void *arg) {
    nrf_drv_gpiote_pin_t pin = *(nrf_drv_gpiote_pin_t *)arg;
    // Check here if the current GPIO is at the pressed level
    if (nrf_gpio_pin_read(pin) == 1) {
        if (pin == BUTTON_1) {
            NRF_LOG_INFO("BUTTON_LEFT");
            m_btn_click_record |= 0x01;
        }
        if (pin == BUTTON_2) {
            NRF_LOG_INFO("BUTTON_RIGHT");
            m_btn_click_record |= 0x02;
        }
    }
}

/**@brief Function for init button and led.
 */
static void button_init(void) {
    ret_code_t err_code;

    // Non-exact timer for initializing button anti-shake
    err_code = app_timer_create(&m_button_check_timer, APP_TIMER_MODE_SINGLE_SHOT, timer_button_event_handle);
    APP_ERROR_CHECK(err_code);

    // Configure SENSE mode, select false for sense configuration
    nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN; // Pulldown

    // Configure key binding POTR
    err_code = nrf_drv_gpiote_in_init(BUTTON_1, &in_config, button_pin_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(BUTTON_1, true);

    err_code = nrf_drv_gpiote_in_init(BUTTON_2, &in_config, button_pin_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(BUTTON_2, true);
}

/**@brief The implementation function to enter deep hibernation
 */
void system_off_enter(void) {
    ret_code_t ret;

    // Disable the HF NFC event first
    NRF_NFCT->INTENCLR = NRF_NFCT_DISABLE_ALL_INT;
    // Then disable the LF LPCOMP event
    NRF_LPCOMP->INTENCLR = LPCOMP_INTENCLR_CROSS_Msk | LPCOMP_INTENCLR_UP_Msk | LPCOMP_INTENCLR_DOWN_Msk | LPCOMP_INTENCLR_READY_Msk;

    // ram保持需要打开，不然功耗有偏差
    // Configure RAM hibernation hold
    uint32_t ram8_retention = // RAM8 Each section has 32KB capacity
                              // POWER_RAM_POWER_S0RETENTION_On << POWER_RAM_POWER_S0RETENTION_Pos ;
                              // POWER_RAM_POWER_S1RETENTION_On << POWER_RAM_POWER_S1RETENTION_Pos |
                              // POWER_RAM_POWER_S2RETENTION_On << POWER_RAM_POWER_S2RETENTION_Pos |
                              // POWER_RAM_POWER_S3RETENTION_On << POWER_RAM_POWER_S3RETENTION_Pos |
                              // POWER_RAM_POWER_S4RETENTION_On << POWER_RAM_POWER_S4RETENTION_Pos |
        POWER_RAM_POWER_S5RETENTION_On << POWER_RAM_POWER_S5RETENTION_Pos;
    ret = sd_power_ram_power_set(8, ram8_retention);
    APP_ERROR_CHECK(ret);

    rgb_marquee_stop();

    // IOs that need to be configured as floating analog inputs ==> no pull-up or pull-down
    uint32_t gpio_cfg_default_nopull[] = {
#if defined(PROJECT_CHAMELEON_ULTRA)
        HF_SPI_SELECT,
        HF_SPI_MISO,
        HF_SPI_MOSI,
        HF_SPI_MOSI,
        LF_OA_OUT,
#endif
        BAT_SENSE_PIN,
    };
    for (int i = 0; i < ARRAY_SIZE(gpio_cfg_default_nopull); i++) {
        nrf_gpio_cfg_default(gpio_cfg_default_nopull[i]);
    }

    // IO that needs to be configured as a push-pull output and pulled high
    uint32_t gpio_cfg_output_high[] = {
#if defined(PROJECT_CHAMELEON_ULTRA)
        HF_ANT_SEL,
#endif
        LED_FIELD, LED_R, LED_G, LED_B,
    };
    for (int i = 0; i < ARRAY_SIZE(gpio_cfg_output_high); i++) {
        nrf_gpio_cfg_output(gpio_cfg_output_high[i]);
        nrf_gpio_pin_set(gpio_cfg_output_high[i]);
    }

    // IOs that need to be configured as push-pull outputs and pulled low
    uint32_t gpio_cfg_output_low[] = {
        LED_1, LED_2, LED_3, LED_4, LED_5, LED_6, LED_7, LED_8, LF_MOD, 
#if defined(PROJECT_CHAMELEON_ULTRA)
        READER_POWER, LF_ANT_DRIVER
#endif
    };
    for (int i = 0; i < ARRAY_SIZE(gpio_cfg_output_low); i++) {
        nrf_gpio_cfg_output(gpio_cfg_output_low[i]);
        nrf_gpio_pin_clear(gpio_cfg_output_low[i]);
    }

    // Wait for a while before hibernating to avoid GPIO circuit configuration fluctuations to wake up the chip
    bsp_delay_ms(50);

    // Print leaving message finally
    NRF_LOG_INFO("Sleep finally, Bye ^.^");
    // Turn off all soft timers
    app_timer_stop_all();

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    // 注意，如果插着jlink或者开着debug，进入低功耗的函数可能会报错，
    // 开启调试时我们应当禁用低功耗状态值检测，或者干脆不进入低功耗
    ret = sd_power_system_off();

    // OK，此处非常重要，如果开启了日志输出并且使能了RTT，则不去检查低功耗模式的错误
#if !(NRF_LOG_ENABLED && NRF_LOG_BACKEND_RTT_ENABLED)
    APP_ERROR_CHECK(ret);
#else
    UNUSED_VARIABLE(ret);
#endif

    // It is not supposed to enter here, but jlink debug mode it can be entered, at most is not normal hibernation just
    // jlink connection, power consumption will rise, and hibernation will also be stuck in this step.
    while (1)
        NRF_LOG_PROCESS();
}

/**@brief Application main function.
 */
int main(void) {
    hw_connect_init();        // Remember to initialize the pins first
    init_leds();              // LED initialization

    log_init();               // Log initialization
    gpio_te_init();           // Initialize GPIO matrix library
    app_timers_init();        // Initialize soft timer
    fds_util_init();          // Initialize fds tool package
    bsp_timer_init();         // Initialize timeout timer
    bsp_timer_start();        // Start BSP TIMER and prepare it for processing business logic
    button_init();            // Button initialization for handling business logic
    rng_drv_and_srand_init(); // Random number generator initialization
    power_management_init();  // Power management initialization
    usb_cdc_init();           // USB cdc emulation initialization
    ble_slave_init();         // Bluetooth protocol stack initialization
    tag_emulation_init();     // Analog card initialization
    rgb_marquee_init();       // Light effect initialization

    // cmd callback register
    on_data_frame_complete(on_data_frame_received);

    set_slot_light_color(2);
    light_up_by_slot();
    
    tag_mode_enter();         // Enter card simulation mode by default

    // usbd event listener
    APP_ERROR_CHECK(app_usbd_power_events_enable());

    // Enter main loop.
    NRF_LOG_INFO("Chameleon working");
    while (1) {
        // Data pack process
        data_frame_process();
        // Log print process
        while (NRF_LOG_PROCESS());
        // USB event process
        while (app_usbd_event_queue_process());
    }
}
