#include "usb_main.h"
#include "dataframe.h"

#include "app_usbd.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_core.h"
#include "app_usbd_serial_num.h"
#include "app_usbd_string_desc.h"

#define NRF_LOG_MODULE_NAME usb_cdc
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

// USB DEFINES START
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst, app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE 0
#define CDC_ACM_COMM_EPIN NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE 1
#define CDC_ACM_DATA_EPIN NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT NRF_DRV_USBD_EPOUT1

/** @brief CDC_ACM class instance */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

// USB DEFINES END

// USB CODE START
volatile bool g_usb_connected = false;
volatile bool g_usb_port_opened = false;
volatile bool g_usb_led_marquee_enable = true;

/** @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst, app_usbd_cdc_acm_user_event_t event) {
    static uint8_t cdc_data_buffer[1];
    // app_usbd_cdc_acm_t const *p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event) {
    case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN: {
        /*
         * 整个USB接收数据的大概关键之处就是 app_usbd_cdc_acm_read
         * app_usbd_cdc_acm_read函数其实不是正经的接收，是给了一个指针，然后等USB的buffer填充到此处
         * 所以需要在 APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN 时先初始化设置头部指针，达到预先设置接收缓冲区的效果
         * 如果在 APP_USBD_CDC_ACM_USER_EVT_RX_DONE 使用下标 0 去访问缓冲区，将会导致丢失第一个发送过来的字节。。
         */
        ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, cdc_data_buffer, 1);
        UNUSED_VARIABLE(ret);
        NRF_LOG_INFO("CDC ACM port opened");
        g_usb_port_opened = true;
        break;
    }

    case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
        NRF_LOG_INFO("CDC ACM port closed");
        g_usb_port_opened = false;
        g_usb_led_marquee_enable = true;
        break;

    case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
        break;

    case APP_USBD_CDC_ACM_USER_EVT_RX_DONE: {
        ret_code_t ret;
        // 先取出第一个字节
        data_frame_receive(cdc_data_buffer, 1);
        do {
            ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, cdc_data_buffer, 1);
            if (ret == NRF_SUCCESS) {
                // 成功取到之后的字节
                data_frame_receive(cdc_data_buffer, 1);
            }
        } while (ret == NRF_SUCCESS);
        break;
    }
    default:
        break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event) {
    switch (event) {
    case APP_USBD_EVT_DRV_SUSPEND:
        NRF_LOG_INFO("USB SUSPEND");
        break;

    case APP_USBD_EVT_DRV_RESUME:
        NRF_LOG_INFO("USB RESUME");
        break;

    case APP_USBD_EVT_STARTED:
        NRF_LOG_INFO("USB STARTED");
        break;

    case APP_USBD_EVT_STOPPED:
        NRF_LOG_INFO("USB STOPPED");
        app_usbd_disable();
        break;

    case APP_USBD_EVT_POWER_DETECTED:
        NRF_LOG_INFO("USB power detected");
        if (!nrf_drv_usbd_is_enabled()) {
            app_usbd_enable();
        }
        g_usb_led_marquee_enable = true;
        break;

    case APP_USBD_EVT_POWER_REMOVED:
        NRF_LOG_INFO("USB power removed");
        g_usb_connected = false;
        g_usb_led_marquee_enable = false;
        app_usbd_stop();
        break;

    case APP_USBD_EVT_POWER_READY:
        NRF_LOG_INFO("USB ready");
        g_usb_connected = true;
        app_usbd_start();
        break;

    default:
        // NRF_LOG_INFO("Other usb event: %d", event);
        break;
    }
}

// USB CODE END

void usb_cdc_init(void) {
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler};

    app_usbd_serial_num_generate();

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const *class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);
}

void usb_cdc_write(const void *p_buf, uint16_t length) {
    ret_code_t err_code = app_usbd_cdc_acm_write(&m_app_cdc_acm, p_buf, length);
    APP_ERROR_CHECK(err_code);
}

// override fputc to printf to cdc serial
/* dont't enable
int fputc(int ch, FILE *f){
    static int ch_static;
    ch_static = ch;

    // must cdc is available
    if (g_usb_port_opened && g_usb_connected) {
        // send and wait done.
        ret_code_t ret;
        do {
            ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, &ch_static, 1);
        } while(ret == NRF_ERROR_BUSY);

        // error log
        if (ret != NRF_SUCCESS) {
            NRF_LOG_ERROR("CDC ACM Unavailable, fputc: %c, return code: %d", ch_static, ret);
        }
    }

    return ch;
}
*/

bool is_usb_working(void) {
    return g_usb_port_opened;
}

void usb_cdc_uninit(void) {
    app_usbd_disable();
    app_usbd_stop();
    while (nrf_drv_usbd_is_enabled());
    app_usbd_uninit();
}