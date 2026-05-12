#include "mxc.h"
#include "led.h"
#include "pmon_gpio.h"

#define PMON_SYS_PORT MXC_GPIO1
#define PMON_SYS_PIN  MXC_GPIO_PIN_6
#define PMON_CNN_PORT MXC_GPIO1
#define PMON_CNN_PIN  MXC_GPIO_PIN_7

static int g_cnn_hold_high = 0;

static void pmon_config_pin(mxc_gpio_regs_t *port, uint32_t mask)
{
    mxc_gpio_cfg_t gpio_out = {0};

    gpio_out.port = port;
    gpio_out.mask = mask;
    gpio_out.pad = MXC_GPIO_PAD_NONE;
    gpio_out.func = MXC_GPIO_FUNC_OUT;
    MXC_GPIO_Config(&gpio_out);
}

void pmon_trig_init(void)
{
    g_cnn_hold_high = 0;
    pmon_config_pin(PMON_SYS_PORT, PMON_SYS_PIN);
    pmon_config_pin(PMON_CNN_PORT, PMON_CNN_PIN);

    MXC_GPIO_OutClr(PMON_SYS_PORT, PMON_SYS_PIN);
    MXC_GPIO_OutClr(PMON_CNN_PORT, PMON_CNN_PIN);

    LED_Off(LED1);
    LED_Off(LED2);
}

void pmon_sys_start(void)
{
    MXC_GPIO_OutSet(PMON_SYS_PORT, PMON_SYS_PIN);
    LED_On(LED1);
}

void pmon_sys_complete(void)
{
    MXC_GPIO_OutClr(PMON_SYS_PORT, PMON_SYS_PIN);
    LED_Off(LED1);
}

void pmon_cnn_start(void)
{
    MXC_GPIO_OutSet(PMON_CNN_PORT, PMON_CNN_PIN);
    LED_On(LED2);
}

void pmon_cnn_complete(void)
{
    if (g_cnn_hold_high) {
        return;
    }

    MXC_GPIO_OutClr(PMON_CNN_PORT, PMON_CNN_PIN);
    LED_Off(LED2);
}

void pmon_cnn_hold_begin(void)
{
    g_cnn_hold_high = 1;
    pmon_cnn_start();
}

void pmon_cnn_hold_end(void)
{
    g_cnn_hold_high = 0;
    MXC_GPIO_OutClr(PMON_CNN_PORT, PMON_CNN_PIN);
    LED_Off(LED2);
}
