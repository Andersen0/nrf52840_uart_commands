#include <sample_usbd.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

static bool blink_enabled = false;
static int blink_rate_ms = 0; // 0 is STEADY ON
static int blink_led_index = -1; // -1 = no LED blinking

static struct k_thread blink_thread_data;
static K_THREAD_STACK_DEFINE(blink_stack, 512);

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)
static const struct gpio_dt_spec leds[] = {
#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
    GPIO_DT_SPEC_GET(LED0_NODE, gpios),
#else
    {0},
#endif
#if DT_NODE_HAS_STATUS(LED1_NODE, okay)
    GPIO_DT_SPEC_GET(LED1_NODE, gpios),
#else
    {0},
#endif
#if DT_NODE_HAS_STATUS(LED2_NODE, okay)
    GPIO_DT_SPEC_GET(LED2_NODE, gpios),
#else
    {0},
#endif
#if DT_NODE_HAS_STATUS(LED3_NODE, okay)
    GPIO_DT_SPEC_GET(LED3_NODE, gpios),
#else
    {0},
#endif
};
#define NUM_LEDS 4

struct led_blink {
    bool enabled;
    int rate_ms;
    int64_t last_toggle;  // timestamp of last toggle
    bool state;
};

struct led_blink led_blinks[NUM_LEDS];

LOG_MODULE_REGISTER(cdc_acm_echo, LOG_LEVEL_INF);

typedef void (*command_handler_t)(const char *args);

struct command_entry {
    const char *name;
    command_handler_t handler;
};

const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define RING_BUF_SIZE 1024
uint8_t ring_buffer[RING_BUF_SIZE];
struct ring_buf ringbuf;
static bool rx_throttled;

void process_command(const char *cmd);
void blink_thread(void *arg1, void *arg2, void *arg3);

static void cmd_blink(const char *args) {
    int led_num;
    int rate;

    if (sscanf(args, "%d %d", &led_num, &rate) != 2) {
        uart_fifo_fill(uart_dev, (uint8_t *)"ERROR: usage BLINK <1-4> <ms>\n", 31);
        return;
    }

    if (led_num < 1 || led_num > 4 || rate < 0) {
        uart_fifo_fill(uart_dev, (uint8_t *)"ERROR: invalid arguments\n", 25);
        return;
    }

    int index = led_num - 1;

    if (rate == 0) {
		led_blinks[index].enabled = false;
		led_blinks[index].state = false;
		gpio_pin_set_dt(&leds[index], 0);
	} else {
		led_blinks[index].enabled = true;
		led_blinks[index].rate_ms = rate;
	}

    blink_led_index = index;
    blink_rate_ms = rate;
    blink_enabled = true;

    char buf[32];
    int len = snprintf(buf, sizeof(buf), "Blinking LED %d at %d ms\n", led_num, rate);
    uart_fifo_fill(uart_dev, (uint8_t *)buf, len);
}


static void cmd_led(const char *args) {
    int led_num;
    char state[8];

    if (sscanf(args, "%d %7s", &led_num, state) == 2) {
        if (led_num < 1 || led_num > 4) {
            uart_fifo_fill(uart_dev, (uint8_t *)"ERROR: invalid LED\n", 20);
            return;
        }

		int index = led_num - 1;

		if (blink_led_index == index) {
			blink_enabled = false;
			blink_led_index = -1;
		}

        const struct gpio_dt_spec *led_spec = &leds[led_num - 1];
        if (!led_spec->port) {
            uart_fifo_fill(uart_dev, (uint8_t *)"ERROR: LED not available\n", 26);
            return;
        }

        if (strcasecmp(state, "ON") == 0) {
            gpio_pin_set_dt(led_spec, 1);
            char buf[32];
            int len = snprintf(buf, sizeof(buf), "LED %d ON\n", led_num);
            uart_fifo_fill(uart_dev, (uint8_t *)buf, len);
        } else if (strcasecmp(state, "OFF") == 0) {
            gpio_pin_set_dt(led_spec, 0);
            char buf[32];
            int len = snprintf(buf, sizeof(buf), "LED %d OFF\n", led_num);
            uart_fifo_fill(uart_dev, (uint8_t *)buf, len);
        } else {
            uart_fifo_fill(uart_dev, (uint8_t *)"ERROR: invalid state\n", 22);
        }
    } else {
        uart_fifo_fill(
            uart_dev,
            (uint8_t *)"ERROR: usage LED <1-4> <ON/OFF>\n",
            34
        );
    }
}



static const struct command_entry command_table[] = {
    { "BLINK", cmd_blink },
    { "LED",   cmd_led }
};

static void interrupt_handler(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (!rx_throttled && uart_irq_rx_ready(dev)) {
            int recv_len, rb_len;
            uint8_t buffer[64];
            size_t len = MIN(ring_buf_space_get(&ringbuf), sizeof(buffer));

            if (len == 0) {
                /* Throttle because ring buffer is full */
                uart_irq_rx_disable(dev);
                rx_throttled = true;
                continue;
            }

            recv_len = uart_fifo_read(dev, buffer, len);
            if (recv_len < 0) {
                LOG_ERR("Failed to read UART FIFO");
                recv_len = 0;
            };

            rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
            if (rb_len < recv_len) {
                LOG_ERR("Drop %u bytes", recv_len - rb_len);
            }

            static char cmd_buffer[64];
            static int cmd_pos = 0;

            rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
            for (int i = 0; i < rb_len; i++) {
                if (buffer[i] == '\n' || buffer[i] == '\r') {
                    cmd_buffer[cmd_pos] = '\0';
                    if (cmd_pos > 0) {
                        process_command(cmd_buffer);
                    }
                    cmd_pos = 0;
                } else if (cmd_pos < sizeof(cmd_buffer) - 1) {
                    cmd_buffer[cmd_pos++] = buffer[i];
                }
            }

            LOG_DBG("tty fifo -> ringbuf %d bytes", rb_len);
            if (rb_len) {
                uart_irq_tx_enable(dev);
            }
        }

        if (uart_irq_tx_ready(dev)) {
            uint8_t buffer[64];
            int rb_len, send_len;

            rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
            if (!rb_len) {
                LOG_DBG("Ring buffer empty, disable TX IRQ");
                uart_irq_tx_disable(dev);
                continue;
            }

            if (rx_throttled) {
                uart_irq_rx_enable(dev);
                rx_throttled = false;
            }

            send_len = uart_fifo_fill(dev, buffer, rb_len);
            if (send_len < rb_len) {
                LOG_ERR("Drop %d bytes", rb_len - send_len);
            }

            LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
        }
    }
}

int main(void) {
    int ret;

    for (int i = 0; i < 4; i++) {
	if (leds[i].port) {
		gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
	}
	}
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    k_thread_create(
        &blink_thread_data,
        blink_stack,
        K_THREAD_STACK_SIZEOF(blink_stack),
        blink_thread,
        NULL, NULL, NULL,
        5, 0, K_NO_WAIT
    );

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("CDC ACM device not ready");
        return 0;
    }

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
    ret = enable_usb_device_next();
#else
    ret = usb_enable(NULL);
#endif
    if (ret != 0) {
        LOG_ERR("Failed to enable USB");
        return 0;
    }

    ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);

    LOG_INF("Wait for DTR");

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
    k_sem_take(&dtr_sem, K_FOREVER);
#else
    while (true) {
        uint32_t dtr = 0U;
        uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
        if (dtr) {
            break;
        } else {
            /* Give CPU resources to low priority threads. */
            k_sleep(K_MSEC(100));
        }
    }
#endif

    LOG_INF("DTR set");

    /* They are optional, we use them to test the interrupt endpoint */
    ret = uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_DCD, 1);
    if (ret) {
        LOG_WRN("Failed to set DCD, ret code %d", ret);
    }

    ret = uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_DSR, 1);
    if (ret) {
        LOG_WRN("Failed to set DSR, ret code %d", ret);
    }

    /* Wait 100ms for the host to do all settings */
    k_msleep(100);

#ifndef CONFIG_USB_DEVICE_STACK_NEXT
#endif

    uart_irq_callback_set(uart_dev, interrupt_handler);
    /* Enable rx interrupts */
    uart_irq_rx_enable(uart_dev);

    return 0;
}

void blink_thread(void *arg1, void *arg2, void *arg3) {
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (1) {
		int64_t now = k_uptime_get();
		for (int i = 0; i < NUM_LEDS; i++) {
			if (led_blinks[i].enabled && (now - led_blinks[i].last_toggle >= led_blinks[i].rate_ms)) {
				led_blinks[i].state = !led_blinks[i].state;
				gpio_pin_set_dt(&leds[i], led_blinks[i].state);
				led_blinks[i].last_toggle = now;
			}
		}
		k_msleep(1);  // tick resolution
	}
}


void process_command(const char *cmd) {
    char command[16];
    const char *args = "";

    // Split first token (command) from args
    if (sscanf(cmd, "%15s", command) == 1) {
        args = cmd + strlen(command);
        while (*args == ' ') args++; // skip spaces

        for (size_t i = 0; i < ARRAY_SIZE(command_table); i++) {
            if (strcasecmp(command, command_table[i].name) == 0) {
                command_table[i].handler(args);
                return;
            }
        }
    }

    uart_fifo_fill(uart_dev, (uint8_t *)"ERROR: unknown command\n", 23);
}
