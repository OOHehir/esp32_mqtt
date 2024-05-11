/*
 * Copyright (c) 2024 Owen O'Hehir
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

#define SLEEP_TIME_MS   1000

#define LED0_NODE DT_ALIAS(led0)
#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

// #ifndef CONFIG_SOC_SERIES_ESP32S3
// #error "Unsupported SoC series"
// #endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void) {
    k_msleep(SLEEP_TIME_MS);

    printf("Startup of board: %s\n", CONFIG_BOARD);

    if (!gpio_is_ready_dt(&led)) {
        printf("GPIO not ready to for LED at %s pin %d\n", led.port->name, led.pin);
        return 0;
    }

    int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printf("Failed to configure LED at %s pin %d\n", led.port->name, led.pin);
        return 0;
    }

    gpio_pin_set_dt(&led, 0);

    while (1) {
        k_msleep(SLEEP_TIME_MS);
        printf(".");
    }
    return 0;
}
