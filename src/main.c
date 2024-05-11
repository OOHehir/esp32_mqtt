/*
 * Copyright (c) 2024 Owen O'Hehir
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mqtt_sample, LOG_LEVEL_DBG);

#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
// #include <zephyr/input/input.h>
// #include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>

#define SLEEP_TIME_MS 1000

#define LED0_NODE DT_ALIAS(led0)
#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

// #ifndef CONFIG_SOC_SERIES_ESP32S3
// #error "Unsupported SoC series"
// #endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* Buffers for MQTT client. */
static uint8_t rx_buffer[256];
static uint8_t tx_buffer[256];

/* MQTT client context */
static struct mqtt_client client_ctx;

/* MQTT Broker address information. */
static struct sockaddr_storage broker;

void mqtt_evt_handler(struct mqtt_client *client, const struct mqtt_evt *evt) {
  printf("MQTT event received: %d\n", evt->type);

  void mqtt_evt_handler(struct mqtt_client *const client,
                        const struct mqtt_evt *evt) {
    int err;

    switch (evt->type) {
    case MQTT_EVT_CONNACK:
      if (evt->result != 0) {
        LOG_ERR("MQTT connect failed %d", evt->result);
        break;
      }

      connected = true;
      LOG_INF("MQTT client connected!");

      break;

    case MQTT_EVT_DISCONNECT:
      LOG_INF("MQTT client disconnected %d", evt->result);

      connected = false;
      clear_fds();

      break;

    case MQTT_EVT_PUBACK:
      if (evt->result != 0) {
        LOG_ERR("MQTT PUBACK error %d", evt->result);
        break;
      }

      LOG_INF("PUBACK packet id: %u", evt->param.puback.message_id);

      break;

    case MQTT_EVT_PUBREC:
      if (evt->result != 0) {
        LOG_ERR("MQTT PUBREC error %d", evt->result);
        break;
      }

      LOG_INF("PUBREC packet id: %u", evt->param.pubrec.message_id);

      const struct mqtt_pubrel_param rel_param = {
          .message_id = evt->param.pubrec.message_id};

      err = mqtt_publish_qos2_release(client, &rel_param);
      if (err != 0) {
        LOG_ERR("Failed to send MQTT PUBREL: %d", err);
      }

      break;

    case MQTT_EVT_PUBCOMP:
      if (evt->result != 0) {
        LOG_ERR("MQTT PUBCOMP error %d", evt->result);
        break;
      }

      LOG_INF("PUBCOMP packet id: %u", evt->param.pubcomp.message_id);

      break;

    case MQTT_EVT_PINGRESP:
      LOG_INF("PINGRESP packet");
      break;

    default:
      break;
    }
  }
}

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

  mqtt_client_init(&client_ctx);

  /* MQTT client configuration */
  client_ctx.broker = &broker;
  client_ctx.evt_cb = mqtt_evt_handler;
  client_ctx.client_id.utf8 = (uint8_t *)"zephyr_mqtt_client";
  client_ctx.client_id.size = sizeof("zephyr_mqtt_client") - 1;
  client_ctx.password = NULL;
  client_ctx.user_name = NULL;
  client_ctx.protocol_version = MQTT_VERSION_3_1_1;
  client_ctx.transport.type = MQTT_TRANSPORT_NON_SECURE;

  /* MQTT buffers configuration */
  client_ctx.rx_buf = rx_buffer;
  client_ctx.rx_buf_size = sizeof(rx_buffer);
  client_ctx.tx_buf = tx_buffer;
  client_ctx.tx_buf_size = sizeof(tx_buffer);

  auto rc = mqtt_connect(&client_ctx);
  if (rc != 0) {
    return rc;
  }

  fds[0].fd = client_ctx.transport.tcp.sock;
  fds[0].events = ZSOCK_POLLIN;
  poll(fds, 1, 5000);

  mqtt_input(&client_ctx);

  if (!connected) {
    mqtt_abort(&client_ctx);
  }

  while (1) {
    k_msleep(SLEEP_TIME_MS);
    printf(".");
  }
  return 0;
}
