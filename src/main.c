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
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <errno.h>
#include "wifi_credentials.h"

// Ensure these are correctly set for your network
#define SSID WIFI_SSID
#define PSK WIFI_PSK

#define SLEEP_TIME_MS 1000

#define LED0_NODE DT_ALIAS(led0)
#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

// #ifndef CONFIG_SOC_SERIES_ESP32S3
// #error "Unsupported SoC series"
// #endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static struct pollfd fds[1];
static int nfds;

static bool connected;

/* Buffers for MQTT client. */
static uint8_t rx_buffer[256];
static uint8_t tx_buffer[256];

/* MQTT client context */
static struct mqtt_client client_ctx;

/* MQTT Broker address information. */
static struct sockaddr_storage broker;

static void clear_fds(void)
{
  nfds = 0;
}

void mqtt_evt_handler(struct mqtt_client *client, const struct mqtt_evt *evt)
{
  printf("MQTT event received: %d\n", evt->type);
  int err;

  switch (evt->type)
  {
  case MQTT_EVT_CONNACK:
    if (evt->result != 0)
    {
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
    if (evt->result != 0)
    {
      LOG_ERR("MQTT PUBACK error %d", evt->result);
      break;
    }

    LOG_INF("PUBACK packet id: %u", evt->param.puback.message_id);

    break;

  case MQTT_EVT_PUBREC:
    if (evt->result != 0)
    {
      LOG_ERR("MQTT PUBREC error %d", evt->result);
      break;
    }

    LOG_INF("PUBREC packet id: %u", evt->param.pubrec.message_id);

    const struct mqtt_pubrel_param rel_param = {
        .message_id = evt->param.pubrec.message_id};

    err = mqtt_publish_qos2_release(client, &rel_param);
    if (err != 0)
    {
      LOG_ERR("Failed to send MQTT PUBREL: %d", err);
    }

    break;

  case MQTT_EVT_PUBCOMP:
    if (evt->result != 0)
    {
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

static K_SEM_DEFINE(wifi_connected, 0, 1);
static K_SEM_DEFINE(ipv4_address_obtained, 0, 1);

static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;

static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
{
  const struct wifi_status *status = (const struct wifi_status *)cb->info;

  if (status->status)
  {
    printk("Connection request failed (%d)\n", status->status);
  }
  else
  {
    printk("Connected\n");
    k_sem_give(&wifi_connected);
  }
}

static void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb)
{
  const struct wifi_status *status = (const struct wifi_status *)cb->info;

  if (status->status)
  {
    printk("Disconnection request (%d)\n", status->status);
  }
  else
  {
    printk("Disconnected\n");
    k_sem_take(&wifi_connected, K_NO_WAIT);
  }
}

static void handle_ipv4_result(struct net_if *iface)
{
  int i = 0;

  for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++)
  {

    char buf[NET_IPV4_ADDR_LEN];

    if (iface->config.ip.ipv4->unicast[i] != NET_ADDR_DHCP)
    {
      continue;
    }

    printk("IPv4 address: %s\n",
           net_addr_ntop(AF_INET,
                         &iface->config.ip.ipv4->unicast[i].address.in_addr,
                         buf, sizeof(buf)));
    printk("Subnet: %s\n",
           net_addr_ntop(AF_INET,
                         &iface->config.ip.ipv4->netmask,
                         buf, sizeof(buf)));
    printk("Router: %s\n",
           net_addr_ntop(AF_INET,
                         &iface->config.ip.ipv4->gw,
                         buf, sizeof(buf)));
  }

  k_sem_give(&ipv4_address_obtained);
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface)
{
  switch (mgmt_event)
  {

  case NET_EVENT_WIFI_CONNECT_RESULT:
    handle_wifi_connect_result(cb);
    break;

  case NET_EVENT_WIFI_DISCONNECT_RESULT:
    handle_wifi_disconnect_result(cb);
    break;

  case NET_EVENT_IPV4_ADDR_ADD:
    handle_ipv4_result(iface);
    break;

  default:
    break;
  }
}

void wifi_connect(void)
{
  struct net_if *iface = net_if_get_default();

  struct wifi_connect_req_params wifi_params = {0};

  wifi_params.ssid = SSID;
  wifi_params.psk = PSK;
  wifi_params.ssid_length = strlen(SSID);
  wifi_params.psk_length = strlen(PSK);
  wifi_params.channel = WIFI_CHANNEL_ANY;
  wifi_params.security = WIFI_SECURITY_TYPE_PSK;
  wifi_params.band = WIFI_FREQ_BAND_2_4_GHZ;
  wifi_params.mfp = WIFI_MFP_OPTIONAL;

  printk("Connecting to SSID: %s\n", wifi_params.ssid);

  if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &wifi_params, sizeof(struct wifi_connect_req_params)))
  {
    printk("WiFi Connection Request Failed\n");
  }
}

void wifi_status(void)
{
  struct net_if *iface = net_if_get_default();

  struct wifi_iface_status status = {0};

  if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status, sizeof(struct wifi_iface_status)))
  {
    printk("WiFi Status Request Failed\n");
  }

  printk("\n");

  if (status.state >= WIFI_STATE_ASSOCIATED)
  {
    printk("SSID: %-32s\n", status.ssid);
    printk("Band: %s\n", wifi_band_txt(status.band));
    printk("Channel: %d\n", status.channel);
    printk("Security: %s\n", wifi_security_txt(status.security));
    printk("RSSI: %d\n", status.rssi);
  }
}

void wifi_disconnect(void)
{
  struct net_if *iface = net_if_get_default();

  if (net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0))
  {
    printk("WiFi Disconnection Request Failed\n");
  }
}

int main(void)
{
  k_msleep(SLEEP_TIME_MS);

  printf("Startup of board: %s\n", CONFIG_BOARD);

  if (!gpio_is_ready_dt(&led))
  {
    printf("GPIO not ready to for LED at %s pin %d\n", led.port->name, led.pin);
    return 0;
  }

  int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
  if (ret < 0)
  {
    printf("Failed to configure LED at %s pin %d\n", led.port->name, led.pin);
    return 0;
  }

  gpio_pin_set_dt(&led, 0);

  int sock;

  net_mgmt_init_event_callback(&wifi_cb, wifi_mgmt_event_handler,
                               NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);

  net_mgmt_init_event_callback(&ipv4_cb, wifi_mgmt_event_handler, NET_EVENT_IPV4_ADDR_ADD);

  net_mgmt_add_event_callback(&wifi_cb);
  net_mgmt_add_event_callback(&ipv4_cb);

  wifi_connect();
  k_sem_take(&wifi_connected, K_FOREVER);
  wifi_status();
  k_sem_take(&ipv4_address_obtained, K_FOREVER);

  printk("Looking up IP addresses:\n");
  struct zsock_addrinfo *res;
  nslookup("google.com", &res);
  print_addrinfo_results(&res);

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

  int rc = mqtt_connect(&client_ctx);
  if (rc != 0)
  {
    return rc;
  }

  fds[0].fd = client_ctx.transport.tcp.sock;
  fds[0].events = ZSOCK_POLLIN;
  poll(fds, 1, 5000);

  mqtt_input(&client_ctx);

  if (!connected)
  {
    mqtt_abort(&client_ctx);
  }

  while (1)
  {
    k_msleep(SLEEP_TIME_MS);
    printf(".");
  }
  return 0;
}
