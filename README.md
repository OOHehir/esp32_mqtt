# esp32_mqtt

MQTT client on the Seeed XIAO ESP32 S3 using Zephyr RTOS — connects to WiFi and communicates with an MQTT broker, with proper event handling and logging.

## Key Technologies

- **MCU:** Seeed XIAO ESP32 S3
- **RTOS:** Zephyr v3.6
- **Protocol:** MQTT (Zephyr MQTT client)
- **Build:** CMake + west

## Getting Started

**Prerequisites:** Zephyr SDK, west, Espressif binary blobs

```bash
west blobs fetch hal_espressif
west build -b xiao_esp32s3
west flash
```

Configure WiFi credentials in `credentials.h` before building.

---

Built by Owen O'Hehir — embedded Linux, IoT, Matter & Rust consulting at [electronicsconsult.com](https://electronicsconsult.com). Available for contract and consulting work.
