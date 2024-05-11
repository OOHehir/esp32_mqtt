#!/bin/bash
# Quick script to set env & build for xiao_esp32s3

if [ "$EUID" -eq 0 ]
  then echo "Don't run as root!"
  exit
fi

source ../zephyrproject/.venv/bin/activate
source ../zephyrproject/zephyr/zephyr-env.sh
source ../zephyrproject/zephyr/scripts/west_commands/completion/west-completion.bash

west update
west blobs fetch hal_espressif

# For > Zephyr v3.6
# west build -b xiao_esp32s3/esp32s3/procpu ./ -p always

# For <= Zephyr v3.6
west build -b xiao_esp32s3 ./ -p always

export ESPTOOL_PORT=/dev/ttyACM0
west flash
west espressif monitor -p /dev/ttyACM0