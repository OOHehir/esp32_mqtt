# esp32Touch
Simple 'out of tree' project on the [Seeed XIAO ESP32 S3](https://docs.zephyrproject.org/latest/boards/seeed/xiao_esp32s3/doc/index.html) using the Zephyr RTOS. It should be straightforward to adapt to other ESP32S3 boards.


This assumes you've already installed & configured Zephyr.

## Working version
Currently need to be
```
    git checkout v3.6-branch
```

## Initialize environment:
As normal, source the environment:
```
    source ../zephyrproject/.venv/bin/activate
    source ../zephyrproject/zephyr/zephyr-env.sh
    source ../zephyrproject/zephyr/scripts/west_commands/completion/west-completion.bash
```
## Binary blobs
Espressif HAL requires binary blobs for peripherals to work. Run the command below to retrieve those files.
```
    west blobs fetch hal_espressif
```
## Build:
Build application & bootloader
```
    west build -p always -b xiao_esp32s3/esp32s3/procpu
```
## Flash
```
    west flash
```

## Modify config
 west build -t guiconfig --board xiao_esp32s3/esp32s3/procpu

 ## Build with twister
 west twister -b xiao_esp32s3/esp32s3/procpu -t build /zephyr/samples/hello_world

## Pinout:

![ESP32S3](images/esp32s3_pinout.jpeg)