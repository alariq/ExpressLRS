#!/bin/bash


#~/.platformio/penv/bin/pio run --environment Unified_ESP32_900_RX_via_UART < /dev/null
#~/.platformio/penv/bin/pio clean

targets=(
Unified_ESP32_900_RX
Unified_ESP32_900_TX
Unified_ESP32C3_900_RX
Unified_ESP32C3_LR1121_RX
Unified_ESP32_LR1121_RX
Unified_ESP32_LR1121_TX
Unified_ESP32S3_900_RX
Unified_ESP32S3_900_TX
Unified_ESP32S3_LR1121_RX
Unified_ESP32S3_LR1121_TX
Unified_ESP8285_900_RX
Unified_ESP8285_900_TX
Unified_ESP8285_LR1121_RX
)

if [ -d ./firmwares ]; then
    echo Deleting existing firmwares
    rm -rf ./firmwares
    mkdir firmwares
fi

for str in ${targets[@]}; do
    echo $str
    mkdir -p ./firmwares/$str
    build_env="${str}_via_UART"
    echo $build_env
    # < /dev/null to remove tty, so build will not ask for a configuration
    ~/.platformio/penv/bin/pio run -s --environment $build_env < /dev/null
    cp ./.pio/build/$build_env/firmware.bin ./firmwares/$str/
done

