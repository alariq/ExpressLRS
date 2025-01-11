#!/bin/bash


#~/.platformio/penv/bin/pio run --environment Unified_ESP32_900_RX_via_UART < /dev/null
#~/.platformio/penv/bin/pio clean

branch_name=$(git symbolic-ref -q HEAD)
branch_name=${branch_name##refs/heads/}
branch_name=${branch_name:-HEAD}
echo Git branch name: ${branch_name}

region="FCC"

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

out_base_dir=./firmwares/$branch_name/$region

if [ -d $out_base_dir ]; then
    echo Deleting existing $branch_name firmwares
    rm -rf $out_base_dir
fi

for str in ${targets[@]}; do
    echo Building $str...
    out_dir=$out_base_dir/$str
    mkdir -p $out_dir
    build_env="${str}_via_UART"
    # < /dev/null to remove tty, so build will not ask for a configuration
    ~/.platformio/penv/bin/pio run -s --environment $build_env < /dev/null
    cp ./.pio/build/$build_env/firmware.bin $out_dir

    if [ -f  ./.pio/build/$build_env/boot_app0.bin ]; then
        cp ./.pio/build/$build_env/boot_app0.bin $out_dir
    fi

    if [ -f  ./.pio/build/$build_env/bootloader.bin ]; then
        cp ./.pio/build/$build_env/bootloader.bin $out_dir
    fi

    if [ -f  ./.pio/build/$build_env/partitions.bin ]; then
        cp ./.pio/build/$build_env/partitions.bin $out_dir
    fi

done

