#!/bin/bash

set -xe

cargo build --release
arm-none-eabi-objcopy -O binary ./target/thumbv7em-none-eabihf/release/openlmr ./target/openlmr.bin
~/dev/radio_tool/radio_tool --wrap -o ./target/openlmr-wrapped.bin -r UV3X0 -s 0x0800C000:./target/openlmr.bin
ls -l target/openlmr.bin
~/dev/radio_tool/radio_tool -f -i ./target/openlmr-wrapped.bin -d 0 || echo "Failed to flash"
