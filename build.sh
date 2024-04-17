#!/bin/bash

set -xe

cargo build --release
arm-none-eabi-objcopy -O binary ./target/thumbv7em-none-eabi/release/openlmr ./target/openlmr.bin
~/dev/radio_tool/radio_tool --wrap -o ./target/openlmr-wrapped.bin -r UV3X0 -s 0x0800C000:./target/openlmr.bin
ls -l target/openlmr.bin
python ~/dev/OpenRTX/scripts/md380_dfu.py upgrade ./target/openlmr-wrapped.bin