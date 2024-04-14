#!/bin/bash

set -xe

cargo build --release
arm-none-eabi-objcopy -O binary ./target/thumbv7em-none-eabi/release/opendmr ./target/opendmr.bin
~/dev/radio_tool/radio_tool --wrap -o ./target/opendmr-wrapped.bin -r UV3X0 -s 0x0800C000:./target/opendmr.bin
python ~/dev/OpenRTX/scripts/md380_dfu.py upgrade ./target/opendmr-wrapped.bin