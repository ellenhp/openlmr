~/dev/radio_tool/radio_tool --wrap -o opendmr-wrapped.bin -r UV3X0 -s 0x0800C000:opendmr.bin
arm-none-eabi-objcopy -O binary ~/dev/opendmr/target/thumbv7em-none-eabi/release/opendmr opendmr.bin
objdump --syms ~/dev/opendmr/target/thumbv7em-none-eabi/release/opendmr
python ~/dev/OpenRTX/scripts/md380_dfu.py upgrade opendmr-wrapped.bin