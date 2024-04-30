#!/bin/bash

set -xe

socat /dev/ttyACM0,raw,b115200 - | defmt-print -e ./target/thumbv7em-none-eabihf/release/openlmr
