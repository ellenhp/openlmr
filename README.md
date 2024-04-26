# OpenLMR

OpenLMR is an open-source firmware for the TYT MD-UV380 and TYT MD-UV390 DMR handheld transceivers. It currently has drivers to interact with the RF chip and baseband, a working USB logging implementation and is based on [RTIC](https://rtic.rs/) so it has async/await based "threading". Right now there's no user interface aside from displaying the RSSI on 146.520 MHz. Transmitting is disabled pending the arrival of an RF dummy load for testing, but in theory I have the commands copied over from OpenRTX to make that work.

### Roadmap

There are a bunch of things I want to get done, ordered roughly by importance.

- [x] Interact with the rf chip (AT1846S).
- [x] Interact with the baseband (C6000).
- [ ] *Load calibration data from non-volatile storage.
- [x] FM receive.
- [ ] *FM transmit.
- [x] LCD drivers.
- [x] Keypad scanning/user input.
- [ ] *Develop codeplug format/DFU flashing capability.
- [ ] *Implement UI to switch channels.
- [ ] Investigate how to push Codec2 over DMR. DMR does not define a codec in the spec, so we will still be "to spec" here, and avoiding the use of patented binary blobs is great for an open project like this.
- [ ] DMR receive.
- [ ] DMR transmit.
- [ ] DMR encryption (for licensed users ONLY). Do not use this feature on the ham bands. You will be hunted down by Amateur Auxiliary or equivalent and face steep penalties.

Items marked with an asterisk are critical for the first versioned release.

### Acknowledgements

This project would not be possible without the work of many others who have reverse engineered this transceiver before me. The [OpenRTX](https://openrtx.org/) and [OpenGD77](https://www.opengd77.com/) projects have been tremendously helpful, and OpenLMR owes its quick progress to the authors and contributors who have worked on those two projects.

Special thanks to everyone in the OpenRTX Matrix space who has answered my questions about this hardware.