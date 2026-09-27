# MeowRobotics USB2FDCAN SDK

The `meow_usb` backend loads the vendor library at runtime with `dlopen`. The x86_64 Linux library shipped in this directory comes from the public Apache-2.0 example repository `SOULDE-Studio/USB2FDCAN-Demo-Zsibot` and exposes `openUSBCAN`, `configUSBCAN`, `sendUSBCAN`, and `readUSBCAN`.

Bundled file SHA-256: `7cff013bf791bfd18a60fe94152050d02d4c014d9bc63466df1054fbb32c1e71`.

For ARM64, replace `x86_64/libusb_fdcan.so` with the ARM64 build supplied by the board seller and pass its absolute path through `library_path`.
