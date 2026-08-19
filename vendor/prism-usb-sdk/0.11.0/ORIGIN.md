# Prism USB SDK 0.11.0 binary provenance

These runtime-only packages were built from Prism USB SDK `0.11.0` at source
commit `e4120a1342d9ca937fd707fe36e8463df4b32185`.

The vendored payload is intentionally limited to the SDK's public headers,
platform-specific `libprism_usb_sdk.so` dynamic library, exported CMake package
configuration files, and USB udev rule. It does not contain the Prism Agent or
USB SDK implementation source, and consumers must not compile the SDK as part
of this repository.

## Platform mapping

| Directory | Build environment | ROS distributions |
| --- | --- | --- |
| `ubuntu-20.04-x86_64` | Ubuntu 20.04, GCC 9, OpenSSL 1.1, libusb 1.0.23 | Noetic |
| `ubuntu-22.04-x86_64` | Ubuntu 22.04, GCC 11, OpenSSL 3.0.2, libusb 1.0.25 | Humble |
| `ubuntu-24.04-x86_64` | Ubuntu 24.04, GCC 13, OpenSSL 3.0.13, libusb 1.0.27 | Jazzy, Kilted |
| `ubuntu-26.04-x86_64` | Ubuntu 26.04, GCC 15, OpenSSL 3.5.5, libusb 1.0.29 | Lyrical, Rolling |

Do not use a binary on a different Ubuntu release merely because the CPU
architecture matches. The platform-specific builds carry different glibc and
OpenSSL requirements.

## Dynamic-library SHA-256

| Platform | SHA-256 of `lib/libprism_usb_sdk.so` |
| --- | --- |
| Ubuntu 20.04 x86_64 | `59cc95e65993f9d7eeadbf424e68161af4b1a66f034ce0a9720eaacdc585f7e1` |
| Ubuntu 22.04 x86_64 | `fca7156b0c4827c763d9f375c6ef13577e470e4a5c7bb184fc704bbc5ba716fd` |
| Ubuntu 24.04 x86_64 | `b03d1dc616e5a489f519fd7816805ef985542329019c05b517c01dc0cbdbe9d0` |
| Ubuntu 26.04 x86_64 | `a4d5ca066c8f4ca9997febc3c11008ddf8d0700c8eaffe2c0136fea7e7ec108f` |

`SHA256SUMS` covers every distributed payload file and intentionally excludes
this provenance document and the checksum file itself.
