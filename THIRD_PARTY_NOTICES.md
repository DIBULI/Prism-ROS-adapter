# Third-party notices

The adapter's Apache-2.0 license applies only to its project-owned source,
messages, scripts and documentation, not to separately supplied dependencies.

## Prism SDK

The [Prism SDK](https://github.com/DIBULI/Prism-SDK) includes public interface
materials and closed-source core libraries under separate licenses.
The SDK remains proprietary when linked into an adapter executable; it is not
covered by the `Apache-2.0` tag in an adapter's `package.xml`.
Retain its [binary license](licenses/Prism-SDK-Binary-LICENSE.txt) and notices
in distributions that contain SDK code.

SDK variants use libusb (LGPL-2.1-or-later), OpenSSL 1.1.1 under its
OpenSSL/SSLeay terms, and, where applicable, miniz under its supplied license.
See the SDK's own `THIRD_PARTY_NOTICES.md` and supplied license files for the
corresponding versions and source references.

This product includes software developed by the OpenSSL Project for use in the
OpenSSL Toolkit (https://www.openssl.org/). This product includes cryptographic
software written by Eric Young (eay@cryptsoft.com).

## ROS and system dependencies

ROS, catkin, ament, generated runtime support and system libraries keep their
respective licenses. They are not relicensed by this repository. Follow the
notices of the exact versions you distribute and any applicable source/relinking
obligations. This notice is not a complete audit of every transitive dependency.
