# ASRSCommunication

Reusable Arduino communication library for the ASRS fork project. It provides the binary protocol, CRC frame codec, UART, SoftwareSerial and ESP-NOW transports, master/slave role APIs, and ESP-NOW session supervision.

The complete deployable fork and rack applications are located at the repository root under `firmware/`.

## Arduino installation

Copy this complete directory to:

```text
Documents/Arduino/libraries/ASRSCommunication/
```

Restart Arduino IDE afterward. Generic library examples will appear under **File > Examples > ASRSCommunication**.

Applications can include the complete public API with:

```cpp
#include <ASRSCommunication.h>
```

## PlatformIO

Copy this directory into a project's `lib/ASRSCommunication/` directory, or reference the Git repository and configure the application to use this library subdirectory.

## Protocol compatibility

Version 0.2.0 preserves the existing ASRS packet layout, command values, CRC-16/CCITT framing, and master/slave APIs. It adds a raw ESP-NOW receive hook and the separate rack-status packet definition so the fork application can receive rack reports without changing tower frames.
