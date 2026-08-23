# ASRSCommunication

ASRSCommunication is an Arduino-compatible C++ library for the Automated Storage and Retrieval System (ASRS) communication protocol. The library contains the shared packet definitions, binary frame codec, UART transport, ESP-NOW transport, and master/slave role logic used by the ASRS controller firmware.

The current release is version `0.1.0`. This version number indicates that the library is usable for staged testing, but the public API and protocol may still evolve during research development.

## Library Layers

| Layer | Files | Responsibility |
| --- | --- | --- |
| Protocol | `ASRS_Protocol.h` | Command codes, status codes, error codes, packet structures, coordinate structures, limit switch structures, travel commands, and byte packing helpers. |
| Frame codec | `ASRS_FrameCodec.h`, `ASRS_FrameCodec.cpp` | Binary frame encoding, synchronization fields, protocol versioning, payload length validation, and CRC-16/CCITT verification. |
| Transport | `ASRS_Comm_UART.*`, `ASRS_Comm_ESPNow.*` | Physical communication over UART or ESP-NOW using the same packet and frame format. |
| Role logic | `ASRS_Master.*`, `ASRS_Slave.*` | Master and slave behaviours independent from the physical transport method. |
| Optional sensor support | `VL53L1X_Manager.*` | Helper for staged tower-board tests using SparkFun VL53L1X sensors. The communication library can be used without this dependency. |
| Optional motion support | `ASRS_Slave.*` with `ASRSMotion` present | Enables direct stepper and limit-switch motion control for the real tower-board slave. Communication-only master and dummy-slave examples compile without the companion motion library. |

## Arduino IDE Installation

Copy the complete `ASRSCommunication` folder into the Arduino libraries directory:

```text
Documents/Arduino/libraries/ASRSCommunication/
```

Restart the Arduino IDE after copying the folder. The examples should then appear under:

```text
File > Examples > ASRSCommunication
```

The packaged template examples do not require additional Arduino Library Manager dependencies. The `Master_UART_UnoNano_Template` example uses Arduino `SoftwareSerial`, which is provided by the classic AVR Arduino core. The `Master_UART_ESP32_Template` example uses `Serial1` hardware UART on ESP32-compatible boards.

The wider research project still contains motion-control and sensor firmware that may require `ASRSMotion`, `SparkFun VL53L1X 4m Laser Distance Sensor`, `AccelStepper`, or `U8g2`. Those project-specific sketches are intentionally not packaged as beginner library examples.

## PlatformIO Usage

For a local PlatformIO project, place the library in the project `lib` directory:

```text
lib/ASRSCommunication/
```

The existing PlatformIO project already uses this layout.

## Maintaining The Library Inside A PlatformIO Project

During active development, this library can be maintained as a separate Git repository inside the PlatformIO project:

```text
Combined Communication Protocol/
└── lib/
    └── ASRSCommunication/   <- independent library repository
```

Use this workflow when changing the library:

1. Open a terminal in `lib/ASRSCommunication`.
2. Modify files in `src/`, `examples/`, or `docs/`.
3. Test the affected Arduino or PlatformIO examples.
4. Update the version in `library.properties` and `library.json` when the change is released.
5. Commit and push from the `ASRSCommunication` folder, not from the firmware project root.

The firmware project can continue to build against the local editable copy in `lib/ASRSCommunication`. When a stable release is required, tag the library repository and optionally replace the local copy with a `lib_deps` entry that points to the tagged GitHub version.

If the library is later published to a Git repository, a PlatformIO project can reference it in `platformio.ini`:

```ini
lib_deps =
  https://github.com/your-name/ASRSCommunication.git
```

## Basic UART Master Example

```cpp
#include <Arduino.h>
#include <ASRSCommunication.h>

ASRS_Comm_UART uartCommunication(Serial1);
ASRS_Master master(uartCommunication);

void setup() {
  Serial.begin(115200);
  uartCommunication.begin(115200, -1, -1);
}

void loop() {
  ASRS_Coordinates coordinates;
  if (master.requestCoordinates(coordinates)) {
    Serial.println(coordinates.x);
  }
  delay(1000);
}
```

On ESP32-S3 boards, pass explicit UART pins to `begin()`. On Arduino boards with fixed hardware UART pins, pass `-1` for both pin arguments.

## Included Examples

| Example | Target board type | Communication interface | Purpose |
| --- | --- | --- | --- |
| `Master_ESPNow_Template` | ESP32 or ESP32-S3 boards only | ESP-NOW | Minimal master template for coordinate request, homing, movement, and limit-switch request. |
| `Master_UART_ESP32_Template` | ESP32 or ESP32-S3 boards only | Hardware UART | Minimal UART master template using configurable ESP32 UART pins. |
| `Master_UART_UnoNano_Template` | Arduino Uno R3 or classic Nano style AVR boards | SoftwareSerial UART | Minimal UART master template that keeps USB Serial available for debugging. |
| `Template_Dummy_Slave` | ESP32 or ESP32-S3 boards only | UART or ESP-NOW | Simulated slave for validating master firmware without the full ASRS mechanical system. |

The template examples are intended as end-user starting points. The project-level PlatformIO `src` templates remain available for repeatable research builds, while the sketches in this library folder are packaged for Arduino IDE library users.

## Versioning Policy

Use semantic versioning while the library develops:

| Change | Example | Version update |
| --- | --- | --- |
| Patch | Correct frame parsing without changing the API | `0.1.0` to `0.1.1` |
| Minor | Add a backward-compatible function or transport | `0.1.0` to `0.2.0` |
| Major | Change packet layout, command values, or public API behaviour | `0.1.0` to `1.0.0` or later major version |

Protocol-level changes must also be documented in `docs/ASRSCommunication_Library_Structure.txt` and in the project-level protocol documentation.
