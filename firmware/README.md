# Arduino Firmware v0.9.5

This directory contains the Arduino Mega 2560 firmware for the Project NUEVO robot controller. The firmware is responsible for:

- hard real-time stepper and DC motor servicing
- sensor acquisition and voltage monitoring
- TLV communication with the Raspberry Pi
- system state management and safety policy
- user I/O, discrete LEDs, and NeoPixel state indication

The current `v0.9.5` profile is the current cleaned-up release after the ISR/UART stabilization work. The code is organized so [arduino/arduino.ino](arduino/arduino.ino) reads as the top-level entry point, while detailed ownership lives in modules under [`arduino/src/`](arduino/src/).

## Documentation Map

- [`docs/architecture.md`](docs/architecture.md): top-level runtime map, startup order, execution layers, and module ownership
- [`docs/communication_and_state.md`](docs/communication_and_state.md): TLV receive/transmit flow, heartbeat handling, `SystemManager`, `SafetyManager`, `StatusReporter`
- [`docs/motion_control.md`](docs/motion_control.md): DC mixed-control pipeline, stepper timing, servo control, encoder usage, odometry
- [`docs/sensors_and_i2c.md`](docs/sensors_and_i2c.md): IMU/ultrasonic implementation, voltage monitoring, shared `Wire` bus, PCA9685 servo interactions, and the Pi-side lidar note
- [`docs/technical_notes.md`](docs/technical_notes.md): low-level constraints, timer ownership, UART/NeoPixel limits, memory/timing notes
- [`docs/pin_table_rev_A.md`](docs/pin_table_rev_A.md): Rev. A pin map
- [`docs/pin_table_rev_B.md`](docs/pin_table_rev_B.md): Rev. B pin map
- [`docs/REV_A_TO_REV_B_CHANGES.md`](docs/REV_A_TO_REV_B_CHANGES.md): hardware migration summary and current Rev. B notes
- [`docs/TIMER3_CONFLICT_ANALYSIS.md`](docs/TIMER3_CONFLICT_ANALYSIS.md): historical note about the Timer3 / LED_RED conflict and its resolution
- [`tests/README.md`](tests/README.md): standalone subsystem test sketches

## Source Layout

```text
firmware/
├── arduino/
│   ├── arduino.ino           # Entry point: setup(), loop(), ISR vectors, soft tasks
│   └── src/
│       ├── config.h          # User-facing compile-time configuration
│       ├── pins.h            # Board pin mapping and OCR register aliases
│       ├── utility.*         # Small shared helpers (startup print, timer tick reads, UART fault sampling)
│       ├── Scheduler.*       # Soft scheduler: fast lane + periodic task dispatch
│       ├── ISRScheduler.*    # Timer and interrupt attachment helpers
│       ├── SystemManager.*   # Firmware state machine and transition policy
│       ├── messages/         # TLV type/payload definitions
│       ├── drivers/          # Per-device driver classes
│       ├── modules/          # Higher-level firmware subsystems
│       └── lib/              # Vendored third-party libraries
├── docs/
│   ├── architecture.md
│   ├── communication_and_state.md
│   ├── motion_control.md
│   ├── sensors_and_i2c.md
│   ├── technical_notes.md
│   ├── pin_table_rev_A.md
│   ├── pin_table_rev_B.md
│   ├── REV_A_TO_REV_B_CHANGES.md
│   └── TIMER3_CONFLICT_ANALYSIS.md
└── tests/
    └── test_*/               # Standalone Arduino sketches for subsystem bring-up
```

## Runtime Overview

The firmware uses three execution layers.

### 1. Hard real-time ISR layer

These paths must stay short and deterministic:

- encoder interrupts for DC motor counting
- `TIMER1_OVF_vect` at `800 Hz`: short DC round-robin latch/apply slot. Process one motor at one time; 200 Hz for each motor.
- `TIMER3_OVF_vect` at `10 kHz`: stepper pulse generation

`Timer4` is PWM hardware only in the current profile. It does not own an ISR.

### 2. Fast cooperative lane in `loop()`

These services run opportunistically every loop pass:

- UART RX drain
- UART TX drain
- DC compute trigger service
- status report chunk emission
- debug log flush

This keeps `loop()` useful without pretending these are fixed-rate periodic tasks.

### 3. Periodic soft tasks

The millis-based scheduler owns jitter-tolerant tasks:

- `taskUART()` at `50 Hz`
- `taskSafety()` at `100 Hz`
- `taskSensors()` at `100 Hz`
- `taskUserIO()` at `20 Hz`
- `StatusReporter::task()` at `1 Hz` when enabled

When the full status reporter is disabled, the firmware can still emit short
event-driven USB debug lines for new loop/UART/control faults. That lightweight
path does not generate the large `[SYSTEM] / [TIMING] / [SENSORS] / [UART]`
report.

Inside `taskSensors()`, the current `v0.9.5` profile runs:

- IMU raw read at `50 Hz`
- Fusion update at `50 Hz` on the alternating sensor tick
- ultrasonic sensors at `50 Hz`
- voltage rails at `10 Hz`

## Major Subsystems

| Module | Responsibility |
|--------|----------------|
| `SystemManager` | Single source of truth for firmware state and transition policy |
| `MessageCenter` | TLV RX/TX, command routing, telemetry packing, liveness bookkeeping |
| `MotorControlCoordinator` | Mixed ISR/loop DC control pipeline bookkeeping |
| `StepperManager` | 10 kHz stepper service and stepper state |
| `SensorManager` | IMU, ultrasonic, voltage, and mag-calibration handling |
| `UserIO` | Input sampling cache, discrete LEDs, NeoPixel state rendering |
| `StatusReporter` | Human-readable `[SYSTEM] / [TIMING] / [UART]` status output |
| `LoopMonitor` | Timing-budget monitoring and overrun flagging |

For the implementation-level docs, start with [`docs/architecture.md`](docs/architecture.md) and then jump to the subsystem-specific documents listed above.

## State Machine

The firmware state machine lives in `SystemManager`:

- `INIT -> IDLE` after boot completes
- `IDLE -> RUNNING` on `START`
- `RUNNING -> IDLE` on `STOP`
- `ERROR/ESTOP -> IDLE` on `RESET`
- `any -> ESTOP` on emergency stop
- `RUNNING -> ERROR` on heartbeat or battery safety fault

Important battery policy:

- the firmware may enter `RUNNING` with no battery present
- drive/servo enable commands are rejected while battery is absent
- once battery power has been seen during a `RUNNING` session, losing it or crossing safety thresholds trips `ERROR`

## Build and Upload

### Main firmware

```bash
arduino-cli compile --fqbn arduino:avr:mega firmware/arduino
```

### Upload

```bash
arduino-cli upload --fqbn arduino:avr:mega -p /dev/ttyUSB0 firmware/arduino
```

Replace `/dev/ttyUSB0` with the correct port for your machine.

## Configuration

Start with [`arduino/src/config.h`](arduino/src/config.h).

The top of the file contains the settings most users are expected to tune first:

- UART baud rate and heartbeat timeout
- soft task cadences
- status reporter enable/rate
- debug serial rate

The rest of the file covers:

- motor and encoder configuration
- telemetry cadences
- PID defaults
- battery thresholds
- sensor enable flags
- debug/bring-up options

## User I/O Policy

- Discrete LEDs are reserved for user/TLV control.
- The NeoPixel is the automatic system-state indicator.
- The current supported WS2812 profile is a single state-indicator pixel with infrequent updates. Multi-pixel WS2812 animation is not part of the supported `v0.9.5` profile on the Mega 2560.

## Test Sketches

The sketches under [`tests/`](tests/) are standalone bring-up tools for specific subsystems. They are documented in [`tests/README.md`](tests/README.md). The main firmware documentation in this folder describes the production firmware, not every historical test sketch behavior.
