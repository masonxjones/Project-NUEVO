# NUEVO Platform — Design Guidelines

This document defines conventions that all sub-projects must follow. Its purpose is to keep firmware, bridge, UI, and SDK consistent as they evolve independently. When in doubt, check here first.

For the wire protocol specification (TLV frame format, type IDs, payload fields), see [COMMUNICATION_PROTOCOL.md](COMMUNICATION_PROTOCOL.md).

---

## 1. Sub-Project Map

| Directory | Language | Role |
|-----------|----------|------|
| `firmware/arduino/` | C++ (Arduino) | Real-time hardware control on Arduino Mega 2560 |
| `nuevo_ui/backend/nuevo_bridge/` | Python (FastAPI) | UART ↔ WebSocket bridge; conversion layer between wire and UI |
| `nuevo_ui/frontend/` | TypeScript (React) | NUEVO control panel UI |
| `tlv_protocol/` | — | Protocol source-of-truth: type IDs, payload schemas, generators |
| `ros2_ws/` | Python/C++ (ROS2) | High-level navigation, perception (Raspberry Pi) |
| `NUEVO board/` | — | PCB schematics and specifications |

### Data flow

```
Arduino firmware
    ↕  UART (TLV v3.0, 250 kbps)
nuevo_bridge  ←→  (conversion: 0-based wire ↔ 1-based user)
    ↕  WebSocket (JSON)
NUEVO UI / Python SDK
```

---

## 2. Source-of-Truth Index

Each concern has exactly one authoritative source. Everything else derives from it.

| Concern | Source of Truth |
|---------|----------------|
| TLV type IDs and field names | `tlv_protocol/TLV_TypeDefs.json` |
| Payload field layouts and sizes | `tlv_protocol/TLV_Payloads.md` |
| Generated C++ type constants | `firmware/arduino/src/messages/TLV_TypeDefs.h` (auto-generated) |
| Generated Python type constants | `nuevo_ui/backend/nuevo_bridge/TLV_TypeDefs.py` (auto-generated) |
| Arduino hardware pin assignments | `firmware/arduino/src/pins.h` |
| Arduino compile-time parameters | `firmware/arduino/src/config.h` |
| PCB hardware specifications | `NUEVO board/SPECIFICATIONS.md` |

**Rule:** Never edit auto-generated files by hand. Change the source and regenerate:

```bash
cd tlv_protocol
python generate_tlv_types.py
# Generates TLV_TypeDefs.h  → firmware/arduino/src/messages/TLV_TypeDefs.h
# Generates TLV_TypeDefs.py → nuevo_ui/backend/nuevo_bridge/TLV_TypeDefs.py
```

After regenerating, update `TLV_Payloads.md` and `COMMUNICATION_PROTOCOL.md` to match any schema changes.

---

## 3. Numbering Convention

**This is the most important cross-project consistency rule.**

### Rule

| Context | Numbering | Examples |
|---------|-----------|---------|
| **User-facing** (UI labels, Python SDK API, documentation) | **1-based** | Motor 1–4, Stepper 1–4, Servo 1–16, Button 1–10, Limit Switch 1–8 |
| **TLV wire fields** (`motorId`, `channel`, `buttonId`, etc.) | **0-based** | motorId 0–3, channel 0–15, buttonId 0–9 |
| **Firmware internal** (array indices, config constants) | **0-based** | `dcMotors[0]`, `DC_MOTOR_1 = 0` |

### Conversion

The **nuevo_bridge is the only place** that performs number conversion. It converts between 0-based wire values and 1-based user-facing values. No other layer should contain conversion logic.

```python
# In nuevo_bridge — outgoing to UI (wire → user)
motor_number = payload.motorId + 1      # 0-based → 1-based

# In nuevo_bridge — incoming from UI (user → wire)
motor_id = command["motorNumber"] - 1   # 1-based → 0-based
```

### Named constants in firmware

Config constants use 1-based names to match user documentation, but their values are 0-based indices:

```c
// config.h — named motor aliases (1-based names, 0-based values)
#define DC_MOTOR_1      0
#define DC_MOTOR_2      1
#define DC_MOTOR_3      2
#define DC_MOTOR_4      3

#define STEPPER_1       0
#define STEPPER_2       1
#define STEPPER_3       2
#define STEPPER_4       3
```

Use these names when assigning motors to roles:

```c
#define ODOM_LEFT_MOTOR     DC_MOTOR_1
#define ODOM_RIGHT_MOTOR    DC_MOTOR_2
```

---

## 4. Naming Conventions

### C++ (firmware)

| Element | Convention | Example |
|---------|------------|---------|
| Macros / compile-time constants | `UPPER_SNAKE_CASE` | `DC_MOTOR_1`, `ENCODER_PPR` |
| Classes | `PascalCase` | `MessageCenter`, `DCMotor` |
| Static class members (private) | `camelCase_` (trailing underscore) | `wheelBaseMm_`, `odomX_` |
| Local variables / parameters | `camelCase` | `dLeft`, `motorId` |
| Function names | `camelCase` | `updateOdometry()`, `processIncoming()` |

### Python (nuevo_bridge, SDK)

| Element | Convention | Example |
|---------|------------|---------|
| Modules / files | `snake_case` | `serial_manager.py`, `payloads.py` |
| Classes | `PascalCase` | `PayloadMotorStatus`, `SerialManager` |
| Functions / methods | `snake_case` | `process_incoming()`, `send_command()` |
| Constants | `UPPER_SNAKE_CASE` | `TLV_TYPE_MOTOR_STATUS`, `BAUD_RATE` |
| Instance variables | `snake_case` | `motor_id`, `wheel_base_mm` |

### TypeScript (NUEVO UI)

| Element | Convention | Example |
|---------|------------|---------|
| React components | `PascalCase` | `MotorPanel`, `StatusBar` |
| Hooks | `useCamelCase` | `useMotorState`, `useWebSocket` |
| Variables / functions | `camelCase` | `motorNumber`, `sendCommand` |
| Constants | `UPPER_SNAKE_CASE` | `WS_URL`, `MAX_MOTORS` |
| Types / interfaces | `PascalCase` | `MotorStatus`, `TelemetryFrame` |

### WebSocket message fields (JSON)

Use `camelCase` for all WebSocket JSON field names:

```json
{ "type": "motorStatus", "motorNumber": 1, "velocityTicksPerSec": 245, "pwmOutput": 180 }
```

Note: `motorNumber` is 1-based (user-facing). The bridge converts from the wire field `motorId` (0-based).

---

## 5. Units

All inter-layer data uses these units unless explicitly annotated otherwise.

| Quantity | Unit | Notes |
|----------|------|-------|
| Distance / position | **mm** | Odometry x/y in mm |
| Wheel dimensions | **mm** | Diameter, wheel base |
| Angle / heading | **rad** | Odometry theta; IMU quaternion uses dimensionless |
| Angular rate | **rad/s** | Gyroscope output |
| Velocity (motor) | **ticks/s** | Raw encoder velocity; convert with mm-per-tick for odometry |
| Linear velocity | **mm/s** | After mm-per-tick conversion |
| Acceleration | **mg** | Raw IMU accelerometer readings (int16); AHRS earth-frame accel uses **g** (float) |
| Magnetic field | **µT** | Magnetometer |
| Time | **ms** | All timestamps and timeouts |
| Pulse width | **µs** | Servo positions |
| Voltage | **mV** | All voltage readings |
| Current | **mA** | All current readings |
| PWM | **−255 to +255** | Signed; positive = forward |

---

## 6. Coordinate System

The robot uses a standard right-hand 2D body frame:

- **+X**: forward (direction of travel)
- **+Y**: left
- **+θ**: counter-clockwise positive (right-hand rule about +Z up)

This matches ROS2 REP-103. Odometry `(x, y, theta)` in mm / mm / rad is published in this frame.

---

## 7. Protocol Change Workflow

When any TLV message needs to change (new field, new type, changed payload size):

1. **Edit `tlv_protocol/TLV_TypeDefs.json`** — add/modify type IDs.
2. **Edit `tlv_protocol/TLV_Payloads.md`** — update payload field tables and sizes.
3. **Run generator** → updates `TLV_TypeDefs.h` and `TLV_TypeDefs.py`.
4. **Update `firmware/.../TLV_Payloads.h`** — add/modify `Payload*` structs to match.
5. **Update `nuevo_bridge/payloads.py`** — add/modify `Payload*` dataclasses to match.
6. **Update `COMMUNICATION_PROTOCOL.md`** — keep the human-readable protocol spec current.
7. **Bump firmware version** in `config.h` (`FIRMWARE_VERSION`).

Never change step 4 or 5 without also updating step 2 and 6. The `.md` files are the documentation that future contributors (and students) will read.

---

## 8. Firmware State Machine

The Arduino firmware has five states. The bridge and UI must track and display these.

| State | Value | Description |
|-------|-------|-------------|
| `SYS_STATE_INIT` | 0 | Initialising hardware |
| `SYS_STATE_IDLE` | 1 | Ready; slow `SYS_STATE` / `SYS_POWER` streams and query responses only |
| `SYS_STATE_RUNNING` | 2 | Full operation; full telemetry active |
| `SYS_STATE_ERROR` | 3 | Non-fatal error |
| `SYS_STATE_ESTOP` | 4 | Emergency stop; requires explicit reset |

Valid transitions: `IDLE → RUNNING` via `SYS_CMD_START`; `RUNNING → IDLE` via `SYS_CMD_STOP`; any `→ ESTOP` via `SYS_CMD_ESTOP`; `ERROR/ESTOP → IDLE` via `SYS_CMD_RESET`.

---

## 9. Safety Rules

These rules apply at every layer.

1. **Heartbeat timeout (500 ms default)**: If the Arduino receives no TLV packet for 500 ms, it disables all actuators and returns to IDLE. The bridge must send a heartbeat if no command is sent within 400 ms.
2. **ESTOP is always honoured**: Any layer receiving `SYS_CMD_ESTOP` must propagate it immediately. The UI must display ESTOP state prominently.
3. **Never leave motors running without feedback**: The bridge runs independently of WebSocket clients — UI connections may come and go freely without affecting robot state. The bridge must send `SYS_CMD_STOP` before closing the serial port on **process shutdown** only.
4. **Validate ranges before sending**: Commands with out-of-range motor IDs, PWM values, or servo positions must be rejected by the bridge before reaching the wire.

---

## 10. Adding a New Sub-Project

When adding a new component (e.g., a Python SDK, a ROS2 driver):

1. Follow the naming conventions for its language (§4).
2. Use 1-based numbers in its public API.
3. Convert to 0-based only at the boundary with the TLV wire protocol.
4. Consume TLV type constants from the generated files (`TLV_TypeDefs.py` or `TLV_TypeDefs.h`). Never hardcode type ID integers.
5. Document the component in this file under §1 (Sub-Project Map).
6. Add build/run instructions to the top-level `README.md`.
