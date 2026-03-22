# TLV Redesign Plan

This document captures the next protocol cleanup goals. It is a planning note,
not the implemented source of truth.

Use:

- [`COMMUNICATION_PROTOCOL.md`](COMMUNICATION_PROTOCOL.md) for the current
  protocol behavior and logical message catalog
- [`../tlv_protocol/TLV_TypeDefs.json`](../tlv_protocol/TLV_TypeDefs.json) for
  authoritative TLV IDs
- [`../tlv_protocol/TLV_Payloads.md`](../tlv_protocol/TLV_Payloads.md) for
  exact payload layouts

---

## 1. Goals

1. Make the TLV set more structured and easier to reason about.
2. Reduce unnecessary streamed data so bandwidth is spent on data the UI
   actually needs at runtime.
3. Separate:
   - fast-changing runtime state
   - slow-changing configuration and identity
   - engineering / debug diagnostics
4. Align message design with the current firmware and `nuevo_bridge`
   architecture instead of older assumptions.

---

## 2. Core Direction

The current protocol works, but several streamed payloads mix unrelated data.
The redesign should move toward a cleaner split:

- **streamed state** for live UI/runtime monitoring
- **query/response snapshots** for configuration, identity, and static metadata
- **separate debug/diagnostic messages** for engineering-only information

This should let us:

- stream important data faster
- slow down or query-only less important data
- make the UI logic simpler
- reduce UART pressure

---

## 3. Candidate Changes

### 3.1 System messages

#### `SYS_STATUS`

Problems:

- It currently mixes runtime state, static firmware info, compile-time config,
  and diagnostics.
- Much of that data does not need to be streamed continuously.

Proposed direction:

- keep a smaller streamed system-state message
- move firmware version, motor direction, configured sensor set, and other
  "who am I" data into a queryable response
- keep voltage data separate, since `SENSOR_VOLTAGE` already exists and is
  conceptually clearer
- add an explicit warning field or warning bitmask

Candidate new pattern:

- `SYS_STATE` or smaller `SYS_STATUS` for live state
- `SYS_INFO_REQ / SYS_INFO_RSP` for static metadata
- optional `SYS_DIAG_REQ / SYS_DIAG_RSP` for RAM, UART counters, loop timing

#### `SYS_CONFIG`

Problems:

- `resetOdometry` does not really behave like configuration

Proposed direction:

- make odometry reset its own command

### 3.2 DC motor messages

#### `DC_STATUS_ALL`

Problems:

- PID gains are bundled into the high-rate status stream
- there is no explicit timestamp

Proposed direction:

- keep measured/runtime state in the streamed message
- move PID values to query/response or explicit reply-after-set behavior
- add timestamp

### 3.3 Stepper messages

#### `STEP_STATUS_ALL`

Problems:

- static config like max speed and accel should not ride in the fast stream
- naming such as `commandedCount` / `targetCount` is confusing
- there is no explicit timestamp

Proposed direction:

- keep position/target/state/homing/live flags in the streamed message
- move config fields to query/response
- rename count fields to something more direct and consistent

### 3.4 IO messages

#### `IO_STATUS`

Problems:

- buttons and limits want a faster update rate
- LED / NeoPixel output mirror does not need the same rate
- current payload grouping is not ideal for user-visible responsiveness

Proposed direction:

- split input state from output state
- stream buttons / limits faster
- stream LEDs / NeoPixel slower or on-change

Candidate new pattern:

- `IO_INPUT_STATE`
- `IO_OUTPUT_STATE` or `LED_STATUS`

---

## 4. Likely Cross-Cutting Protocol Rules

These are not finalized yet, but they are likely needed to make the redesign
consistent:

- streamed runtime messages should carry timestamps consistently
- static configuration should prefer `REQ / RSP` pairs
- set commands that change configuration may optionally trigger an immediate
  matching response snapshot
- runtime diagnostics may move into a separate debug-only message family

---

## 5. Expected Implementation Impact

This redesign will touch more than TLV payloads.

Likely affected areas:

- `docs/COMMUNICATION_PROTOCOL.md`
- `tlv_protocol/TLV_TypeDefs.json`
- `tlv_protocol/TLV_Payloads.md`
- `tlv_protocol/generate_tlv_types.py`
- firmware `MessageCenter`
- firmware status/telemetry scheduling
- `nuevo_bridge` payloads and routing
- UI topic handling and rate assumptions

So this should be treated as a coordinated protocol revision, not a simple
payload tweak.

---

## 6. Next Step

Before editing the machine-readable TLV definitions, define the new logical
message catalog in [`COMMUNICATION_PROTOCOL.md`](COMMUNICATION_PROTOCOL.md)
first.
