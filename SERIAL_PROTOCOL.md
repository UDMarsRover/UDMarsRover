# UDMRT Serial Protocol Specification

Version: 0.1 (Draft)  
Source Files of Reference:
- Host: `src/rover/rover/drive/UDMRTMotorSerial.py`
- Firmware: `src/rover/rover/drive/arduino/motor_controller/motor_controller.ino`

---
## 1. Overview
All UART messages are **fixed-length 25-byte packets**:

| Byte | Meaning | Notes |
|------|---------|-------|
| 0 | Message Type | Command (host→MCU) or Status (MCU→host) code |
| 1–24 | Payload | Layout depends on Message Type |

Current transport parameters: `115200 8N1`, no framing sentinel, no checksum.  
Framing is implicit via fixed length (host performs blocking `read(25)`).

> Future improvement recommended: Add preamble + version + CRC (see §12).

---
## 2. Message Type Summary

### Control (Host → MCU)
| Code | Name | Status |
|------|------|--------|
| `0x01` | Velocity Set | Implemented (host + firmware) |
| `0x02` | Idle Mode (Coast/Brake) | Implemented (host); NOT parsed in firmware yet |
| `0x03` | Current Limit | Reserved / TBD |
| `0x04` | Clear Faults | Reserved / TBD |

### Status (MCU → Host)
| Code | Name | Status |
|------|------|--------|
| `0x85` | Velocity Status | Implemented (host mock only); NOT sent by firmware |
| `0x86` | Other Status (Temp/Voltage/Current) | Partially implemented (firmware sends only first motor’s 4 bytes) |

---
## 3. Data Representation

| Field | Type | Endianness | Notes |
|-------|------|------------|-------|
| Velocities (control / status) | IEEE-754 float32 | Little-endian | Copied with `memcpy` (MCU) / `np.float32(...).tobytes()` (host) |
| Temperature | int8 | N/A | °C |
| Voltage (fixed-point) | 12-bit unsigned | Bit-packed | Full-scale 12.0 V |
| Current (fixed-point) | 12-bit unsigned | Bit-packed | Full-scale 3.0 A |

Fixed-point scaling formulas:
- Encoding: `fixed = round(value / FS * 4095)`
- Decoding: `value = fixed * FS / 4095`

Where `FS` = 12.0 for voltage, `FS` = 3.0 for current.

---
## 4. Velocity Set Packet (`0x01`)
**Direction:** Host → MCU  
**Purpose:** Set target velocity for each of 6 motors (units depend on downstream CAN motor controller configuration — e.g., RPM or rad/s).

| Byte(s) | Description |
|---------|-------------|
| 0 | `0x01` (type) |
| 1–4 | Motor 1 velocity (float32 LE) |
| 5–8 | Motor 2 velocity |
| 9–12 | Motor 3 velocity |
| 13–16 | Motor 4 velocity |
| 17–20 | Motor 5 velocity |
| 21–24 | Motor 6 velocity |

Total: 1 + 6×4 = 25 bytes.

**Firmware Handling:** Extracts floats and forwards each to CAN using mode constant `Smart_Velocity_Set`.

**Host Construction (reference):**
```python
def construct_velocity_set(vels):
    assert len(vels) == 6
    pkt = bytearray([0x01])
    for v in vels:
        pkt.extend(np.float32(v).tobytes())  # little-endian
    return pkt
```

---
## 5. Idle Mode Packet (`0x02`)
**Direction:** Host → MCU  
**Purpose:** Request braking or coasting state (NOT yet parsed in firmware).

| Byte(s) | Description |
|---------|-------------|
| 0 | `0x02` (type) |
| 1 | Mode: `0x00` = Coast, `0x01` = Brake |
| 2–24 | Zero padding |

---
## 6. Velocity Status Packet (`0x85`)
**Direction:** MCU → Host  
**Purpose:** Report measured velocities (NOT currently emitted by firmware).

| Byte(s) | Description |
|---------|-------------|
| 0 | `0x85` (type) |
| 1–24 | 6 × float32 LE velocity feedback |

---
## 7. Other Status Packet (`0x86`)
**Direction:** MCU → Host  
**Purpose:** Report temperature, voltage, and current for each motor (compact bit-packed format).  
**Current firmware state:** Only one 4-byte block (motor 1) is populated and transmitted; remaining 5 blocks are zeroed (incomplete implementation).

### 7.1 Canonical Per-Motor Block (4 bytes)
| Byte (relative) | Bits | Meaning |
|-----------------|------|---------|
| 0 | 7..0 | Temperature (int8 °C) |
| 1 | 7..0 | Voltage bits [11:4] |
| 2 | 7..4 | Voltage bits [3:0] |
| 2 | 3..0 | Current bits [11:8] |
| 3 | 7..0 | Current bits [7:0] |

Packet layout:
| Byte(s) | Description |
|---------|-------------|
| 0 | `0x86` (type) |
| 1–4 | Motor 1 block |
| 5–8 | Motor 2 block |
| 9–12 | Motor 3 block |
| 13–16 | Motor 4 block |
| 17–20 | Motor 5 block |
| 21–24 | Motor 6 block |

### 7.2 Decoding Formulas
```
V_fixed = (B1 << 4) | (B2 >> 4)
I_fixed = ((B2 & 0x0F) << 8) | B3
Voltage = V_fixed * 12.0 / 4095
Current = I_fixed * 3.0 / 4095
```

### 7.3 Reference Decoder (Corrected)
```python
def decode_other_status(packet):
    assert packet[0] == 0x86
    stats = []
    for i in range(6):
        base = 1 + i * 4
        temp = np.frombuffer(packet, dtype=np.int8, count=1, offset=base)[0]
        B1 = packet[base + 1]
        B2 = packet[base + 2]
        B3 = packet[base + 3]
        v_fixed = (B1 << 4) | (B2 >> 4)
        i_fixed = ((B2 & 0x0F) << 8) | B3
        voltage = v_fixed * 12.0 / 4095
        current = i_fixed * 3.0 / 4095
        stats.append((temp, voltage, current))
    return stats
```

> NOTE: The current host implementation (`deconstruct_other_status_frame`) uses a different bit extraction order and produces incorrect physical values; it should be updated per the above.

---
## 8. Reserved Control Packets
| Code | Meaning | Proposed Notes |
|------|---------|----------------|
| `0x03` | Current Limit | Define whether per-motor (6 × 16-bit?) or global. |
| `0x04` | Clear Faults | Acknowledge & clear latched error flags. |

---
## 9. Example Exchange Flow
1. Host sends Velocity Set (0x01) every control cycle (e.g. 50 Hz).
2. MCU applies velocities via CAN to each motor controller.
3. MCU periodically sends Other Status (0x86) (currently only partial data).
4. Host parses telemetry and updates UI / safety logic.

---
## 10. CAN Interaction Context (Informational Only)
- Firmware relays each velocity to CAN using extended ID = `Smart_Velocity_Set + device_id`.
- Heartbeat frames are CAN-only (`0x2052C80`) and not serialized over UART.
- Status frames from motor controllers (IDs `status_0` .. `status_4` + device offset) are partially ingested to compose serial status.

---
## 11. Known Issues & Gaps
| Area | Issue | Impact | Action |
|------|-------|--------|--------|
| Other Status Build | Only motor 1 data populated | Misleading telemetry | Implement full 6-motor aggregation in firmware |
| Other Status Decode | Host bit-order mismatch | Incorrect voltage/current | Fix decoder per §7.2 |
| Velocity Status | Not emitted by MCU | No real feedback | Add periodic 0x85 frame generation |
| Idle Mode | Not parsed on MCU | Command inert | Implement packet type 0x02 handler |
| Integrity | No checksum | Silent corruption possible | Add CRC-8 (polynomial 0x07) |
| Framing | No sentinel/version | Harder protocol evolution | Add preamble + version byte |

---
## 12. Recommended Enhancements (Next Revisions)
| Priority | Enhancement | Description |
|----------|------------|-------------|
| High | CRC-8 | Append 1 byte; adjust payload or increase frame size |
| High | Firmware full 0x86 data | Populate all 6 motor blocks before transmit |
| Medium | Implement 0x02 | Apply brake/coast via CAN mode or GPIO |
| Medium | Implement 0x85 | Provide true measured velocities |
| Low | Versioning | Byte 1 reserved for protocol version; shift payloads |
| Low | Sequence number | Detect dropped frames |

---
## 13. Change Log
- 0.1: Initial draft extracted from current code base (incomplete implementation noted).

---
## 14. Quick Reference Table
| Type | Dir | Code | Payload Bytes 1–24 |
|------|-----|------|--------------------|
| Velocity Set | H→M | 0x01 | 6 × float32 LE |
| Idle Mode | H→M | 0x02 | Mode (1 byte) + padding |
| Current Limit | H→M | 0x03 | TBD |
| Clear Faults | H→M | 0x04 | TBD |
| Velocity Status | M→H | 0x85 | 6 × float32 LE |
| Other Status | M→H | 0x86 | 6 × (Temp int8 + packed V/I) |

---
## 15. Validation Checklist
- [ ] Every frame exactly 25 bytes
- [ ] Byte 0 matches known message type
- [ ] Velocity packets: 1 + 24 bytes, each 4-byte chunk decodes to finite float
- [ ] Other status: Each motor block 4 bytes; bitfields consistent with §7.2
- [ ] No unexpected / orphaned message codes observed

---
## 16. Appendix: Bit Packing Diagram (Other Status)
```
Motor Block (4 bytes):

Byte0:  T7  T6  T5  T4  T3  T2  T1  T0   (Temperature)
Byte1:  V11 V10 V9  V8  V7  V6  V5  V4   (Voltage high)
Byte2:  V3  V2  V1  V0  C11 C10 C9  C8   (Voltage low + Current high)
Byte3:  C7  C6  C5  C4  C3  C2  C1  C0   (Current low)
```

---
*End of Specification.*
