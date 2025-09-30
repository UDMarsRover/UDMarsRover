# Drive Speed Control & Bluetooth Controller Integration (Ubuntu)

Version: 0.1 (Draft)

Related Source Files:
- `src/rover/rover/drive/bt_drive.py`
- `src/rover/rover/drive/pro_controller.py`
- `src/rover/rover/drive/UDMRTMotorSerial.py`

---
## 1. Purpose
This document explains:
1. The current speed / differential drive mixing algorithm used to convert Nintendo Switch Pro Controller inputs into left/right motor velocity targets.
2. How to connect, manage, and troubleshoot the Bluetooth controller on Ubuntu.
3. Suggested improvements (dead zones, smoothing, safety) for future iterations.

---
## 2. Input Flow Overview
```
Nintendo Pro Controller (Bluetooth)
        ↓ (SDL / Pygame axes)
`pro_controller.py` (polls every 50 ms)
        ↓ callbacks: LS_x, LS_y
`bt_drive.BTDrive` (mixing + scaling)
        ↓ 6-motor velocity vector (R R R L L L)
`UDMRTMotorSerial.send_velocity_set()` → UART → MCU → CAN bus → Motor controllers
```

- The controller loop is **blocking** (runs forever inside `NintendoProController.run()`), so `BTDrive` relies on callbacks invoked during polling.
- Left Stick axes mapping (as used):
  - `LS_x` (index 0): Strafe / turn component (X axis)
  - `LS_y` (index 1): Forward/backward component (Y axis)
  - Axis values are floats in range `[-1.0, 1.0]` from Pygame / SDL.

---
## 3. Speed Mixing Algorithm
Defined in `BTDrive.calculate_velocities(x, y)`:
```python
def calculate_velocities(self, x, y):
    left_velocity  = ((-y) + 0.5 * x) * self.max_velocity
    right_velocity = ((-y) - 0.5 * x) * self.max_velocity
    # Saturation to ±max_velocity
```
### 3.1 Rationale
- Standard differential (tank) drive mix derived from forward (F) and turn (T):
  - Raw forward command: `F = -y` (invert Y so pushing forward (>0) becomes positive motion)
  - Turn command: `T = x`
  - Left  = `F + k*T`
  - Right = `F - k*T`
- Here the turn scaling constant `k = 0.5` reduces over-rotation when the stick is fully deflected sideways.

### 3.2 Scaling
- `self.max_velocity = 300` is a linear scalar applied post-mix.
- Output saturation clamps each side to `[-max_velocity, max_velocity]`.
- Resulting velocities list sent to the motor layer:
  - `[right, right, right, left, left, left]`
  - Assumes first three motors belong to the right side, last three to left side.

### 3.3 Units
- The code treats values as logical velocities; unit semantics (e.g. RPM vs rad/s vs wheel linear m/s) are enforced downstream in the motor controller / firmware translation layer.

### 3.4 Edge Cases / Behavior
| Scenario | LS_x | LS_y | Left | Right | Effect |
|----------|------|------|------|-------|--------|
| Forward full | 0 | -1 | +300 | +300 | Straight forward |
| Reverse full | 0 | +1 | -300 | -300 | Straight reverse |
| In-place rotate right | +1 | 0 | +150 | -150 | Pivot (reduced by 0.5 factor) |
| In-place rotate left | -1 | 0 | -150 | +150 | Opposite pivot |
| Forward + turn right | +1 | -1 | 300 + 150 = 450 → clamp 300 | 300 - 150 = 150 | Arcing right |

### 3.5 Limitations
- No dead zone handling → Minor stick noise causes micro-movements.
- No acceleration limiting → Instant jumps may cause wheel slip or current spikes.
- No field-oriented control (pure differential frame).
- Turn scaling fixed at 0.5 (not adaptive to forward magnitude).

---
## 4. Suggested Algorithm Enhancements
| Enhancement | Benefit | Example Implementation Hint |
|-------------|---------|-----------------------------|
| Dead zone | Ignore small jitter | `if abs(x) < 0.08: x = 0` |
| Nonlinear response | Fine control near center | `x = math.copysign(x*x, x)` |
| Dynamic turn scale | Preserve forward speed | `k = 1 - 0.5*abs(F)` |
| Accel limiter (slew) | Smooth starts/stops | Limit Δv per frame (e.g. 20 units per 50 ms) |
| Watchdog timeout | Safety if controller disconnects | Zero velocities if no update in N ms |
| Brake / idle buttons | Rapid stop mode | Map A/B to 0 velocity or brake frame |

**Example Dead Zone + Slew (pseudo):**
```python
DEAD = 0.07
SLEW = 25.0  # units per cycle
now = time.time()
if abs(x) < DEAD: x = 0
if abs(y) < DEAD: y = 0
raw_left, raw_right = mix(x, y)
left  = clamp(prev_left  + clip(raw_left  - prev_left,  -SLEW, SLEW), -MAX, MAX)
right = clamp(prev_right + clip(raw_right - prev_right, -SLEW, SLEW), -MAX, MAX)
```

---
## 5. Nintendo Switch Pro Controller on Ubuntu
### 5.1 Required Packages
Install core Bluetooth stack and joystick utilities:
```bash
sudo apt update
sudo apt install -y bluetooth bluez bluez-tools blueman joystick python3-pip
pip install pygame
```

### 5.2 Pairing via Terminal (`bluetoothctl`)
```bash
bluetoothctl
[bluetooth]# power on
[bluetooth]# agent on
[bluetooth]# default-agent
[bluetooth]# scan on     # Wait for 'Pro Controller' MAC (e.g., XX:XX:XX:XX:XX:XX)
[bluetooth]# pair XX:XX:XX:XX:XX:XX
[bluetooth]# trust XX:XX:XX:XX:XX:XX
[bluetooth]# connect XX:XX:XX:XX:XX:XX
[bluetooth]# quit
```

### 5.3 Verifying Input Device
```bash
ls /dev/input/js* /dev/input/event* 2>/dev/null
python3 - <<'EOF'
import pygame, time
pygame.init(); pygame.joystick.init()
print('Joysticks:', pygame.joystick.get_count())
if pygame.joystick.get_count():
    j = pygame.joystick.Joystick(0); j.init()
    print('Name:', j.get_name(), 'Axes:', j.get_numaxes(), 'Buttons:', j.get_numbuttons())
EOF
```
If no joystick is detected, ensure:
- Controller is trusted / connected (`bluetoothctl info <MAC>` shows `Connected: yes`).
- `sudo systemctl status bluetooth` is active.
- No other host is holding the device (unpair from other machines).

### 5.4 Auto-Reconnect Tips
Create a udev or systemd user service that runs a small script invoking `bluetoothctl connect <MAC>` when disconnected. Example systemd user service:
```
~/.config/systemd/user/pro-controller-reconnect.service
[Unit]
Description=Reconnect Nintendo Pro Controller
After=bluetooth.target

[Service]
Type=simple
ExecStart=/usr/bin/bash -c 'while true; do bluetoothctl connect <MAC> || true; sleep 5; done'

[Install]
WantedBy=default.target
```
Enable:
```bash
systemctl --user daemon-reload
systemctl --user enable --now pro-controller-reconnect.service
```

### 5.5 SDL / Pygame Environment
- The code forces headless operation: `SDL_VIDEODRIVER=dummy`.
- If encountering `pygame.error: No available video device`, ensure the environment variable is exported before launching your script or keep current code order.

### 5.6 Latency Considerations
- Poll rate: 50 ms (configured in `pro_controller.py`). Effective control loop ~20 Hz.
  - For smoother control, reduce to e.g. 20 ms (50 Hz). Ensure CPU usage stays acceptable.
- Bluetooth interference (2.4 GHz) can introduce jitter; prefer line-of-sight or reduce crowded channels.

### 5.7 Security & Reliability
- Disable pairing mode after use; the controller will auto-connect to the last trusted host.
- Avoid running as root; only user-level access to `/dev/input/js*` is needed (input group membership may help on some distros: `sudo usermod -a -G input $USER`).

---
## 6. Integrating a Different Controller
1. Replace `NintendoProController` with an abstraction layer.
2. Map new controller axes to logical `forward` and `turn`.
3. Normalize ranges to `[-1, 1]`.
4. Reuse the same mixing function.

---
## 7. Safety & Watchdogs (Recommended)
| Mechanism | Description |
|-----------|-------------|
| Input timeout | Zero velocities if no stick update within 250 ms |
| Serial heartbeat | MCU expects periodic packet; stops motors if stale |
| E-stop binding | Map a button (e.g. `B`) to immediate zero + brake packet |
| Sanity check | Reject NaN / inf before sending velocities |

**Watchdog Example (Pseudo):**
```python
last_update = time.time()
if time.time() - last_update > 0.25:
    self.serial_conn.send_velocity_set([0]*6)
```

---
## 8. Common Troubleshooting
| Symptom | Possible Cause | Resolution |
|---------|----------------|-----------|
| Controller not detected | Bluetooth service down | `sudo systemctl restart bluetooth` |
| Axis stuck near ±0.003 | Noise | Implement dead zone |
| Sudden jumps in velocity | No ramp limiting | Add slew limiter |
| High latency | Poll rate too low | Decrease pygame wait delay |
| Serial exception on start | Wrong `/dev/serial` symlink | Run `ls -l /dev/serial/by-id` and update path |

---
## 9. Extension Ideas
- Integrate ROS2 topic publication (`/cmd_vel`) for interoperability.
- Add logging ring buffer of last N velocity commands.
- Implement field-centric drive using IMU yaw.
- Teleop GUI with live stick visualization.

---
## 10. Quick Reference (API Snippets)
```python
# Construct and send zero velocity (stop)
self.serial_conn.send_velocity_set([0.0]*6)

# Adjust max velocity dynamically
bt_drive.max_velocity = 400

# Modify turn scaling
def calculate_velocities(self, x, y):
    F = -y
    k = 0.5 * (1 - 0.3 * abs(F))  # reduce turn at high forward speed
    L = (F + k * x) * self.max_velocity
    R = (F - k * x) * self.max_velocity
    return clamp(L), clamp(R)
```

---
## 11. Summary
- The current system uses a simple linear differential mixing model with fixed scaling and clamping.
- Controller integration is minimalistic and can be enhanced with dead zones, smoothing, and safety watchdogs.
- Bluetooth setup on Ubuntu relies on standard `bluez` tools; stability can be improved with auto-reconnect strategies.

---
*End of Document.*
