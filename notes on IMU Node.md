# IMU (BNO08x) on Raspberry Pi — Implementation Notes

## What We Installed

- **Extended I²C backend (Linux):**  
  ```bash
  pip install --user adafruit-extended-bus
  ```
- **Adafruit BNO08x driver (public release):**  
  `adafruit-circuitpython-bno08x==1.2.10`  
  > Note: This release **does not** support `report_interval` / `batch_interval` kwargs on `enable_feature()`. The node handles this automatically (fallback).

- **Already present / required:** `adafruit-blinka`, `adafruit-circuitpython-busdevice`

---

## Why ExtendedI2C

The RPi’s standard I²C path (Blinka + `busio.I2C`) can hiccup under the BNO08x’s SHTP framing (long/fragmented packets), causing errors like *“Unprocessable Batch bytes”* or `Errno 5`.  
`adafruit_extended_bus.ExtendedI2C(1)` uses the Linux `I2C_RDWR` ioctl and is more tolerant of short reads and packet splits.

---

## Verifying the Backend in Use

On startup the node logs which backend it actually opened:

```
[IMU] I2C backend: ExtendedI2C (from .../adafruit_extended_bus.py)
[IMU] BNO08X backend: BNO08X_I2C (from .../adafruit_bno08x/i2c.py)
```

A one-time Blinka warning is normal on Linux:
```
RuntimeWarning: I2C frequency is not settable in python, ignoring!
```

---

## Topics & Parameters

**Published topics**
- `/imu/data` — `sensor_msgs/Imu`
- `/imu/euler` — `geometry_msgs/Vector3` (roll, pitch, yaw)
- `/imu/heading` — `std_msgs/String` (cardinal + degrees)
- `/imu/health_status` — `std_msgs/String`  
  Values include:  
  `IMU OK`, `IMU HICCUP …`, `IMU UNSTABLE (…)`, `IMU SOFT RESET`, `IMU STAGNANT`, `IMU RESTARTING …`, `IMU OFFLINE`

**Parameters**
- `poll_period_sec` (default **0.5**) — read/publish period  
- `stagnant_timeout_sec` (default **3.0**) — if data doesn’t change for this long, node restarts

---

## Initialization Strategy

1. Prefer **`ExtendedI2C(1)`**; fall back to `busio.I2C` if ExtendedI2C is missing.
2. Create `BNO08X_I2C(i2c, address=0x4B)`.
3. Enable features **one by one**, modest rate, **no batching**:
   - Accelerometer, Gyroscope, Rotation Vector.
4. The node **tries** newer `enable_feature(feature, report_interval=…, batch_interval=…)` and **falls back** to the legacy signature when unsupported.  
   You’ll see logs like:
   ```
   [IMU] enable_feature ok via attempt 4 (feature=0x1)
   ```
   which is expected with `adafruit-circuitpython-bno08x==1.2.10`.

---

## Runtime Behavior & Resilience

- **Data validation** before publish:
  - Quaternion ~ unit length (±0.2)
  - Accel magnitude within 5–20 m/s²
  - Gyro < ~35 rad/s (≈2000 °/s)  
  Fail → “Invalid IMU data” hiccup; publish skipped.

- **Hiccup classification & recovery**
  - On common I²C/SHTP errors (`Errno 5`, `Remote I/O`, “Unprocessable Batch bytes”), try a **soft I²C reset**:
    1. Close & reopen I²C
    2. Recreate `BNO08X_I2C`
    3. Re-enable features (no batching, moderate rate)
    4. On success, publish **`IMU SOFT RESET`** then **`IMU OK`**

- **Failure window & escalation**
  - Track failures in a 30 s sliding window.  
    ≥3 failures → soft reset (up to 3 attempts) → minimal init (Accel only) → process restart (exec self).

- **Stagnation watchdog**
  - If data is unchanged for `stagnant_timeout_sec`, publish **`IMU STAGNANT`** and restart.

- **Explicit health transitions**
  - After any recovery, the node actively publishes **`IMU OK`** so downstream nodes see the return-to-green.

---

## Clean Shutdown / OFFLINE

- On `Ctrl+C` or normal shutdown, the node publishes **`IMU OFFLINE`** once, then tears down publishers and exits cleanly.  
- The node now responds properly to SIGINT and avoids double shutdown of `rclpy`.

---

## Known / Expected Logs

- **“Unknown Report Type 0x7B/0x7D”** blocks can appear during mode transitions — handled by recovery logic.
- Sporadic read errors with numeric codes (e.g., 123/125/129/133/227/254/255/0) can still occur on marginal wiring or busy shared buses; the node soft-resets and announces status transitions.

---

## Operational Guidance

- Keep `poll_period_sec` conservative on long/loaded I²C busses (e.g., **0.2–0.5 s**). Higher rates increase bus pressure and error likelihood.
- Removing ExtendedI2C is supported (fallback to `busio`), but you may see more hiccups on the same hardware.
- We intentionally **did not** switch to a fork of the BNO08x driver. The public 1.2.10 release is stable and the node’s enable-feature logic adapts to its legacy signature.

---

## Quick Install Recap (fresh system)

```bash
# Use the same Python that runs your ROS 2 nodes
python3 -m pip install --user   adafruit-extended-bus   adafruit-circuitpython-bno08x   adafruit-blinka   adafruit-circuitpython-busdevice
```

---

## Quick Sanity Checks

```bash
# Start the node and verify backends & feature enables
ros2 run auv_pkg imu_node

# Watch health/status transitions
ros2 topic echo /imu/health_status

# Watch data
ros2 topic echo /imu/data
```

---

## Current Status (TL;DR)

- **ExtendedI2C** in use → more tolerant I²C on Linux.
- **Graceful recovery**: soft reset → minimal init → self-restart.
- **Health topic** reflects transitions, including **`IMU OFFLINE`** on shutdown.
- **Validation & throttling** guard against bad frames and bus overload.
