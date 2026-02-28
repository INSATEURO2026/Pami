# SIMA (Ninja) Robot — Code Specification

## Table of Contents

1. [How to Test](#how-to-test)
   - [Step 1: Encoder & Odometry Tuning (encoder_tuning.ino)](#step-1-encoder--odometry-tuning-separate-script--no-motor-movement)
   - [Step 2: Single-Waypoint Test](#step-2-single-waypoint-test-lift-the-robot)
   - [Step 3: On-Ground Single Waypoint](#step-3-on-ground-single-waypoint)
   - [Step 4: Full Waypoint Sequence](#step-4-full-waypoint-sequence)
   - [Step 5: Stepper & Pump](#step-5-stepper--pump)
2. [Code Explanation](#code-explanation)
   - [Includes & Libraries](#1-includes--libraries)
   - [Pin Definitions](#2-pin-definitions)
   - [Tuning Constants](#3-tuning-constants)
   - [Wheel / Encoder Parameters](#4-wheel--encoder-physical-parameters)
   - [Waypoints](#5-waypoints)
   - [Position & Error Variables](#6-position--error-variables)
   - [Encoder & Trapezoid Variables](#7-encoder--trapezoid-variables)
   - [PID Controllers](#8-pid-controllers)
   - [4x Quadrature Encoder ISRs](#9-4x-quadrature-encoder-isrs)
   - [Utility Functions](#10-utility-functions)
   - [Setup](#11-setup)
   - [State Machine](#12-state-machine)
   - [Motor Functions](#13-motor-functions)
   - [Odometry](#14-odometry--update_coordinates)
   - [Error Calculation](#15-error-calculation--calcul_erreur)
   - [Trapezoid Profile](#16-trapezoid-profile--trapeze)
   - [Pump Control](#17-pump-control)
3. [Encoder Tuning Script Reference](#encoder-tuning-script-reference)

---

## How to Test

### Step 1: Encoder & Odometry Tuning (Separate Script — No Motor Movement)

Since the encoders and motors share the same power rail, the motors turn on whenever the board is powered. A **dedicated tuning script** (`encoder_tuning.ino`) solves this: it immediately brakes all motors at startup and keeps them braked, so you can push the robot by hand and read encoder data safely.

#### 1a. Flash the Tuning Script

1. Open `encoder_tuning.ino` in Arduino IDE / PlatformIO (it lives in the same folder).
2. Flash it to the ESP32.
3. Open Serial Monitor at **115200 baud**.
4. You should see:

```
==============================================
  SIMA Encoder Tuning & Odometry Test
  Motors are BRAKED — push the robot by hand
==============================================
Commands: r=reset, d=raw counts, c=constants, h=help
```

5. The motors will be **braked** (both H-bridge direction pins LOW, PWM = 0). The wheels should resist when you try to spin them — that's the brake. But you can still push the robot on the ground.

#### 1b. Verify Encoder Direction

1. Type `r` to reset odometry.
2. Push the robot **straight forward** by hand.
3. `x` should **increase**. If it decreases, swap the A/B wires on **both** encoders.
4. Spin only the **right** wheel forward → `teta` should decrease (robot turns right). If it goes the wrong way, swap the A/B wires on the right encoder.
5. Spin only the **left** wheel forward → `teta` should increase (robot turns left). If wrong, swap left encoder wires.
6. Type `d` at any time to see raw encoder tick counts and verify both channels are counting.

#### 1c. Calibrate Linear Distance (ENCODER_COEFFICIENT)

1. Type `r` to reset.
2. Mark a starting point on the floor.
3. Push the robot **exactly 1000 mm** (1 meter) forward in a straight line.
4. Read `x` from the Serial output. It should say ~1000.
5. If it reads a different value (e.g. 950), compute the corrected coefficient:

$$\text{new\_coeff} = \text{old\_coeff} \times \frac{1000}{\text{measured\_x}}$$

6. Edit `ENCODER_COEFFICIENT` in **both** `encoder_tuning.ino` and `ninjaavecreverse.ino`, reflash the tuning script, and repeat until `x` reads ~1000.

#### 1d. Calibrate Rotation (ALPHA_CORRECTION)

1. Type `r` to reset.
2. Rotate the robot **exactly 360°** on the spot (pick a visual reference, end where you started).
3. Read `teta` — it should return to ~0°.
   - If `teta` drifts **positive** (e.g. +5°), the correction factor is too high → **decrease** `ALPHA_CORRECTION`.
   - If `teta` drifts **negative** (e.g. −5°), the factor is too low → **increase** `ALPHA_CORRECTION`.
4. Repeat until a full 360° spin returns `teta` to within ±2°.

#### 1e. Verify With Shapes

For extra confidence:

- Push the robot in a **1 m square** (forward 1m, turn 90°, repeat 4 times). After completing the square, `x` and `y` should both be close to 0.
- Push the robot in a **straight line and back**. `x` should return to ~0.

#### Serial Commands Reference

| Command | Action |
|---------|--------|
| `r` | Reset odometry to (0, 0, 0) |
| `d` | Print raw encoder counts (ticks) |
| `c` | Print current calibration constants |
| `h` | Print help |

### Step 2: Single-Waypoint Test (Lift the Robot)

> **From this step onward, flash `ninjaavecreverse.ino` (the main code) back onto the ESP32.**

1. **Lift the robot** so wheels spin in the air.
2. Change the code to just 1 waypoint: `NB_POINTS = 1`, `path_x = {300}`, `path_y = {0}`.
3. Flash and observe:
   - SETPATH prints, then after 500ms → ROTATE.
   - Wheels spin in opposite directions (robot tries to aim).
   - Once Cap1 < 0.03 → FORWARD.
   - Wheels spin same direction (tries to go forward 300mm).
   - Since wheels are in the air, the encoders will run forever — that's OK, you're verifying state transitions happen.

### Step 3: On-Ground Single Waypoint

1. Place robot on the floor.
2. Use a simple target: `path_x = {300}`, `path_y = {0}` (go straight 300mm forward).
3. Flash, pull the start cord, and watch. The robot should:
   - Briefly rotate (already facing forward, so very short).
   - Drive forward ~30 cm.
   - Stop at CHECKPOINT → FINISH.
4. Measure with a ruler. It should be within ~2cm of 300mm.

### Step 4: Full Waypoint Sequence

Once single-point works, restore all 5 waypoints and test the full sequence. Watch Serial throughout. Key things to monitor:

- `Dist` should decrease during FORWARD/BACKWARD.
- `dist_p` and `SPdist` should be close to each other.
- `teta` during ROTATE should converge to `SPAngle`.

### Step 5: Stepper & Pump

- At waypoint 4 (`currentPoint == 4`), the pump turns ON and the stepper runs. Verify the mechanical arm/gripper actuates correctly.
- At FINISH, the pump turns OFF. Verify suction releases.

### Troubleshooting Serial Cheat Sheet

| Serial message | Meaning |
|---|---|
| `SETPATH pt=N target=(X,Y) dist=D` | Loading waypoint N, target is (X,Y), D mm away |
| `ROTATE SPa=... teta=... Cap1=...` | Rotating: SPa = desired heading, teta = current, Cap1 = error |
| `ROTATE → FORWARD` | Heading aligned, starting to drive |
| `FWD x=... y=... Dist=... dist_p=... SPd=...` | Driving: position, remaining distance, progress, reference |
| `FWD arrived → CHECKPOINT` | Reached the waypoint |
| `REVERSE ...` | Driving backward (waypoint 2) |
| `Trajet termine!` | All 5 waypoints done |

---

## Code Explanation

### 1. Includes & Libraries

**File location: Lines 33–35**

```cpp
#include <Wire.h>       // I²C bus (reserved for future IMU)
#include <math.h>       // sin(), cos(), sqrt(), atan2(), M_PI
#include <PID_v1.h>     // Brett Beauregard's PID library
```

---

### 2. Pin Definitions

**File location: Lines 38–61**

| Pin | Variable | Connected to |
|-----|----------|-------------|
| 18, 17 | `ENCODER_RA`, `ENCODER_RB` | Right wheel encoder channels A and B |
| 16, 15 | `ENCODER_LA`, `ENCODER_LB` | Left wheel encoder channels A and B |
| 9, 46 | `IN1`, `IN2` | Left motor H-bridge direction inputs |
| 8 | `ENA` | Left motor PWM speed pin |
| 10, 12 | `IN3`, `IN4` | Right motor H-bridge direction inputs |
| 6 | `ENB` | Right motor PWM speed pin |
| 35, 36, 37 | `P1`, `P2`, `ENP` | Pump motor: direction + enable |
| 2 | `LED_PIN` | Onboard LED for visual debugging |
| 20, 19 | `dirPin`, `stepPin` | Stepper motor direction + step pulse |

- **`stepsPerRevolution = 200`**: The stepper motor has 200 steps per full turn (1.8° per step), standard for NEMA-style steppers.

---

### 3. Tuning Constants

**File location: Lines 64–90**

| Constant | Value | Meaning |
|----------|-------|---------|
| `PWM_MAX` | 200 | Maximum PWM sent to motors (out of 255). Capped below 255 to protect hardware. |
| `PWM_STOP_DEADBAND` | 5 | If the PID asks for a command smaller than ±5, send 0 PWM instead (avoids motor buzzing). |
| `PWM_MIN` | 65 (or 75 with pump) | The minimum PWM that actually makes the motor turn. Below this, the motor stalls. |
| `V_MAX` | 600.0 mm/s | Maximum cruise speed during straight-line segments. |
| `ACC_MAX` | 300.0 mm/s² | How fast the robot ramps UP in speed. |
| `DEC_MAX` | 400.0 mm/s² | How fast the robot ramps DOWN. Higher than ACC so it stops before overshooting. |
| `SAMPLING` | 5 ms | The main loop runs the control code every 5 ms (200 Hz). |
| `ANGLE_THRESH_ROTATE` | 0.03 rad (~1.7°) | In ROTATE state: if the heading error is below this, the robot is "aimed" and switches to FORWARD. |
| `ANGLE_THRESH_ROTAFT` | 0.02 rad (~1.1°) | In ROTATEAFTER: if final heading error is this small, stop. |
| `DIST_ARRIVE_THRESH` | 10.0 mm | If the Euclidean distance to the target is below 10mm, consider arrived. |
| `DIST_REF_MARGIN` | 20.0 mm | Secondary arrival check: if the trapeze reference distance is within 20mm of `Target_dist`, also consider arrived. |
| `ALPHA_CORRECTION` | 1.05 | Empirical multiplier for the rotation term in odometry. Compensates for mechanical imperfections (wheel diameter mismatch, play). |
| `SETPATH_DELAY_MS` | 500 ms | Pause between loading a new waypoint and starting to move (lets the system settle). |
| `STOP_DELAY_MS` | 100 ms | Pause after stopping at a waypoint before loading the next. |

---

### 4. Wheel / Encoder Physical Parameters

**File location: Lines 93–100**

| Constant | Value | Meaning |
|----------|-------|---------|
| `WHEEL_PERIMETER` | 138.23 mm | Circumference of the driving wheels: C = π × d. |
| `RESOLUTION` | 1400 | Encoder counts per wheel revolution in 4x decoding mode (350 CPR × 4 = 1400). |
| `ENCODER_COEFFICIENT` | 0.098736 mm/count | Distance the wheel travels per encoder tick: 138.23 / 1400. |
| `WHEEL_BASE` | 119 mm | Distance between the two wheel contact points. Used to convert differential wheel travel into rotation angle: α = (ΔR − ΔL) / WHEEL_BASE. |
| `wheel_gain` | 1.063 | The left motor is ~6.3% weaker than the right. This multiplier is applied to the left PWM to compensate, so both wheels produce equal torque. |

---

### 5. Waypoints

**File location: Lines 103–109**

```cpp
path_x[5]    = { 300,  200, -150, 100, -500 }
path_y[5]    = {   0,  200, -150,   0,    0 }
path_teta[5] = { 1.57, 0.0, 0.0, 0.0, 0.0 }
```

Each waypoint is **relative to the robot's position when it starts that segment** (because `x,y` are reset to 0 in SETPATH).

| Point | (x, y) mm | Description |
|-------|-----------|-------------|
| 0 | (300, 0) | Go straight forward 300mm. Final heading target: 1.57 rad (90° = face left). |
| 1 | (200, 200) | Go diagonally (45°) ~283mm. |
| 2 | (-150, -150) | **BACKWARD** segment. Target is behind & below → robot reverses ~212mm. |
| 3 | (100, 0) | Straight forward 100mm. |
| 4 | (-500, 0) | Straight forward along −X, 500mm. |

**Variables:**

- **`currentPoint`**: Index (0–4) of which waypoint the robot is currently navigating toward.
- **`NB_POINTS`**: Total number of waypoints (5).

---

### 6. Position & Error Variables

**File location: Lines 112–138**

| Variable | Type | Meaning |
|----------|------|---------|
| `x`, `y` | double | Robot's estimated position in mm (relative to segment start, reset for each waypoint). |
| `teta` | double | Robot's heading in radians. 0 = facing +X. Positive = counter-clockwise. Range [−π, +π]. |
| `x_prev`, `y_prev`, `teta_prev` | double | Position/heading from the **previous** loop cycle. Used to compute deltas. |
| `linearVel` | double | Instantaneous forward speed in mm/s, computed from wheel travel / dt. |
| `angularVel` | double | Angular velocity (reserved, not currently used). |
| `xc`, `yc` | double | Current target waypoint coordinates (loaded from `path_x[currentPoint]`, `path_y[currentPoint]`). |
| `tetac` | double | Desired final heading after reaching the waypoint (loaded from `path_teta[currentPoint]`). |
| `x_err`, `y_err` | double | Position error: `xc - x` and `yc - y`. |
| `teta_err` | double | Heading error (reserved, not currently computed). |
| `Dist` | double | Euclidean distance from robot to target: sqrt(x_err² + y_err²) in mm. |
| `bearing` | double | Angle from *current position* to target: `atan2(y_err, x_err)`. Updated dynamically during motion. |
| `Cap` | double | Bearing error = `bearing - teta`. How far off-heading the robot is *during forward driving*. |
| `Cap1` | double | Angle from *origin* to target minus current heading. Used in ROTATE state to decide when the initial rotation is done. |
| `Cap2` | double | Heading error for ROTATEAFTER: `tetac - teta`. How far the robot's heading is from the desired final orientation. |
| `SPAngle` | double | Setpoint for the angle PID (the desired heading at this instant). |
| `SPdist` | double | Setpoint for the distance reference (the desired distance-traveled at this instant). |
| `measuredVel` | double | Measured velocity (reserved, not currently used). |
| `pumpOn` | bool | Whether the pump is currently active. |

---

### 7. Encoder & Trapezoid Variables

**File location: Lines 144–155**

| Variable | Type | Meaning |
|----------|------|---------|
| `encoderCount_R` | volatile long | Raw cumulative tick count from the right encoder ISR. `volatile` because ISRs modify it at any time. |
| `encoderCount_L` | volatile long | Raw cumulative tick count from the left encoder ISR. |
| `rampUp` | double | The distance (mm) needed to accelerate from 0 to `V_MAX`: V_MAX² / (2 × ACC_MAX) = 600² / 600 = 600 mm. |
| `rampDown` | double | Distance to decelerate from `V_MAX` to 0: V_MAX² / (2 × DEC_MAX) = 600² / 800 = 450 mm. |
| `vitesse` | double | The **current reference velocity** in mm/s. The trapeze profile ramps this up and down. |
| `Target_dist` | double | The total straight-line distance for the current segment: sqrt(xc² + yc²). Doesn't change until the next waypoint. |
| `remaining_dist` | double | How many mm are left along the segment direction. Used by the trapeze to decide accel/cruise/decel. |
| `dt` | double | Time elapsed since the last loop iteration, in seconds. Typically ~0.005 s (5ms). |
| `dist_p` | double | **How far the robot has progressed** along the intended segment, in mm. Computed as the dot-product projection of position onto the segment vector. If the robot drifts sideways, this only measures the useful forward component. |
| `dist_ref` | double | The **reference distance** generated by integrating `vitesse × dt` each cycle. The PID tries to make `dist_p` track `dist_ref`. This is what makes the robot follow the trapeze speed profile. |
| `leftCmd`, `rightCmd` | int | Final signed motor commands (−255 to +255) sent to `driveMotors()`. |
| `stateTimerStart` | unsigned long | Timestamp (ms) when the current non-blocking delay started. |
| `stateTimerActive` | bool | Whether a non-blocking delay is currently in progress. |

---

### 8. PID Controllers

**File location: Lines 158–178**

There are **4 PID controllers**, each as a triplet of `(input, output, setpoint)`:

| PID | Input (measurement) | Setpoint (desired) | Output range | Purpose |
|-----|--------------------|--------------------|--------------|---------|
| `PID_angle` | `teta` (current heading) | Desired heading | −200 to +200 | Pure rotation (ROTATE, ROTATEAFTER states). Output is differential motor command. |
| `PID_angleF` | `teta` (current heading) | `bearing` or backward heading | −200 to +200 | Heading **correction** while driving forward or backward. Lower gains (Kp=9 vs 20) for subtle steering. |
| `PID_dist` | `dist_p` (progress along segment) | `dist_ref` (trapeze reference) | −200 to +200 | Forward driving speed. Output = base motor speed command. |
| `PID_distB` | `dist_p` (progress) | `dist_ref` | −200 to +200 | Backward driving speed. Separate PID with softer gains (Kp=0.3) because reverse is trickier. |

**PID Gains:**

| Controller | Kp | Ki | Kd | Notes |
|------------|----|----|-----|-------|
| `PID_angle` | 20 | 0 | 1 | Aggressive for fast rotation. Kd=1 damps overshoot. |
| `PID_angleF` | 9 | 0 | 0 | Gentle steering during motion. No derivative. |
| `PID_dist` | 5 | 0 | 0 | Normal forward. Kp=12 in pump mode. |
| `PID_distB` | 0.3 | 0 | 0 | Very gentle for reverse. |

Gain definitions:

- **Kp** (Proportional): Output proportional to error. Higher = more aggressive.
- **Ki** (Integral): Output proportional to accumulated error over time. All zeros here = no integral action, which avoids windup.
- **Kd** (Derivative): Output proportional to rate-of-change of error. `KdAngle=1` adds damping to rotation, preventing overshoot.

---

### 9. 4x Quadrature Encoder ISRs

**File location: Lines 183–196**

A rotary encoder has two channels (A and B) that produce square waves 90° out of phase. With **4x decoding**, we trigger on every edge (rising AND falling) of both channels:

```
      ┌──┐  ┌──┐  ┌──┐
  A:  │  │  │  │  │  │    ← 1 period = 1 original "tick"
  ────┘  └──┘  └──┘  └──
         ┌──┐  ┌──┐
  B:     │  │  │  │       ← shifted 90° from A
  ───────┘  └──┘  └──
      ↑  ↑  ↑  ↑             ← 4 edges per period (4x resolution)
```

**Direction logic:**

- **Channel A edge**: if A and B are the **same** level → forward (+1), else → backward (−1).
- **Channel B edge**: if A and B are the **same** level → backward (−1), else → forward (+1).

This gives 4 counts per encoder cycle. The 350 CPR encoder now gives **1400 counts per revolution = ~0.099 mm per count** (sub-millimeter precision).

`IRAM_ATTR` places the ISR in ESP32 instruction RAM for fastest possible execution.

---

### 10. Utility Functions

**File location: Lines 201–231**

#### `normalizeAngle(a)`

Wraps any angle into [−π, +π]. Example: 3.5 rad becomes 3.5 − 2π ≈ −2.78 rad. This prevents the PID from seeing a huge error when crossing the ±180° boundary.

#### `pwmFromSigned(val)`

Converts a signed PID output (−255 to +255) to unsigned PWM:

- If `|val| < 5` → return 0 (stop motor, don't buzz).
- Otherwise → map 0–255 to `PWM_MIN`–`PWM_MAX` (65–200). This ensures the motor always gets enough voltage to actually turn.

#### `updatePIDGains()`

Switches `PID_dist` between normal (Kp=5) and pump-loaded (Kp=12) gains depending on `pumpOn`.

#### `resetPID(pid)`

Toggles MANUAL → AUTOMATIC, which the PID_v1 library uses to clear the accumulated integral term. Called at every state transition to prevent "windup" (where the I-term builds up during one state and causes overshoot in the next).

---

### 11. Setup

**File location: Lines 237–298**

Runs once at power-on:

1. **Serial** at 115200 baud for debug output.
2. **Pin modes**: encoders as `INPUT_PULLUP`, motors/pump/stepper/LED as `OUTPUT`.
3. **Pump off**: all pump pins LOW.
4. **Attach interrupts**: all 4 encoder channels on `CHANGE` (4x quadrature).
5. **Precompute** `rampUp` and `rampDown` (constant values used in trapeze).
6. **PID setup**: all 4 PIDs get output limits (−200, +200), sample time (10ms), and are set to `AUTOMATIC` mode.

---

### 12. State Machine

**File location: Lines 303–625**

The robot's brain is a state machine with 9 states:

```
SETPATH → ROTATE → FORWARD → CHECKPOINT → STOP → SETPATH → ...
                          ↘ BACKWORD → CHECKPOINT
                                         ↓ (at pt 4)
                                       STEPPER → STOP
                                         ↓ (at pt 5)
                                       FINISH (stays forever)
```

#### SETPATH (Load a Waypoint)

**Lines 329–382**

This state prepares for a new navigation segment:

1. Reset `x, y, x_prev, y_prev` to 0 (waypoints are relative).
2. Reset `dist_p = 0` (no progress yet) and `dist_ref = 0` (trapeze reference starts at 0).
3. Call `Pompe()` to ensure the pump is ON.
4. Load `xc, yc, tetac` from the waypoint arrays.
5. Compute `Target_dist = sqrt(xc² + yc²)` — straight-line distance to target.
6. Reset all 4 PIDs (clear integral windup).
7. Start a 500ms non-blocking timer. When it expires → go to ROTATE (or BACKWORD for point 2).

#### ROTATE (Face the Target)

**Lines 385–413**

The robot turns on the spot to face the waypoint:

1. **`SPAngle = atan2(yc, xc)`**: The desired heading = angle from origin (0,0) to the target.
2. Feed `SPAngle` as setpoint and `teta` as input to `PID_angle`.
3. `PID_angle.Compute()` outputs a correction value.
4. Drive motors with `(-cmd, +cmd)` — left goes backwards, right goes forwards → robot turns counter-clockwise (or reversed if cmd is negative).
5. When `|Cap1| < 0.03 rad` (≈1.7°) → heading is aligned, switch to FORWARD.

#### BACKWORD (Drive in Reverse)

**Lines 416–451**

Used for waypoint 2 (target is behind the starting position):

1. **Trapeze()** computes current reference velocity.
2. `dist_ref += vitesse × dt` — integrate the velocity reference over time to get the distance reference.
3. **Distance PID**: setpoint = `dist_ref`, input = `dist_p` → output = how much PWM to apply.
4. **Heading PID**: setpoint = `atan2(yc,xc) + PI` — add π because the robot's **back** should face the target.
5. `distCmd` is **negated** (driving backwards).
6. Motor commands: `leftCmd = distCmd − angleCorrection`, `rightCmd = distCmd + angleCorrection`.
7. Arrival: when `Dist < 10mm` or `dist_ref` is close to `Target_dist`.

#### FORWARD (Drive Forward)

**Lines 454–497**

The main driving state:

1. **Trapeze()** sets `vitesse` (current speed) based on where the robot is in the accel/cruise/decel profile.
2. `dist_ref += vitesse × dt` — the reference distance grows as the trapeze dictates.
3. **Distance PID**: makes `dist_p` (measured progress) track `dist_ref` (desired progress). Output = forward PWM.
4. **Heading PID**: `bearing = atan2(y_err, x_err)` — the angle from the *current position* to the target. `PID_angleF` keeps the robot aimed at the target even if it drifts.
5. `leftCmd = distPWM − anglePWM`, `rightCmd = distPWM + anglePWM`. If the robot veers right, `anglePWM` becomes positive, so left gets less and right gets more → corrects back to the left.
6. Two arrival checks: Euclidean distance < 10mm, OR reference distance within 20mm of target.

#### ROTATEAFTER (Final Heading Adjustment)

**Lines 500–514**

After arriving, optionally rotate to the desired final heading `tetac`. Uses `PID_angle` the same as ROTATE but with `tetac` instead of `atan2(yc,xc)`. Exits when `|Cap2| < 0.02 rad`.

> **Note:** ROTATEAFTER is never entered in the current flow because CHECKPOINT goes to STOP. It exists for future use if you want final heading control.

#### CHECKPOINT (Next Waypoint Decision)

**Lines 517–533**

1. `stopmotors()` — hard brake.
2. `currentPoint++` — advance to the next waypoint.
3. Decision tree:
   - Point 4 → turn on pump, run STEPPER.
   - Point ≥ 5 → all done → FINISH.
   - Otherwise → STOP (brief pause before next waypoint).

#### STEPPER (Actuate Gripper/Arm)

**Lines 536–560**

Drives a stepper motor (likely a mechanical arm/gripper):

- **Phase 1**: 4 full revolutions clockwise at slow speed (4000 µs per step = 250 steps/sec).
- 1-second pause.
- **Phase 2**: 4 full revolutions counter-clockwise at double speed.
- 1-second pause.
- Then → STOP.

The `break` after this case is critical — without it, execution would "fall through" into the STOP case immediately.

#### STOP (Brief Pause)

**Lines 563–574**

Non-blocking 100ms pause:

1. First entry: call `stopmotors()`, record `stateTimerStart`.
2. Each subsequent loop: check if 100ms has passed.
3. When done → SETPATH (load next waypoint).

#### FINISH (Done)

**Lines 577–583**

All waypoints complete. Calls `stopmotors()` and `PompeOff()` every loop. The robot stays here forever. LED stays ON as a visual indicator.

---

### 13. Motor Functions

**File location: Lines 589–641**

#### `driveMotors(leftPWM, rightPWM)`

- Clamps each command to −255..+255.
- Positive → `IN1=HIGH, IN2=LOW` (forward). Negative → `IN1=LOW, IN2=HIGH` (reverse). Zero → both LOW (brake).
- Left motor PWM is multiplied by `wheel_gain = 1.063` to compensate for its weaker performance.
- `pwmFromSigned()` converts the command magnitude to an actual PWM value above the motor's minimum threshold.

#### `stopmotors()`

All direction pins LOW, both PWM pins to 0. Hard stop.

---

### 14. Odometry — `Update_coordinates()`

**File location: Lines 647–688**

This is the core — it answers **"where am I?"** every 5ms.

1. **Atomic read**: `noInterrupts()` → snapshot both encoder counts → `interrupts()`. This prevents one count being read before an ISR fires and the other after.

2. **Convert to mm**: `encR = snapR × 0.098736`. `encL = snapL × 0.098736`.

3. **Compute deltas**: `deltaR = encR − encR_prev` (how far the right wheel moved since last cycle). Same for left.

4. **Forward displacement**:

$$D = \frac{\Delta R + \Delta L}{2}$$

Average of both wheels. If both go forward 1mm, the robot moved forward 1mm.

5. **Rotation**:

$$\alpha = 1.05 \times \frac{\Delta R - \Delta L}{\text{WHEEL\_BASE}}$$

If the right wheel goes further than the left, the robot turned left (positive angle). The 1.05 corrects for real-world imperfections.

6. **Position update** (mid-angle integration):

$$\theta_{mid} = \theta_{prev} + \frac{\alpha}{2}$$

$$x = x_{prev} + D \cdot \cos(\theta_{mid})$$

$$y = y_{prev} + D \cdot \sin(\theta_{mid})$$

Using θ_mid instead of θ_prev is a second-order approximation that's more accurate when the robot is turning and moving simultaneously.

7. **Segment projection** (enhancement):

$$dist\_p = \frac{x \cdot xc + y \cdot yc}{\text{Target\_dist}}$$

This is the dot product of the position vector with the unit segment vector. It measures how far the robot has progressed **along the intended path**, ignoring lateral drift. If the robot is at (95, 10) heading to (100, 0), the progress is 95mm not sqrt(95² + 10²) = 95.5mm.

---

### 15. Error Calculation — `Calcul_erreur()`

**File location: Lines 690–708**

| Variable | Formula | Meaning |
|----------|---------|---------|
| `x_err` | `xc − x` | How far to go in X |
| `y_err` | `yc − y` | How far to go in Y |
| `Dist` | sqrt(x_err² + y_err²) | Straight-line distance remaining |
| `bearing` | `atan2(y_err, x_err)` | Absolute angle from robot to target |
| `Cap1` | `atan2(yc, xc) − teta` | Angle error for initial rotation (from origin, not current pos) |
| `Cap` | `bearing − teta` | Heading error during driving (from current pos) |
| `Cap2` | `tetac − teta` | Heading error for ROTATEAFTER |

---

### 16. Trapezoid Profile — `Trapeze()`

**File location: Lines 714–778**

Controls **how fast** the robot should be going at each moment. Think of it as a velocity plan:

```
velocity
  ↑
V_MAX ─ ─ ─ ─ ─ ─ ─┐─ ─ ─ ─ ─ ─ ─┐
                   / │               │ \
                  /  │  CRUISE       │  \
                 /   │               │   \
                /    │               │    \
  0 ──────────/─────┤───────────────┤─────\──────→ distance
           ACCEL   rampUp         rampDown  DECEL
```

**Key variables:**

- **`Vpeak`**: If the segment is too short to reach `V_MAX`, compute the peak velocity of a triangular profile:

$$V_{peak} = \sqrt{\frac{2 \cdot ACC \cdot DEC \cdot D}{ACC + DEC}}$$

- **TRIANGULAR mode**: The robot accelerates to `Vpeak` then immediately decelerates — never reaches cruise speed.
- **TRAPEZOID mode**: Three phases:
  - `TRAP_ACCEL`: `vitesse += ACC_MAX × dt` until reaching `V_MAX`.
  - `TRAP_CRUISE`: Hold `vitesse = V_MAX` until `remaining_dist ≤ rampDown`.
  - `TRAP_DECEL`: `vitesse -= DEC_MAX × dt` until reaching 0.

`remaining_dist` is computed as `Target_dist − dist_p` (segment-projected), making it consistent with the PID's view of progress.

---

### 17. Pump Control

**File location: Lines 784–812**

#### `Pompe()`

Turns pump ON: sets `P1` HIGH, `P2` LOW, `ENP` HIGH. Also raises `PWM_MIN` to 75 (the pump adds mechanical load, so the motors need more voltage to overcome friction). **Idempotent** — calling it when already ON does nothing.

#### `PompeOff()`

Turns pump OFF, resets `PWM_MIN` to 65, reverts PID gains. Separate function (the original had dead code where the OFF branch was unreachable).

---

## Summary: Mental Model

The key mental model is:

**SETPATH → ROTATE → FORWARD → CHECKPOINT**, repeated for each waypoint, with:

- The **trapeze** controlling speed (smooth accel/cruise/decel).
- **PID controllers** controlling heading and distance.
- **Odometry** tracking where the robot is.
- **State machine** orchestrating the sequence.

---

## Encoder Tuning Script Reference

**File:** `encoder_tuning.ino` (same folder as `ninjaavecreverse.ino`)

### Why It Exists

The encoder power and motor power share the same rail on this board. That means you **cannot** read encoders without powering the motor driver. The tuning script works around this by immediately setting all H-bridge direction pins LOW and PWM to 0 — this puts the motors in **brake mode** (short-circuit across the motor terminals). The wheels resist turning slightly, but you can still push the robot on the ground.

The script re-asserts `analogWrite(ENA, 0)` and `analogWrite(ENB, 0)` every loop iteration as a safety measure.

### What It Does

| Feature | Detail |
|---------|--------|
| Motor state | All motors braked (direction LOW, PWM 0). Reasserted every loop. |
| Pump / Stepper | Forced OFF at startup. |
| Encoder decoding | 4x quadrature (same ISRs as main code). |
| Odometry | Computed at 200 Hz using mid-angle integration (identical algorithm to main code). |
| Serial output | Prints `x`, `y`, `teta` (degrees + radians), `dist`, `vel` every 50ms — but only when the robot moves (avoids flooding). |
| Interactive commands | `r` = reset, `d` = raw counts, `c` = constants, `h` = help. |

### Calibration Constants to Edit

These constants appear at the top of `encoder_tuning.ino`. After tuning, copy the final values into `ninjaavecreverse.ino` as well.

| Constant | Default | What to adjust |
|----------|---------|----------------|
| `ENCODER_COEFFICIENT` | 0.098736 | If `x` reads wrong after a 1m push |
| `ALPHA_CORRECTION` | 1.05 | If `teta` drifts after a full 360° spin |
| `WHEEL_BASE` | 119.0 mm | If rotation is consistently wrong even after ALPHA tuning |
| `WHEEL_PERIMETER` | 138.23 mm | If you change wheels |
