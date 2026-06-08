# BLS3355 Hip Servo — Electrical Reference & Safe Operating Guide

**Project:** UCR MEDDL Lab — Wheel-Legged Balancing Robot  
**Date:** June 2026 | **Servos:** 2× BLS3355 (Left hip ID 1 / Right hip ID 2)  
**Purpose:** Quick reference for demo day and to prevent another servo burn-out.

---

## 1. Key Specs (from product page)

| Parameter | Value |
| :--- | :--- |
| Operating voltage | **4.8 V – 8.4 V** (2S LiPo, **DO NOT exceed 8.4 V**) |
| Recommended voltage | **7.4 V** (2S LiPo nominal) |
| Stall torque @ 7.4 V | 55 kg·cm |
| Stall torque @ 8.4 V | 61 kg·cm |
| Stall current @ 7.4 V | **5.2 A per servo** |
| Stall current @ 8.4 V | 6.2 A per servo |
| No-load speed @ 7.4 V | 0.10 sec / 60° |
| Idle current (any voltage, not moving) | 5 mA per servo |
| Control signal | PWM, 500–2500 μs, 50 Hz |
| Neutral position | 1500 μs |
| Total range | 270° (500–2500 μs) |
| Weight | 85 g |
| IP rating | IP67 (waterproof) |

---

## 2. Power Supply Setup

| Item | Value |
| :--- | :--- |
| Battery | 12 V, 9600 mAh, **10 A max discharge** |
| Buck converter | DROK DC Buck, 12 V → **7.4 V**, 12 A rated |
| Converter setting | **Set to 7.4 V — do not set above 7.4 V** |
| Servo signal source | Arduino Uno R3 — D3 (left hip), D11 (right hip) |

> **Why 7.4 V and not 7.8–8.0 V?**  
> At 7.8–8.0 V both servos stalling simultaneously draws ~8.6 A from the battery  
> plus wheel motors + Pi ≈ **10.5–11 A total**, which exceeds the 10 A battery limit  
> and will trip the BMS. At 7.4 V the same scenario draws ~8–9 A — within the limit.

---

## 3. Current Draw by Operating Condition

**Torque constant @ 7.4 V:** 55 kg·cm ÷ 5.2 A = **10.6 kg·cm per amp**

Body mass supported per hip: 1.7 kg upper body ÷ 2 = **0.85 kg per servo**

### Per-servo current

| Condition | Current per servo | Notes |
| :--- | :--- | :--- |
| Idle (holding position, no gravity load) | **5 mA** | Robot powered but not standing |
| Holding upright stance (~5 cm moment arm) | **~0.4 A** | Normal balanced standing |
| Holding mid-crouch (~10 cm moment arm) | **~0.8 A** | Normal raised/lowered position |
| Holding deep crouch (~15 cm moment arm) | **~1.2 A** | Near lower travel limit |
| Actively repositioning (no external load) | **~0.8–1.5 A** | Moving between positions |
| **Stall (joint at hard limit)** | **5.2 A** | Maximum — avoid this |

### Both servos combined (what the buck converter sees)

| Scenario | Both servos | Notes |
| :--- | :--- | :--- |
| Idle / parked | ~10 mA | |
| Normal balancing and standing | **~1 A** | Typical demo condition |
| Hip raise / lower in progress | **~2–3 A** | Transient during movement |
| Both stalling simultaneously | **10.4 A** | Converter at limit — avoid |

### Full system current from 12 V battery (estimated)

| Component | Typical | Peak |
| :--- | :--- | :--- |
| BLS3355 servos (via 7.4 V buck) | 0.6 A | 7.1 A |
| JGB-520 wheel motors | 1–2 A | 4–5 A |
| Raspberry Pi 4 | 0.5–1 A | 1.5 A |
| Arduino Uno R3 | ~0.05 A | ~0.05 A |
| **Total** | **~2.2–4 A** | **~10 A** |

---

## 4. What Causes Stall — and How to Avoid It

**Stall** = servo motor is powered but cannot move. It draws maximum current continuously and will overheat and burn out if sustained for more than a few seconds.

### What triggers stall on this robot

| Trigger | How to avoid |
| :--- | :--- |
| Commanding past the physical joint limit | `HIP_MAX_OFFSET_US = 325` clamp in `robot_server.py` prevents this |
| Robot tipping and leg catching the ground | Keep `FALL_ANGLE = 35°` in firmware; add foam padding under chassis during testing |
| Two servos fighting each other (bad wiring) | Verify D3 = left (ID 1), D11 = right (ID 2) before powering |
| Voltage too low (servo can't overcome load) | Keep converter at 7.4 V; check battery charge before demos |
| Running 3S LiPo (12.6 V) directly to servos | **Never connect servo power directly to the main 12 V battery** |

### Warning signs before a burnout

- Servo getting hot to the touch (>50 °C)
- Robot jerking or oscillating at a joint
- Battery voltage sagging under load
- Burning smell

**If any of these occur: send ESTOP immediately (B / Circle on controller) and power off the servo supply.**

---

## 5. Wiring Quick-Reference

```
12 V Battery
    │
    ├──────────────────────────────► Arduino Uno R3 (via USB to Pi)
    │                                  D3  ──► Left hip servo signal
    │                                  D11 ──► Right hip servo signal
    │                                  GND ──► Servo ground (shared)
    │
    └──► DROK Buck Converter (set 7.4 V)
              │
              ├──► Servo RED wires  (7.4 V power)
              └──► Servo BLACK wires (GND, tied to Arduino GND)
```

> **Critical:** Servo signal wires (white/yellow) go to Arduino D3/D11.  
> Servo power (red) comes from the DROK converter output, **NOT** the Arduino 5 V pin.  
> Arduino GND and converter GND must be connected together (common ground).

---

## 6. Pulse Width ↔ Angle Reference

| Pulse width | Angle (from CCW limit) | Angle from neutral |
| :--- | :--- | :--- |
| 500 μs | 0° | −135° |
| 1000 μs | 67.5° | −67.5° |
| **1500 μs** | **135° (neutral)** | **0°** |
| 2000 μs | 202.5° | +67.5° |
| 2500 μs | 270° | +135° |

**Formula:**  `angle_from_neutral_deg = (pulse_us − 1500) × (270 / 2000)`

---

## 7. Software Limits (configured in `robot_server.py`)

| Parameter | Value | Meaning |
| :--- | :--- | :--- |
| `HIP_STEP_US` | 20 μs | ~2.7° per button press |
| `HIP_MAX_OFFSET_US` | 325 μs | ±43.9° max travel from default |
| `HIP_RAISE_DIR` | 1 | Flip to −1 if raise/lower is reversed |
| Default pulse (config) | 1500 μs | Neutral position until calibrated |

Update `config/robot.yaml` → `bls.left.default_pos` and `bls.right.default_pos` after physical calibration.  
Use `python3 scripts/default_stance.py` to send servos to their configured default.

---

## 8. Pre-Demo Checklist

- [ ] DROK converter output verified at **7.4 V** with a multimeter before connecting servos
- [ ] Servo power (red/black) connected to converter output — **not** Arduino or Pi pins
- [ ] D3 → left servo signal, D11 → right servo signal, common GND connected
- [ ] Arduino firmware flashed with Timer 2 ISR servo code (current `robot_control.ino`)
- [ ] `default_pos` calibrated in `config/robot.yaml` and tested with `default_stance.py`
- [ ] Hip travel tested: confirm raise/lower moves the correct direction; flip `HIP_RAISE_DIR` if not
- [ ] Battery charged; voltage checked (should read 11.5–12.6 V under no load)
- [ ] ESTOP tested: B / Circle on controller immediately stops motors and returns hips to neutral
