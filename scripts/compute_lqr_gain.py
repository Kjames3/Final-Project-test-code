#!/usr/bin/env python3
"""
Offline LQR gain solver for the wheel-legged balancer.

Builds the linearized inverted-pendulum-on-wheels plant from the physical
parameters in config/robot.yaml, solves the continuous-time algebraic Riccati
equation, and writes the resulting state-feedback gain K = [Kx, Kv, Kp, Kd]
(plus the torque→PWM scalar and balance offset) to config/lqr_gains.json.

This is DESIGN-TIME code — it runs once on the laptop whenever the robot's
mass/geometry changes. It is NOT needed on the Pi or Arduino at run time: the
robot only ever uses the four resulting numbers. Dependencies are numpy +
(optionally) scipy; a numpy-only Riccati fallback is included so the script
runs even on a stripped-down install.

Usage:
    python scripts/compute_lqr_gain.py
"""

import os
import json
import shutil

import numpy as np

import _bootstrap  # noqa: F401  (adds project root to sys.path)
from src.utils.config import load_config

G = 9.81  # gravity, m/s²

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
GAINS_FILE = os.path.join(ROOT_DIR, "config", "lqr_gains.json")

DEG2RAD = np.pi / 180.0


# ── Riccati solver ────────────────────────────────────────────────────────
def solve_care(A, B, Q, R):
    """Solve AᵀP + PA − PBR⁻¹BᵀP + Q = 0 for the stabilizing P.

    Prefers scipy; falls back to the Hamiltonian eigenvector method (numpy
    only) so the script has no hard scipy dependency.
    """
    try:
        from scipy.linalg import solve_continuous_are
        return solve_continuous_are(A, B, Q, R)
    except ImportError:
        n = A.shape[0]
        Ri = np.linalg.inv(R)
        H = np.block([[A, -B @ Ri @ B.T],
                      [-Q, -A.T]])
        w, v = np.linalg.eig(H)
        stable = np.argsort(w.real)[:n]      # n eigenvalues with most-negative real part
        U = v[:, stable]
        P = np.real(U[n:, :] @ np.linalg.inv(U[:n, :]))
        return 0.5 * (P + P.T)               # symmetrize


# ── Plant + gain ───────────────────────────────────────────────────────────
def compute_gain(phys: dict) -> dict:
    """Build the cart-pole-on-wheels plant and return the LQR result."""
    M_total = float(phys["total_mass_kg"])
    m_wheel = float(phys["wheel_mass_kg"])
    r       = float(phys["wheel_radius_m"])
    L       = float(phys["com_height_m"])
    Ib      = float(phys["pitch_inertia_kgm2"])

    m = M_total - 2.0 * m_wheel        # pitching mass (chassis + components + legs)
    M = 3.0 * m_wheel                  # 2·m_w + 2·Iw/r²  (solid-disk wheels ⇒ ≈3·m_w)

    I = Ib + m * L * L
    denom = I * (M + m) - (m * L) ** 2

    A = np.array([
        [0, 1, 0,                            0],
        [0, 0, m * m * L * L * G / denom,     0],
        [0, 0, 0,                            1],
        [0, 0, (M + m) * m * G * L / denom,   0],
    ])
    B = np.array([[0],
                  [(I - m * L * r) / (r * denom)],
                  [0],
                  [(m * L - r * (M + m)) / (r * denom)]])

    Q = np.diag([float(x) for x in phys["q_weights"]])
    R = np.array([[float(phys["r_weight"])]])

    P = solve_care(A, B, Q, R)
    K = (np.linalg.inv(R) @ B.T @ P).ravel()

    eig_ol = np.linalg.eigvals(A)
    eig_cl = np.linalg.eigvals(A - B @ K.reshape(1, 4))

    return {
        "K": K, "A": A, "B": B,
        "m": m, "M": M, "L": L, "I": I, "r": r,
        "eig_ol": eig_ol, "eig_cl": eig_cl,
    }


def main():
    cfg = load_config()
    if "physical" not in cfg:
        raise SystemExit("config/robot.yaml has no `physical:` block — add one (see template).")
    phys = cfg["physical"]

    res = compute_gain(phys)
    K = res["K"]
    Kx, Kv, Kp, Kd = (float(v) for v in K)

    stable = np.all(res["eig_cl"].real < 0)
    ks = float(phys.get("torque_to_pwm", 7.8))

    # ── report ──────────────────────────────────────────────────────────
    print("=" * 64)
    print("  LQR GAIN DESIGN — inverted-pendulum-on-wheels")
    print("=" * 64)
    print(f"  m (pitch)= {res['m']:.3f} kg   M (roll)= {res['M']:.3f} kg")
    print(f"  L = {res['L']:.3f} m   I = Ib+mL² = {res['I']:.3f} kg·m²   r = {res['r']:.4f} m")
    print(f"  natural freq √(g/L) = {np.sqrt(G/res['L']):.2f} rad/s "
          f"({np.sqrt(G/res['L'])/(2*np.pi):.2f} Hz)")
    print("-" * 64)
    print(f"  K = [ Kx={Kx:+.2f}  Kv={Kv:+.2f}  Kp={Kp:+.2f}  Kd={Kd:+.2f} ]  (N·m per SI state)")
    print(f"  open-loop   eig: {np.round(res['eig_ol'], 2)}")
    print(f"  closed-loop eig: {np.round(res['eig_cl'], 2)}")
    print(f"  STABLE: {'YES ✓' if stable else 'NO ✗  — gains rejected'}")
    print("-" * 64)
    # PWM-space gains (what the firmware applies after ·ks) vs the legacy PID
    print(f"  with ks = {ks:.2f} PWM/N·m  →  applied gains vs legacy PID:")
    print(f"    pitch : {abs(ks*Kp*DEG2RAD):6.1f} PWM/deg     (legacy kpAngle = 45.0)")
    print(f"    rate  : {abs(ks*Kd*DEG2RAD):6.1f} PWM/(deg/s) (legacy kdAngle =  1.8)")
    print(f"    vel   : {abs(ks*Kv):6.1f} PWM/(m/s)    (legacy kpAngle·kpSpeed = 810)")
    print(f"    pos   : {abs(ks*Kx):6.1f} PWM/m")
    print("=" * 64)

    if not stable:
        raise SystemExit("Closed loop is unstable — not writing gains. Check physical params.")

    # ── persist (preserve hardware-tuned ks / balance_offset if present) ──
    existing = {}
    if os.path.exists(GAINS_FILE):
        try:
            with open(GAINS_FILE) as f:
                existing = json.load(f)
        except Exception:
            pass
        shutil.copyfile(GAINS_FILE, GAINS_FILE + ".bak")
        print(f"  backed up existing gains → {os.path.basename(GAINS_FILE)}.bak")

    gains = {
        "kx": round(Kx, 4),
        "kv": round(Kv, 4),
        "kp": round(Kp, 4),
        "kd": round(Kd, 4),
        "ks": float(existing.get("ks", ks)),                       # keep tuned ks on recompute
        "balance_offset": float(existing.get("balance_offset", 2.5)),
    }
    os.makedirs(os.path.dirname(GAINS_FILE), exist_ok=True)
    with open(GAINS_FILE, "w") as f:
        json.dump(gains, f, indent=4)
    print(f"  wrote {GAINS_FILE}")
    print(f"  {json.dumps(gains)}")


if __name__ == "__main__":
    main()
