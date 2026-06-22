#!/usr/bin/env python3
import math
import config


def clamp(v, lo=0, hi=180):
    return max(lo, min(hi, int(round(v))))


def angles_to_servo(q1, a2, a3, a4, q5=90.0, grip=120):
    home = list(config.HOME_SERVO)
    dq1 = -q1 if config.INVERT_BASE else q1
    dq2 = -a2 if config.INVERT_SHOULDER else a2
    dq3 = -a3 if config.INVERT_ELBOW else a3
    dq4 = -a4 if config.INVERT_WRIST_PITCH else a4
    s = home.copy()
    s[0] = clamp(home[0] + dq1, config.SERVO_MIN, config.SERVO_MAX)
    s[1] = clamp(home[1] + dq2, config.SERVO_MIN, config.SERVO_MAX)
    s[2] = clamp(home[2] + dq3, config.SERVO_MIN, config.SERVO_MAX)
    if getattr(config, "FIX_WRIST_PITCH", False):
        s[3] = clamp(config.FIXED_WRIST_PITCH_SERVO, config.SERVO_MIN, config.SERVO_MAX)
    else:
        s[3] = clamp(home[3] + dq4, config.SERVO_MIN, config.SERVO_MAX)
    s[4] = clamp(home[4] + (-q5 if config.INVERT_WRIST_ROLL else q5), config.SERVO_MIN, config.SERVO_MAX)
    s[5] = clamp(int(round(grip)), config.SERVO_MIN, config.SERVO_MAX)
    return s


def fk_nominal(q1, a2, a3, pitch_deg):
    q4 = pitch_deg - a2 - a3
    q1r = math.radians(q1)
    q2r = math.radians(a2)
    q3r = math.radians(a3)
    q4r = math.radians(q4)
    L45 = config.L4_REAL + config.L5_REAL if not config.USE_EFFECTIVE_WRIST else config.EFFECTIVE_L45
    phi = q2r + q3r + q4r
    r = (
        config.L2 * math.cos(q2r) +
        config.L3 * math.cos(q2r + q3r) +
        L45 * math.cos(phi)
    )
    z = (
        config.L1 +
        config.L2 * math.sin(q2r) +
        config.L3 * math.sin(q2r + q3r) +
        L45 * math.sin(phi)
    )
    x = r * math.cos(q1r)
    y = r * math.sin(q1r)
    return x, y, z, math.degrees(phi)


def solve_ik_nominal(x, y, z, pitch_deg):
    q1 = math.degrees(math.atan2(y, x))
    r = math.hypot(x, y)
    phi = math.radians(pitch_deg)
    L45 = config.L4_REAL + config.L5_REAL
    wx = r - L45 * math.cos(phi)
    wz = (z - config.L1) - L45 * math.sin(phi)
    denom = 2.0 * config.L2 * config.L3
    if abs(denom) < 1e-9:
        return None, "bad link geometry"
    D = (wx * wx + wz * wz - config.L2 * config.L2 - config.L3 * config.L3) / denom
    if D < -1.0 or D > 1.0:
        return None, f"unreachable D={D:.4f}"
    candidates = []
    for sign in (+1.0, -1.0):
        q3 = math.degrees(sign * math.acos(clamp(D, -1.0, 1.0)))
        q3r = math.radians(q3)
        q2 = math.degrees(
            math.atan2(wz, wx) -
            math.atan2(config.L3 * math.sin(q3r), config.L2 + config.L3 * math.cos(q3r))
        )
        q4 = pitch_deg - q2 - q3
        if (config.Q2_MIN <= q2 <= config.Q2_MAX and
            config.Q3_MIN <= q3 <= config.Q3_MAX and
            config.Q4_MIN <= q4 <= config.Q4_MAX):
            candidates.append((q1, q2, q3, pitch_deg, 90.0))
    if not candidates:
        return None, "no valid branch in joint limits"
    preferred = [c for c in candidates if c[1] >= 0.0 and c[2] >= 0.0]
    pool = preferred if preferred else ([c for c in candidates if c[2] >= 0.0] or candidates)
    best = min(pool, key=lambda t: (abs(t[3]), abs(t[1]) + abs(t[2])))
    return best, "ok"


def print_ik_report(x, y, pick, z_mm=None, pitch_deg=None):
    print("=" * 60)
    print("[IK REPORT]")
    print(f"workspace X={x:.2f}, Y={y:.2f}")
    if z_mm is not None:
        print(f"target Z={z_mm:.2f}")
    if pitch_deg is not None:
        print(f"target pitch={pitch_deg:.2f}")
    ik_q, msg = solve_ik_nominal(x, y, z_mm if z_mm is not None else config.L1, pitch_deg if pitch_deg is not None else -60.0)
    print("ik msg:", msg)
    if ik_q is None:
        return
    q1, a2, a3, pitch_out, q5 = ik_q
    a4 = 90.0 - pitch_out
    servos = angles_to_servo(q1, a2, a3, a4, q5=q5, grip=120)
    vx, vy, vz, vp = fk_nominal(q1, a2, a3, pitch_out)
    print(f"IK angles: q1={q1:.2f}, a2={a2:.2f}, a3={a3:.2f}, a4={a4:.2f}, q5={q5:.2f}")
    print("IK servo:", servos)
    print(f"FK verify: X={vx:.2f}, Y={vy:.2f}, Z={vz:.2f}, pitch={vp:.2f}")
