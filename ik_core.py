#!/usr/bin/env python3
import math
import config

def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))

class IKResult:
    def __init__(self, ok, q1=0.0, q2=0.0, q3=0.0, q4=0.0, used_pitch=0.0, msg=""):
        self.ok = ok
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        self.q4 = q4
        self.used_pitch = used_pitch
        self.msg = msg

class HoverIK:
    def __init__(self):
        self.L1 = config.L1
        self.L2 = config.L2
        self.L3 = config.L3
        if config.USE_EFFECTIVE_WRIST:
            self.L45 = config.EFFECTIVE_L45
        else:
            self.L45 = config.L4 + config.L5

    def in_joint_limits(self, q1, q2, q3, q4):
        return (
            config.Q1_MIN <= q1 <= config.Q1_MAX and
            config.Q2_MIN <= q2 <= config.Q2_MAX and
            config.Q3_MIN <= q3 <= config.Q3_MAX and
            config.Q4_MIN <= q4 <= config.Q4_MAX
        )

    def fk(self, q1_deg, q2_deg, q3_deg, q4_deg):
        q1 = math.radians(q1_deg)
        q2 = math.radians(q2_deg)
        q3 = math.radians(q3_deg)
        q4 = math.radians(q4_deg)

        phi = q2 + q3 + q4
        r = (
            self.L2 * math.cos(q2) +
            self.L3 * math.cos(q2 + q3) +
            self.L45 * math.cos(phi)
        )
        z = (
            self.L1 +
            self.L2 * math.sin(q2) +
            self.L3 * math.sin(q2 + q3) +
            self.L45 * math.sin(phi)
        )

        x = r * math.cos(q1)
        y = r * math.sin(q1)
        return x, y, z, math.degrees(phi)

    def solve_one(self, x, y, z, pitch_deg):
        q1 = math.degrees(math.atan2(y, x))
        r = math.hypot(x, y)
        phi = math.radians(pitch_deg)

        wx = r - self.L45 * math.cos(phi)
        wz = (z - self.L1) - self.L45 * math.sin(phi)

        d2 = wx * wx + wz * wz
        denom = 2.0 * self.L2 * self.L3
        if abs(denom) < 1e-9:
            return IKResult(False, msg="bad link geometry")

        D = (d2 - self.L2 * self.L2 - self.L3 * self.L3) / denom
        if D < -1.0 or D > 1.0:
            return IKResult(False, msg=f"unreachable D={D:.4f}")

        candidates = []
        for sign in (+1.0, -1.0):
            q3 = math.degrees(sign * math.acos(clamp(D, -1.0, 1.0)))
            q3r = math.radians(q3)
            q2 = math.degrees(
                math.atan2(wz, wx) -
                math.atan2(self.L3 * math.sin(q3r), self.L2 + self.L3 * math.cos(q3r))
            )
            q4 = pitch_deg - q2 - q3
            if self.in_joint_limits(q1, q2, q3, q4):
                candidates.append((q1, q2, q3, q4))

        if not candidates:
            return IKResult(False, msg="no valid branch in joint limits")

        preferred = [c for c in candidates if c[1] >= 0.0 and c[2] >= 0.0]
        if preferred:
            pool = preferred
        else:
            positive_q3 = [c for c in candidates if c[2] >= 0.0]
            pool = positive_q3 if positive_q3 else candidates

        best = min(pool, key=lambda t: (abs(t[3]), abs(t[1]) + abs(t[2])))
        return IKResult(
            True,
            q1=best[0],
            q2=best[1],
            q3=best[2],
            q4=best[3],
            used_pitch=pitch_deg,
            msg="ok"
        )

    def solve_hover(self, x, y, z, requested_pitch_deg):
        pitch_candidates = [
            requested_pitch_deg,
            -80.0,
            -70.0,
            -60.0,
            -50.0,
        ]

        tried = []
        for p in pitch_candidates:
            res = self.solve_one(x, y, z, p)
            tried.append((p, res.ok, res.msg))
            if res.ok:
                return res

        msg = " ; ".join([f"pitch={p:.1f} ok={ok} {m}" for p, ok, m in tried])
        return IKResult(False, msg=msg)
