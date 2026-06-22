#!/usr/bin/env python3
import time
import config

def clamp_int(v, vmin, vmax):
    return max(vmin, min(vmax, int(round(v))))

class MotionSequences:
    def __init__(self, server):
        self.server = server

    def _move_pose(self, x, y, z, pitch, roll, grip, label):
        res = self.server.ik.solve_hover(x, y, z, pitch)
        if not res.ok:
            return False, f"{label}: ik failed: {res.msg}"

        servos = self.server.joints_to_servos(res.q1, res.q2, res.q3, res.q4, roll, grip)

        if config.VERBOSE:
            fx, fy, fz, fp = self.server.ik.fk(res.q1, res.q2, res.q3, res.q4)
            print("-" * 60)
            print(f"[SEQ:{label}] target x={x:.2f} y={y:.2f} z={z:.2f} pitch={pitch:.2f} roll={roll:.2f} grip={grip:.2f}")
            print(f"[SEQ:{label}] q1={res.q1:.2f} q2={res.q2:.2f} q3={res.q3:.2f} q4={res.q4:.2f}")
            print(f"[SEQ:{label}] fk x={fx:.2f} y={fy:.2f} z={fz:.2f} pitch={fp:.2f}")
            print(f"[SEQ:{label}] servo {servos}")

        ok = self.server.driver.move6(servos, config.STEP_MOVE_TIME_MS)
        if not ok:
            return False, f"{label}: move failed"

        time.sleep(config.STEP_WAIT_SEC)
        return True, f"{label}: ok"

    def run_pick_and_place(self, x, y):
        roll = config.DEFAULT_ROLL_DEG
        steps = [
            ("APPROACH", x, y, config.APPROACH_Z, config.PICK_PITCH, roll, config.GRIP_OPEN),
            ("DOWN", x, y, config.PICK_Z, config.PICK_PITCH, roll, config.GRIP_OPEN),
            ("CLOSE", x, y, config.PICK_Z, config.PICK_PITCH, roll, config.GRIP_CLOSE),
            ("LIFT", x, y, config.LIFT_Z, config.PICK_PITCH, roll, config.GRIP_CLOSE),
            ("PLACE", config.PLACE_X, config.PLACE_Y, config.PLACE_Z, config.PLACE_PITCH, roll, config.GRIP_CLOSE),
            ("OPEN", config.PLACE_X, config.PLACE_Y, config.PLACE_Z, config.PLACE_PITCH, roll, config.GRIP_OPEN),
            ("HOME", None, None, None, None, None, None),
        ]

        logs = []
        for step in steps:
            label = step[0]
            if label == "HOME":
                self.server.go_home()
                time.sleep(config.STEP_WAIT_SEC)
                logs.append("HOME: ok")
                continue

            _, sx, sy, sz, spitch, sroll, sgrip = step
            ok, msg = self._move_pose(sx, sy, sz, spitch, sroll, sgrip, label)
            logs.append(msg)
            if not ok:
                return False, " ; ".join(logs)

        return True, " ; ".join(logs)
