#!/usr/bin/env python3
import math
import socket
import traceback

import config
from ik_core import HoverIK
from motion_sequences import MotionSequences
from pose_state import save_pose, load_pose


def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


def clamp_int(v, vmin, vmax):
    return max(vmin, min(vmax, int(round(v))))

class ArmDriver:
    def __init__(self):
        self.arm = None
        self.available = False
        self.method = None
        if config.DRY_RUN:
            print("[ARM] DRY_RUN=True, hardware move disabled")
            return
        try:
            from Arm_Lib import Arm_Device
            self.arm = Arm_Device()
            self.available = True
            if hasattr(self.arm, "Arm_serial_servo_write6_array"):
                self.method = "write6_array"
            elif hasattr(self.arm, "Arm_serial_servo_write6"):
                self.method = "write6"
            print(f"[ARM] Arm_Lib loaded, method={self.method}")
        except Exception as e:
            print(f"[ARM] Arm_Lib not available, fallback to dry-run: {e}")
            self.available = False

    def move6(self, s, move_time_ms):
        if not self.available:
            print(f"[DRY-RUN MOVE] servos={s} time_ms={move_time_ms}")
            return True
        try:
            if self.method == "write6_array":
                self.arm.Arm_serial_servo_write6_array(s, move_time_ms)
                return True
            if self.method == "write6":
                self.arm.Arm_serial_servo_write6(s[0], s[1], s[2], s[3], s[4], s[5], move_time_ms)
                return True
            print("[ARM] No supported write method found, dry-run only")
            print(f"[DRY-RUN MOVE] servos={s} time_ms={move_time_ms}")
            return False
        except Exception as e:
            print(f"[ARM] move error: {e}")
            traceback.print_exc()
            return False

class DofbotServer:
    def __init__(self):
        self.ik = HoverIK()
        self.driver = ArmDriver()
        self.home = list(config.HOME_SERVO)
        self.seq = MotionSequences(self)

    def safe_target(self, x, y, z):
        if not (config.SAFE_X_MIN <= x <= config.SAFE_X_MAX):
            return False, f"x out of range [{config.SAFE_X_MIN}, {config.SAFE_X_MAX}]"
        if not (config.SAFE_Y_MIN <= y <= config.SAFE_Y_MAX):
            return False, f"y out of range [{config.SAFE_Y_MIN}, {config.SAFE_Y_MAX}]"
        if not (config.SAFE_Z_MIN <= z <= config.SAFE_Z_MAX):
            return False, f"z out of range [{config.SAFE_Z_MIN}, {config.SAFE_Z_MAX}]"
        return True, "ok"

    def map_roll_to_servo5(self, roll_deg):
        v = self.home[4] + (-roll_deg if config.INVERT_WRIST_ROLL else roll_deg)
        return clamp_int(v, config.SERVO_MIN, config.SERVO_MAX)

    def map_grip_to_servo6(self, grip):
        if grip >= 0.5:
            v = self.home[5]
        else:
            v = 120
        return clamp_int(v, config.SERVO_MIN, config.SERVO_MAX)

    def joints_to_servos(self, q1, q2, q3, q4, roll_deg, grip):
        s = list(self.home)
        dq1 = -q1 if config.INVERT_BASE else q1
        dq2 = -q2 if config.INVERT_SHOULDER else q2
        dq3 = -q3 if config.INVERT_ELBOW else q3
        dq4 = -q4 if config.INVERT_WRIST_PITCH else q4
        s[0] = clamp_int(self.home[0] + dq1, config.SERVO_MIN, config.SERVO_MAX)
        s[1] = clamp_int(self.home[1] + dq2, config.SERVO_MIN, config.SERVO_MAX)
        s[2] = clamp_int(self.home[2] + dq3, config.SERVO_MIN, config.SERVO_MAX)
        if getattr(config, "FIX_WRIST_PITCH", False):
            s[3] = clamp_int(config.FIXED_WRIST_PITCH_SERVO, config.SERVO_MIN, config.SERVO_MAX)
        else:
            s[3] = clamp_int(self.home[3] + dq4, config.SERVO_MIN, config.SERVO_MAX)
        s[4] = self.map_roll_to_servo5(roll_deg)
        s[5] = self.map_grip_to_servo6(grip)
        return s

    def go_home(self):
        print(f"[HOME] {self.home}")
        self.driver.move6(self.home, config.MOVE_TIME_MS)

    def handle_line(self, line):
        text = line.strip()
        if text == "GETPOS":
            p = load_pose()
            return f'{p["x"]} {p["y"]} {p["z"]} {p["pitch"]} {p["roll"]} {p["grip"]}\n'

        if text.startswith("SERVO_MS "):
            parts = text.split()
            if len(parts) != 8:
                return "ERR bad SERVO_MS format, need: SERVO_MS ms s1 s2 s3 s4 s5 s6\n"
            try:
                move_ms = int(round(float(parts[1])))
                move_ms = max(300, min(3000, move_ms))
                servos = [int(round(float(v))) for v in parts[2:8]]
                servos = [max(0, min(180, v)) for v in servos]
            except Exception:
                return "ERR SERVO_MS parse failed\n"
            moved = self.driver.move6(servos, move_ms)
            if not moved:
                return "ERR SERVO_MS move failed\n"
            return f"OK SERVO_MS {move_ms} {servos}\n"

        if text.startswith("SERVO "):
            parts = text.split()
            if len(parts) != 7:
                return "ERR bad SERVO format, need: SERVO s1 s2 s3 s4 s5 s6\n"
            try:
                servos = [int(round(float(v))) for v in parts[1:7]]
                servos = [max(0, min(180, v)) for v in servos]
            except Exception:
                return "ERR SERVO parse failed\n"
            moved = self.driver.move6(servos, config.MOVE_TIME_MS)
            if not moved:
                return "ERR SERVO move failed\n"
            return f"OK SERVO {servos}\n"

        parts = text.split()
        if len(parts) != 6:
            return f"ERR bad format, need 6 numbers: got {len(parts)}\n"
        try:
            x = float(parts[0])
            y = float(parts[1])
            z = float(parts[2])
            pitch = float(parts[3])
            roll = float(parts[4])
            grip = float(parts[5])
        except Exception:
            return "ERR parse float failed\n"

        ok, msg = self.safe_target(x, y, z)
        if not ok:
            return f"ERR unsafe target: {msg}\n"

        if getattr(config, "SEQUENCE_MODE", False):
            if config.VERBOSE:
                print("-" * 60)
                print(f"[TARGET] x={x:.2f} y={y:.2f} z={z:.2f} pitch={pitch:.2f} roll={roll:.2f} grip={grip:.2f}")
            print("[MODE] SEQUENCE_MODE=True")
            ok, seq_msg = self.seq.run_pick_and_place(x, y)
            if not ok:
                return f"ERR sequence failed: {seq_msg}\n"
            return f"OK sequence: {seq_msg}\n"

        res = self.ik.solve_hover(x, y, z, pitch)
        if not res.ok:
            return f"ERR ik failed: {res.msg}\n"

        servos = self.joints_to_servos(res.q1, res.q2, res.q3, res.q4, roll, grip)

        if config.VERBOSE:
            fx, fy, fz, fp = self.ik.fk(res.q1, res.q2, res.q3, res.q4)
            print("-" * 60)
            print(f"[TARGET] x={x:.2f} y={y:.2f} z={z:.2f} pitch={pitch:.2f} roll={roll:.2f} grip={grip:.2f}")
            print(f"[IK] q1={res.q1:.2f} q2={res.q2:.2f} q3={res.q3:.2f} q4={res.q4:.2f} used_pitch={res.used_pitch:.2f}")
            print(f"[FK] x={fx:.2f} y={fy:.2f} z={fz:.2f} pitch={fp:.2f}")
            print(f"[SERVO] {servos}")

        moved = self.driver.move6(servos, config.MOVE_TIME_MS)
        if moved:
            save_pose(x, y, z, pitch, roll, grip)
        if not moved:
            return "ERR move failed\n"

        return f"OK q=({res.q1:.2f},{res.q2:.2f},{res.q3:.2f},{res.q4:.2f}) pitch_used={res.used_pitch:.2f} servo={servos}\n"

    def serve_forever(self):
        if config.STARTUP_HOME:
            self.go_home()

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((config.SERVER_HOST, config.SERVER_PORT))
        sock.listen(5)

        print("[BOOT] Safe DOFBOT server starting")
        print(f"[BOOT] Listening on {config.SERVER_HOST}:{config.SERVER_PORT}")
        print(f"[BOOT] HOME = {self.home}")
        print(f"[BOOT] INVERT_BASE = {config.INVERT_BASE}")
        print(f"[BOOT] INVERT_SHOULDER = {config.INVERT_SHOULDER}")
        print(f"[BOOT] REAL: L1={config.L1}, L2={config.L2}, L3={config.L3}, L4={config.L4}, L5={config.L5}")
        print(f"[BOOT] TEST: L1={config.L1}, L2={config.L2}, L3={config.L3}, L4={'40.0' if config.USE_EFFECTIVE_WRIST else config.L4}, L5={'0.0' if config.USE_EFFECTIVE_WRIST else config.L5}")

        while True:
            conn, addr = sock.accept()
            print(f"[CONNECT] {addr}")
            with conn:
                buf = b""
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        text = line.decode("utf-8", errors="ignore").strip()
                        if not text:
                            continue
                        print(f"[RAW] {text}")
                        try:
                            reply = self.handle_line(text)
                        except Exception as e:
                            traceback.print_exc()
                            reply = f"ERR exception: {e}\n"
                        conn.sendall(reply.encode("utf-8"))

if __name__ == "__main__":
    DofbotServer().serve_forever()
