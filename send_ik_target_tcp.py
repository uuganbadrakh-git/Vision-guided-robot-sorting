#!/usr/bin/env python3
import socket
import sys
from ik_nominal import solve_ik_nominal, angles_to_servo, fk_nominal

JETSON_IP = "192.168.55.1"
PORT = 5000

DEFAULT_GRIP = 120
DEFAULT_MS = 1800

def send_servo_ms(servos, move_ms):
    msg = "SERVO_MS " + str(int(move_ms)) + " " + " ".join(str(int(v)) for v in servos)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((JETSON_IP, PORT))
        s.sendall((msg + "\n").encode("utf-8"))
        reply = s.recv(1024).decode("utf-8", errors="ignore").strip()
    print("sent =", msg)
    print("reply =", reply)


def main():
    if len(sys.argv) not in [5, 6, 7]:
        print("usage:")
        print(" python3 send_ik_target_tcp.py X Y Z pitch")
        print(" python3 send_ik_target_tcp.py X Y Z pitch grip")
        print(" python3 send_ik_target_tcp.py X Y Z pitch grip ms")
        print()
        print("example:")
        print(" python3 send_ik_target_tcp.py 107.4 4.25 112.3 -70 120 1800")
        sys.exit(1)

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    pitch = float(sys.argv[4])

    grip = DEFAULT_GRIP
    move_ms = DEFAULT_MS
    if len(sys.argv) >= 6:
        grip = int(float(sys.argv[5]))
    if len(sys.argv) >= 7:
        move_ms = int(float(sys.argv[6]))

    ik_q, msg = solve_ik_nominal(x, y, z, pitch)
    print("=" * 60)
    print("[IK TARGET SEND]")
    print(f"target: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, pitch={pitch:.2f}")
    print("ik msg:", msg)

    if ik_q is None:
        print("IK failed. Not sending.")
        return

    q1, a2, a3, pitch_out, q5 = ik_q
    a4 = 90.0 - pitch_out
    servos = angles_to_servo(q1, a2, a3, a4, q5=q5, grip=grip)
    vx, vy, vz, vp = fk_nominal(q1, a2, a3, pitch_out)

    print(f"IK angles: q1={q1:.2f}, a2={a2:.2f}, a3={a3:.2f}, a4={a4:.2f}, pitch={pitch_out:.2f}")
    print("IK servo :", servos)
    print(f"FK verify: X={vx:.2f}, Y={vy:.2f}, Z={vz:.2f}, pitch={vp:.2f}")
    print("=" * 60)

    send_servo_ms(servos, move_ms)

if __name__ == "__main__":
    main()
