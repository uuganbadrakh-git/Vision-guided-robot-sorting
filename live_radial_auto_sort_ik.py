#!/usr/bin/env python3
import cv2
import socket
import time
import numpy as np
from pupil_apriltags import Detector
from radial_pick_model import RadialPickModel
from ik_nominal import print_ik_report

CAM_INDEX = 2

JETSON_IP = "192.168.55.1"
PORT = 5000

# Motion constants
LIFT_S4_ADD = 45
PICK_S4_UP_OFFSET = 1

# IK report only. Does not control final sorting motion.
IK_PICK_Z_MM = 130.0
IK_PITCH_DEG = -62.0

# Local delta corrections.
# Format:
# (s1, s2, s3, s4, ds1, ds2, ds3, ds4)
# These are offsets, not absolute servo values.
LOCAL_DELTA_POINTS = [
    (91, 47, 32, 41, 0, 0, 0, -3), # learned: s4 -3
    (53, 77, 4, 14, 0, 0, 0, -1), # ALL_CORRECTED_POINT run 22 X=67.9 Y=-49.2 s4 -1
    (59, 79, 1, 13, 0, 0, 0, 1), # ALL_CORRECTED_POINT run 1 X=68.6 Y=-39.3 s4 +1,2
    (63, 67, 15, 19, 0, 0, 0, -1), # ALL_CORRECTED_POINT run 21 X=93.1 Y=-45.7 s4 -1
    (82, 80, 0, 13, 0, 0, 0, -1), # ALL_CORRECTED_POINT run 2 X=68.8 Y=-8.8 s4 -1
    (90, 80, 0, 13, 0, 0, 0, -1), # ALL_CORRECTED_POINT run 24 X=70.3 Y=1.8 s4 -1
    (92, 48, 31, 39, 0, 0, 0, -1), # ALL_CORRECTED_POINT run 6 X=144.4 Y=6.3 s4 -1
    (106, 75, 5, 15, 0, 0, 0, 1), # ALL_CORRECTED_POINT run 27 X=83.0 Y=26.0 s4 +1
    (59, 79, 1, 13, 0, 0, 0, 1), # FINAL_MAIN_FIX run 1: X=68.57, Y=-39.28, s4 +1,2
    (82, 80, 0, 13, 0, 0, 0, -1), # FINAL_MAIN_FIX run 2: X=68.83, Y=-8.78, s4 -1
    (92, 48, 31, 39, 0, 0, 0, -1), # FINAL_MAIN_FIX run 6: X=144.44, Y=6.33, s4 -1
    (63, 67, 15, 19, 0, 0, 0, -1), # FINAL_MAIN_FIX run 21: X=93.06, Y=-45.69, s4 -1
    (53, 77, 4, 14, 0, 0, 0, -1), # FINAL_MAIN_FIX run 22: X=67.92, Y=-49.23, s4 -1
    (106, 75, 5, 15, 0, 0, 0, 1), # FINAL_MAIN_FIX run 27: X=83.01, Y=25.95, s4 +1
    (59, 79, 1, 13, 0, 0, 0, 1), # final failed-point fix run 1: X=68.57, Y=-39.28, s4 +1,2
    (82, 80, 0, 13, 0, 0, 0, -1), # final failed-point fix run 2: X=68.83, Y=-8.78, s4 -1
    (92, 48, 31, 39, 0, 0, 0, -1), # final failed-point fix run 6: X=144.44, Y=6.33, s4 -1
    (63, 67, 15, 19, 0, 0, 0, -1), # final failed-point fix run 21: X=93.06, Y=-45.69, s4 -1
    (53, 77, 4, 14, 0, 0, 0, -1), # final failed-point fix run 22: X=67.92, Y=-49.23, s4 -1
    (106, 75, 5, 15, 0, 0, 0, 1), # final failed-point fix run 27: X=83.01, Y=25.95, s4 +1
    (104, 80, 0, 13, 0, 0, 0, -1), # latest accuracy: run 4 s4 -1
    (91, 78, 2, 14, 2, 0, 0, 0), # latest accuracy: run 8 s1 +2
    (79, 58, 25, 23, 0, 0, 0, -1), # latest accuracy: run 11 s4 -1
    (91, 47, 32, 40, 0, 0, 0, -1), # latest accuracy: run 16 s4 -1
    (80, 50, 30, 35, 0, 0, 0, -1), # latest accuracy: run 21 s4 -1
    (72, 77, 4, 14, 0, 0, 0, -2), # latest accuracy: run 25 s4 -2
    (90, 78, 3, 14, 0, 0, 0, 2), # latest accuracy: run 26 s4 +2
    (72, 75, 5, 15, 0, 0, 0, 1), # latest accuracy: run 28 s4 +1
    (120, 79, 1, 13, 0, 0, 0, 1), # latest accuracy: run 29 s4 +1
    (70, 80, 0, 13, 0, 0, 0, -3), # from accuracy run 2: s4 -3
    (90, 80, 0, 13, 0, 0, 0, -3), # from accuracy run 3: s4 -3
    (107, 80, 0, 13, 0, 0, 0, -3), # from accuracy run 4: s4 -3
    (76, 80, 0, 13, 0, 0, 0, -2), # from accuracy run 8: s4 -2
    (120, 75, 5, 15, 0, 0, 0, 2), # from accuracy run 12: s4 +2
    (90, 73, 8, 16, 0, 0, 0, -2), # from accuracy run 15: s4 -2
    (91, 71, 10, 17, 0, 0, 0, 1), # from accuracy run 16: s4 +1
    (90, 70, 11, 17, 0, 0, 0, -1), # from accuracy run 17: s4 -1
    (91, 47, 31, 40, 0, 0, 0, -1), # from accuracy run 28: s4 -1
    (90, 70, 11, 17, 0, 0, 0, 0), # center: S4 +2 from previous final
]
LOCAL_DELTA_TOL = 6
LOCAL_WEIGHTED_RADIUS = 18.0
LOCAL_WEIGHT_EPS = 1e-6

# Continuous approach:
# robot moves toward object and bends down together, not as two separate motions.
CONTINUOUS_APPROACH_STEPS = 4
CONTINUOUS_APPROACH_MS = 500

CENTER_SERVO = [89, 91, 91, 90, 89, 120]

# User-tested box poses
RED_BOX_SERVO = [40, 80, 15, 35, 89, 137]
BLUE_BOX_SERVO = [140, 80, 15, 35, 89, 137]

GRIP_OPEN = 110
GRIP_CLOSE = 137
STEP_WAIT = 1.2

AUTO_CLEAR_MAX_OBJECTS = 10
AUTO_CLEAR_WAIT_AFTER_SORT = 0.6
AUTO_CLEAR_FRAME_FLUSH = 8
AUTO_CLEAR_SAME_TARGET_MM = 20.0
APPROACH_MOVE_MS = 2600
CLEAR_LOOP_WAIT = 0.5
MAX_CLEAR_OBJECTS = 10
SAME_TARGET_MM = 18.0
CLEAR_WAIT_SEC = 2.0
MAX_CLEAR_OBJECTS = 20

CAMERA_MATRIX = np.array([
    [952.86066325, 0.0, 414.71566199],
    [0.0, 954.58459356, 187.15852363],
    [0.0, 0.0, 1.0],
], dtype=np.float64)

DIST_COEFFS = np.array([0.28570763, -0.62357284, -0.02281565, 0.02562266, 0.00581731], dtype=np.float64)

TAG_SIZE_MM = 95.0

WORLD_TAG_CENTERS = {
    0: np.array([0.0, 0.0], dtype=np.float32),
    1: np.array([150.0, 0.0], dtype=np.float32),
    2: np.array([150.0, 150.0], dtype=np.float32),
    3: np.array([0.0, 150.0], dtype=np.float32),
}

# Red HSV
RED_LOW1 = np.array([0, 99, 167], dtype=np.uint8)
RED_HIGH1 = np.array([47, 255, 255], dtype=np.uint8)
RED_LOW2 = np.array([140, 99, 167], dtype=np.uint8)
RED_HIGH2 = np.array([180, 255, 255], dtype=np.uint8)

# Blue HSV, tune if needed
BLUE_LOW = np.array([90, 70, 60], dtype=np.uint8)
BLUE_HIGH = np.array([130, 255, 255], dtype=np.uint8)

A = np.load("workspace_to_robot_affine.npz")["A"]
pick_model = RadialPickModel("radial_pick_points.json")

detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0,
)

locked_target = None
current_servos = CENTER_SERVO.copy()

def clamp(v):
    return max(0, min(180, int(round(v))))

def tcp_send(msg):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((JETSON_IP, PORT))
        s.sendall((msg.strip() + "\n").encode("utf-8"))
        reply = s.recv(1024).decode("utf-8", errors="ignore").strip()
    return reply


def estimate_move_ms(target):
    global current_servos
    diffs = [abs(int(target[i]) - int(current_servos[i])) for i in range(6)]
    max_diff = max(diffs)
    if max_diff <= 10:
        return 700
    if max_diff <= 25:
        return 1100
    if max_diff <= 45:
        return 1600
    return 2200


def send_servo(servos, label):
    global current_servos
    servos = [clamp(v) for v in servos]
    move_ms = estimate_move_ms(servos)
    msg = "SERVO_MS " + str(move_ms) + " " + " ".join(str(v) for v in servos)
    reply = tcp_send(msg)
    print(label, msg)
    print("[REPLY]", reply)
    current_servos = servos.copy()
    time.sleep(move_ms / 1000.0 + 0.25)
    return reply


def send_servo_timed(servos, label, move_ms=650, extra_wait=0.05):
    global current_servos
    servos = [clamp(v) for v in servos]
    move_ms = max(300, min(3000, int(move_ms)))
    msg = "SERVO_MS " + str(move_ms) + " " + " ".join(str(v) for v in servos)
    reply = tcp_send(msg)
    print(label, msg)
    print("[REPLY]", reply)
    current_servos = servos.copy()
    time.sleep(move_ms / 1000.0 + extra_wait)
    return reply


def lerp(a, b, t):
    return a + (b - a) * t


def ease(t):
    return t * t * (3.0 - 2.0 * t)


def continuous_approach_to_pick(pick):
    global current_servos
    start = current_servos.copy()
    start[5] = GRIP_OPEN
    end = [
        pick["s1"],
        pick["s2"],
        pick["s3"],
        clamp(pick["s4"] + PICK_S4_UP_OFFSET),
        89,
        GRIP_OPEN,
    ]
    steps = max(1, int(CONTINUOUS_APPROACH_STEPS))
    for i in range(1, steps + 1):
        t = ease(i / float(steps))
        pose = [
            clamp(lerp(start[0], end[0], t)),
            clamp(lerp(start[1], end[1], t)),
            clamp(lerp(start[2], end[2], t)),
            clamp(lerp(start[3], end[3], t)),
            89,
            GRIP_OPEN,
        ]
        send_servo_timed(pose, f"[APPROACH {i}/{steps}]", move_ms=CONTINUOUS_APPROACH_MS, extra_wait=0.03)


def workspace_to_robot(xw, yw):
    p = np.array([xw, yw, 1.0], dtype=np.float64)
    out = A @ p
    return float(out[0]), float(out[1])


def build_homography_and_roi(gray, vis):
    tags = detector.detect(gray, estimate_tag_pose=False)
    img_pts = []
    world_pts = []
    centers = {}
    found_ids = []
    for tag in tags:
        tid = tag.tag_id
        if tid not in WORLD_TAG_CENTERS:
            continue
        cx, cy = WORLD_TAG_CENTERS[tid]
        half = TAG_SIZE_MM / 2.0
        img_corners = tag.corners.astype(np.float32)
        obj_corners = np.array([
            [cx - half, cy - half],
            [cx + half, cy - half],
            [cx + half, cy + half],
            [cx - half, cy + half],
        ], dtype=np.float32)
        img_pts.extend(img_corners.tolist())
        world_pts.extend(obj_corners.tolist())
        centers[tid] = np.array(tag.center, dtype=np.float32)
        found_ids.append(tid)
        c = tuple(np.round(tag.center).astype(int))
        cv2.circle(vis, c, 5, (0,255,0), -1)
        cv2.putText(vis, f"ID{tid}", (c[0]+6, c[1]-6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    H = None
    if len(img_pts) >= 8:
        H, _ = cv2.findHomography(np.array(img_pts, dtype=np.float32), np.array(world_pts, dtype=np.float32), method=0)
    roi_poly = None
    if all(k in centers for k in [0,1,2,3]):
        roi_poly = np.array([centers[0], centers[1], centers[2], centers[3]], dtype=np.int32)
    return H, roi_poly, found_ids


def color_masks(frame):
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    b, g, r = cv2.split(blur.astype(np.float32))
    total = r + g + b + 1.0
    red_ratio = r / total
    blue_ratio = b / total
    red = ((red_ratio > 0.42) & (r > g + 15) & (r > b + 15)).astype(np.uint8) * 255
    blue = ((blue_ratio > 0.38) & (b > r + 12) & (b > g + 8)).astype(np.uint8) * 255
    kernel = np.ones((5, 5), np.uint8)
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, kernel, iterations=1)
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, kernel, iterations=2)
    blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, kernel, iterations=1)
    blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, kernel, iterations=2)
    return red, blue


def apply_roi(mask, roi_poly):
    if roi_poly is None:
        return np.zeros_like(mask)
    roi_mask = np.zeros_like(mask)
    cv2.fillPoly(roi_mask, [roi_poly], 255)
    return cv2.bitwise_and(mask, roi_mask)


def detect_object(frame, roi_poly):
    red_mask, blue_mask = color_masks(frame)
    red_mask = apply_roi(red_mask, roi_poly)
    blue_mask = apply_roi(blue_mask, roi_poly)
    candidates = []
    for name, mask in [("red", red_mask), ("blue", blue_mask)]:
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            continue
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if area < 120 or area > 6000:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            ratio = w / float(h)
            if ratio < 0.40 or ratio > 2.50:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            px = int(M["m10"] / M["m00"])
            py = int(M["m01"] / M["m00"])
            candidates.append({"color": name, "px": px, "py": py, "area": area, "cnt": cnt, "mask": mask})
    if not candidates:
        return None, red_mask, blue_mask
    best = max(candidates, key=lambda x: x["area"])
    return best, red_mask, blue_mask


def pixel_to_workspace(H, px, py):
    p = np.array([[[px, py]]], dtype=np.float32)
    out = cv2.perspectiveTransform(p, H)[0,0]
    return float(out[0]), float(out[1])


def apply_local_delta_correction(pick):
    q = pick.copy()
    total_w = 0.0
    ds1_sum = 0.0
    ds2_sum = 0.0
    ds3_sum = 0.0
    ds4_sum = 0.0
    used = []
    for s1c, s2c, s3c, s4c, ds1, ds2, ds3, ds4 in LOCAL_DELTA_POINTS:
        d = ((q["s1"] - s1c) ** 2 + (q["s2"] - s2c) ** 2 + (q["s3"] - s3c) ** 2 + (q["s4"] - s4c) ** 2) ** 0.5
        if d > LOCAL_WEIGHTED_RADIUS:
            continue
        w = 1.0 / (d * d + LOCAL_WEIGHT_EPS)
        total_w += w
        ds1_sum += w * ds1
        ds2_sum += w * ds2
        ds3_sum += w * ds3
        ds4_sum += w * ds4
        used.append((s1c, s2c, s3c, s4c, ds1, ds2, ds3, ds4, d))
    if total_w <= 0.0:
        return q
    ds1 = round(ds1_sum / total_w)
    ds2 = round(ds2_sum / total_w)
    ds3 = round(ds3_sum / total_w)
    ds4 = round(ds4_sum / total_w)
    old = (q["s1"], q["s2"], q["s3"], q["s4"])
    q["s1"] = clamp(q["s1"] + ds1)
    q["s2"] = clamp(q["s2"] + ds2)
    q["s3"] = clamp(q["s3"] + ds3)
    q["s4"] = clamp(q["s4"] + ds4)
    new = (q["s1"], q["s2"], q["s3"], q["s4"])
    print(f"[WEIGHTED DELTA] {old} -> {new} delta=({ds1},{ds2},{ds3},{ds4}) used={len(used)}")
    return q


def make_down_servos(pick, grip):
    if "apply_local_delta_correction" in globals():
        p = apply_local_delta_correction(pick)
    else:
        p = pick
    return [
        p["s1"],
        p["s2"],
        p["s3"],
        clamp(p["s4"] + PICK_S4_UP_OFFSET),
        89,
        grip,
    ]


def make_hover_servos(pick, grip):
    if "apply_local_delta_correction" in globals():
        p = apply_local_delta_correction(pick)
    else:
        p = pick
    return [
        p["s1"],
        p["s2"],
        p["s3"],
        clamp(p["s4"] + LIFT_S4_ADD),
        89,
        grip,
    ]


def box_for_color(color):
    if color == "red":
        return RED_BOX_SERVO, "RED BOX"
    if color == "blue":
        return BLUE_BOX_SERVO, "BLUE BOX"
    return None, "UNKNOWN"


def full_sort_sequence(target):
    color = target["color"]
    pick = target["pick"]
    box_servo, label = box_for_color(color)
    if box_servo is None:
        print("[BLOCKED] unknown color")
        return
    print("=" * 60)
    print(f"[AUTO SORT] color={color}")
    print(f"robot X={target['x']:.2f}, Y={target['y']:.2f}, r={pick['r']:.2f}")
    print(f"pick S=[{pick['s1']},{pick['s2']},{pick['s3']},{pick['s4']},89,{GRIP_OPEN}]")
    print(f"box S={box_servo}")
    print("=" * 60)
    print_ik_report(target["x"], target["y"], pick, z_mm=112.3, pitch_deg=-70.0)
    approach_open = make_down_servos(pick, GRIP_OPEN)
    send_servo_timed(approach_open, "[1 CONTINUOUS APPROACH OPEN]", move_ms=APPROACH_MOVE_MS, extra_wait=0.10)
    close_pose = make_down_servos(pick, GRIP_CLOSE)
    send_servo_timed(close_pose, "[2 CLOSE]", move_ms=700, extra_wait=0.20)
    lift_closed = make_hover_servos(pick, GRIP_CLOSE)
    send_servo_timed(lift_closed, "[3 LIFT CLOSED]", move_ms=550, extra_wait=0.15)
    box_closed = list(box_servo)
    box_closed[5] = GRIP_CLOSE
    send_servo_timed(box_closed, f"[4 MOVE {label} CLOSED]", move_ms=900, extra_wait=0.20)
    box_open = list(box_servo)
    box_open[5] = GRIP_OPEN
    send_servo_timed(box_open, f"[5 OPEN {label}]", move_ms=350, extra_wait=0.25)
    send_servo_timed(CENTER_SERVO, "[6 RETURN CENTER]", move_ms=700, extra_wait=0.20)


def read_current_target_once(cap):
    ok, frame = cap.read()
    if not ok:
        return None
    und = cv2.undistort(frame, CAMERA_MATRIX, DIST_COEFFS)
    vis = und.copy()
    gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)
    H, roi_poly, found_ids = build_homography_and_roi(gray, vis)
    det, red_mask, blue_mask = detect_object(und, roi_poly)
    if det is None or H is None:
        return None
    px = det["px"]
    py = det["py"]
    color = det["color"]
    xw, yw = pixel_to_workspace(H, px, py)
    xr, yr = workspace_to_robot(xw, yw)
    pick = pick_model.predict(xr, yr)
    return {
        "color": color,
        "x": xr,
        "y": yr,
        "pick": pick,
    }


def flush_camera_frames(cap, n=AUTO_CLEAR_FRAME_FLUSH):
    for _ in range(n):
        cap.read()
        time.sleep(0.03)


def target_distance(a, b):
    dx = float(a["x"]) - float(b["x"])
    dy = float(a["y"]) - float(b["y"])
    return (dx * dx + dy * dy) ** 0.5


def read_fresh_target(cap, previous_targets):
    flush_camera_frames(cap)
    best = None
    for _ in range(20):
        t = read_current_target_once(cap)
        if t is None:
            time.sleep(0.08)
            continue
        repeated = False
        for old in previous_targets:
            if t["color"] == old["color"] and target_distance(t, old) < AUTO_CLEAR_SAME_TARGET_MM:
                repeated = True
                break
        if not repeated:
            return t
        best = t
        time.sleep(0.08)
    return None


def clear_area_sequence(cap):
    print("=" * 60)
    print("[AUTO CLEAR] started")
    print("=" * 60)
    sorted_targets = []
    count = 0
    while count < AUTO_CLEAR_MAX_OBJECTS:
        target = read_fresh_target(cap, sorted_targets)
        if target is None:
            print("[AUTO CLEAR] no fresh red/blue target found")
            break
        count += 1
        print(f"[AUTO CLEAR] sorting {count}: color={target['color']} X={target['x']:.2f} Y={target['y']:.2f} r={target['pick']['r']:.2f}")
        full_sort_sequence(target)
        sorted_targets.append(target)
        time.sleep(AUTO_CLEAR_WAIT_AFTER_SORT)
        flush_camera_frames(cap)
    print("=" * 60)
    print(f"[AUTO CLEAR] finished, sorted {count} object(s)")
    print("=" * 60)


def main():
    global locked_target
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"camera index {CAM_INDEX} open failed")
    print("Live radial AUTO sorting + IK report")
    print("a = clear all detected red/blue objects, then wait")
    print("s = lock target only")
    print("q = quit")
    print("red -> red box, blue -> blue box")
    current_target = None
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        und = cv2.undistort(frame, CAMERA_MATRIX, DIST_COEFFS)
        vis = und.copy()
        gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)
        H, roi_poly, found_ids = build_homography_and_roi(gray, vis)
        det, red_mask, blue_mask = detect_object(und, roi_poly)
        current_target = None
        cv2.putText(vis, "a auto sort | s lock | q quit", (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)
        cv2.putText(vis, f"Tags: {found_ids}", (20,60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,0), 2)
        if det is not None and H is not None:
            px = det["px"]
            py = det["py"]
            color = det["color"]
            xw, yw = pixel_to_workspace(H, px, py)
            xr, yr = workspace_to_robot(xw, yw)
            pick = pick_model.predict(xr, yr)
            current_target = {
                "color": color,
                "x": xr,
                "y": yr,
                "pick": pick,
            }
            draw_color = (0,0,255) if color == "red" else (255,0,0)
            cv2.drawContours(vis, [det["cnt"]], -1, draw_color, 2)
            cv2.circle(vis, (px, py), 6, draw_color, -1)
            cv2.putText(vis, f"color={color}", (20,95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, draw_color, 2)
            cv2.putText(vis, f"robot=({xr:.1f},{yr:.1f}) r={pick['r']:.1f}", (20,130), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,255), 2)
            cv2.putText(vis, f"S=[{pick['s1']},{pick['s2']},{pick['s3']},{pick['s4']},89,{GRIP_OPEN}]", (20,165), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        if locked_target is not None:
            cv2.putText(vis, f"LOCKED {locked_target['color']}", (20,205), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200,255,200), 2)
        cv2.imshow("auto_sort", vis)
        cv2.imshow("red_mask", red_mask)
        cv2.imshow("blue_mask", blue_mask)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            if current_target is None:
                print("[BLOCKED] no target")
                continue
            locked_target = current_target.copy()
            print("[LOCKED]", locked_target)
        elif key == ord("a"):
            clear_area_sequence(cap)
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
