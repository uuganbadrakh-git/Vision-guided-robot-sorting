#!/usr/bin/env python3
import json
import os
import threading

POSE_FILE = os.path.expanduser("~/dofbot_nano/current_pose.json")

_lock = threading.Lock()

DEFAULT_POSE = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "pitch": 0.0,
    "roll": 0.0,
    "grip": 0,
}

def ensure_pose_file():
    os.makedirs(os.path.dirname(POSE_FILE), exist_ok=True)
    if not os.path.exists(POSE_FILE):
        with open(POSE_FILE, "w", encoding="utf-8") as f:
            json.dump(DEFAULT_POSE, f, indent=2)


def save_pose(x, y, z, pitch, roll, grip):
    ensure_pose_file()
    data = {
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "pitch": float(pitch),
        "roll": float(roll),
        "grip": int(grip),
    }
    with _lock:
        with open(POSE_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)


def load_pose():
    ensure_pose_file()
    with _lock:
        with open(POSE_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
