#!/usr/bin/env python3
import json
import math

class RadialPickModel:
    def __init__(self, path="radial_pick_points.json"):
        with open(path, "r", encoding="utf-8") as f:
            self.points = json.load(f)
        self.points = sorted(self.points, key=lambda p: float(p["r"]))

    def clamp(self, v, lo=0, hi=180):
        return max(lo, min(hi, int(round(v))))

    def xy_to_s1(self, x, y):
        angle = math.degrees(math.atan2(y, x))
        s1 = 89 + angle
        return self.clamp(s1), angle

    def interp_value(self, r, key):
        pts = self.points
        if r <= float(pts[0]["r"]):
            return float(pts[0][key])
        if r >= float(pts[-1]["r"]):
            return float(pts[-1][key])

        for i in range(len(pts) - 1):
            p0 = pts[i]
            p1 = pts[i + 1]
            r0 = float(p0["r"])
            r1 = float(p1["r"])
            if r0 <= r <= r1:
                t = (r - r0) / (r1 - r0)
                v0 = float(p0[key])
                v1 = float(p1[key])
                return v0 + t * (v1 - v0)

        return float(pts[-1][key])

    def predict(self, x, y):
        r = math.sqrt(x * x + y * y)
        s1, angle = self.xy_to_s1(x, y)
        s2 = self.clamp(self.interp_value(r, "s2"))
        s3 = self.clamp(self.interp_value(r, "s3"))
        s4 = self.clamp(self.interp_value(r, "s4"))
        open_grip = self.clamp(self.interp_value(r, "open"))
        close_grip = self.clamp(self.interp_value(r, "close"))
        return {
            "x": x,
            "y": y,
            "r": r,
            "angle": angle,
            "s1": s1,
            "s2": s2,
            "s3": s3,
            "s4": s4,
            "s5": 89,
            "open": open_grip,
            "close": close_grip,
        }

if __name__ == "__main__":
    model = RadialPickModel()
    tests = [
        (74.25, -22.67),
        (118.44, -21.75),
        (134.56, 9.39),
        (174.50, 12.05),
    ]
    for x, y in tests:
        p = model.predict(x, y)
        print(
            f"X={x:.2f}, Y={y:.2f}, r={p['r']:.2f}, "
            f"S=[{p['s1']},{p['s2']},{p['s3']},{p['s4']},89,{p['open']}], "
            f"close={p['close']}"
        )
