"""Dependency-free planar two-link robotic arm tracking demo."""

import csv
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"


def inverse_kinematics(x: float, y: float, l1: float = 1.0, l2: float = 0.8):
    radius_sq = x * x + y * y
    cosine = (radius_sq - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    cosine = max(-1.0, min(1.0, cosine))
    theta2 = math.acos(cosine)
    theta1 = math.atan2(y, x) - math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    return theta1, theta2


def forward_kinematics(theta1: float, theta2: float, l1: float = 1.0, l2: float = 0.8):
    return (
        l1 * math.cos(theta1) + l2 * math.cos(theta1 + theta2),
        l1 * math.sin(theta1) + l2 * math.sin(theta1 + theta2),
    )


def svg_path(rows):
    width, height, margin = 720, 480, 40
    xs = [r[1] for r in rows] + [r[3] for r in rows]
    ys = [r[2] for r in rows] + [r[4] for r in rows]
    xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
    def point(x, y):
        px = margin + (x - xmin) / (xmax - xmin) * (width - 2 * margin)
        py = height - margin - (y - ymin) / (ymax - ymin) * (height - 2 * margin)
        return f"{px:.1f},{py:.1f}"
    desired = " ".join(point(r[1], r[2]) for r in rows)
    actual = " ".join(point(r[3], r[4]) for r in rows)
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="white"/><text x="40" y="24" font-family="sans-serif" font-size="16">Two-link arm trajectory tracking</text>
<polyline points="{desired}" fill="none" stroke="#2563eb" stroke-width="5"/><polyline points="{actual}" fill="none" stroke="#ef4444" stroke-width="2" stroke-dasharray="7 5"/>
<text x="520" y="24" fill="#2563eb" font-family="sans-serif">desired</text><text x="605" y="24" fill="#ef4444" font-family="sans-serif">actual</text></svg>'''


def svg_joint_angles(rows):
    width, height, margin = 720, 420, 45
    values = [math.degrees(r[5]) for r in rows] + [math.degrees(r[6]) for r in rows]
    low, high = min(values), max(values)
    def point(step, value):
        x = margin + step / (len(rows) - 1) * (width - 2 * margin)
        y = height - margin - (value - low) / (high - low) * (height - 2 * margin)
        return f"{x:.1f},{y:.1f}"
    theta1 = " ".join(point(r[0], math.degrees(r[5])) for r in rows)
    theta2 = " ".join(point(r[0], math.degrees(r[6])) for r in rows)
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="white"/><text x="45" y="24" font-family="sans-serif" font-size="16">Joint-angle commands</text>
<polyline points="{theta1}" fill="none" stroke="#7c3aed" stroke-width="3"/><polyline points="{theta2}" fill="none" stroke="#059669" stroke-width="3"/>
<text x="560" y="24" fill="#7c3aed" font-family="sans-serif">theta 1</text><text x="635" y="24" fill="#059669" font-family="sans-serif">theta 2</text></svg>'''


def main():
    RESULTS.mkdir(exist_ok=True)
    rows = []
    errors = []
    for step in range(121):
        t = 2 * math.pi * step / 120
        desired_x = 0.95 + 0.35 * math.cos(t)
        desired_y = 0.55 + 0.28 * math.sin(t)
        theta1, theta2 = inverse_kinematics(desired_x, desired_y)
        actual_x, actual_y = forward_kinematics(theta1, theta2)
        error = math.hypot(desired_x - actual_x, desired_y - actual_y)
        errors.append(error)
        rows.append((step, desired_x, desired_y, actual_x, actual_y, theta1, theta2, error))
    with (RESULTS / "trajectory.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["step", "desired_x", "desired_y", "actual_x", "actual_y", "theta1_rad", "theta2_rad", "error"])
        writer.writerows(rows)
    metrics = {
        "samples": len(rows),
        "rmse_position": math.sqrt(sum(e * e for e in errors) / len(errors)),
        "max_position_error": max(errors),
        "reachable_samples": sum(math.hypot(r[1], r[2]) <= 1.8 for r in rows),
        "desired_path_length": sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(rows, rows[1:])),
        "actual_path_length": sum(math.hypot(b[3] - a[3], b[4] - a[4]) for a, b in zip(rows, rows[1:])),
        "theta1_range_deg": [min(math.degrees(r[5]) for r in rows), max(math.degrees(r[5]) for r in rows)],
        "theta2_range_deg": [min(math.degrees(r[6]) for r in rows), max(math.degrees(r[6]) for r in rows)],
        "data": "deterministic synthetic circular trajectory",
    }
    (RESULTS / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    (RESULTS / "trajectory.svg").write_text(svg_path(rows), encoding="utf-8")
    (RESULTS / "joint_angles.svg").write_text(svg_joint_angles(rows), encoding="utf-8")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
