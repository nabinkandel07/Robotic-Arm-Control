# Robotic Arm Control

A dependency-free Python demonstration of inverse-kinematics control for a planar two-link robotic arm. The demo generates a circular Cartesian target, solves the elbow-down inverse kinematics, verifies the solution with forward kinematics, and exports the complete trajectory.

## Run

```bash
python src/run_demo.py
```

Python 3.10 or newer is recommended. No third-party packages are required.

## Reproducible demo result

| Metric | Result |
|---|---:|
| Trajectory samples | 121 |
| Position RMSE | `1.462e-16` units |
| Maximum position error | `3.140e-16` units |
| Reachable targets | 121 / 121 |

The near-zero error is expected because this is an analytical kinematics simulation without motor dynamics, sensor noise, collision constraints, or joint limits. It is a software baseline, not a physical-robot validation.

Generated artifacts are committed under `results/`: `metrics.json`, `trajectory.csv`, `trajectory.svg`, and `joint_angles.svg`.

For an academic review, see [methodology](docs/METHODOLOGY.md), [result interpretation](docs/RESULTS.md), and the [data dictionary](docs/DATA_DICTIONARY.md). Run the verification suite with `python -m unittest discover -s tests -v`.

## Layout

```text
src/run_demo.py       inverse and forward kinematics simulation
results/metrics.json  summary metrics
results/trajectory.csv full numerical trajectory
results/trajectory.svg trajectory visualization
results/joint_angles.svg joint-command visualization
docs/                  methodology, results, and field definitions
tests/                 independent kinematics checks
```

## Next steps

Add joint limits, velocity and acceleration constraints, a PID or model-predictive controller, and hardware measurements from a real arm.
