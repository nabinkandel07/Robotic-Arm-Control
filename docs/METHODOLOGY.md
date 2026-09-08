# Methodology

## Research question

Can analytical inverse kinematics produce joint commands that follow a reachable Cartesian trajectory for a planar two-link arm?

## Model

The arm uses link lengths `L1 = 1.0` and `L2 = 0.8`. Forward kinematics are:

```text
x = L1 cos(theta1) + L2 cos(theta1 + theta2)
y = L1 sin(theta1) + L2 sin(theta1 + theta2)
```

Inverse kinematics applies the law of cosines to obtain `theta2`, then resolves `theta1` from two `atan2` terms. The implementation selects the positive-`acos` elbow configuration and clips the cosine argument to the numerical interval `[-1, 1]`.

## Experiment

The target is a 121-point closed ellipse centered at `(0.95, 0.55)` with radii `0.35` and `0.28`. Every target is checked against the maximum reach `L1 + L2`. Forward kinematics reconstruct the endpoint from the commanded angles, and Euclidean endpoint error is measured.

## Reproducibility

Run `python src/run_demo.py` from the repository root. The script has no random component or external dependency. Run `python -m unittest discover -s tests -v` to verify inverse/forward round trips.

## Scope and validity

This is a kinematic baseline. It does not model torque, inertia, gearbox backlash, latency, sampling, sensor noise, joint limits, singularity avoidance, or collision constraints. Physical claims require experiments on a specified robot and calibrated measurement system.
