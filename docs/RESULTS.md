# Results and interpretation

## Main result

All 121 targets were reachable. Desired and reconstructed paths both measured `1.98509` units. Position RMSE was `1.462e-16` units and maximum error was `3.140e-16` units, which is floating-point roundoff for this analytical simulation.

![Desired and reconstructed trajectory](../results/trajectory.svg)

The commanded angle ranges were:

| Joint | Minimum | Maximum |
|---|---:|---:|
| `theta1` | -31.735° | 1.912° |
| `theta2` | 75.040° | 131.202° |

![Joint angles](../results/joint_angles.svg)

## Interpretation

The result verifies internal consistency between the inverse- and forward-kinematics equations for the selected reachable path. It does not measure controller robustness. A more demanding study should add dynamic plant behavior, perturbations, actuator limits, and comparisons among control strategies.
