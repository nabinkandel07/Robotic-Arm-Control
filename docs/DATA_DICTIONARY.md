# Data dictionary

`results/trajectory.csv` contains one row per target sample.

| Column | Unit | Meaning |
|---|---|---|
| `step` | index | Sequential sample number, 0–120 |
| `desired_x`, `desired_y` | model units | Requested end-effector coordinates |
| `actual_x`, `actual_y` | model units | Coordinates reconstructed from joint commands |
| `theta1_rad`, `theta2_rad` | radians | Analytical joint-angle commands |
| `error` | model units | Euclidean distance between requested and reconstructed endpoint |

`results/metrics.json` records aggregate statistics and explicitly identifies the data as a deterministic synthetic trajectory.
