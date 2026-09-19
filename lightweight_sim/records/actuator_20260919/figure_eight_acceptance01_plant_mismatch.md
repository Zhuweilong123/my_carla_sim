# acceptance01_plant_mismatch

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 3; simulated seconds: 107.35; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2147 | 0.02482 | 1.41504 | 2.64057 |
| startup | 199 | 0.02692 | 1.42833 | 8.66135 |
| steady | 1948 | 0.02459 | 1.41368 | 0.14578 |
| crossing_1 | 147 | 0.00276 | 0.10802 | 0.01807 |
| crossing_2 | 147 | 0.00285 | 0.10875 | 0.01762 |
| seam | 121 | 0.02074 | 0.92694 | 10.31989 |
| other | 1732 | 0.02706 | 1.55567 | 1.09680 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 34.38 deg/s. Maximum lateral acceleration: 14.11 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
