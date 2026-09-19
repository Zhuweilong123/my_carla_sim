# screening01_adapted

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.03612 | 1.44615 | 4.98869 |
| startup | 199 | 0.04055 | 1.51550 | 8.65724 |
| steady | 401 | 0.03370 | 1.41046 | 0.20946 |
| crossing_1 | 49 | 0.00347 | 0.11740 | 0.01123 |
| crossing_2 | 49 | 0.00346 | 0.11724 | 0.02792 |
| seam | 30 | 0.02280 | 1.06930 | 20.72408 |
| other | 472 | 0.04029 | 1.60716 | 2.08278 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 27.81 deg/s. Maximum lateral acceleration: 13.93 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
