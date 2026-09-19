# acceptance01_adapted

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 10; simulated seconds: 354.95; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 7099 | 0.03599 | 1.53471 | 1.46605 |
| startup | 199 | 0.04055 | 1.51550 | 8.65724 |
| steady | 6900 | 0.03585 | 1.53526 | 0.22302 |
| crossing_1 | 490 | 0.00339 | 0.11619 | 0.02644 |
| crossing_2 | 490 | 0.00338 | 0.11632 | 0.02761 |
| seam | 370 | 0.02182 | 0.96185 | 5.90237 |
| other | 5749 | 0.03959 | 1.68718 | 0.64166 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 34.38 deg/s. Maximum lateral acceleration: 14.50 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
