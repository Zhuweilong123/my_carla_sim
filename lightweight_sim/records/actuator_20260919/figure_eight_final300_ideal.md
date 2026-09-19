# final300_ideal

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 10; simulated seconds: 355.05; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 7101 | 0.01702 | 1.61121 | 1.46606 |
| startup | 199 | 0.02051 | 1.61188 | 8.65761 |
| steady | 6902 | 0.01691 | 1.61119 | 0.22408 |
| crossing_1 | 490 | 0.00191 | 0.13384 | 0.02668 |
| crossing_2 | 491 | 0.00193 | 0.13370 | 0.02787 |
| seam | 370 | 0.01304 | 1.01050 | 5.90270 |
| other | 5750 | 0.01860 | 1.77121 | 0.64204 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 135.16 deg/s. Maximum lateral acceleration: 20.81 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
