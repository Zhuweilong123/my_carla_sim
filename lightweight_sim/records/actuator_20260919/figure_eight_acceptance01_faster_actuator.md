# acceptance01_faster_actuator

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 3; simulated seconds: 107.15; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2143 | 0.04788 | 1.50720 | 2.56328 |
| startup | 199 | 0.04920 | 1.60353 | 8.38248 |
| steady | 1944 | 0.04774 | 1.49699 | 0.22385 |
| crossing_1 | 148 | 0.02435 | 0.13437 | 0.02376 |
| crossing_2 | 147 | 0.02473 | 0.12957 | 0.02835 |
| seam | 120 | 0.03053 | 1.05394 | 10.03136 |
| other | 1728 | 0.05173 | 1.65442 | 1.07711 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 34.38 deg/s. Maximum lateral acceleration: 32.25 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
