# final300_faster_actuator

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 3; simulated seconds: 107.15; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2143 | 0.04020 | 1.51696 | 2.57898 |
| startup | 199 | 0.04582 | 1.63861 | 8.43454 |
| steady | 1944 | 0.03958 | 1.50395 | 0.22247 |
| crossing_1 | 147 | 0.00404 | 0.12021 | 0.02357 |
| crossing_2 | 147 | 0.00402 | 0.11915 | 0.02767 |
| seam | 121 | 0.02400 | 1.05685 | 10.09709 |
| other | 1728 | 0.04428 | 1.66529 | 1.05329 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 34.38 deg/s. Maximum lateral acceleration: 28.84 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
