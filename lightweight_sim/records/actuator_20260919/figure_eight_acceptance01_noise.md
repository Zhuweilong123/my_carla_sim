# acceptance01_noise

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 3; simulated seconds: 107.20; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2144 | 0.04311 | 1.56430 | 2.64523 |
| startup | 199 | 0.04409 | 1.59116 | 8.65329 |
| steady | 1945 | 0.04301 | 1.56153 | 0.22800 |
| crossing_1 | 147 | 0.02013 | 0.47895 | 0.02752 |
| crossing_2 | 147 | 0.01281 | 0.32678 | 0.03130 |
| seam | 121 | 0.03449 | 1.07793 | 10.31651 |
| other | 1729 | 0.04662 | 1.71012 | 1.10831 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 34.38 deg/s. Maximum lateral acceleration: 17.38 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
