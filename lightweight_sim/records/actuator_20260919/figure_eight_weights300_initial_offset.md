# weights300_initial_offset

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.08202 | 1.71453 | 4.92280 |
| startup | 199 | 0.12817 | 2.13571 | 8.54272 |
| steady | 401 | 0.04375 | 1.46112 | 0.21032 |
| crossing_1 | 49 | 0.00502 | 0.11416 | 0.01053 |
| crossing_2 | 49 | 0.00500 | 0.11366 | 0.02755 |
| seam | 30 | 0.30434 | 3.63469 | 20.49580 |
| other | 472 | 0.05158 | 1.70130 | 2.02630 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 34.38 deg/s. Maximum lateral acceleration: 13.92 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
