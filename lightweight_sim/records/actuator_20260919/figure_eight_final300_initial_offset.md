# final300_initial_offset

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 3; simulated seconds: 107.15; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2143 | 0.05910 | 1.61997 | 2.61192 |
| startup | 199 | 0.12817 | 2.13571 | 8.54272 |
| steady | 1944 | 0.04657 | 1.55758 | 0.22359 |
| crossing_1 | 147 | 0.00498 | 0.11392 | 0.02320 |
| crossing_2 | 147 | 0.00496 | 0.11430 | 0.02741 |
| seam | 120 | 0.15384 | 1.99589 | 10.24852 |
| other | 1729 | 0.05179 | 1.72453 | 1.07975 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 34.38 deg/s. Maximum lateral acceleration: 13.92 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
