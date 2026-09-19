# acceptance01_initial_offset

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 3; simulated seconds: 107.15; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2143 | 0.05796 | 1.72899 | 2.58621 |
| startup | 199 | 0.15383 | 2.99570 | 8.45829 |
| steady | 1944 | 0.03579 | 1.54167 | 0.22269 |
| crossing_1 | 147 | 0.00333 | 0.11550 | 0.02294 |
| crossing_2 | 147 | 0.00332 | 0.11609 | 0.02734 |
| seam | 121 | 0.17185 | 2.96696 | 10.14094 |
| other | 1728 | 0.04579 | 1.75746 | 1.04576 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.187%. Maximum steering rate: 34.38 deg/s. Maximum lateral acceleration: 13.68 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
