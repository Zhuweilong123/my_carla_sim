# acceptance01_slower_actuator

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 3; simulated seconds: 107.20; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2144 | 0.04210 | 1.50301 | 2.64337 |
| startup | 199 | 0.04557 | 1.54169 | 8.64826 |
| steady | 1945 | 0.04173 | 1.49900 | 0.22367 |
| crossing_1 | 147 | 0.00400 | 0.10786 | 0.02367 |
| crossing_2 | 147 | 0.00423 | 0.11069 | 0.02794 |
| seam | 121 | 0.02651 | 1.04952 | 10.31184 |
| other | 1729 | 0.04633 | 1.64989 | 1.10584 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 34.38 deg/s. Maximum lateral acceleration: 14.42 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
