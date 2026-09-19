# final300_slower_actuator

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 3; simulated seconds: 107.20; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2144 | 0.05655 | 1.52007 | 2.64550 |
| startup | 199 | 0.05819 | 1.53091 | 8.65492 |
| steady | 1945 | 0.05638 | 1.51895 | 0.22508 |
| crossing_1 | 147 | 0.00617 | 0.11197 | 0.02367 |
| crossing_2 | 147 | 0.00616 | 0.11281 | 0.02787 |
| seam | 121 | 0.02808 | 1.01006 | 10.31747 |
| other | 1729 | 0.06248 | 1.67083 | 1.10849 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 26.67 deg/s. Maximum lateral acceleration: 13.98 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
