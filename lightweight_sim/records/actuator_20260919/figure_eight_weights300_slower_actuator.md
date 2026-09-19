# weights300_slower_actuator

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.05471 | 1.46294 | 4.98741 |
| startup | 199 | 0.05819 | 1.53091 | 8.65492 |
| steady | 401 | 0.05290 | 1.42801 | 0.21151 |
| crossing_1 | 49 | 0.00618 | 0.11188 | 0.01119 |
| crossing_2 | 49 | 0.00618 | 0.11106 | 0.02793 |
| seam | 30 | 0.02623 | 1.08538 | 20.71951 |
| other | 472 | 0.06127 | 1.62577 | 2.08179 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 18.92 deg/s. Maximum lateral acceleration: 13.66 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
