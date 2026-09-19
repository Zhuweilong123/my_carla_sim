# weights300_faster_actuator

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.04027 | 1.51299 | 4.86050 |
| startup | 199 | 0.04582 | 1.63861 | 8.43454 |
| steady | 401 | 0.03722 | 1.44660 | 0.20906 |
| crossing_1 | 49 | 0.00391 | 0.11760 | 0.00992 |
| crossing_2 | 49 | 0.00389 | 0.11727 | 0.02717 |
| seam | 30 | 0.02440 | 1.26309 | 20.27690 |
| other | 472 | 0.04495 | 1.67501 | 1.97444 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 34.38 deg/s. Maximum lateral acceleration: 28.84 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
