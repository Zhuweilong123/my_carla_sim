# screening01_ideal

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.01771 | 1.51738 | 4.98893 |
| startup | 199 | 0.02051 | 1.61188 | 8.65761 |
| steady | 401 | 0.01615 | 1.46823 | 0.21052 |
| crossing_1 | 49 | 0.00187 | 0.13539 | 0.01088 |
| crossing_2 | 50 | 0.00200 | 0.13947 | 0.02818 |
| seam | 30 | 0.01783 | 1.04282 | 20.72506 |
| other | 471 | 0.01946 | 1.69110 | 2.08512 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 105.04 deg/s. Maximum lateral acceleration: 17.50 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
