# weights300_plant_mismatch

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.03367 | 1.37493 | 4.99005 |
| startup | 199 | 0.03529 | 1.43399 | 8.66251 |
| steady | 401 | 0.03283 | 1.34466 | 0.13772 |
| crossing_1 | 49 | 0.00422 | 0.10654 | 0.01947 |
| crossing_2 | 49 | 0.00434 | 0.10763 | 0.01756 |
| seam | 30 | 0.02097 | 1.02086 | 20.72728 |
| other | 472 | 0.03754 | 1.52790 | 2.08493 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 23.12 deg/s. Maximum lateral acceleration: 13.40 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
