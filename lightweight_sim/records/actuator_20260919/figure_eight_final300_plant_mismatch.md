# final300_plant_mismatch

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 3; simulated seconds: 107.35; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2147 | 0.03521 | 1.43152 | 2.64095 |
| startup | 199 | 0.03529 | 1.43399 | 8.66251 |
| steady | 1948 | 0.03520 | 1.43127 | 0.14643 |
| crossing_1 | 147 | 0.00425 | 0.10658 | 0.01808 |
| crossing_2 | 147 | 0.00437 | 0.10828 | 0.01766 |
| seam | 121 | 0.02259 | 0.92594 | 10.32098 |
| other | 1732 | 0.03871 | 1.57430 | 1.09724 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 32.96 deg/s. Maximum lateral acceleration: 13.58 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
