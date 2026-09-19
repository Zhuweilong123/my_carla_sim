# weights300_adapted

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.04605 | 1.44293 | 4.98936 |
| startup | 199 | 0.05051 | 1.51323 | 8.65836 |
| steady | 401 | 0.04367 | 1.40674 | 0.21028 |
| crossing_1 | 49 | 0.00512 | 0.11596 | 0.01126 |
| crossing_2 | 49 | 0.00511 | 0.11539 | 0.02788 |
| seam | 30 | 0.02234 | 1.03064 | 20.72621 |
| other | 472 | 0.05156 | 1.60511 | 2.08347 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 23.12 deg/s. Maximum lateral acceleration: 13.63 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
