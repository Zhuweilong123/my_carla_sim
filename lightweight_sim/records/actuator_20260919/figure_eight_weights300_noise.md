# weights300_noise

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.04665 | 1.46535 | 4.98834 |
| startup | 199 | 0.05353 | 1.54224 | 8.65653 |
| steady | 401 | 0.04282 | 1.42566 | 0.21166 |
| crossing_1 | 49 | 0.01730 | 0.47184 | 0.00704 |
| crossing_2 | 49 | 0.01825 | 0.27455 | 0.02909 |
| seam | 30 | 0.02121 | 1.02377 | 20.72317 |
| other | 472 | 0.05169 | 1.62235 | 2.08231 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 34.38 deg/s. Maximum lateral acceleration: 14.81 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
