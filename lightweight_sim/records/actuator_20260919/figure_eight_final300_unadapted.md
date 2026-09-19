# final300_unadapted

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 0; simulated seconds: 4.80; passed: False

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 96 | 0.99803 | 15.78582 | 12.43737 |
| startup | 96 | 0.99803 | 15.78582 | 12.43737 |
| seam | 30 | 0.02785 | 1.18590 | 20.71707 |
| other | 66 | 1.20353 | 19.02162 | 5.46913 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 30.208%. Maximum steering rate: 34.38 deg/s. Maximum lateral acceleration: 18.43 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
