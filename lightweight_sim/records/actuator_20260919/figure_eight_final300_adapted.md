# final300_adapted

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 10; simulated seconds: 354.95; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 7099 | 0.04657 | 1.51899 | 1.46635 |
| startup | 199 | 0.05051 | 1.51323 | 8.65836 |
| steady | 6900 | 0.04645 | 1.51915 | 0.22386 |
| crossing_1 | 490 | 0.00504 | 0.11506 | 0.02647 |
| crossing_2 | 490 | 0.00505 | 0.11550 | 0.02771 |
| seam | 371 | 0.02490 | 0.97500 | 5.89501 |
| other | 5748 | 0.05132 | 1.66913 | 0.64223 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 34.23 deg/s. Maximum lateral acceleration: 13.96 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
