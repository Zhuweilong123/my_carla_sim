# final300_noise

Protocol: route_projection_v2; Git: 784c1d57819db21323491a626d5bcd96fe17c795

Completed laps: 3; simulated seconds: 107.15; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2143 | 0.04985 | 1.55644 | 2.64666 |
| startup | 199 | 0.05353 | 1.54224 | 8.65653 |
| steady | 1944 | 0.04946 | 1.55789 | 0.22577 |
| crossing_1 | 147 | 0.01766 | 0.34903 | 0.02465 |
| crossing_2 | 147 | 0.01444 | 0.28263 | 0.02928 |
| seam | 119 | 0.03306 | 0.98515 | 10.40565 |
| other | 1730 | 0.05440 | 1.70790 | 1.10856 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 34.38 deg/s. Maximum lateral acceleration: 15.31 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
