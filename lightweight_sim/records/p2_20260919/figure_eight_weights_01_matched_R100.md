# weights_01_matched_R100

Protocol: route_projection_v2; Git: 101a30ce395f2cb8548c871fd2aea9168b3d9767

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.02207 | 1.77210 | 4.98360 |
| startup | 199 | 0.02147 | 1.83116 | 8.64292 |
| steady | 401 | 0.02237 | 1.74206 | 0.30156 |
| crossing_1 | 49 | 0.00220 | 0.13957 | 0.00387 |
| crossing_2 | 49 | 0.00236 | 0.13836 | 0.04299 |
| seam | 30 | 0.01272 | 0.90133 | 20.71046 |
| other | 472 | 0.02466 | 1.98402 | 2.07589 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 524.19 deg/s. Maximum lateral acceleration: 57.90 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
