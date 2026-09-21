# smooth_01_smooth_R1

Protocol: route_projection_v2; Git: 101a30ce395f2cb8548c871fd2aea9168b3d9767

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.01495 | 1.58055 | 4.98810 |
| startup | 199 | 0.01657 | 1.66973 | 8.65589 |
| steady | 401 | 0.01407 | 1.53437 | 0.21601 |
| crossing_1 | 49 | 0.00196 | 0.14216 | 0.00988 |
| crossing_2 | 49 | 0.00209 | 0.13533 | 0.02849 |
| seam | 30 | 0.01551 | 1.04869 | 20.72275 |
| other | 472 | 0.01637 | 1.76116 | 2.08183 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 191.52 deg/s. Maximum lateral acceleration: 25.02 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
