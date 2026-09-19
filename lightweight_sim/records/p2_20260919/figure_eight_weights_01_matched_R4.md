# weights_01_matched_R4

Protocol: route_projection_v2; Git: 101a30ce395f2cb8548c871fd2aea9168b3d9767

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.02345 | 1.86817 | 4.98312 |
| startup | 199 | 0.02490 | 1.90733 | 8.63400 |
| steady | 401 | 0.02270 | 1.84843 | 0.40028 |
| crossing_1 | 49 | 0.00190 | 0.13055 | 0.02082 |
| crossing_2 | 49 | 0.00184 | 0.13170 | 0.05798 |
| seam | 30 | 0.01286 | 0.88406 | 20.70324 |
| other | 472 | 0.02623 | 2.09363 | 2.07895 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 2.333%. Maximum steering rate: 620.96 deg/s. Maximum lateral acceleration: 65.86 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
