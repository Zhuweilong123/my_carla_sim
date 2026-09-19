# p2_candidate_position_noise

Protocol: route_projection_v2; Git: 101a30ce395f2cb8548c871fd2aea9168b3d9767

Completed laps: 3; simulated seconds: 107.15; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2143 | 0.02611 | 1.72539 | 2.63070 |
| startup | 199 | 0.03054 | 1.71897 | 8.59774 |
| steady | 1944 | 0.02561 | 1.72605 | 0.24894 |
| crossing_1 | 147 | 0.02240 | 0.63839 | 0.05281 |
| crossing_2 | 147 | 0.01875 | 0.51817 | 0.05433 |
| seam | 120 | 0.02481 | 1.19075 | 10.30880 |
| other | 1729 | 0.02701 | 1.87987 | 1.09612 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 326.32 deg/s. Maximum lateral acceleration: 34.64 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
