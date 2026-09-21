# ablation_01_matched_current

Protocol: route_projection_v2; Git: 101a30ce395f2cb8548c871fd2aea9168b3d9767

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.02404 | 1.86911 | 4.98316 |
| startup | 199 | 0.02539 | 1.91133 | 8.63332 |
| steady | 401 | 0.02334 | 1.84779 | 0.40823 |
| crossing_1 | 49 | 0.00189 | 0.12963 | 0.02200 |
| crossing_2 | 49 | 0.00172 | 0.13103 | 0.05918 |
| seam | 30 | 0.01286 | 0.88303 | 20.70278 |
| other | 472 | 0.02689 | 2.09473 | 2.07935 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 2.667%. Maximum steering rate: 624.59 deg/s. Maximum lateral acceleration: 66.01 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
