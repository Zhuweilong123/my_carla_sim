# p2_candidate_plant_mismatch

Protocol: route_projection_v2; Git: 101a30ce395f2cb8548c871fd2aea9168b3d9767

Completed laps: 3; simulated seconds: 107.35; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2147 | 0.01269 | 1.49109 | 2.64073 |
| startup | 199 | 0.01538 | 1.50068 | 8.66176 |
| steady | 1948 | 0.01239 | 1.49011 | 0.14650 |
| crossing_1 | 147 | 0.00194 | 0.13002 | 0.01818 |
| crossing_2 | 147 | 0.00189 | 0.13178 | 0.01792 |
| seam | 121 | 0.01378 | 0.98905 | 10.32043 |
| other | 1732 | 0.01363 | 1.63854 | 1.09693 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 117.68 deg/s. Maximum lateral acceleration: 17.69 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
