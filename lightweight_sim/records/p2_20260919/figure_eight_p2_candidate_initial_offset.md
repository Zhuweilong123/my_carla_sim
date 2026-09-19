# p2_candidate_initial_offset

Protocol: route_projection_v2; Git: 101a30ce395f2cb8548c871fd2aea9168b3d9767

Completed laps: 3; simulated seconds: 107.15; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2143 | 0.02488 | 1.66332 | 2.53066 |
| startup | 199 | 0.06290 | 1.96601 | 8.27514 |
| steady | 1944 | 0.01665 | 1.62916 | 0.22361 |
| crossing_1 | 147 | 0.00195 | 0.13130 | 0.02327 |
| crossing_2 | 147 | 0.00193 | 0.13258 | 0.02768 |
| seam | 119 | 0.07824 | 1.75009 | 9.91411 |
| other | 1730 | 0.01857 | 1.79261 | 1.08262 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.047%. Maximum steering rate: 446.62 deg/s. Maximum lateral acceleration: 52.81 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
