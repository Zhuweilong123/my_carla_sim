# p2_candidate_delay_50ms

Protocol: route_projection_v2; Git: 101a30ce395f2cb8548c871fd2aea9168b3d9767

Completed laps: 3; simulated seconds: 107.05; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2141 | 0.01709 | 1.54956 | 2.36469 |
| startup | 199 | 0.02225 | 1.69850 | 7.72329 |
| steady | 1942 | 0.01647 | 1.53348 | 0.22888 |
| crossing_1 | 147 | 0.00343 | 0.18832 | 0.02238 |
| crossing_2 | 147 | 0.00343 | 0.18850 | 0.02727 |
| seam | 118 | 0.02394 | 1.43863 | 9.41028 |
| other | 1729 | 0.01791 | 1.68108 | 0.93837 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 264.36 deg/s. Maximum lateral acceleration: 91.37 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
