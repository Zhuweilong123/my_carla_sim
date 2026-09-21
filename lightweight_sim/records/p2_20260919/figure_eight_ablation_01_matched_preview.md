# ablation_01_matched_preview

Protocol: route_projection_v2; Git: 101a30ce395f2cb8548c871fd2aea9168b3d9767

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.05575 | 1.86890 | 4.95159 |
| startup | 199 | 0.04928 | 1.90367 | 8.54718 |
| steady | 401 | 0.05870 | 1.85140 | 0.65705 |
| crossing_1 | 49 | 0.06534 | 1.46227 | 0.53144 |
| crossing_2 | 49 | 0.06547 | 1.44907 | 0.56310 |
| seam | 30 | 0.01322 | 0.91835 | 20.64537 |
| other | 472 | 0.05525 | 1.98655 | 2.00350 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 53.833%. Maximum steering rate: 1145.92 deg/s. Maximum lateral acceleration: 87.77 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
