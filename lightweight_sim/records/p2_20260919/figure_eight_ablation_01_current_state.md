# ablation_01_current_state

Protocol: route_projection_v2; Git: 101a30ce395f2cb8548c871fd2aea9168b3d9767

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.02645 | 2.13101 | 3.11594 |
| startup | 199 | 0.03164 | 2.69693 | 5.37711 |
| steady | 401 | 0.02345 | 1.78475 | 0.42285 |
| crossing_1 | 49 | 0.00155 | 0.12061 | 0.03561 |
| crossing_2 | 49 | 0.00165 | 0.11823 | 0.05874 |
| seam | 25 | 0.06731 | 5.59034 | 14.52646 |
| other | 477 | 0.02534 | 2.01776 | 1.07357 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 4.667%. Maximum steering rate: 1145.92 deg/s. Maximum lateral acceleration: 313.18 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
