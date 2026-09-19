# weights_01_matched_R10

Protocol: route_projection_v2; Git: 101a30ce395f2cb8548c871fd2aea9168b3d9767

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.02324 | 1.85494 | 4.98329 |
| startup | 199 | 0.02418 | 1.89961 | 8.63502 |
| steady | 401 | 0.02276 | 1.83237 | 0.39246 |
| crossing_1 | 49 | 0.00194 | 0.13204 | 0.01918 |
| crossing_2 | 49 | 0.00186 | 0.13356 | 0.05676 |
| seam | 30 | 0.01286 | 0.88599 | 20.70407 |
| other | 472 | 0.02599 | 2.07855 | 2.07894 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 2.167%. Maximum steering rate: 616.12 deg/s. Maximum lateral acceleration: 65.23 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
