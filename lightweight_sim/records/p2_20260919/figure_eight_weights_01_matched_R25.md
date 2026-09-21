# weights_01_matched_R25

Protocol: route_projection_v2; Git: 101a30ce395f2cb8548c871fd2aea9168b3d9767

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.02251 | 1.83702 | 4.98311 |
| startup | 199 | 0.02288 | 1.88317 | 8.63697 |
| steady | 401 | 0.02232 | 1.81368 | 0.36681 |
| crossing_1 | 49 | 0.00201 | 0.13466 | 0.01555 |
| crossing_2 | 49 | 0.00211 | 0.13408 | 0.05249 |
| seam | 30 | 0.01284 | 0.89009 | 20.70575 |
| other | 472 | 0.02516 | 2.05808 | 2.07733 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 1.500%. Maximum steering rate: 609.27 deg/s. Maximum lateral acceleration: 64.76 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
