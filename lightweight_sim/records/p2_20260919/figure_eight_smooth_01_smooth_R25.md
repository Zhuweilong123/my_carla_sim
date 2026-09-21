# smooth_01_smooth_R25

Protocol: route_projection_v2; Git: 101a30ce395f2cb8548c871fd2aea9168b3d9767

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.01563 | 1.55468 | 4.98843 |
| startup | 199 | 0.01768 | 1.64599 | 8.65662 |
| steady | 401 | 0.01451 | 1.50731 | 0.21318 |
| crossing_1 | 49 | 0.00194 | 0.14036 | 0.01036 |
| crossing_2 | 49 | 0.00206 | 0.13435 | 0.02799 |
| seam | 30 | 0.01631 | 1.04905 | 20.72364 |
| other | 472 | 0.01711 | 1.73165 | 2.08230 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 150.90 deg/s. Maximum lateral acceleration: 21.37 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
