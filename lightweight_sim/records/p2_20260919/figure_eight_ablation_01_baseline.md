# ablation_01_baseline

Protocol: route_projection_v2; Git: 101a30ce395f2cb8548c871fd2aea9168b3d9767

Completed laps: 0; simulated seconds: 30.00; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 600 | 0.12828 | 2.35019 | 3.08296 |
| startup | 199 | 0.12627 | 2.98424 | 5.25229 |
| steady | 401 | 0.12926 | 1.96083 | 0.72889 |
| crossing_1 | 49 | 0.15614 | 1.47238 | 0.56162 |
| crossing_2 | 49 | 0.16047 | 1.45143 | 0.57939 |
| seam | 25 | 0.10134 | 6.35845 | 13.94777 |
| other | 477 | 0.12252 | 2.09513 | 1.30099 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 69.000%. Maximum steering rate: 1145.92 deg/s. Maximum lateral acceleration: 300.22 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
