# p1_nominal

Protocol: route_projection_v2; Git: a79f6df245e983a23bf7a955605e40452f4f7191

Completed laps: 10; simulated seconds: 353.55; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 7071 | 0.12941 | 2.03022 | 1.15891 |
| startup | 199 | 0.12627 | 2.98424 | 5.25229 |
| steady | 6872 | 0.12950 | 1.99581 | 0.76361 |
| crossing_1 | 488 | 0.15869 | 1.46347 | 0.57936 |
| crossing_2 | 486 | 0.15932 | 1.46163 | 0.57906 |
| seam | 364 | 0.13900 | 2.25171 | 3.72203 |
| other | 5733 | 0.12302 | 2.09725 | 0.84849 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 68.562%. Maximum steering rate: 1145.92 deg/s. Maximum lateral acceleration: 300.22 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
