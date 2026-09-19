# p1_dense_reference

Protocol: route_projection_v2; Git: a79f6df245e983a23bf7a955605e40452f4f7191

Completed laps: 3; simulated seconds: 106.50; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2130 | 0.12903 | 2.13326 | 1.76371 |
| startup | 199 | 0.12627 | 2.98424 | 5.25229 |
| steady | 1931 | 0.12931 | 2.02534 | 0.76699 |
| crossing_1 | 147 | 0.15657 | 1.46633 | 0.57367 |
| crossing_2 | 147 | 0.15684 | 1.46431 | 0.58238 |
| seam | 116 | 0.13484 | 3.24217 | 6.50654 |
| other | 1720 | 0.12326 | 2.13533 | 0.96947 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 68.920%. Maximum steering rate: 1145.92 deg/s. Maximum lateral acceleration: 300.22 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
