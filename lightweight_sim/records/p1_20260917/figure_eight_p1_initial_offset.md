# p1_initial_offset

Protocol: route_projection_v2; Git: a79f6df245e983a23bf7a955605e40452f4f7191

Completed laps: 3; simulated seconds: 106.40; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2128 | 0.12994 | 2.15668 | 1.53896 |
| startup | 199 | 0.14359 | 3.25022 | 4.43821 |
| steady | 1929 | 0.12845 | 2.01030 | 0.76201 |
| crossing_1 | 147 | 0.15881 | 1.45667 | 0.57368 |
| crossing_2 | 147 | 0.15557 | 1.46763 | 0.58105 |
| seam | 115 | 0.15984 | 3.79818 | 5.56315 |
| other | 1719 | 0.12233 | 2.10409 | 0.89691 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 68.045%. Maximum steering rate: 1145.92 deg/s. Maximum lateral acceleration: 320.49 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
