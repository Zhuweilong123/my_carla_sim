# p1_delay_50ms

Protocol: route_projection_v2; Git: a79f6df245e983a23bf7a955605e40452f4f7191

Completed laps: 3; simulated seconds: 103.15; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2063 | 0.14140 | 3.86823 | 3.45393 |
| startup | 199 | 0.12694 | 3.69834 | 7.30005 |
| steady | 1864 | 0.14286 | 3.88592 | 2.74116 |
| crossing_1 | 141 | 0.12153 | 3.69061 | 2.54232 |
| crossing_2 | 141 | 0.11696 | 3.55712 | 2.62111 |
| seam | 114 | 0.13904 | 3.60671 | 9.40004 |
| other | 1667 | 0.14497 | 3.92502 | 2.75556 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 63.548%. Maximum steering rate: 1145.92 deg/s. Maximum lateral acceleration: 115.07 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
