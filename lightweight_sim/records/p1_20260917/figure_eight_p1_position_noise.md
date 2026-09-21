# p1_position_noise

Protocol: route_projection_v2; Git: a79f6df245e983a23bf7a955605e40452f4f7191

Completed laps: 3; simulated seconds: 106.50; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2130 | 0.11154 | 2.22111 | 1.66979 |
| startup | 199 | 0.10971 | 3.15392 | 4.92074 |
| steady | 1931 | 0.11173 | 2.10158 | 0.76169 |
| crossing_1 | 146 | 0.09097 | 1.52728 | 0.57010 |
| crossing_2 | 145 | 0.10642 | 1.58316 | 0.57649 |
| seam | 114 | 0.11248 | 3.65448 | 6.09935 |
| other | 1725 | 0.11346 | 2.19109 | 0.96374 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 69.531%. Maximum steering rate: 1145.92 deg/s. Maximum lateral acceleration: 302.40 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
