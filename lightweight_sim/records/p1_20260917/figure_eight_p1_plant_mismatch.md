# p1_plant_mismatch

Protocol: route_projection_v2; Git: a79f6df245e983a23bf7a955605e40452f4f7191

Completed laps: 3; simulated seconds: 106.85; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2137 | 0.08084 | 1.78443 | 1.75181 |
| startup | 199 | 0.07863 | 2.19890 | 5.56381 |
| steady | 1938 | 0.08107 | 1.73628 | 0.45310 |
| crossing_1 | 147 | 0.09854 | 1.26420 | 0.33992 |
| crossing_2 | 147 | 0.09635 | 1.27661 | 0.34792 |
| seam | 116 | 0.08194 | 2.35161 | 6.87874 |
| other | 1727 | 0.07759 | 1.81492 | 0.77398 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 65.559%. Maximum steering rate: 1145.92 deg/s. Maximum lateral acceleration: 221.94 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
