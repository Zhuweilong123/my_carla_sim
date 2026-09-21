# p2_candidate_dense_reference

Protocol: route_projection_v2; Git: 101a30ce395f2cb8548c871fd2aea9168b3d9767

Completed laps: 3; simulated seconds: 107.20; passed: True

Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.
Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.

| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |
| --- | ---: | ---: | ---: | ---: |
| all | 2144 | 0.01716 | 1.60175 | 2.64621 |
| startup | 199 | 0.02051 | 1.61188 | 8.65761 |
| steady | 1945 | 0.01678 | 1.60071 | 0.22366 |
| crossing_1 | 147 | 0.00192 | 0.13355 | 0.02338 |
| crossing_2 | 148 | 0.00196 | 0.13577 | 0.02783 |
| seam | 120 | 0.01373 | 1.02115 | 10.36316 |
| other | 1729 | 0.01874 | 1.76237 | 1.10877 |

Wrong-branch samples: 0; unexpected progress jumps: 0; Riccati failures: 0.

Steering saturation fraction: 0.000%. Maximum steering rate: 115.62 deg/s. Maximum lateral acceleration: 17.50 m/s^2.

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. It does not certify ride quality or physical feasibility.
