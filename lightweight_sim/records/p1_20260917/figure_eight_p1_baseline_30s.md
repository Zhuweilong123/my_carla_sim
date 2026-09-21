# p1_baseline_30s

Protocol: route_projection_v2; Git: a79f6df245e983a23bf7a955605e40452f4f7191

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

P95, peaks with time/region, parameters, source hashes and environment are in JSON. Control timings are wall-clock diagnostics and are not deterministic. Performance values are measurements, not additional pass thresholds.
