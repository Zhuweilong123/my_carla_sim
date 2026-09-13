# Figure-eight tracking optimization record

Measurement convention: direct closed-loop simulation on `figure_eight_three_lane`,
30 seconds, 0.05 second time step, target speed 30 km/h. The benchmark uses the
vehicle controller directly so that the lateral and heading tracking behavior is
isolated from planner effects.

| Round | Main change | Duration | Offroad | Lateral RMS (m) | Heading RMS (deg) |
| --- | --- | ---: | --- | ---: | ---: |
| Baseline | Original sampled-point controller | 15.8 s | yes | 0.5666 | 10.7079 |
| Opt 1 | Tuned gains and 0.10 s preview | 30.0 s | no | 0.7082 | 6.2955 |
| Opt 2 | Continuous segment projection and heading-consistent matching | 30.0 s | no | 0.1462 | 1.8074 |
| Opt 3 | Opt 2 plus 0.05 s preview, `k_lat=20`, `k_heading=0.5` | 30.0 s | no | 0.0602 | 1.5229 |

The baseline ended early because the vehicle went offroad, so its RMS values are
not directly comparable with the complete 30-second runs. Stability and the two
error metrics are both considered when selecting the final gains.

Detailed samples and metrics are stored in the matching `figure_eight_*.csv` and
`figure_eight_*.json` files.
