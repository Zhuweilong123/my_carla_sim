# 50 km/h speed-loop optimization record

All runs use the same dynamic figure-eight scene, 30 seconds, 0.05 second time
step, and a 50 km/h target. The lateral Riccati LQR is unchanged; only the
longitudinal speed loop is changed.

| Version | Main change | Peak speed | Final speed | Lateral RMS | Heading RMS | Offroad |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| Original | P control | 54.37 km/h | 53.09 km/h | 0.1218 m | 2.2862 deg | no |
| Opt 1 | PD + strong jerk limit | 55.67 km/h | 55.15 km/h | 0.1206 m | 2.4019 deg | no |
| Opt 2 | PD + relaxed jerk limit | 54.92 km/h | 54.53 km/h | 0.1219 m | 2.3075 deg | no |
| Opt 3 | Higher proportional damping | 54.31 km/h | 53.67 km/h | 0.1192 m | 2.2984 deg | no |
| Final | Opt 3 + half `r*vy` compensation | 51.25 km/h | 50.57 km/h | 0.1124 m | 2.3460 deg | no |

The final controller uses `K_P=1.15`, `K_D=0.55`, filtered speed derivative,
40 m/s^3 maximum jerk, and a 0.5 lateral-dynamic coupling compensation gain.
The full coupling compensation was rejected because it caused underspeed
(46.94 km/h final speed).

## Route-lock evaluation

The next version keeps the final speed loop and dynamic Riccati LQR unchanged,
and changes only reference-line anchoring at the figure-eight crossing. The
controller now uses an unwrapped route progress, a local search window
(`2` segments behind and `48` ahead), and heading-continuity hysteresis. The
30-second re-run is archived as
`figure_eight_evaluation_50kmh_route_lock_v2.json/csv`:

| Version | Peak speed | Final speed | Lateral RMS | Heading RMS | Max backtrack | Offroad/collision |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Route lock v2 | 51.43 km/h | 50.57 km/h | 0.1154 m | 2.3505 deg | 0.0000 segment | no / no |

The recorded route progress grew from `0.068` to `97.308` segments, with no
negative progress delta and a maximum single-step advance of `0.375` segment.
