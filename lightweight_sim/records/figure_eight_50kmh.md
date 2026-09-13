# Figure-eight 50 km/h dynamic evaluation

This run uses the dynamic vehicle model and the current projected-reference
Riccati LQR controller. It is a separate high-speed evaluation and should not
be mixed with the earlier 30 km/h optimization records.

| Setting | Result |
| --- | ---: |
| Target speed | 50 km/h |
| Duration | 30 s |
| Time step | 0.05 s |
| Lateral error RMS | 0.1218 m |
| Maximum lateral error | 0.2148 m |
| Heading error RMS | 2.2862 deg |
| Maximum heading error | 14.5525 deg |
| DARE converged | yes |
| DARE iterations | 28 |
| Collision / offroad | no / no |
| Final speed | 53.09 km/h |

The vehicle remains stable at the requested high-speed condition. The final
speed is about 3.1 km/h above target, so longitudinal speed regulation is the
next high-speed refinement target.

Detailed samples and Riccati data are stored in
`figure_eight_evaluation_50kmh.csv` and `figure_eight_evaluation_50kmh.json`.
