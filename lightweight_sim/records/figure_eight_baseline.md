# Figure-eight tracking baseline

Date: 2026-09-13
Scenario: `figure_eight_three_lane`
Controller: `LQR_controller`
Target speed: 30 km/h
Time step: 0.05 s
Method: direct closed-loop tracking of the global reference path. Nearby reference candidates are disambiguated by heading agreement.

## Results

| Metric | Mean absolute | RMS | Max absolute |
|---|---:|---:|---:|
| Lateral error `ed` | 0.293 m | 0.567 m | 4.293 m |
| Heading error `ephi` | 2.92 deg | 10.71 deg | 76.19 deg |

- Measured duration: 15.8 s / requested 30.0 s
- Samples: 316
- Offroad: yes
- Collision: no

Per-step data: [figure_eight_baseline.csv](figure_eight_baseline.csv). Summary: [figure_eight_baseline.json](figure_eight_baseline.json).
