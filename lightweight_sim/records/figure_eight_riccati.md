# Dynamic Riccati LQR validation

The lightweight simulator now uses the CARLA controller's full dynamic LQR
pipeline for the figure-eight scenario:

1. Build the speed-dependent bicycle-model matrices `A` and `B`.
2. Discretize them with the bilinear transform.
3. Iterate the discrete algebraic Riccati equation (DARE).
4. Compute `K = (R + B^T P B)^-1 B^T P A`.
5. Apply `delta = delta_ff - K x` with steering saturation.

The error state is `[e_d, e_d_dot, e_phi, e_phi_dot]`. The public controller
keeps continuous reference-line projection and heading-consistent matching from
the previous figure-eight improvements.

Vehicle parameters use the lightweight simulator order:
`(a, b, m, Cf, Cr, Iz)`.

## Validation

The `figure_eight` benchmark uses the dynamic vehicle model for 30 seconds at
0.05 second time steps and a target speed of 30 km/h.

| Metric | Result |
| --- | ---: |
| Lateral error RMS | 0.0891 m |
| Heading error RMS | 4.0042 deg |
| Maximum lateral error | 0.3360 m |
| Maximum heading error | 15.5346 deg |
| Offroad | no |
| Collision | no |
| DARE converged | yes |
| DARE iterations | 37 |

The numeric details and final feedback gain are stored in
`figure_eight_riccati.json`; per-step data is in `figure_eight_riccati.csv`.
