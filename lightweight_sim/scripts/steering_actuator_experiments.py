"""A/B actuator experiments. All parameters are uncalibrated assumptions."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lightweight_sim.engine.analysis.evaluation import run_evaluation
from lightweight_sim.engine.simulator.steering import steering_profile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--suite", action="store_true")
    parser.add_argument("--screen-mismatch", action="store_true")
    parser.add_argument("--lqr-r", type=float, default=300.0)
    args = parser.parse_args()
    nominal = steering_profile("assumed")
    cases = [
        ("ideal", dict()),
        ("unadapted", dict(steering_params=nominal, actuator_compensation=False)),
        ("adapted", dict(steering_params=nominal)),
    ]
    if args.suite or args.screen_mismatch:
        cases += [
            ("noise", dict(steering_params=nominal, noise_m=0.05)),
            ("initial_offset", dict(steering_params=nominal, lateral_offset=0.5, heading_offset_deg=5)),
            ("slower_actuator", dict(steering_params=replace(nominal, time_constant_s=0.20, delay_s=0.10),
                                    controller_steering_params=nominal)),
            ("faster_actuator", dict(steering_params=replace(nominal, time_constant_s=0.10, delay_s=0),
                                    controller_steering_params=nominal)),
            ("plant_mismatch", dict(steering_params=nominal, mass_scale=1.1, stiffness_scale=0.9)),
        ]
    for name, options in cases:
        if name not in ("ideal", "unadapted"):
            options["lqr_r"] = args.lqr_r
        settings = dict(laps=(10 if name in ("ideal", "unadapted", "adapted") else 3), duration=None) if args.suite else dict(laps=0, duration=30)
        result = run_evaluation(args.output_dir, args.label+"_"+name, **settings, **options)
        print(json.dumps({key: result[key] for key in ("label", "passed", "termination", "duration_s", "steady", "actuator")}), flush=True)


if __name__ == "__main__":
    main()
