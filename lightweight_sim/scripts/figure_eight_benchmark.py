#!/usr/bin/env python3
"""Repeatable closed-loop tracking benchmark for the figure-eight scenario."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from lightweight_sim.engine.algorithms.controller.combined import VehicleController
from lightweight_sim.engine.simulator.data_types import ControlCommand
from lightweight_sim.engine.simulator.engine import SimulationEngine
from lightweight_sim.engine.simulator.scenarios import make_scenario


def _wrapped_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def tracking_error(state, path):
    """Measure error using a nearby reference point with heading agreement."""
    distances = [
        (point[0] - state.x) ** 2 + (point[1] - state.y) ** 2
        for point in path
    ]
    nearest = min(distances)
    candidates = [
        index
        for index, distance in enumerate(distances)
        if distance <= nearest + 9.0
    ]
    index = min(
        candidates,
        key=lambda item: abs(_wrapped_angle(state.phi - path[item][2]))
        + 0.05 * math.sqrt(distances[item]),
    )
    reference = path[index]
    dx = state.x - reference[0]
    dy = state.y - reference[1]
    ed = -math.sin(reference[2]) * dx + math.cos(reference[2]) * dy
    ephi = _wrapped_angle(state.phi - reference[2])
    return ed, ephi


def run_benchmark(
    duration: float,
    output_dir: Path,
    label: str,
    vehicle_model: str = "dynamic",
) -> dict:
    config = make_scenario("figure_eight")
    config.vehicle_model = vehicle_model
    engine = SimulationEngine(config)
    path = engine.world.ref_path_as_tuples
    controller = VehicleController(
        (1.015, 1.895, 1412.0, -148970.0, -82204.0, 1537.0),
        controller_type=config.controller,
        target_speed_kmh=config.target_speed,
    )
    controller.update_ref_path(path)

    dt = 0.05
    rows = []
    progress_deltas = []
    previous_route_progress = None
    for step in range(max(1, int(duration / dt))):
        state = engine.get_state()
        steer, throttle, brake = controller.step(
            state.x, state.y, state.phi, state.vx, state.vy, state.r
        )
        engine.step(
            ControlCommand(steer=steer, throttle=throttle, brake=brake),
            dt=dt,
        )
        state = engine.get_state()
        ed, ephi = tracking_error(state, path)
        route_progress = float(
            getattr(controller.lat, "route_progress", controller.lat.min_index)
        )
        if previous_route_progress is not None:
            progress_deltas.append(route_progress - previous_route_progress)
        previous_route_progress = route_progress
        rows.append(
            {
                "step": step + 1,
                "time_s": (step + 1) * dt,
                "x_m": state.x,
                "y_m": state.y,
                "speed_kmh": state.speed_kmh,
                "steer_deg": math.degrees(state.steer),
                "ed_m": ed,
                "ephi_deg": math.degrees(ephi),
                "route_progress": route_progress,
                "offroad": engine.offroad_occurred,
                "collision": engine.collision_occurred,
            }
        )
        if engine.is_done:
            break

    def values(key):
        return [float(row[key]) for row in rows]

    def metrics(key):
        data = values(key)
        absolute = [abs(item) for item in data]
        return {
            "mean_abs": sum(absolute) / len(absolute),
            "rms": math.sqrt(sum(item * item for item in data) / len(data)),
            "max_abs": max(absolute),
        }

    speed_values = values("speed_kmh")

    summary = {
        "label": label,
        "scenario": config.name,
        "duration_requested_s": duration,
        "duration_measured_s": len(rows) * dt,
        "samples": len(rows),
        "dt_s": dt,
        "target_speed_kmh": config.target_speed,
        "vehicle_model": config.vehicle_model,
        "speed_kmh": {
            "mean": sum(speed_values) / len(speed_values),
            "max": max(speed_values),
            "final": speed_values[-1],
        },
        "lateral_error_m": metrics("ed_m"),
        "heading_error_deg": metrics("ephi_deg"),
        "route_progress": {
            "start": rows[0]["route_progress"],
            "end": rows[-1]["route_progress"],
            "max_forward_step": max(progress_deltas, default=0.0),
            "max_backtrack": min(0.0, min(progress_deltas, default=0.0)),
        },
        "offroad": engine.offroad_occurred,
        "collision": engine.collision_occurred,
        "riccati": {
            "converged": controller.lat.riccati_converged,
            "iterations": controller.lat.riccati_iterations,
            "K": controller.lat.K.tolist(),
        },
        "final_state": {
            "x_m": engine.get_state().x,
            "y_m": engine.get_state().y,
            "speed_kmh": engine.get_state().speed_kmh,
        },
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / f"figure_eight_{label}.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    json_path = output_dir / f"figure_eight_{label}.json"
    json_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"samples: {csv_path}")
    print(f"summary: {json_path}")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float)
    parser.add_argument("--label", default="baseline")
    parser.add_argument("--protocol", choices=("legacy", "v2"), default="v2")
    parser.add_argument("--laps", type=int)
    parser.add_argument("--speed", type=float, default=50.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--warmup", type=float, default=10.0)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--lateral-offset", type=float, default=0.0)
    parser.add_argument("--heading-offset-deg", type=float, default=0.0)
    parser.add_argument("--delay-steps", type=int, default=0)
    parser.add_argument("--noise-m", type=float, default=0.0)
    parser.add_argument("--mass-scale", type=float, default=1.0)
    parser.add_argument("--stiffness-scale", type=float, default=1.0)
    parser.add_argument("--spacing", type=float)
    parser.add_argument("--feedback-horizon-s", type=float)
    parser.add_argument("--lqr-discretization", choices=("plant", "bilinear"))
    parser.add_argument("--lqr-r", type=float)
    parser.add_argument("--smooth-reference-heading", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--steering-profile", choices=("ideal", "assumed"), default="ideal")
    parser.add_argument("--actuator-compensation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--suite", action="store_true", help="Nominal 10 laps and five 3-lap perturbation cases")
    parser.add_argument(
        "--vehicle-model",
        choices=("kinematic", "dynamic"),
        default="dynamic",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "records",
    )
    args = parser.parse_args()
    if args.protocol == "legacy":
        if args.steering_profile != "ideal":
            parser.error("actuator experiments require --protocol v2")
        run_benchmark(args.duration or 30.0, args.output_dir, args.label,
                      vehicle_model=args.vehicle_model)
        return
    from lightweight_sim.engine.analysis.evaluation import run_evaluation
    from lightweight_sim.engine.simulator.steering import steering_profile
    options = dict(laps=args.laps if args.laps is not None else (0 if args.duration else 10),
                   duration=args.duration, speed=args.speed, dt=args.dt, warmup=args.warmup,
                   seed=args.seed, lateral_offset=args.lateral_offset,
                   heading_offset_deg=args.heading_offset_deg, delay_steps=args.delay_steps,
                   noise_m=args.noise_m, mass_scale=args.mass_scale,
                   stiffness_scale=args.stiffness_scale, spacing=args.spacing,
                   vehicle_model=args.vehicle_model,
                   feedback_horizon_s=args.feedback_horizon_s,
                   lqr_discretization=args.lqr_discretization, lqr_r=args.lqr_r,
                   smooth_reference_heading=args.smooth_reference_heading,
                   steering_params=steering_profile(args.steering_profile),
                   actuator_compensation=args.actuator_compensation)
    cases = [(args.label, options)]
    if args.suite:
        cases = [(args.label+"_nominal", {**options, "laps": 10, "duration": None})]
        for name, change in [
            ("initial_offset", dict(lateral_offset=0.5, heading_offset_deg=5.0)),
            ("delay_50ms", dict(delay_steps=1)),
            ("position_noise", dict(noise_m=0.05)),
            ("plant_mismatch", dict(mass_scale=1.1, stiffness_scale=0.9)),
            ("dense_reference", dict(spacing=1.0)),
        ]:
            cases.append((args.label+"_"+name, {**options, "laps": 3, "duration": None, **change}))
    summaries = []
    for label, settings in cases:
        result = run_evaluation(args.output_dir, label, **settings)
        summaries.append(result)
        print(json.dumps({key: result[key] for key in
              ("label", "protocol", "completed_laps", "duration_s", "passed", "speed_kmh",
               "steady", "wrong_branch_samples", "unexpected_jump_samples")}, indent=2), flush=True)
    if args.suite:
        suite_path = args.output_dir/f"{args.label}_suite.json"
        suite_path.write_text(json.dumps([
            {key: r[key] for key in ("label", "passed", "completed_laps", "steady", "speed_kmh")}
            for r in summaries], indent=2)+"\n", encoding="utf-8")
    if not all(r["passed"] for r in summaries):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
