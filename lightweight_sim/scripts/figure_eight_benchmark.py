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
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--label", default="baseline")
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
    run_benchmark(
        args.duration,
        args.output_dir,
        args.label,
        vehicle_model=args.vehicle_model,
    )


if __name__ == "__main__":
    main()
