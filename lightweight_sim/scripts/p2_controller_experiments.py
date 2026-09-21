"""Reproducible ablation of LQR preview and sampled-plant model mismatch."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lightweight_sim.engine.analysis.evaluation import run_evaluation, provenance, archive_sources, PARAMS
from lightweight_sim.engine.algorithms.controller.lat_lqr import LateralLQRController
from lightweight_sim.engine.simulator.data_types import VehicleState
from lightweight_sim.engine.simulator.vehicle import EgoVehicle


CASES = {
    "baseline": dict(feedback_horizon_s=0.05, lqr_discretization="bilinear"),
    "current_state": dict(feedback_horizon_s=0.0, lqr_discretization="bilinear"),
    "matched_preview": dict(feedback_horizon_s=0.05, lqr_discretization="plant"),
    "matched_current": dict(feedback_horizon_s=0.0, lqr_discretization="plant"),
}


def plant_step(z, steer, speed, dt):
    vehicle = EgoVehicle(VehicleState(x=20., y=z[0], vy=z[1], phi=z[2], r=z[3], vx=speed))
    n = max(1, math.ceil(speed*dt/0.5))
    for _ in range(n):
        state = vehicle.step(steer, 0.0, dt/n, "dynamic")
    return np.array([state.y, state.vy, state.phi, state.r])


def sampled_jacobian(speed, options, dt=0.05):
    controller = LateralLQRController(PARAMS, ts=dt, R=options.get("lqr_r", 1.0))
    controller.feedback_horizon_s = options["feedback_horizon_s"]
    controller.discretization = options["lqr_discretization"]
    controller.smooth_reference_heading = options.get("smooth_reference_heading", False)
    path = [(0., 0., 0., 0.), (10000., 0., 0., 0.)]
    eps = 1e-6
    def step(z):
        controller.reset_tracking()
        steer = controller.control(20., z[0], z[2], speed, z[1], z[3], path)
        return plant_step(z, steer, speed, dt)
    closed = np.column_stack([(step(np.eye(4)[i]*eps)-step(-np.eye(4)[i]*eps))/(2*eps)
                              for i in range(4)])
    eigenvalues = np.linalg.eigvals(closed)
    return dict(speed_kmh=speed*3.6, settings=options,
                closed_loop_matrix=closed.tolist(),
                eigenvalues=[[float(v.real), float(v.imag)] for v in eigenvalues],
                spectral_radius=float(max(abs(eigenvalues))),
                K=controller.K.tolist(), riccati_converged=controller.riccati_converged)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--label", default="ablation")
    parser.add_argument("--duration", type=float, default=30.)
    parser.add_argument("--mode", choices=("ablation", "weights", "smooth"), default="ablation")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    diagnostic = args.output_dir/(args.label+"_linearization.json")
    if diagnostic.exists():
        raise FileExistsError(diagnostic)
    metadata = provenance()
    archive_sources(metadata, args.output_dir)
    cases = CASES if args.mode == "ablation" else {
        f"matched_R{weight}": dict(feedback_horizon_s=0.0, lqr_discretization="plant", lqr_r=float(weight))
        for weight in (4, 10, 25, 100)}
    if args.mode == "smooth":
        cases = {f"smooth_R{weight}": dict(feedback_horizon_s=0., lqr_discretization="plant",
                         lqr_r=float(weight), smooth_reference_heading=True)
                 for weight in (1, 25, 100)}
    for options in cases.values():
        options.setdefault("lqr_r", 1.0)
        options.setdefault("smooth_reference_heading", False)
    data = dict(provenance=metadata, description="Central-difference actual held-input closed loop at zero straight-line error; no steering saturation",
                runs=[sampled_jacobian(speed/3.6, options)
                      for speed in (21.6, 30, 40, 50, 60) for options in cases.values()])
    diagnostic.write_text(json.dumps(data, indent=2)+"\n", encoding="utf-8")
    for name, options in cases.items():
        result = run_evaluation(args.output_dir, args.label+"_"+name,
                                laps=0, duration=args.duration, **options)
        print(json.dumps(dict(label=result["label"], passed=result["passed"],
                    lateral_rms=result["steady"]["lateral_error_m"],
                    heading_rms=result["steady"]["heading_error_deg"],
                    saturation=result["steer_saturation_fraction"],
                    steer_rate=result["all"]["steer_rate_deg_s"]["max_abs"],
                    speed=result["speed_kmh"])), flush=True)


if __name__ == "__main__":
    main()
