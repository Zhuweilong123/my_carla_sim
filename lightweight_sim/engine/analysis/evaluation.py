"""Reproducible, multi-lap figure-eight evaluation and report generation."""
from collections import deque
from dataclasses import asdict, replace
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import platform
import random
import subprocess
import time

import numpy as np

from .tracking import TrackingMonitor, FigureEightOracle, PROTOCOL
from ..algorithms.controller.combined import VehicleController
from ..algorithms.utils.route import RouteGeometry, wrap_angle
from ..simulator.data_types import ControlCommand
from ..simulator.engine import SimulationEngine
from ..simulator.scenarios import make_scenario

ROOT = Path(__file__).resolve().parents[2]
PARAMS = (1.015, 1.895, 1412.0, -148970.0, -82204.0, 1537.0)


def stats(rows, key):
    if not rows:
        return None
    values = np.asarray([row[key] for row in rows], dtype=float)
    worst = int(np.argmax(np.abs(values)))
    return dict(mean=float(values.mean()), mean_abs=float(np.abs(values).mean()),
                rms=float(np.sqrt(np.mean(values**2))),
                p95_abs=float(np.percentile(np.abs(values), 95)),
                max_abs=float(np.abs(values).max()),
                peak_time_s=rows[worst]["time_s"], peak_region=rows[worst]["region"])


def group_metrics(rows):
    return dict(samples=len(rows), lateral_error_m=stats(rows, "ed_m"),
                heading_error_deg=stats(rows, "ephi_deg"),
                speed_error_kmh=stats(rows, "speed_error_kmh"),
                steer_rate_deg_s=stats(rows, "steer_rate_deg_s"),
                lateral_accel_m_s2=stats(rows, "lateral_accel_m_s2"),
                control_ms=stats(rows, "control_ms"))


def provenance():
    def git(*args):
        return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()
    hashes = {}
    for directory in ("engine", "visualization", "scripts", "config"):
        for path in sorted((ROOT/directory).rglob("*")):
            if path.is_file() and path.suffix in (".py", ".yaml", ".sh"):
                hashes[path.relative_to(ROOT).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    return dict(git_commit=git("rev-parse", "HEAD"),
                working_tree_status=git("status", "--short"),
                source_sha256=hashes,
                source_tree_sha256=hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest(),
                python=platform.python_version(), numpy=np.__version__, platform=platform.platform())


def resample(path, spacing):
    if spacing is None:
        return path
    geometry = RouteGeometry(path)
    # Subdivide each original segment, preserving every vertex and the exact
    # road shape. Thus this experiment isolates sample density, not smoothing.
    result = []
    for i, (a, b) in enumerate(zip(path, path[1:])):
        count = max(1, math.ceil((geometry.s[i+1]-geometry.s[i])/spacing))
        for j in range(count):
            t = j/count
            result.append((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]),
                           a[2]+t*wrap_angle(b[2]-a[2]), a[3]+t*(b[3]-a[3])))
    return result+[path[-1]]


def run_evaluation(output_dir, label, *, laps=10, duration=None, speed=50.0,
                   dt=0.05, warmup=10.0, seed=2026, lateral_offset=0.0,
                   heading_offset_deg=0.0, delay_steps=0, noise_m=0.0,
                   mass_scale=1.0, stiffness_scale=1.0, spacing=None,
                   vehicle_model="dynamic"):
    if (laps < 0 or (duration is not None and duration <= 0) or speed <= 0
            or dt <= 0 or warmup < 0 or delay_steps < 0 or noise_m < 0
            or mass_scale <= 0 or stiffness_scale <= 0 or (spacing is not None and spacing <= 0)):
        raise ValueError("invalid evaluation settings")
    if not label or Path(label).name != label or label in (".", ".."):
        raise ValueError("label must be a filename component")
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    prefix = output_dir/f"figure_eight_{label}"
    if prefix.with_suffix(".json").exists():
        raise FileExistsError(f"archive already exists: {prefix}")
    config = make_scenario("figure_eight")
    config.target_speed, config.vehicle_model = speed, vehicle_model
    config.ego_start_x -= lateral_offset  # Left normal at the initial +y tangent.
    config.ego_start_phi += math.radians(heading_offset_deg)
    engine = SimulationEngine(config)
    engine.ego.params.m *= mass_scale
    engine.ego.params.Cf *= stiffness_scale
    engine.ego.params.Cr *= stiffness_scale
    path = resample(engine.world.ref_path_as_tuples, spacing)
    geometry = RouteGeometry(path)
    controller = VehicleController(PARAMS, target_speed_kmh=speed)
    controller.lat.ts = controller.lon.dt = dt
    controller.update_ref_path(path)
    monitor = TrackingMonitor(path)
    oracle = FigureEightOracle()
    rng = random.Random(seed)
    history = deque(maxlen=delay_steps+1)
    max_duration = duration or max(30.0, (laps or 1)*geometry.length/(speed/3.6)*1.5+20)
    rows = []
    previous_s = 0.0
    previous_steer = 0.0
    metadata = provenance()
    for step in range(math.ceil(max_duration/dt)):
        truth = engine.get_state()
        history.append(replace(truth))
        sensed = replace(history[0])
        sensed.x += rng.gauss(0, noise_m)
        sensed.y += rng.gauss(0, noise_m)
        started = time.perf_counter()
        steer, throttle, brake = controller.step(
            sensed.x, sensed.y, sensed.phi, sensed.vx, sensed.vy, sensed.r)
        elapsed_ms = (time.perf_counter()-started)*1000
        engine.step(ControlCommand(steer, throttle, brake), dt=dt)
        state = engine.get_state()
        measured = monitor.update(state)
        phase, oracle_heading, region = oracle.update(state.x, state.y)
        control_heading = controller.lat.last_ref_heading
        wrong = (oracle.wrong_branch(control_heading, oracle_heading, region)
                 or oracle.wrong_branch(measured["reference_heading_rad"], oracle_heading, region))
        progress_delta = measured["route_s_m"]-previous_s
        row = dict(step=step+1, time_s=state.timestamp, x_m=state.x, y_m=state.y,
                   yaw_rad=state.phi, speed_kmh=state.speed_kmh,
                   speed_error_kmh=state.speed_kmh-speed,
                   steer_deg=math.degrees(state.steer),
                   steer_saturated=abs(steer) >= controller.lat.max_steer-1e-6,
                   steer_rate_deg_s=math.degrees(state.steer-previous_steer)/dt,
                   lateral_accel_m_s2=(state.vy-truth.vy)/dt+state.vx*state.r,
                   control_ms=elapsed_ms, region=region,
                   oracle_laps=phase/(2*math.pi), wrong_branch=bool(wrong),
                   route_delta_m=progress_delta,
                   unexpected_jump=abs(progress_delta) > max(3.0, state.speed*dt*3),
                   controller_ed_pred_m=controller.lat.last_ed,
                   controller_ephi_pred_deg=math.degrees(controller.lat.last_ephi),
                   riccati_converged=bool(controller.lat.riccati_converged),
                   offroad=engine.offroad_occurred, collision=engine.collision_occurred,
                   **measured)
        rows.append(row)
        previous_s, previous_steer = measured["route_s_m"], state.steer
        if engine.is_done or (laps and phase >= laps*2*math.pi):
            break
    steady = [r for r in rows if r["time_s"] >= warmup]
    completed = max(0, math.floor(rows[-1]["oracle_laps"]+1e-6))
    required_completed = completed >= laps if laps else len(rows)*dt >= max_duration-1e-6
    summary = dict(protocol=PROTOCOL, label=label, provenance=metadata,
                   settings=dict(laps=laps, duration=duration, max_duration_s=max_duration,
                       speed_kmh=speed, dt_s=dt, warmup_s=warmup, seed=seed,
                       lateral_offset_m=lateral_offset, heading_offset_deg=heading_offset_deg,
                       delay_steps=delay_steps, noise_std_m=noise_m, mass_scale=mass_scale,
                       stiffness_scale=stiffness_scale, max_segment_spacing_m=spacing),
                   scenario=asdict(config), plant_parameters=asdict(engine.ego.params),
                   controller_parameters=dict(vehicle=PARAMS, Q=controller.lat.Q.tolist(),
                       R=controller.lat.R.tolist(), speed_loop={k: getattr(controller.lon, k)
                       for k in ("K_P", "K_I", "K_D", "dt", "max_accel", "max_decel", "max_jerk", "coupling_gain")}),
                   reference=dict(points=len(path), length_m=geometry.length,
                       sha256=hashlib.sha256(json.dumps(path).encode()).hexdigest()),
                   samples=len(rows), duration_s=len(rows)*dt, completed_laps=completed,
                   speed_kmh=dict(mean=float(np.mean([r["speed_kmh"] for r in rows])),
                       max=max(r["speed_kmh"] for r in rows), final=rows[-1]["speed_kmh"],
                       overshoot=max(0.0, max(r["speed_error_kmh"] for r in rows))),
                   termination="collision" if engine.collision_occurred else "offroad" if engine.offroad_occurred
                       else "completed" if required_completed else "duration_limit",
                   all=group_metrics(rows), startup=group_metrics([r for r in rows if r["time_s"] < warmup]),
                   steady=group_metrics(steady),
                   regions={name: group_metrics([r for r in rows if r["region"] == name])
                            for name in ("crossing_1", "crossing_2", "seam", "other")},
                   wrong_branch_samples=sum(r["wrong_branch"] for r in rows),
                   unexpected_jump_samples=sum(r["unexpected_jump"] for r in rows),
                   max_backtrack_m=max(0.0, -min(r["route_delta_m"] for r in rows)),
                   riccati_failures=sum(not r["riccati_converged"] for r in rows),
                   steer_saturation_fraction=sum(r["steer_saturated"] for r in rows)/len(rows),
                   offroad=engine.offroad_occurred, collision=engine.collision_occurred)
    summary["passed"] = bool(required_completed and not summary["offroad"] and not summary["collision"]
                             and not summary["wrong_branch_samples"] and not summary["unexpected_jump_samples"]
                             and not summary["riccati_failures"])
    summary["pass_scope"] = "completion, route integrity, collision/offroad and solver convergence only; not ride quality"
    with gzip.open(str(prefix)+".csv.gz", "wt", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    with Path(str(prefix)+"_reference.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["x_m", "y_m", "heading_rad", "curvature_1_m"])
        writer.writerows(path)
    prefix.with_suffix(".json").write_text(json.dumps(summary, indent=2)+"\n", encoding="utf-8")
    report = [f"# {label}", "", f"Protocol: {PROTOCOL}; Git: {metadata['git_commit']}", "",
              f"Completed laps: {completed}; simulated seconds: {len(rows)*dt:.2f}; passed: {summary['passed']}", "",
              "Error uses actual state and segment tangent; controller predicted errors are separate CSV columns.",
              "Regions use an independent analytic figure-eight phase tracker. Warmup is a fixed time window.", "",
              "| Window | Samples | Lateral RMS m | Heading RMS deg | Speed-error RMS km/h |",
              "| --- | ---: | ---: | ---: | ---: |"]
    for name, group in [("all", summary["all"]), ("startup", summary["startup"]),
                         ("steady", summary["steady"]), *summary["regions"].items()]:
        if group["samples"]:
            report.append(f"| {name} | {group['samples']} | {group['lateral_error_m']['rms']:.5f} | "
                          f"{group['heading_error_deg']['rms']:.5f} | {group['speed_error_kmh']['rms']:.5f} |")
    report += ["", f"Wrong-branch samples: {summary['wrong_branch_samples']}; "
               f"unexpected progress jumps: {summary['unexpected_jump_samples']}; "
               f"Riccati failures: {summary['riccati_failures']}.", "",
               f"Steering saturation fraction: {summary['steer_saturation_fraction']:.3%}. "
               f"Maximum steering rate: {summary['all']['steer_rate_deg_s']['max_abs']:.2f} deg/s. "
               f"Maximum lateral acceleration: {summary['all']['lateral_accel_m_s2']['max_abs']:.2f} m/s^2.", "",
               "P95, peaks with time/region, parameters, source hashes and environment are in JSON. "
               "Control timings are wall-clock diagnostics and are not deterministic. "
               "The pass flag only covers completion, route integrity, collision/offroad and Riccati convergence. "
               "It does not certify ride quality or physical feasibility."]
    prefix.with_suffix(".md").write_text("\n".join(report)+"\n", encoding="utf-8")
    return summary
