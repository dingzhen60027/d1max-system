#!/usr/bin/env python3
"""Bounded offline PCT regression. Starts no ROS, networking or robot control."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS / "src/d1max_pct_planner"))
TOMOGRAM = WS / "maps/processed/sc_pgo_20260919_pct_flat_floor_v6_20260923/tomogram.npz"
CONTRACT = TOMOGRAM.parent / "audit/native_v6_final_2_8_20260923/acceptance_plan.json"
VENDOR = WS / "src/pct_planner_vendor"
STRAIGHTS = [
    ("straight_23m", [-4.111784362792967, 7.0277099609375, 0.],
     [-15.075141906738281, 27.616825103759766, 0.]),
    ("straight_18m", [-4.111784362792967, 7.0277099609375, 0.],
     [-12.5, 23.1, .03835880011320114]),
    ("straight_11m", [-6.437280654907227, 11.653109550476074, .06450923532247543],
     [-11.833974838256836, 21.566173553466797, 0.]),
]


def write(path, result):
    with Path(path).open("x") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def child(args):
    from d1max_pct_planner.tomogram_map import TomogramMap
    from d1max_pct_planner.tomogram_route import TomogramRoute
    from d1max_pct_planner.path_quality import path_quality
    plan = json.loads(args.output.joinpath("contract.json").read_text())
    case = next(item for item in plan["cases"] if item["name"] == args.child)
    result = {"case": args.child, "passed": False, "robot_or_ros_used": False}
    started = time.monotonic()
    try:
        tomo = TomogramMap(TOMOGRAM, minimum_headroom_m=.55, max_ground_step_m=.17)
        route = TomogramRoute(tomo, VENDOR, astar_cost_weight=args.astar_cost_weight,
                              optimizer_cost_margin=args.optimizer_cost_margin,
                              max_heading_rate=args.max_heading_rate,
                              path_refinement=args.path_refinement)
        value = route.plan(case["start"], case["goal"], case["start_layer"], case["goal_layer"])
        result.update(passed=True, result=value, quality=path_quality(value["path"]))
        # Continuity in an entire hallway is different from a building-scale detour.
        if case.get("maximum_length_m") is not None and value["length_m"] > case["maximum_length_m"]:
            result.update(passed=False, error_code="local_gap_detour_too_long")
        result["independent_path_validation"] = tomo.validate_path(value["path"], value["layer_ids"])
        if case["category"] == "straight":
            result["direct_chord_validation"] = tomo.validate_path(
                [case["start"], case["goal"]], [case["start_layer"], case["goal_layer"]])
    except Exception as exc:
        result.update(error_code=getattr(exc, "code", type(exc).__name__),
                      error=str(exc), details=getattr(exc, "details", {}))
    result["elapsed_s"] = time.monotonic() - started
    write(args.output / (args.child + ".json"), result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--astar-cost-weight", type=float, default=2.)
    parser.add_argument("--optimizer-cost-margin", type=float, default=8.)
    parser.add_argument("--max-heading-rate", type=float, default=10.)
    parser.add_argument("--path-refinement", choices=("none", "visibility_c2"), default="none")
    parser.add_argument("--case", action="append")
    parser.add_argument("--child", help=argparse.SUPPRESS)
    args = parser.parse_args()
    args.output = args.output.resolve()
    if args.child:
        child(args)
        return
    from d1max_pct_planner.native_runtime import prepare_native_environment
    from d1max_pct_planner.planner_core import validate_native_parameters
    parameters = validate_native_parameters(args.astar_cost_weight, args.optimizer_cost_margin)
    cases = [{"name": c["name"], "category": c["category"],
              "start": c["start"]["selected_xyz"], "goal": c["goal"]["selected_xyz"],
              "start_layer": 0, "goal_layer": 0,
              "maximum_length_m": c["maximum_local_route_length_m"]}
             for c in json.loads(CONTRACT.read_text())["cases"]]
    cases += [{"name": n, "start": a, "goal": b, "start_layer": 0,
               "goal_layer": 0, "category": "straight"} for n, a, b in STRAIGHTS]
    last_user = json.loads((WS / 'log/pct_preview/20260923_101221_25a7db3835ad/path_000038.json').read_text())
    cases.append({'name': 'last_user_45m', 'start': last_user['start_xyz'],
                  'goal': last_user['goal_xyz'], 'start_layer': last_user['start_layer'],
                  'goal_layer': last_user['goal_layer'], 'category': 'preserved_user_goal'})
    if args.case:
        unknown = set(args.case) - {c["name"] for c in cases}
        if unknown:
            parser.error("Unknown cases: " + str(unknown))
        cases = [c for c in cases if c["name"] in args.case]
    environment = prepare_native_environment(VENDOR, os.environ)
    environment.update(OPENBLAS_NUM_THREADS="2", OMP_NUM_THREADS="4")
    args.output.mkdir(parents=True, exist_ok=False)
    write(args.output / "contract.json", {"cases": cases, "parameters": {
        **parameters, "max_heading_rate": args.max_heading_rate,
        "path_refinement": args.path_refinement},
        "tomogram": str(TOMOGRAM), "prior_contract": str(CONTRACT)})
    rows = []
    for case in cases:
        name = case["name"]
        command = [sys.executable, str(Path(__file__).resolve()), "--child", name,
                   "--output", str(args.output), "--astar-cost-weight", str(args.astar_cost_weight),
                   "--optimizer-cost-margin", str(args.optimizer_cost_margin),
                   "--max-heading-rate", str(args.max_heading_rate),
                   "--path-refinement", args.path_refinement]
        with (args.output / (name + ".log")).open("x") as log:
            try:
                completed = subprocess.run(command, env=environment, stdout=log, stderr=log, timeout=35)
                code = completed.returncode
            except subprocess.TimeoutExpired:
                code = "timeout"
        path = args.output / (name + ".json")
        record = json.loads(path.read_text()) if path.exists() else {"passed": False, "error_code": code}
        row = {"name": name, "passed": record["passed"], "error_code": record.get("error_code"),
               "quality": record.get("quality"), "elapsed_s": record.get("elapsed_s")}
        rows.append(row)
        metrics = row.get("quality") or {}
        print(json.dumps({"name": name, "passed": row["passed"],
                          "error_code": row["error_code"],
                          "xy_length_m": metrics.get("xy_length_m"),
                          "curvature_p95_per_m": metrics.get("curvature_p95_per_m")}), flush=True)
    write(args.output / "summary.json", {"cases": rows,
          "passed_cases": sum(row["passed"] for row in rows), "total_cases": len(rows)})


if __name__ == "__main__":
    main()
