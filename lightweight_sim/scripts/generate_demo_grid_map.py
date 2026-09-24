"""Generate the built-in, lane-level 3-by-3 demo city grid map."""

from __future__ import annotations

import json
import math
from pathlib import Path


MAP_PATH = Path(__file__).resolve().parents[1] / "config" / "maps" / "demo_grid.json"
GRID = (0.0, 100.0, 200.0)
LANE_WIDTH = 3.5
LANE_CENTERS = (-(1.5 * LANE_WIDTH), -(0.5 * LANE_WIDTH))
PORTAL_DISTANCE = 12.0
TURN_CONTROL_DISTANCE = 9.0
DIRECTIONS = {
    "E": (1.0, 0.0),
    "N": (0.0, 1.0),
    "W": (-1.0, 0.0),
    "S": (0.0, -1.0),
}


def _intersection(row, col):
    return (GRID[col], GRID[row])


def _intersection_id(row, col):
    return f"i{row}_{col}"


def _departure_id(row, col, direction, lane):
    return f"d{row}_{col}_{direction}_l{lane}"


def _arrival_id(row, col, direction, lane):
    return f"a{row}_{col}_{direction}_l{lane}"


def _lane_offset(direction, lane):
    # Positive offset is to the right of the direction of travel.
    return -LANE_CENTERS[lane]


def _portal(row, col, direction, lane, departure):
    x, y = _intersection(row, col)
    dx, dy = DIRECTIONS[direction]
    right = (dy, -dx)
    longitudinal = PORTAL_DISTANCE if departure else -PORTAL_DISTANCE
    offset = _lane_offset(direction, lane)
    return (
        x + dx * longitudinal + right[0] * offset,
        y + dy * longitudinal + right[1] * offset,
    )


def _directions_at(row, col):
    result = []
    if col < len(GRID) - 1:
        result.append("E")
    if row < len(GRID) - 1:
        result.append("N")
    if col > 0:
        result.append("W")
    if row > 0:
        result.append("S")
    return result


def _incoming_directions(row, col):
    return [
        direction
        for direction, (dx, dy) in DIRECTIONS.items()
        if 0 <= col - int(dx) < len(GRID) and 0 <= row - int(dy) < len(GRID)
    ]


def _turn(incoming, outgoing):
    if incoming == outgoing:
        return "straight"
    first, second = DIRECTIONS[incoming], DIRECTIONS[outgoing]
    cross = first[0] * second[1] - first[1] * second[0]
    if first[0] * second[0] + first[1] * second[1] < 0.0:
        return "u_turn"
    return "left" if cross > 0.0 else "right"


def _bezier(start, incoming, end, outgoing):
    first = DIRECTIONS[incoming]
    last = DIRECTIONS[outgoing]
    control = TURN_CONTROL_DISTANCE
    points = []
    for index in range(13):
        t = index / 12.0
        u = 1.0 - t
        p1 = (start[0] + control * first[0], start[1] + control * first[1])
        p2 = (end[0] - control * last[0], end[1] - control * last[1])
        x = u**3 * start[0] + 3 * u**2 * t * p1[0] + 3 * u * t**2 * p2[0] + t**3 * end[0]
        y = u**3 * start[1] + 3 * u**2 * t * p1[1] + 3 * u * t**2 * p2[1] + t**3 * end[1]
        points.append([round(x, 3), round(y, 3)])
    return points


def _sample_line(start, end, spacing=5.0):
    length = math.dist(start, end)
    steps = max(1, math.ceil(length / spacing))
    return [
        [round(start[0] + (end[0] - start[0]) * index / steps, 3),
         round(start[1] + (end[1] - start[1]) * index / steps, 3)]
        for index in range(steps + 1)
    ]


def generate_map():
    nodes = [
        {"id": _intersection_id(row, col), "x": x, "y": y}
        for row, y in enumerate(GRID)
        for col, x in enumerate(GRID)
    ]
    for row in range(len(GRID)):
        for col in range(len(GRID)):
            for direction in _directions_at(row, col):
                for lane in range(2):
                    point = _portal(row, col, direction, lane, True)
                    nodes.append({
                        "id": _departure_id(row, col, direction, lane),
                        "x": round(point[0], 3),
                        "y": round(point[1], 3),
                    })
            for direction in _incoming_directions(row, col):
                for lane in range(2):
                    point = _portal(row, col, direction, lane, False)
                    nodes.append({
                        "id": _arrival_id(row, col, direction, lane),
                        "x": round(point[0], 3),
                        "y": round(point[1], 3),
                    })

    edges = []
    connectors_by_arrival = {}
    for row in range(len(GRID)):
        for col in range(len(GRID)):
            for direction in _directions_at(row, col):
                dx, dy = DIRECTIONS[direction]
                next_col = col + int(dx)
                next_row = row + int(dy)
                if not (0 <= next_col < len(GRID) and 0 <= next_row < len(GRID)):
                    continue
                road_id = f"{_intersection_id(row, col)}_{_intersection_id(next_row, next_col)}"
                for lane in range(2):
                    edge_id = f"{road_id}_{direction}_l{lane}"
                    start = _portal(row, col, direction, lane, True)
                    end = _portal(next_row, next_col, direction, lane, False)
                    edge = {
                        "id": edge_id,
                        "road_id": road_id,
                        "lane_id": f"{edge_id}_lane",
                        "from": _departure_id(row, col, direction, lane),
                        "to": _arrival_id(next_row, next_col, direction, lane),
                        "lane_index": lane,
                        "speed_limit_kmh": 50.0,
                        "maneuver": "straight",
                        "centerline": _sample_line(start, end),
                        "successors": [],
                    }
                    edges.append(edge)

    for row in range(len(GRID)):
        for col in range(len(GRID)):
            for incoming in _incoming_directions(row, col):
                for lane in range(2):
                    incoming_node = _arrival_id(row, col, incoming, lane)
                    for outgoing in _directions_at(row, col):
                        movement = _turn(incoming, outgoing)
                        # Rightmost lanes feed right turns or continue straight;
                        # the inner lane feeds left turns or continues straight.
                        if movement == "right" and lane != 0:
                            continue
                        if movement == "left" and lane != 1:
                            continue
                        if movement not in {"right", "left", "straight"}:
                            continue
                        dx, dy = DIRECTIONS[outgoing]
                        next_col = col + int(dx)
                        next_row = row + int(dy)
                        if not (0 <= next_col < len(GRID) and 0 <= next_row < len(GRID)):
                            continue
                        end = _portal(row, col, outgoing, lane, True)
                        start = _portal(row, col, incoming, lane, False)
                        connector_id = f"j{row}_{col}_{incoming}_{outgoing}_l{lane}"
                        connector = {
                            "id": connector_id,
                            "road_id": connector_id,
                            "lane_id": f"{connector_id}_lane",
                            "from": incoming_node,
                            "to": _departure_id(row, col, outgoing, lane),
                            "lane_index": lane,
                            "speed_limit_kmh": 30.0 if movement != "straight" else 50.0,
                            "maneuver": movement,
                            "centerline": (
                                [[round(start[0], 3), round(start[1], 3)], [round(end[0], 3), round(end[1], 3)]]
                                if movement == "straight"
                                else _bezier(start, incoming, end, outgoing)
                            ),
                            "successors": [],
                        }
                        edges.append(connector)
                        connectors_by_arrival.setdefault(incoming_node, []).append(connector)

    by_departure = {}
    for edge in edges:
        by_departure.setdefault(edge["from"], []).append(edge)
    for edge in edges:
        if edge["id"].startswith("j"):
            edge["successors"] = [candidate["id"] for candidate in by_departure.get(edge["to"], [])]
        else:
            edge["successors"] = [candidate["id"] for candidate in connectors_by_arrival.get(edge["to"], [])]

    return {
        "map_id": "demo_grid",
        "description": "3x3 urban grid; two-way streets with two lanes per direction",
        "lane_width": LANE_WIDTH,
        "num_lanes": 2,
        "road_network_num_lanes": 4,
        "road_network": (
            [[[GRID[0], y], [GRID[-1], y]] for y in GRID]
            + [[[x, GRID[0]], [x, GRID[-1]]] for x in GRID]
        ),
        "nodes": nodes,
        "edges": edges,
    }


if __name__ == "__main__":
    MAP_PATH.write_text(json.dumps(generate_map(), indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {MAP_PATH} ({len(generate_map()['edges'])} directed lane edges)")
