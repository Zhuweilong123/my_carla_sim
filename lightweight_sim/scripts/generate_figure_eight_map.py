"""Generate the routing map that matches the built-in figure-eight scene."""

import json
import math
from pathlib import Path


def main() -> None:
    half_length = 78.0
    half_height = 42.0
    sample_count = 241
    centerline = [
        [
            half_length * math.cos(2.0 * math.pi * index / (sample_count - 1)),
            half_height * math.sin(4.0 * math.pi * index / (sample_count - 1)),
        ]
        for index in range(sample_count)
    ]
    road_map = {
        "map_id": "figure_eight",
        "lane_width": 3.5,
        "num_lanes": 3,
        "nodes": [{"id": "crossing_start", "x": half_length, "y": 0.0}],
        "edges": [
            {
                "id": "figure_eight_lane_1",
                "road_id": "figure_eight",
                "lane_id": "figure_eight_lane_1",
                "from": "crossing_start",
                "to": "crossing_start",
                "lane_index": 1,
                "speed_limit_kmh": 40.0,
                "maneuver": "curve",
                "centerline": centerline,
                "successors": ["figure_eight_lane_1"],
            }
        ],
    }
    output = Path(__file__).resolve().parents[1] / "config" / "maps" / "figure_eight.json"
    output.write_text(json.dumps(road_map, indent=2) + "\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
