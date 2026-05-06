from __future__ import annotations

import json
from pathlib import Path

from pipe_routing.io import load_routing_case
from pipe_routing.multi_pipe import route_pipes_sequentially
from pipe_routing.visualize import write_result_html


def main() -> None:
    root = Path(__file__).resolve().parent
    input_path = root / "data" / "demo_case.json"
    output_dir = root / "outputs"
    output_dir.mkdir(parents=True, exist_ok=True)

    case = load_routing_case(input_path)
    result = route_pipes_sequentially(case)

    json_path = output_dir / "result.json"
    html_path = output_dir / "result.html"

    with json_path.open("w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    write_result_html(case, result, html_path)

    for pipe in result["pipes"]:
        if pipe["success"]:
            print(
                f"{pipe['id']} success=True length={pipe['length']:.1f} "
                f"bend_count={pipe['bend_count']} conflict_count={pipe['conflict_count']}"
            )
        else:
            print(f"{pipe['id']} success=False error=\"{pipe['error']}\"")


if __name__ == "__main__":
    main()
