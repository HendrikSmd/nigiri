#!/usr/bin/env python3
"""Plot execution-time distributions from nigiri benchmark CSV files."""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_times(path: Path) -> tuple[str, list[float]]:
    with path.open(newline="") as stream:
        first_line = stream.readline()
        if first_line.startswith("#"):
            label = first_line[1:].strip()
        else:
            label = path.stem
            stream.seek(0)
        rows = csv.DictReader(stream)
        try:
            return label, [float(row["execution_time_ms"]) for row in rows]
        except KeyError as error:
            raise ValueError(
                f"{path}: missing execution_time_ms CSV column"
            ) from error


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Create a violin plot of benchmark execution times."
    )
    parser.add_argument(
        "inputs", nargs="+", type=Path, help="CSV files from convert-benchmark"
    )
    parser.add_argument(
        "-o", "--output", type=Path, default=Path("benchmark-violins.png")
    )
    args = parser.parse_args()

    labels = []
    data = []
    for path in args.inputs:
        label, times = read_times(path)
        if not times:
            raise ValueError(f"{path}: no benchmark points found")
        labels.append(label)
        data.append(times)

    figure, axis = plt.subplots(figsize=(max(6, 1.5 * len(data)), 5))
    axis.violinplot(data, showmeans=True, showmedians=True, showextrema=True)
    axis.set_xticks(range(1, len(labels) + 1), labels, rotation=30, ha="right")
    axis.set_ylabel("Execution time (ms)")
    axis.set_yscale('log')
    axis.set_title("Benchmark execution-time distribution")
    axis.grid(axis="y", alpha=0.25)
    figure.tight_layout()
    figure.savefig(args.output, dpi=150)
    plt.close(figure)


if __name__ == "__main__":
    main()
