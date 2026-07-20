#!/usr/bin/env python3
"""Plot the all-location and partition-level importance distributions."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def read_distribution(path: Path) -> tuple[int, list[float], list[float]]:
    with path.open() as stream:
        data = json.load(stream)

    try:
        level = int(data["level"])
        importances_all = [float(value) for value in data["importances_all"]]
        importances_level = [float(value) for value in data["importances_level"]]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            f"{path}: expected level, importances_all, and importances_level"
        ) from error

    if not importances_all or not importances_level:
        raise ValueError(f"{path}: both importance vectors must be non-empty")
    return level, importances_all, importances_level


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Create an overlaid distribution plot from para output."
    )
    parser.add_argument("input", type=Path, help="JSON file from para distribution")
    parser.add_argument(
        "-o", "--output", type=Path, default=Path("importance-distributions.png")
    )
    parser.add_argument("--bins", type=int, default=40, help="number of histogram bins")
    parser.add_argument(
        "--percentile",
        type=float,
        default=99.0,
        help="upper percentile to retain (default: 99; use 100 to keep all data)",
    )
    args = parser.parse_args()

    if args.bins <= 0:
        parser.error("--bins must be positive")
    if not 0 < args.percentile <= 100:
        parser.error("--percentile must be greater than 0 and at most 100")

    level, importances_all, importances_level = read_distribution(args.input)
    values = np.asarray(importances_all + importances_level)
    cutoff = float(np.percentile(values, args.percentile))
    importances_all_plot = [value for value in importances_all if value <= cutoff]
    importances_level_plot = [value for value in importances_level if value <= cutoff]
    if not importances_all_plot or not importances_level_plot:
        raise ValueError("the percentile cutoff removed all values from one distribution")

    plot_values = np.asarray(importances_all_plot + importances_level_plot)
    bin_edges = np.histogram_bin_edges(plot_values, bins=args.bins)
    maximum = float(plot_values.max())
    x_max = maximum if maximum > 0 else 1.0

    figure, axis = plt.subplots(figsize=(8, 5))
    axis.hist(
        importances_all_plot,
        bins=bin_edges,
        density=True,
        color="tab:blue",
        alpha=0.85,
        histtype="step",
        linewidth=1.8,
        label=f"all locations (n={len(importances_all_plot)}/{len(importances_all)})",
    )
    axis.hist(
        importances_level_plot,
        bins=bin_edges,
        density=True,
        color="tab:orange",
        alpha=0.85,
        histtype="step",
        linewidth=1.8,
        label=f"level {level} (n={len(importances_level_plot)}/{len(importances_level)})",
    )
    axis.set_xlim(0, x_max)
    axis.set_xlabel("Location importance")
    axis.set_ylabel("Density")
    axis.set_title(f"Location-importance distributions (up to {args.percentile:g}th percentile)")
    axis.legend()
    axis.grid(axis="y", alpha=0.25)
    figure.tight_layout()
    figure.savefig(args.output, dpi=150)
    plt.close(figure)


if __name__ == "__main__":
    main()
