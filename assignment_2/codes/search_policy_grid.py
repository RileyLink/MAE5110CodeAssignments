"""Historical full-table grid search; final uniform policy has its own validator.

Probe velocities around the 0/1-, 1/2-, and 2/3-impact boundaries to detect
step-count changes, then compare coarser policies with exact-map references.
"""

import json
from pathlib import Path

import numpy as np
from build_lookup_policy import (
    ALPHA_MAX,
    CAPTURE,
    GAMMA,
    LIMIT,
    G,
    L,
    build,
    polish,
    rollout,
)
from plot_analytical_capture import boundaries

OUT = Path(__file__).resolve().parents[1] / "figures"


def tests():
    """Return section-velocity probes and analytical stopping-count thresholds."""
    alpha = ALPHA_MAX
    upper = float(boundaries(np.array([GAMMA - alpha]))[1][0])
    one_step_threshold = np.sqrt(
        (upper - 1e-10) ** 2 / np.cos(2 * alpha) ** 2
        - 2 * G / L * (1 - np.cos(GAMMA + alpha))
    )
    return_scale = np.cos(2 * alpha) ** 2
    return_offset = return_scale * 2 * G / L * (
        1 - np.cos(GAMMA + alpha)
    ) - 2 * G / L * (1 - np.cos(GAMMA - alpha))
    two_step_threshold = np.sqrt((one_step_threshold**2 - return_offset) / return_scale)
    # Fixed probes on both sides of analytically located transition candidates.
    offsets = np.array([-1e-3, -1e-4, -1e-5, -1e-6, 1e-6, 1e-5, 1e-4, 1e-3])
    points = np.unique(
        np.r_[
            np.linspace(0, LIMIT, 10001),
            *(t + offsets for t in [CAPTURE, one_step_threshold, two_step_threshold]),
        ]
    )
    return points, [CAPTURE, one_step_threshold, two_step_threshold]


def evaluate(velocity_count, action_count, points):
    w, alpha, v, p, c, _iterations = build(velocity_count, action_count)
    del c
    v, p, c = polish(w, alpha, p)
    result = rollout(points, w, p, max_steps=20)
    return result, w, alpha, v, p, c


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    points, thresholds = tests()
    print("tests", len(points), "thresholds", thresholds, flush=True)
    previous = None
    for velocity_count, action_count in [
        (2401, 401),
        (4801, 401),
        (4801, 801),
        (9601, 801),
    ]:
        result, w, alpha, v, p, c = evaluate(velocity_count, action_count, points)
        print(
            velocity_count,
            action_count,
            "fail",
            int(np.sum(result < 0)),
            "diff previous",
            None if previous is None else int(np.sum(result != previous)),
            flush=True,
        )
        np.savez(
            OUT / f"reference_{velocity_count}_{action_count}.npz",
            result=result,
            omega=w,
            alpha=alpha,
            value=v,
            policy=p,
        )
        previous = result
        del c

    rows = []
    best = []
    reference = previous
    for cells in range(4, 401):
        for velocity_count in range(2, cells // 2 + 1):
            if cells % velocity_count:
                continue
            action_count = cells // velocity_count
            result, *unused = evaluate(velocity_count, action_count, points)
            row = {
                "nw": velocity_count,
                "na": action_count,
                "cells": cells,
                "failures": int(np.sum(result < 0)),
                "disagreements": int(np.sum((result >= 0) & (result != reference))),
            }
            rows.append(row)
            if row["failures"] == 0 and row["disagreements"] == 0:
                best.append(row)
        if best:
            break
    (OUT / "grid_search_results.json").write_text(
        json.dumps(
            {
                "test_count": len(points),
                "thresholds": thresholds,
                "reference_failures": int(np.sum(reference < 0)),
                "results": rows,
                "best": best,
            },
            indent=2,
        )
        + "\n"
    )
    print("Smallest passing tables:", best, flush=True)
