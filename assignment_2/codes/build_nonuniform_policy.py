"""Historical alternative: section-velocity intervals with constant step count.

The submitted implementation uses the three-node uniform table instead.
This script retains the analytical interval construction for comparison.
"""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from build_lookup_policy import ALPHA_MAX, ALPHA_MIN, CAPTURE, GAMMA, LIMIT, G, L
from plot_analytical_capture import boundaries

OUT = Path(__file__).resolve().parents[1] / "figures"


def main():
    def capture_interval(alpha):
        """Initial section velocities whose next impact enters the standing RoA."""
        lower_velocity, upper_velocity = boundaries(np.array([GAMMA - alpha]))
        impact_factor = np.cos(2 * alpha)
        stance_energy_gain = 2 * G / L * (1 - np.cos(GAMMA + alpha))
        return float(
            np.sqrt(
                max(0, (lower_velocity[0] / impact_factor) ** 2 - stance_energy_gain)
            )
        ), float(np.sqrt((upper_velocity[0] / impact_factor) ** 2 - stance_energy_gain))

    _, small_angle_upper = capture_interval(ALPHA_MIN)
    large_angle_lower, one_step_upper = capture_interval(ALPHA_MAX)
    action_switch = (large_angle_lower + small_angle_upper) / 2
    return_scale = np.cos(2 * ALPHA_MAX) ** 2
    return_offset = (
        2
        * G
        / L
        * (
            return_scale * (1 - np.cos(GAMMA + ALPHA_MAX))
            - (1 - np.cos(GAMMA - ALPHA_MAX))
        )
    )
    two_step_upper = float(
        np.sqrt((one_step_upper * one_step_upper - return_offset) / return_scale)
    )
    three_step_upper = float(
        np.sqrt((two_step_upper * two_step_upper - return_offset) / return_scale)
    )
    edges = [CAPTURE, one_step_upper, two_step_upper]
    # Analytical inequalities certify one-step overlap and the next two preimages.
    assert (
        CAPTURE
        < large_angle_lower
        < action_switch
        < small_angle_upper
        < one_step_upper
        < two_step_upper
        < LIMIT
        < three_step_upper
    )
    # For omega >= small_angle_upper, the return radicand decreases with alpha over the action range.
    # Bound its derivative from above, dropping an additional negative term.
    derivative_upper = -4 * np.cos(2 * ALPHA_MAX) * np.sin(2 * ALPHA_MIN) * (
        small_angle_upper * small_angle_upper
        + 2 * G / L * (1 - np.cos(GAMMA + ALPHA_MIN))
    ) + 2 * G / L * np.cos(2 * ALPHA_MIN) ** 2 * np.sin(GAMMA + ALPHA_MAX)
    assert derivative_upper < 0
    # Positive lower bound for d(U(alpha)^2)/dalpha; omit another positive term.
    capture_derivative_lower = (
        2
        * G
        / L
        * (
            (np.sin(ALPHA_MIN - GAMMA) + 0.1) / np.cos(2 * ALPHA_MIN) ** 2
            - np.sin(GAMMA + ALPHA_MAX)
        )
    )
    assert capture_derivative_lower > 0
    rows = [
        (0, CAPTURE, 0),
        (CAPTURE, one_step_upper, 1),
        (one_step_upper, two_step_upper, 2),
        (two_step_upper, LIMIT, 3),
    ]
    data = {
        "method": "analytical nonuniform intervals",
        "capture_threshold": CAPTURE,
        "small_angle_upper": small_angle_upper,
        "large_angle_lower": large_angle_lower,
        "switch": action_switch,
        "step_boundaries": [CAPTURE, one_step_upper, two_step_upper],
        "next_boundary": three_step_upper,
        "return_derivative_upper_bound": derivative_upper,
        "capture_derivative_lower_bound": capture_derivative_lower,
        "rows": [{"left": x, "right": y, "steps": n} for x, y, n in rows],
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "nonuniform_policy.json").write_text(json.dumps(data, indent=2) + "\n")
    np.savetxt(
        OUT / "nonuniform_policy.csv",
        np.array(rows),
        delimiter=",",
        header="omega_left,omega_right,footstrikes",
        comments="",
    )
    fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True, layout="constrained")
    colors = ["#c9e7df", "#e9bd89", "#bdb0dc", "#dfb9ce"]
    for i, ((x, y, n), color) in enumerate(zip(rows, colors), 1):
        for ax in axs:
            ax.axvspan(x, y, color=color, alpha=0.6)
        axs[1].plot([x, y], [n] * 2, color="#653c84", lw=3)
        axs[0].text((x + y) / 2, 24.1, f"Cell {i}", ha="center", fontsize=14)
    axs[0].plot(
        [CAPTURE, action_switch, action_switch, LIMIT],
        np.degrees([ALPHA_MIN, ALPHA_MIN, ALPHA_MAX, ALPHA_MAX]),
        color="#234a66",
        lw=3,
    )
    for ax in axs:
        for x in edges:
            ax.axvline(x, color="#777777", ls=":", lw=1.3)
        ax.tick_params(labelsize=14)
        ax.set_xlim(0, LIMIT)
    axs[0].set_ylim(22, 26.3)
    axs[0].set_ylabel(r"Action $\alpha$ (degrees)", fontsize=19)
    axs[0].set_title(
        "Nonuniform grid: four constant-step-count intervals", fontsize=21, pad=15
    )
    axs[0].text(CAPTURE / 2, 25.5, "Stand", ha="center", fontsize=13)
    axs[1].set_ylabel("Footstrikes to RoA", fontsize=19)
    axs[1].set_yticks([0, 1, 2, 3])
    axs[1].set_ylim(-0.25, 3.35)
    axs[1].set_xlabel(r"Section velocity $\omega_k$ (rad/s)", fontsize=19)
    fig.supxlabel(
        "Cell edges: " + ", ".join(f"{x:.5f}" for x in [0, *edges, LIMIT]) + " rad/s\n"
        "Use interval membership, not nearest-node lookup. Re-evaluate after each step.",
        fontsize=13,
    )
    fig.savefig(OUT / "nonuniform_policy.png", dpi=180)
    plt.close(fig)
    print(json.dumps(data, indent=2))


if __name__ == "__main__":
    main()
