"""Analytical state-action table, backward policy, and exact-map rollout checks."""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from plot_analytical_capture import ALPHA_MAX, ALPHA_MIN, GAMMA, G, L, boundaries

OUT = Path(__file__).resolve().parents[1] / "figures"
LIMIT = np.sqrt(2 * G / L)
CAPTURE = float(boundaries(np.array([0.0]))[1][0])
INF = 10000  # Sentinel cost greater than the 100-impact search horizon.
CAPTURE_TOLERANCE = 1e-10  # rad/s, strict buffer around analytical boundaries.


def transition(omega, alpha):
    """Map (theta=0, omega>0, alpha) through one passive stance and impact.

    Return capture-at-impact flag, valid forward-section return flag, exact
    next section velocity, and post-impact velocity margin to the RoA boundary.
    Callers handle states already captured at the initial section separately.
    The 1e-10 rad/s buffer excludes floating-point ambiguity at the separatrices;
    it is not the finite standing tolerance or an empirical tuning parameter.
    """
    theta = GAMMA - alpha
    post = np.cos(2 * alpha) * np.sqrt(
        omega**2 + 2 * G / L * (1 - np.cos(GAMMA + alpha))
    )
    low, high = boundaries(theta)
    capture = (post > low + CAPTURE_TOLERANCE) & (post < high - CAPTURE_TOLERANCE)
    squared = post**2 - 2 * G / L * (1 - np.cos(theta))
    next_velocity = np.sqrt(np.maximum(squared, 0))
    valid = (~capture) & (squared > 0) & (next_velocity <= LIMIT)
    return capture, valid, next_velocity, np.minimum(post - low, high - post)


def build(velocity_count, action_count):
    """Backward dynamic programming on uniform velocity and landing-angle nodes."""
    w = np.linspace(0, LIMIT, velocity_count)
    alpha = np.linspace(ALPHA_MIN, ALPHA_MAX, action_count)
    capture, valid, next_velocity, margin = transition(w[:, None], alpha[None, :])
    successor_indices = np.clip(
        np.rint(next_velocity / (w[1] - w[0])).astype(int), 0, velocity_count - 1
    )
    value = np.full(velocity_count, INF, dtype=int)
    value[w < CAPTURE - CAPTURE_TOLERANCE] = 0
    for iteration in range(100):
        cost = np.where(capture, 1, np.where(valid, 1 + value[successor_indices], INF))
        new = np.minimum(value, cost.min(axis=1))
        if np.array_equal(value, new):
            break
        value = new
    else:
        raise RuntimeError("Dynamic programming did not converge")
    cost = np.where(capture, 1, np.where(valid, 1 + value[successor_indices], INF))
    # Among minimum-step actions, prefer deeper capture or a lower next velocity.
    score = np.where(capture, margin, -next_velocity)
    chosen = np.argmax(np.where(cost == value[:, None], score, -np.inf), axis=1)
    policy = alpha[chosen]
    policy[(value == 0) | (value >= INF)] = np.nan
    return w, alpha, value, policy, cost, iteration


def rollout(starts, w, policy, max_steps=100):
    """Execute a landing-angle policy from actual section velocities.

    Count impacts until capture using analytical transitions. Only action lookup
    uses the nearest node; successor velocities are never rounded in the rollout.
    A result of -1 denotes failure or no capture before max_steps.
    """
    omega = starts.copy()
    count = np.full(omega.shape, -1, dtype=int)
    count[omega < CAPTURE - CAPTURE_TOLERANCE] = 0
    active = count < 0
    for step in range(1, max_steps + 1):
        rows = np.flatnonzero(active)
        if not rows.size:
            break
        state_indices = np.clip(
            np.rint(omega[rows] / (w[1] - w[0])).astype(int), 0, len(w) - 1
        )
        # The actual state has already failed the RoA test. A nearby terminal
        # grid point has no stepping action; use the nearest nonterminal cell.
        first_walking = int(np.searchsorted(w, CAPTURE - CAPTURE_TOLERANCE))
        state_indices = np.maximum(state_indices, first_walking)
        actions = policy[state_indices]
        has_action = np.isfinite(actions)
        active[rows[~has_action]] = False
        rows = rows[has_action]
        if not rows.size:
            continue
        captured, valid, next_velocity, _ = transition(omega[rows], actions[has_action])
        count[rows[captured]] = step
        active[rows[captured | ~valid]] = False
        omega[rows[valid]] = next_velocity[valid]
        # Safety check of actual section state, independent of the rounded grid index.
        at_goal = valid & (next_velocity < CAPTURE - CAPTURE_TOLERANCE)
        count[rows[at_goal]] = step
        active[rows[at_goal]] = False
    return count


def polish(w, alpha, policy):
    """Policy improvement using exact transitions instead of rounded successors."""
    capture, valid, next_velocity, margin = transition(w[:, None], alpha[None, :])
    for _ in range(12):
        future = rollout(next_velocity.ravel(), w, policy).reshape(next_velocity.shape)
        cost = np.where(capture, 1, np.where(valid & (future >= 0), 1 + future, INF))
        value = cost.min(axis=1)
        value[w < CAPTURE - CAPTURE_TOLERANCE] = 0
        score = np.where(capture, margin, -next_velocity)
        choice = np.argmax(np.where(cost == value[:, None], score, -np.inf), axis=1)
        new = alpha[choice]
        new[(value == 0) | (value >= INF)] = np.nan
        if np.array_equal(new, policy, equal_nan=True):
            return value, new, cost
        policy = new
    raise RuntimeError("Exact-map policy improvement did not stabilize")


def draw(w, alpha, value, policy, cost):
    display_w = np.unique(np.r_[np.linspace(0, LIMIT, 10001), w])
    display_ids = np.maximum(
        np.rint(display_w / (w[1] - w[0])).astype(int),
        np.searchsorted(w, CAPTURE - CAPTURE_TOLERANCE),
    )
    display_policy = policy[np.clip(display_ids, 0, len(w) - 1)].copy()
    display_policy[display_w < CAPTURE - CAPTURE_TOLERANCE] = np.nan
    display_count = rollout(display_w, w, policy, max_steps=20)
    fig, axes = plt.subplots(2, 1, figsize=(12, 9), sharex=True, layout="constrained")
    axes[0].plot(display_w, np.degrees(display_policy), lw=2.4, color="#23699b")
    axes[0].set_ylabel(r"Selected $\alpha$ (degrees)", fontsize=19)
    axes[0].set_title("Minimum-step landing-angle lookup policy", fontsize=23, pad=14)
    axes[0].set_ylim(np.degrees(ALPHA_MIN) - 0.3, np.degrees(ALPHA_MAX) + 0.3)
    axes[1].step(
        display_w,
        np.where(display_count >= 0, display_count, np.nan),
        where="mid",
        lw=2.8,
        color="#694399",
    )
    axes[1].set_ylabel("Footstrikes to capture\n(verified policy)", fontsize=19)
    axes[1].set_xlabel(
        r"Section velocity $\omega_k$ (rad/s), at $\theta=0$", fontsize=19
    )
    axes[1].set_yticks(range(int(value[value < INF].max()) + 1))
    for ax in axes:
        ax.axvspan(0, CAPTURE, color="#acdacf", alpha=0.7)
        ax.axvline(CAPTURE, c="#438c7c", ls="--", lw=1.4)
        ax.set_xlim(0, LIMIT)
        ax.tick_params(labelsize=15)
        ax.grid(alpha=0.2)
    axes[0].text(
        0.38,
        0.06,
        "Green: activate standing control; no landing angle needed",
        transform=axes[0].transAxes,
        va="bottom",
        fontsize=14,
        bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.95},
    )
    fig.savefig(OUT / "lookup_policy.png", dpi=180)
    plt.close(fig)
    from plot_uniform_velocity import draw_figure

    draw_figure(w, alpha, cost)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    velocity_count, action_count = 2401, 401
    w, alpha, value, policy, cost, iterations = build(velocity_count, action_count)
    value, policy, cost = polish(w, alpha, policy)
    actual = rollout(w, w, policy)
    mids = (w[:-1] + w[1:]) / 2
    midactual = rollout(mids, w, policy)
    data = {
        "method": "backward grid policy followed by exact-map policy improvement",
        "velocity_points": velocity_count,
        "action_points": action_count,
        "velocity_spacing": float(w[1] - w[0]),
        "action_spacing_rad": float(alpha[1] - alpha[0]),
        "backward_iterations": iterations,
        "capture_velocity": CAPTURE,
        "maximum_finite_steps": int(value[value < INF].max()),
        "unresolved_grid_states": int(np.sum(value >= INF)),
        "grid_rollout_failures": int(np.sum((value < INF) & (actual < 0))),
        "grid_step_count_mismatches": int(np.sum((value < INF) & (actual != value))),
        "midpoint_rollout_failures": int(np.sum(midactual < 0)),
        "midpoint_tests": len(mids),
    }
    np.savez(
        OUT / "lookup_policy.npz",
        omega=w,
        alpha=alpha,
        value=value,
        policy=policy,
        cost=cost,
    )
    np.savetxt(
        OUT / "lookup_policy.csv",
        np.column_stack([w, policy, np.where(value < INF, value, np.nan), actual]),
        delimiter=",",
        header="omega_rad_s,alpha_rad,min_steps_grid,exact_map_steps",
        comments="",
    )
    (OUT / "lookup_policy_summary.json").write_text(json.dumps(data, indent=2) + "\n")
    draw(w, alpha, value, policy, cost)
    print(json.dumps(data, indent=2))


if __name__ == "__main__":
    main()
