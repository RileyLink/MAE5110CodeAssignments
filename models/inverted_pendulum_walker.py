"""InvertedPendulumWalker starter model, with visualization provided.

Implement the model functions for Assignment 2. The visualizer works independently
of those functions; it draws a supplied state without advancing the simulation.
"""

import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from matplotlib.animation import FuncAnimation, PillowWriter


def generate_params():
    """
    Generates useful parameters
    """
    params = {
        "gravity": 9.81,  # gravity m/s^2)
        "length": 1,  # rod length (m)
        "mass": 1,  # point mass at end of rod (kg)
        "incline": 0.06,
        "N_spokes": 10,
        "angle_of_attack": np.pi/8,
        "ankle_torque": 0
    }
    return params


def dynamics(t, state, params):
    """
    Calculates the dynamics x_dot = [theta_dot, theta_ddot] for a spokeless wheel
    
    args:
        t: time, ununsed since these dynamics are autonomous
        state: Contains the state x=[theta,theta_dot] where theta = 0 is the vertical axis
        params: useful parameters including gravity, length of the spokes, and angle of the axis (incline).

    Returns: 
        state_derivative: array of the derivative [theta_dot, theta double dot]
    """
    gravity = params["gravity"]
    length = params["length"]
    ankle_torque = params["ankle_torque"]
    mass = params["mass"]

    theta = state[0]       # Angle from world vertical
    theta_dot = state[1]

    theta_ddot = (gravity / length) * np.sin(theta) + ankle_torque / (mass * length**2)
    state_derivative = np.array([theta_dot, theta_ddot])
    return state_derivative

def event_guard(previous_state, next_state, params):
    incline = params["incline"]
    alpha = params["angle_of_attack"]

    previous_theta = previous_state[0]
    next_theta = next_state[0]
    next_theta_dot = next_state[1]

    # Express angles relative to the ramp normal
    previous_relative = previous_theta - incline
    next_relative = next_theta - incline

    forward_collision = (
        previous_relative < alpha <= next_relative
        and next_theta_dot > 0
    )

    backward_collision = (
        previous_relative > -alpha >= next_relative
        and next_theta_dot < 0
    )

    return forward_collision or backward_collision


def event_dynamics(state, params):
    theta = state[0]
    theta_dot = state[1]

    alpha = params["angle_of_attack"]

    if theta_dot > 0:
        # Forward collision
        theta -= 2 * alpha

    elif theta_dot < 0:
        # Backward collision
        theta += 2 * alpha

    theta_dot *= np.cos(2 * alpha)

    return np.array([theta, theta_dot])


def calculate_energy(state, params):
    gravity = params["gravity"]
    length = params["length"]
    mass = params["mass"]

    theta = state[0]
    theta_dot = state[1]

    kinetic_energy = 0.5 * mass * (length * theta_dot) ** 2
    potential_energy = mass * gravity * length * np.cos(theta)

    return kinetic_energy, potential_energy

def visualize(
    state,
    params,
    ax=None,
    *,
    show_swing=True,
    stance_position=(0.0, 0.0),
    view_limits=None,
):
    """Draw one walker pose and return a Matplotlib Axes.

    Parameters
    ----------
    state : array-like, shape (2,)
        [theta, angular_velocity], in radians and radians/second. Theta is
        measured clockwise from upward vertical; positive x points right.
    params : dict
        ``length`` is the leg length in meters. ``incline`` is the ground's
        downhill slope angle in radians (positive slopes descend to the right).
        ``angle_of_attack`` is HALF the angle between the stance and forward swing
        legs, in radians; it is needed only when show_swing=True.
        ``ankle_torque`` (optional, default 0) is displayed in N m, with positive
        torque acting in the positive theta direction. Other keys are ignored.
    ax : matplotlib.axes.Axes, optional
        Axes to clear and reuse. If omitted, create a figure. This function
        neither shows nor saves it: use plt.show() or ax.figure.savefig(...).
    show_swing : bool
        Draw a straight forward swing leg at the supplied angle_of_attack. Set False
        while the swing leg is held clear or while balancing. Swing motion is
        not part of the two-state model and is not inferred from theta.
    stance_position : pair of floats
        Current stance foot's (x, y) in meters, default (0, 0). The two-state
        model does not track translation; supply foot positions if desired.
        Ground passes through this point at the supplied incline.
    view_limits : (xmin, xmax, ymin, ymax), optional
        Fixed camera bounds in meters. By default the view follows the stance
        foot with bounds that fit both legs at any angle. Supply the same bounds
        each frame for a stationary world view.

    Notes
    -----
    Draws the supplied pose; contact events belong in the simulation.
    Reuse ax for frame sequences; use evenly spaced simulation times for playback
    at a fixed frame rate, and pass the parameters actually used at each frame.
    """
    state = np.asarray(state, dtype=float)
    foot = np.asarray(stance_position, dtype=float)
    if state.shape != (2,) or not np.all(np.isfinite(state)):
        raise ValueError("state must contain two finite values: [theta, velocity].")
    if foot.shape != (2,) or not np.all(np.isfinite(foot)):
        raise ValueError("stance_position must contain two finite values: [x, y].")
    length = float(params["length"])
    incline = float(params["incline"])
    torque = float(params.get("ankle_torque", 0.0))
    if not np.isfinite(length) or length <= 0:
        raise ValueError("length must be finite and positive.")
    if not np.isfinite(incline) or abs(incline) >= np.pi / 2:
        raise ValueError("incline must be finite and between -pi/2 and pi/2.")
    if not np.isfinite(torque):
        raise ValueError("ankle_torque must be finite.")
    if show_swing:
        angle_of_attack = float(params["angle_of_attack"])
        if not np.isfinite(angle_of_attack):
            raise ValueError("angle_of_attack must be finite.")

    if view_limits is None:
        radius = 2.15 * length
        view_limits = (
            foot[0] - radius,
            foot[0] + radius,
            foot[1] - radius,
            foot[1] + radius,
        )
    limits = np.asarray(view_limits, dtype=float)
    if (
        limits.shape != (4,)
        or not np.all(np.isfinite(limits))
        or limits[0] >= limits[1]
        or limits[2] >= limits[3]
    ):
        raise ValueError(
            "view_limits must be (xmin, xmax, ymin, ymax) with increasing bounds."
        )

    if ax is None:
        _, ax = plt.subplots(figsize=(6, 6), layout="constrained")
    ax.clear()
    theta, angular_velocity = state
    hub = foot + length * np.array([np.sin(theta), np.cos(theta)])

    ground_x = np.array(limits[:2])
    ground_y = foot[1] - np.tan(incline) * (ground_x - foot[0])
    ax.fill_between(ground_x, ground_y, limits[2], color="#eee7dc", zorder=0)
    ax.plot(ground_x, ground_y, color="#7b6651", linewidth=2, label="Ground")
    ax.plot(
        [foot[0], foot[0]],
        [foot[1], foot[1] + 1.25 * length],
        ":",
        color="0.7",
        linewidth=1,
        label="Vertical",
    )

    if show_swing:
        swing_angle = theta - 2 * angle_of_attack
        swing_foot = hub - length * np.array([np.sin(swing_angle), np.cos(swing_angle)])
        swing_color = "#df8a25"
        ax.plot(
            [hub[0], swing_foot[0]],
            [hub[1], swing_foot[1]],
            "--",
            color=swing_color,
            linewidth=2.5,
            label="Swing leg",
            zorder=3,
        )
        ax.plot(
            *swing_foot,
            "o",
            color=swing_color,
            markersize=7,
            zorder=4,
            label="Swing foot",
        )

    stance_color = "#23699b"
    ax.plot(
        [foot[0], hub[0]],
        [foot[1], hub[1]],
        color=stance_color,
        linewidth=4,
        label="Stance leg",
        zorder=4,
    )
    ax.plot(*foot, "s", color="#333333", markersize=8, zorder=5, label="Stance foot")
    ax.plot(
        *hub,
        "o",
        color=stance_color,
        markeredgecolor="white",
        markersize=17,
        zorder=6,
        label="Hub",
    )
    ax.text(
        0.03,
        0.97,
        f"$\\theta$ = {theta:.3f} rad\n"
        f"$\\dot\\theta$ = {angular_velocity:.3f} rad/s\n"
        f"$\\tau$ = {torque:.3f} N m",
        transform=ax.transAxes,
        va="top",
        fontsize=10,
        bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.85},
    )
    ax.set(
        xlim=limits[:2],
        ylim=limits[2:],
        xlabel="x (m)",
        ylabel="y (m)",
        title="Inverted pendulum walker",
    )
    ax.set_aspect("equal", adjustable="box")
    return ax


def animate(state_traj,time_traj,params,timestep,completed_steps):
    fig, ax = plt.subplots(figsize=(8, 5), layout="constrained")


    def draw_frame(index):
        # The massless swing leg is repositioned instantaneously at each impact.
        visualize(state_traj[:, index], params, ax=ax)
        ax.set_title(f"t = {time_traj[index]:.2f} s")


    # Simulate at a small timestep, but render only 25 frames per second.
    fps = 25
    frame_stride = round(1 / (fps * timestep))
    frame_indices = list(range(0, time_traj.size, frame_stride))
    if frame_indices[-1] != time_traj.size - 1:
        frame_indices.append(time_traj.size - 1)

    animation = FuncAnimation(
        fig, draw_frame, frames=frame_indices, interval=1000 / fps, repeat=False
    )
    output = Path("output/assignment_2")
    output.mkdir(parents=True, exist_ok=True)
    animation.save(output / "walker.gif", writer=PillowWriter(fps=fps))

    # To save an MP4 instead, install FFmpeg and use:
    # animation.save(output / "walker.mp4", writer="ffmpeg", fps=fps)
    print(f"Saved {output / 'walker.gif'} ({completed_steps} footstrikes).")
    plt.show()