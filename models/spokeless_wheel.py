import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch

STANDING = 0
ROLLING = 1
UNRESOLVED = 2

def dynamics(t, state, params):
    gravity = params["gravity"]
    length = params["length"]
    gamma = params["gamma"]

    theta = state[0]
    theta_dot = state[1]

    theta_ddot = (1/length) * (gravity * np.sin(theta+gamma))

    state_derivative = np.array([theta_dot, theta_ddot])
    return state_derivative

def check_event(state, params):
    theta = state[0]
    theta_dot = state[1]
    N_spokes = params["N_spokes"]
    two_alpha = 2*np.pi / N_spokes
    #angle_at_two_axles_touching = np.pi/2 - two_alpha/2  #np.arcsin(length * np.sin(two_alpha) / np.sqrt(2*length**2*(1-np.cos(two_alpha))))
    # if np.abs(np.pi/2 - theta - angle_at_two_axles_touching) <= 0.01:
    #     theta_dot = theta_dot * np.cos(two_alpha)
    #     theta = theta-two_alpha
    event_bool = False

    if theta >= two_alpha/2 and theta_dot > 0: # Forward Collision
        theta = theta - two_alpha
        theta_dot = theta_dot * np.cos(two_alpha)
        event_bool = True
    elif theta <= -two_alpha/2 and theta_dot < 0: # Backward Collision
        theta = theta + two_alpha
        theta_dot = theta_dot * np.cos(two_alpha)
        event_bool = True

    return event_bool, np.array([theta, theta_dot])

def generate_params():
    params = {
        "gravity": 9.81,  # gravity m/s^2)
        "length": 1,  # rod length (m)
        "mass": 1,  # point mass at end of rod (kg)
        "gamma": np.pi/6,
        "N_spokes": 10
    }
    return params

def calculate_energy(state, params):
    """Compute energies for a state ``(2,)`` or trajectory ``(2, N)``."""
    gravity = params["gravity"]
    length = params["length"]
    mass = params["mass"]
    gamma = params["gamma"]

    angle = state[0]  # indexes entire row "vectorized" if state is (2, N)
    angular_velocity = state[1]

    kinetic_energy = 0.5 * mass * (length * angular_velocity) ** 2
    potential_energy = mass * gravity * length * np.cos(angle+gamma)
    return kinetic_energy, potential_energy


### Poincare ##########################################################################################
def collect_poincare_pairs(
    initial_state,
    params,
    integrator,
    timestep=1e-3,
    sim_time=5.0
):
    """Simulate one trajectory and collect consecutive forward-impact speeds."""
    alpha = np.pi / params["N_spokes"]
    initial_state = np.asarray(initial_state, dtype=float)

    if not np.isclose(initial_state[0], -alpha):
        raise ValueError("initial_state must start just after impact at theta = -alpha")

    events, time_traj, state_traj = integrator(
        timestep,
        sim_time,
        initial_state,
        dynamics,
        params,
        check_event
    )

    event_mask = np.asarray(events, dtype=bool).reshape(-1)
    event_indices = np.flatnonzero(event_mask)

    # check_event stores post-impact states. Positive velocity selects
    # forward impacts and excludes backward impacts.
    forward_indices = event_indices[state_traj[1, event_indices] > 0]
    forward_event_times = time_traj[forward_indices]
    forward_event_speeds = state_traj[1, forward_indices]

    # The initial state is itself the first post-impact section point.
    section_speeds = np.concatenate((
        np.array([initial_state[1]]),
        forward_event_speeds
    ))

    theta_dot_k = section_speeds[:-1]
    theta_dot_next = section_speeds[1:]

    return theta_dot_k, theta_dot_next, forward_event_times


def plot_poincare_map(theta_dot_k, theta_dot_next):
    """Plot theta_dot[k+1] = P(theta_dot[k]) and the identity line."""
    theta_dot_k = np.asarray(theta_dot_k)
    theta_dot_next = np.asarray(theta_dot_next)

    if theta_dot_k.size == 0:
        raise ValueError("The trajectory did not contain a forward return")

    limits = np.concatenate((theta_dot_k, theta_dot_next))
    lower = np.min(limits)
    upper = np.max(limits)
    padding = max(0.05 * (upper - lower), 0.05)
    identity_limits = [lower - padding, upper + padding]

    fig, ax = plt.subplots()
    ax.scatter(theta_dot_k, theta_dot_next, label="Return-map samples")
    ax.plot(identity_limits, identity_limits, "k--", label="Identity line")
    ax.set_xlim(identity_limits)
    ax.set_ylim(identity_limits)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel(r"$\dot{\theta}_k$")
    ax.set_ylabel(r"$\dot{\theta}_{k+1}=P(\dot{\theta}_k)$")
    ax.set_title("Poincare Return Map")
    ax.grid()
    ax.legend()
    plt.tight_layout()
    plt.show()

    return fig, ax



def evaluate_poincare_map(
    theta_dot_k,
    params,
    integrator,
    timestep=1e-3,
    max_time=5.0
):
    alpha = np.pi / params["N_spokes"]
    initial_state = np.array([-alpha, theta_dot_k])

    events, _, state_traj = integrator(
        timestep,
        max_time,
        initial_state,
        dynamics,
        params,
        check_event
    )

    event_indices = np.flatnonzero(events)

    # Keep forward impacts
    forward_indices = event_indices[
        state_traj[1, event_indices] > 0
    ]

    if len(forward_indices) == 0:
        return None

    first_forward_impact = forward_indices[0]

    return state_traj[1, first_forward_impact]

def estimate_floquet_multiplier(
    theta_dot_fixed,
    params,
    integrator,
    epsilon=1e-2
):
    p_left = evaluate_poincare_map(
        theta_dot_fixed - epsilon,
        params,
        integrator
    )

    p_right = evaluate_poincare_map(
        theta_dot_fixed + epsilon,
        params,
        integrator
    )

    if p_left is None or p_right is None:
        raise ValueError("A perturbed state did not return to the section")

    derivative = (p_right - p_left) / (2 * epsilon)

    return derivative

### ROA #############################################################################################

def classify_trajectory(state_traj, alpha):
    # Examine the final 20% of the complete trajectory
    tail_start = int(0.8 * state_traj.shape[1])

    theta_tail = state_traj[0, tail_start:]
    velocity_tail = state_traj[1, tail_start:]

    tolerance = 0.02

    # Nearly no motion throughout the final section
    if np.max(np.abs(velocity_tail)) < tolerance:
        return STANDING

    # Continues moving forward through approximately complete steps
    if (np.mean(velocity_tail) > tolerance and np.ptp(theta_tail) > alpha):
        return ROLLING

    return UNRESOLVED

def calculate_roa(
    params,
    model,
    integrator,
    n_theta=11,
    n_velocity=11,
    velocity_limit=6.0,
    timestep=1e-3,
    sim_time=5
):
    alpha = np.pi / params["N_spokes"]

    # Physically valid initial stance angles
    theta_values = np.linspace(
        -alpha + 1e-6,
        alpha - 1e-6,
        n_theta
    )

    velocity_values = np.linspace(
        -velocity_limit,
        velocity_limit,
        n_velocity
    )

    roa = np.full(
        (n_velocity, n_theta),
        UNRESOLVED,
        dtype=int
    )

    for row, theta_dot_0 in enumerate(velocity_values):
        for column, theta_0 in enumerate(theta_values):
            initial_state = np.array([theta_0, theta_dot_0])

            integrator_result = integrator(
                timestep,
                sim_time,
                initial_state,
                model.dynamics,
                params,
                model.check_event
            )
            state_traj = integrator_result[-1]

            roa[row, column] = classify_trajectory(
                state_traj,
                alpha
            )

    return theta_values, velocity_values, roa


def plot_roa_sweep(
    slopes,
    spoke_counts,
    model,
    integrator,
    n_theta=11,
    n_velocity=11,
    timestep=1e-5,
    sim_time=5
):
    base_params = model.generate_params()

    colors = ListedColormap([
        "royalblue",   # standing
        "darkorange",  # rolling
        "lightgray"    # unresolved
    ])

    norm = BoundaryNorm(
        [-0.5, 0.5, 1.5, 2.5],
        colors.N
    )

    fig, axes = plt.subplots(
        len(slopes),
        len(spoke_counts),
        figsize=(4 * len(spoke_counts), 3.5 * len(slopes)),
        squeeze=False,
        sharey=True
    )

    for row, gamma in enumerate(slopes):
        for column, n_spokes in enumerate(spoke_counts):
            params = base_params.copy()
            params["gamma"] = gamma
            params["N_spokes"] = n_spokes

            theta_values, velocity_values, roa = calculate_roa(
                params,
                model,
                integrator,
                n_theta=n_theta,
                n_velocity=n_velocity,
                timestep=timestep,
                sim_time=sim_time
            )

            ax = axes[row, column]

            ax.pcolormesh(
                np.rad2deg(theta_values),
                velocity_values,
                roa,
                cmap=colors,
                norm=norm,
                shading="nearest"
            )

            ax.set_title(
                f"γ={np.rad2deg(gamma):.1f}°, N={n_spokes}"
            )
            ax.set_xlabel("Initial θ (degrees)")
            ax.set_ylabel("Initial θ̇ (rad/s)")

    legend = [
        Patch(color="royalblue", label="Standing"),
        Patch(color="darkorange", label="Rolling"),
        Patch(color="lightgray", label="Unresolved")
    ]

    fig.legend(handles=legend, loc="upper center", ncol=3)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()
