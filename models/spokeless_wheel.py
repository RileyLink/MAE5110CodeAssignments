import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch
from matplotlib.animation import FuncAnimation
import time

def dynamics(t, state, params):
    """
    Calculates the dynamics x_dot = [theta_dot, theta_ddot] for a spokeless wheel
    
    args:
        t: time, ununsed since these dynamics are autonomous
        state: Contains the state x=[theta,theta_dot] where theta is the angle away from the vertical axis of the
                current spoke the wheel is on, i.e. the axis is always normal to the ramp
        params: useful parameters including gravity, length of the spokes, and angle of the axis (gamma).

    Returns: 
        state_derivative: array of the derivative [theta_dot, theta double dot]
    """
    gravity = params["gravity"]
    length = params["length"]
    gamma = params["gamma"]

    theta = state[0]
    theta_dot = state[1]

    theta_ddot = (1/length) * (gravity * np.sin(theta+gamma))

    state_derivative = np.array([theta_dot, theta_ddot])
    return state_derivative

def check_event(state, params):
    """
    Checks whether a step collision event has occured. Does this by seeing whether theta = alpha and if we are falling forward or backward

    args:
        state: Contains the state x=[theta,theta_dot]
        params: useful parameters

    returns:
        event_bool: boolean array of whether an event occured at that step
        updated_state: updated state values according to whether an event happened or not
    """
    theta = state[0]
    theta_dot = state[1]
    N_spokes = params["N_spokes"]
    alpha = np.pi / N_spokes
    event_bool = False

    if theta >= alpha and theta_dot > 0: # Forward Collision
        theta = theta - 2*alpha
        theta_dot = theta_dot * np.cos(2*alpha)
        event_bool = True
    elif theta <= -alpha and theta_dot < 0: # Backward Collision
        theta = theta + 2*alpha
        theta_dot = theta_dot * np.cos(2*alpha)
        event_bool = True

    updated_state = np.array([theta, theta_dot])

    return event_bool, updated_state

def generate_params():
    """
    Generates useful parameters
    """
    params = {
        "gravity": 9.81,  # gravity m/s^2)
        "length": 1,  # rod length (m)
        "mass": 1,  # point mass at end of rod (kg)
        "gamma": np.pi/6,
        "N_spokes": 10
    }
    return params

def calculate_energy(state, params):
    """
    Computes the kinetic and potential energy for the spokeless wheel by using the inverted pendulum model
    
    args:
        state: Contains the state x=[theta,theta_dot]
        params: useful parameters

    returns:
        kinetic_energy: Kinetic energy of the inverted pendulum calculated by T = 1/2 * mass * (length*theta_dot)^2
        potential_energy: Gravitational potential energy of the inverted pendulum calculated via V = mglcos(theta+gamma)
    """
    gravity = params["gravity"]
    length = params["length"]
    mass = params["mass"]
    gamma = params["gamma"]

    angle = state[0]  # indexes entire row "vectorized" if state is (2, N)
    angular_velocity = state[1]

    kinetic_energy = 0.5 * mass * (length * angular_velocity) ** 2
    potential_energy = mass * gravity * length * np.cos(angle+gamma)
    return kinetic_energy, potential_energy

def plot_poincare_section(x0, params, integrator, timestep=1e-5, sim_time=5.0, show=False):
    """
    Simulate a trajectory use a Poincare section at the event (in this case a collision of a spoke and the ramp). 
    This collision acts as our Poincare section, only observing events that go through it. Mathematically, P(x_k) = x_{k+1}

    This then makes a plot of trajectories through the Poincare section, titled with different gamma and N values.
    This also classifies whether the trajectory is "Forward Walking", "Rocking towards rest", or just at "Rest"

    In the case of "Forward Walking" the floquet multiplier is calculated to determine whether this is stable or unstable.

    args:
        x0: The initial state of the spokeless wheel
        params: useful parameters
        integrator: an integrator function to simulate the dynamics forward (as of right now this is only either euler or rk4)
        timestep: integration timestep, defaults to 1e-5
        sim_time: How long to integrate the dynamics for, defaults to 5.0

    returns:
        fig: Matplotlib Figure object containing the complete Poincare plot
        ax: Matplotlib Axes object containing the plotted points, lines, labels, and annotations
    """
    # Integrate Dynamics
    events, time_traj, state_traj = integrator(timestep, sim_time, x0, dynamics, params, check_event)
    event_indices = np.flatnonzero(events)
    theta_dot_k = state_traj[1, event_indices] # Poincare section trajectories x_k

    if theta_dot_k.size == 0:
        theta_dot_k = np.array([0])

    # Check whether the velocity is small at end of trajectory (rest) or positive near end (walking)
    velocity_tolerance = 0.1
    recent_impacts = theta_dot_k[-min(5, theta_dot_k.size):]
    at_rest = np.max(np.abs(recent_impacts)) < velocity_tolerance# Check whether max vel is small
    forward_walking = np.all(recent_impacts > 0)
    if at_rest:
        classification = "Rest"
    elif forward_walking:
        classification = "Forward Walking"
    else:
        classification = "Not Yet Converged"

    # Construct the Poincare points. If there is no complete return pair, show the rest reference point.
    if theta_dot_k.size >= 2:
        x = theta_dot_k[:-1]
        y = theta_dot_k[1:]
        point_label = "Poincare Points"
    else:
        x = np.array([0.0])
        y = np.array([0.0])
        point_label = "Rest Reference"

    lower = min(x.min(), y.min(), 0.0) - 1
    upper = max(x.max(), y.max(), 0.0) + 1
    identity_limits = [lower, upper]
    fig, ax = plt.subplots()
    ax.scatter(x, y, color="blue", label=point_label)
    ax.plot(identity_limits, identity_limits, "k--", linewidth=1, label="Identity Line")
    ax.axvline(y[-1], color="red", linestyle=":", linewidth=1, label="Final Velocity")
    information_text = f"Classification: {classification}"

    # Floquet multiplier is only for forward walking and a good convergence of P(x_k+1) = x_k
    if classification == "Forward Walking":
        epsilon = 1e-3 * max(abs(y[-1]), 1.0)
        multiplier = estimate_floquet_multiplier(y[-1], params, integrator, timestep=timestep, sim_time=5, epsilon=epsilon)
        stability = "Stable" if abs(multiplier) < 1 else "Unstable"
        if abs(y[-1] - x[-1]) < epsilon:
            information_text += f"\nFloquet multiplier: {multiplier:.4f}\n{stability}"
        else:
            information_text += f"\nSlope: {multiplier:.4f}\nCycle not fully converged"
    else:
        information_text += "\nFloquet multiplier: N/A"

    ax.text(0.02, 0.98, information_text, transform=ax.transAxes, ha="left", va="top", bbox={"facecolor": "white", "alpha": 0.8})
    ax.set_xlim(identity_limits)
    ax.set_ylim(identity_limits)
    ax.set_xlabel(r"$\dot{\theta}_k$")
    ax.set_ylabel(r"$\dot{\theta}_{k+1}$")
    ax.set_title(f"Gamma: {np.rad2deg(params['gamma'])}, Spokes: {params['N_spokes']}, Initial state: {x0}")
    ax.legend()
    plt.tight_layout()
    if show == True:
        plt.show()

    return fig, ax


def estimate_floquet_multiplier(theta_dot_limit_cycle, params, integrator, timestep = 1e-3, sim_time = 5, epsilon=1e-3):
    """
    Estimate the Floquet multiplier of the forward-walking limit cycle using the derivative (since we are in one dimension)

    The limit-cycle velocity is perturbed by plus and minus epsilon. Each
    perturbed state is integrated to the next impact, and the derivative of the
    return map is approximated by:
        multiplier = (P(theta_dot + epsilon) - P(theta_dot - epsilon)) / (2 * epsilon)

    args:
        theta_dot_limit_cycle: walking fixed point
        params: Model parameters
        integrator: Integration function used to simulate each perturbed trajectory
        timestep: Integration timestep
        sim_time: Maximum time allowed for each perturbed trajectory
        epsilon: Angular-velocity perturbation used for the finite difference

    returns:
        multiplier: Estimated Floquet multiplier
    """
    alpha = np.pi / params["N_spokes"]
    x0_left = np.array([-alpha,theta_dot_limit_cycle - epsilon])
    x0_right = np.array([-alpha,theta_dot_limit_cycle + epsilon])

    # Simulate trajector (THIS IS CURRENTLY A WASTE OF COMPUTE BECAUSE WE ONLY NEED THE NEXT EVENT)
    events_left, _, state_traj_left = integrator(timestep, sim_time, x0_left, dynamics, params, check_event)
    events_right, _, state_traj_right = integrator(timestep, sim_time, x0_right, dynamics, params, check_event)
    indices_left = np.flatnonzero(events_left)
    indices_right = np.flatnonzero(events_right)

    # Ignore an event at the initial state if its there
    indices_left = indices_left[indices_left > 0]
    indices_right = indices_right[indices_right > 0]

    # Post-impact theta_dot at the next collision
    theta_dot_minus_left = state_traj_left[1, indices_left[0]]
    theta_dot_minus_right = state_traj_right[1, indices_right[0]]

    multiplier = (theta_dot_minus_right - theta_dot_minus_left) / (2 * epsilon)

    return multiplier


### ROA #############################################################################################

def plot_roa(params, integrator, theta_dot_limits, n_theta=25, n_theta_dot=25, timestep=1e-3, sim_time=10.0, show=False):
    """
    Calculate and plot a grid-based region of attraction over a range of initial conditions

    Initial angles are sampled between -alpha and +alpha. For all initial conditions we classify according to:
    Classification values:
        0: Rest
        1: Not yet converged
        2: Forward walking

    The function also prints the number of completed trajectories, elapsed
    time, and estimated remaining computation time after each grid row.

    args:
        params: Model parameters, including gamma and N_spokes
        integrator: Integration function used to simulate each initial condition
        theta_dot_limits: Minimum and maximum initial angular velocities
        n_theta: Number of initial angle values between -alpha and +alpha
        n_theta_dot: Number of initial angular-velocity values
        timestep: Integration timestep for each trajectory
        sim_time: Total simulation time for each trajectory
        show: Whether to display the completed ROA plot

    returns:
        fig: Matplotlib Figure object containing the ROA plot
        ax: Matplotlib Axes object containing the classification grid
        classification_grid: Integer array containing the outcome of each initial condition
    """
    # Initial Conditions
    alpha = np.pi / params["N_spokes"]
    theta_values = np.linspace(-alpha, alpha, n_theta)
    theta_dot_values = np.linspace(theta_dot_limits[0], theta_dot_limits[1], n_theta_dot)

    # 0 = Rest, 1 = Not yet converged, 2 = Forward walking
    classification_grid = np.zeros((n_theta_dot, n_theta), dtype=int)
    velocity_tolerance = 0.1
    total_trajectories = n_theta * n_theta_dot
    start_time = time.perf_counter()
    for i, theta_dot_0 in enumerate(theta_dot_values):
        for j, theta_0 in enumerate(theta_values):
            x0 = np.array([theta_0, theta_dot_0])
            events, time_traj, state_traj = integrator(timestep, sim_time, x0, dynamics, params, check_event)
            event_indices = np.flatnonzero(events)
            theta_dot_k = state_traj[1, event_indices]

            if theta_dot_k.size == 0:
                theta_dot_k = np.array([0])

            # Check whether the velocity is small at end of trajectory (rest) and positive near end (walking)
            velocity_tolerance = 0.1
            recent_impacts = theta_dot_k[-min(5, theta_dot_k.size):]
            at_rest = np.max(np.abs(recent_impacts)) < velocity_tolerance# Check whether max vel is small
            forward_walking = np.all(recent_impacts > 0)
            if at_rest:
                classification_grid[i, j] = 0
            elif forward_walking:
                classification_grid[i, j] = 2
            else:
                classification_grid[i, j] = 1

        # Timing
        completed = (i + 1) * n_theta
        remaining = total_trajectories - completed
        elapsed_time = time.perf_counter() - start_time
        average_time = elapsed_time / completed
        estimated_remaining_time = average_time * remaining
        percent_complete = 100 * completed / total_trajectories

        print(f"Completed {completed}/{total_trajectories} trajectories ({percent_complete:.1f}%) | Remaining: {remaining} | Elapsed: {elapsed_time / 60:.1f} min | ETA: {estimated_remaining_time / 60:.1f} min", flush=True)

    total_time = time.perf_counter() - start_time
    print(f"ROA calculation complete in {total_time / 60:.1f} minutes.")

    # Plotting 
    colors = ["lightgray", "orange", "green"]
    labels = ["Rest", "Not Yet Converged", "Forward walking"]
    colormap = ListedColormap(colors)

    fig, ax = plt.subplots(figsize=(9, 7))

    image = ax.imshow(classification_grid, origin="lower", extent=[-alpha, alpha, theta_dot_limits[0], theta_dot_limits[1]], aspect="auto", interpolation="nearest", cmap=colormap, vmin=-0.5, vmax=2.5)

    colorbar = fig.colorbar(image, ax=ax, ticks=[0, 1, 2])
    colorbar.ax.set_yticklabels(labels)

    ax.set_xlabel(r"Initial $\theta$ (rad)")
    ax.set_ylabel(r"Initial $\dot{\theta}$ (rad/s)")
    ax.set_title(f"Region of Attraction: Gamma = {np.rad2deg(params['gamma'])}, Spokes = {params['N_spokes']}")

    plt.tight_layout()

    if show == True:
        plt.show()

    return fig, ax, classification_grid


def plot_energy_roa(params, theta_dot_limits, n_theta=100, n_theta_dot=100, show=False, max_collisions=100):
    """
    Calculate and plot an energy-based approximation of the region of attraction 

    Initial angles are sampled between -alpha and +alpha. For all initial conditions we use total energy to classify according to:
    Classification values:
        0: Rest or no sustained forward walking
        1: Forward walking

    One drawback of this model is that it classifies trajectories off its initial state and therefore cannot see the case
    where you start going backwards and then rock back to going forwards, and eventually walking

    args:
        params: Model parameters
        theta_dot_limits: Minimum and maximum initial angular velocities
        n_theta: Number of initial angle values between -alpha and +alpha
        n_theta_dot: Number of initial angular-velocity values
        show: Whether to display the completed energy-based ROA plot
        max_collisions: maximum number of collisions

    returns:
        fig: Matplotlib Figure object containing the energy-based ROA plot
        ax: Matplotlib Axes object containing the classification grid
        theta_values: Array of initial angle values used for the grid
        theta_dot_values: Array of initial angular-velocity values used for the grid
        classification_grid: Integer array containing the predicted outcome of each initial condition
    """
    gravity = params["gravity"]
    length = params["length"]
    gamma = params["gamma"]
    alpha = np.pi / params["N_spokes"]

    theta_values = np.linspace(-alpha, alpha, n_theta)
    theta_dot_values = np.linspace(theta_dot_limits[0], theta_dot_limits[1], n_theta_dot)

    # 0 = Rest / no walking, 1 = Forward walking
    classification_grid = np.zeros((n_theta_dot, n_theta), dtype=int)

    collision_factor = np.cos(2 * alpha)
    energy_tolerance = 1e-10 * max(gravity * length, 1.0)

    def potential_energy_per_mass(theta):
        return gravity * length * np.cos(theta + gamma)

    def maximum_potential_between(theta_start, theta_end):
        lower = min(theta_start, theta_end)
        upper = max(theta_start, theta_end)
        potential_values = [potential_energy_per_mass(lower), potential_energy_per_mass(upper)]

        # theta = -gamma is the top of the potential-energy barrier
        if lower <= -gamma <= upper:
            potential_values.append(gravity * length)

        return max(potential_values)

    potential_left = potential_energy_per_mass(-alpha)
    potential_right = potential_energy_per_mass(alpha)
    full_step_barrier = maximum_potential_between(-alpha, alpha)

    # Check whether a nonzero forward walking fixed point exists
    energy_gain_speed_squared = 2 * gravity / length * (np.cos(gamma - alpha) - np.cos(gamma + alpha))
    minimum_step_speed_squared = max(0.0, 2 * (full_step_barrier - potential_left) / length**2)

    if 0 < collision_factor < 1 and energy_gain_speed_squared > 0:
        limit_cycle_speed_squared = collision_factor**2 * energy_gain_speed_squared / (1 - collision_factor**2)
        walking_cycle_exists = limit_cycle_speed_squared > minimum_step_speed_squared
    else:
        limit_cycle_speed_squared = 0.0
        walking_cycle_exists = False

    angle_tolerance = 1e-12

    def next_collision_direction(theta, theta_dot, energy):
        """Return +1 for forward, -1 for backward, or 0 for no return."""
        # These handle initial states placed directly on an outward-moving
        # collision boundary.
        if theta >= alpha - angle_tolerance and theta_dot > 0:
            return 1
        if theta <= -alpha + angle_tolerance and theta_dot < 0:
            return -1

        if theta_dot > 0:
            forward_barrier = maximum_potential_between(theta, alpha)
            margin = energy - forward_barrier
            if margin > energy_tolerance:
                return 1
            if abs(margin) <= energy_tolerance:
                return 0

            # It cannot cross the upright barrier, so it turns backward.
            return -1

        if theta_dot < 0:
            backward_barrier = maximum_potential_between(-alpha, theta)
            margin = energy - backward_barrier
            if margin > energy_tolerance:
                return -1
            if abs(margin) <= energy_tolerance:
                return 0

            # It cannot cross the upright barrier, so it turns forward.
            return 1

        acceleration = gravity / length * np.sin(theta + gamma)
        if acceleration > 0:
            return 1
        if acceleration < 0:
            return -1
        return 0

    # On a sufficiently steep slope, potential decreases immediately after a
    # forward reset; otherwise the wheel must vault the potential barrier.
    no_forward_barrier = (
        full_step_barrier - potential_left <= energy_tolerance
        and np.sin(gamma - alpha) > 0
    )

    for i, theta_dot_0 in enumerate(theta_dot_values):
        for j, theta_0 in enumerate(theta_values):
            theta = theta_0
            theta_dot = theta_dot_0

            # Follow the exact energy map through both forward and backward
            # impacts. This avoids treating an initial backward step as rest.
            for _ in range(max_collisions):
                energy = (
                    0.5 * length**2 * theta_dot**2
                    + potential_energy_per_mass(theta)
                )
                direction = next_collision_direction(theta, theta_dot, energy)

                # The separatrix approaches the upright equilibrium and never
                # reaches another collision.
                if direction == 0:
                    break

                impact_potential = (
                    potential_right if direction > 0 else potential_left
                )
                impact_speed_squared = max(
                    0.0,
                    2 * (energy - impact_potential) / length**2
                )
                pre_impact_speed = direction * np.sqrt(impact_speed_squared)
                post_impact_speed = collision_factor * pre_impact_speed

                if direction > 0:
                    # The new stance begins at theta = -alpha.
                    theta = -alpha
                    theta_dot = post_impact_speed
                    post_impact_energy = (
                        0.5 * length**2 * theta_dot**2 + potential_left
                    )

                    if no_forward_barrier:
                        enough_energy_for_next_step = True
                    else:
                        enough_energy_for_next_step = (
                            post_impact_energy
                            > full_step_barrier + energy_tolerance
                        )

                    if walking_cycle_exists and enough_energy_for_next_step:
                        classification_grid[i, j] = 1

                    # If this forward post-impact state cannot clear the next
                    # barrier, subsequent rocking only loses more energy.
                    break

                # A backward impact starts the next stance at theta = +alpha.
                # Continue iterating: it may take more backward steps, turn
                # around, and eventually enter the forward-walking basin.
                theta = alpha
                theta_dot = post_impact_speed

    colors = ["lightgray", "green"]
    labels = ["Rest", "Forward walking"]
    colormap = ListedColormap(colors)

    theta_grid, theta_dot_grid = np.meshgrid(theta_values, theta_dot_values)

    fig, ax = plt.subplots(figsize=(9, 7))

    image = ax.pcolormesh(theta_grid, theta_dot_grid, classification_grid, shading="nearest", cmap=colormap, vmin=-0.5, vmax=1.5)

    colorbar = fig.colorbar(image, ax=ax, ticks=[0, 1])
    colorbar.ax.set_yticklabels(labels)

    ax.set_xlabel(r"Initial $\theta$ (rad)")
    ax.set_ylabel(r"Initial $\dot{\theta}$ (rad/s)")
    ax.set_title(f"Energy-Based ROA: Gamma = {np.rad2deg(gamma)}, Spokes = {params['N_spokes']}")

    plt.tight_layout()

    if show == True:
        plt.show()

    return fig, ax, theta_values, theta_dot_values, classification_grid

#########################


def animate_pendulum(x, y, gamma, max_frames=500):
    """
    Animate the motion of a the spokeless wheel
    args:
        x: Array containing the horizontal position of the pendulum mass
        y: Array containing the vertical position of the pendulum mass
        gamma: Downward ground-slope angle in radians
        max_frames: Maximum number of trajectory frames included in the animation
    returns:
        animation: Matplotlib FuncAnimation object containing the pendulum animation
    """
    fig, ax = plt.subplots()

    limit = 1.1 * max(np.max(np.abs(x)), np.max(np.abs(y)))
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_aspect("equal")
    ax.grid()

    # Ground slopes downward to the right
    ground_x = np.array([-limit, limit])
    ground_y = -np.tan(gamma) * ground_x
    ax.plot(ground_x, ground_y, color="brown", linewidth=3)

    rod, = ax.plot([], [], "k-", linewidth=2)
    mass, = ax.plot([], [], "ro", markersize=10)

    frame_indices = np.linspace(
        0,
        len(x) - 1,
        min(max_frames, len(x)),
        dtype=int
    )

    def update(index):
        rod.set_data([0, x[index]], [0, y[index]])
        mass.set_data([x[index]], [y[index]])
        return rod, mass

    animation = FuncAnimation(
        fig,
        update,
        frames=frame_indices,
        interval=20,
        repeat=False,
        blit=False
    )

    plt.show()
    return animation
