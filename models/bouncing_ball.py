import numpy as np
def check_event(state):
    height = state[0]
    vel = state[1]
    event_bool = False
    if height <= 0 and vel < 0: 
        height = 0
        vel = -state[1]
        event_bool = True

    return event_bool, np.array([height, vel])

def dynamics(t, state, params):
    gravity = params["gravity"]
    height = state[0]
    vel = state[1]
    return np.array([vel, -gravity])


def generate_params():
    params = {
        "gravity": 9.81,  # gravity m/s^2)
        "mass": 1,  # point mass at end of rod (kg)
    }
    return params


def calculate_energy(state, params):
    """Compute energies for a state ``(2,)`` or trajectory ``(2, N)``."""
    gravity = params["gravity"]
    mass = params["mass"]

    height = state[0]  # indexes entire row "vectorized" if state is (2, N)
    velocity = state[1]

    kinetic_energy = 0.5 * mass * velocity ** 2
    potential_energy = mass * gravity * height
    return kinetic_energy, potential_energy
