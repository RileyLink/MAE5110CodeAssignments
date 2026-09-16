import numpy as np
import matplotlib.pyplot as plt
import timeit

from models import pendulum as model
# from models import bouncing_ball as model
from integrators import explicit_euler as euler
from integrators import rk4 as rk4

# Basic simulation of the pendulum

params = {
    "gravity": 9.81,  # gravity m/s^2)
    "length": 1,  # rod length (m)
    "mass": 0.2,  # point mass at end of rod (kg)
    "damping_coeff": 0.0,  # damping coefficient (kg*m^2/s)
}


# some set-up
N = 1000
initial_state = np.array([np.pi / 4, 0.0])
kinetic_energy = np.zeros((2,N))
potential_energy = np.zeros((2,N))
dt_values = np.linspace(1e-5, 1, N)
dt_largest = []

sim_time = 5
x0 = [np.pi / 4, 0]
for k in range(2):
    i=0
    for timestep in dt_values:
        if k==0:
            name = "Euler"
            time_traj, state_traj = euler(timestep, sim_time, x0, model.dynamics, params, model.check_event)
        else:
            name = "rk4"
            time_traj, state_traj = rk4(timestep, sim_time, x0, model.dynamics, params, model.check_event)

        # Calculate first and last potential energies for each time step
        kinetic_energy1, potential_energy1 = model.calculate_energy(state_traj, params)
        kinetic_energy[0,i] = kinetic_energy1[0]
        potential_energy[0,i] = potential_energy1[0]
        kinetic_energy[1,i] = kinetic_energy1[-1]
        potential_energy[1,i] = potential_energy1[-1]
        i+=1

    # Calculate relative error between starting energy and final energy to see if its conserved
    total_energy = potential_energy + kinetic_energy
    energy_error = total_energy[1, :] - total_energy[0, :]
    relative_error = np.abs(
        (total_energy[1, :] - total_energy[0, :]) / total_energy[0, :]
    )

    # If energy error is larger than 1% then we say energy is not conserved and the integration is broken
    exceeds = relative_error >= 0.01
    if np.any(exceeds):
        first_bad_idx = np.argmax(exceeds)
        first_too_large = dt_values[first_bad_idx]

        # print(f"{name} first timestep that exceeds 1% error: {first_too_large:.2e}")

        if first_bad_idx > 0:
            largest_acceptable_dt = dt_values[first_bad_idx - 1]
            print(f"{name} largest timestep before exceeding 1% error tolerance: {largest_acceptable_dt:.2e}")
            dt_largest.append(largest_acceptable_dt)
        else:
            dt_largest.append(1e-5)
    else:
        print(f"{name}: None of the tested timesteps exceeded the 1% error tolerance.")
        largest_acceptable_dt = dt_values[-1]
        dt_largest.append(largest_acceptable_dt)

# plt.figure()
# plt.plot(dt_values, relative_error*100, label = "Energy Error")
# plt.xlabel("Timestep")
# plt.ylabel("Energy Relative Error Percentage")
# plt.legend()
# plt.show()

def time_integrator(integrator, timestep, n=1):
    return timeit.timeit(
        lambda: integrator(timestep, sim_time, x0, model.dynamics, params, model.check_event),
        number=n) / n

# Use SAME timestep
timestep_same = 1e-5
runtime_euler = time_integrator(euler, timestep_same)
runtime_rk4 = time_integrator(rk4, timestep_same)

print(f"Average Euler runtime (dt={timestep_same:.2e}): {runtime_euler:.4f} s")
print(f"Average RK4 runtime (dt={timestep_same:.2e}): {runtime_rk4:.4f} s")

# Use largest acceptable timestep for each integrator
runtime_euler = time_integrator(euler, dt_largest[0])
runtime_rk4 = time_integrator(rk4, dt_largest[1])

print(f"Average Euler runtime (dt={dt_largest[0]:.2e}): {runtime_euler:.4f} s")
print(f"Average RK4 runtime (dt={dt_largest[1]:.2e}): {runtime_rk4:.4f} s")


time_traj, state_traj = rk4(dt_largest[1], sim_time, x0, model.dynamics, params, model.check_event)
kinetic_energy, potential_energy = model.calculate_energy(state_traj, params)

plt.figure()
plt.plot(time_traj, potential_energy, label="Potential energy")
plt.plot(time_traj, kinetic_energy, label="Kinetic energy")
plt.plot(time_traj, potential_energy + kinetic_energy, label="Total energy")
plt.xlabel("Time (s)")
plt.ylabel("Energy (J)")
plt.title("Energy over time")
plt.legend()
plt.tight_layout()
plt.show()

# TODO: make a phase portrait plot
dynamics_traj = np.zeros_like(state_traj)

for i, t in enumerate(time_traj):
    dynamics_traj[:, i] = model.dynamics(t, state_traj[:, i], params)

plt.figure()
plt.plot(state_traj[0, :], dynamics_traj[0, :],label="State 1 Phase Portrait")
plt.plot(state_traj[1, :], dynamics_traj[1, :], label="State 2 Phase Portrait")
plt.xlabel("State")
plt.ylabel("Dynamics")
plt.legend()
plt.title("Phase Portrait")
plt.tight_layout()
plt.show()

# plt.figure()
# plt.plot(state_traj[0,:], model.dynamics(
#         time_traj[0], state_traj[:,:], params)[0,:], label = "Angle Phase Portrait")
# plt.plot(state_traj[1,:], model.dynamics(
#         time_traj[0], state_traj[:,:], params)[1,:], label = "Velocity Phase Portrait")
# plt.xlabel("Pendulum State x")
# plt.ylabel("Pendulum Dynamics f(x)")
# plt.legend()
# plt.tight_layout()
# plt.show()
