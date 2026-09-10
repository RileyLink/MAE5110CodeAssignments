import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
import timeit
import sys
from models import spokeless_wheel as model
from integrators import explicit_euler as euler
from integrators import rk4 as rk4

################################# SANITY CHECK FUNCTIONS AND MAIN IS FIRST #############################

# Sanity Check initial values
output_folder = Path("assignment_1_figures")
output_folder.mkdir(parents=True, exist_ok=True)
params = model.generate_params()
gamma = np.deg2rad(0)
N_spokes = 9
params["gamma"] = gamma
params["N_spokes"] = N_spokes
length = params["length"]
alpha = np.pi/params["N_spokes"]
x0 = np.array([0,3])
events, time_traj, state_traj = rk4(1e-3, 10, x0, model.dynamics, params, model.check_event)

# Sanity Check 1. Energy Plots ##########################################################################
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
filename_energy = (f"Energy_gamma_{np.rad2deg(gamma):.1f}_spokes_{N_spokes:02d}.png")
#plt.savefig(output_folder / filename_energy, dpi=300, bbox_inches="tight")
plt.show()

# Sanity Check 2. Animations ##########################################################################
theta = state_traj[0]
time = np.linspace(0, 2, len(theta))
x = length * np.sin(gamma + theta)
y = length * np.cos(gamma + theta)
animation = model.animate_pendulum(x, y, params["gamma"])

# Sanity Check 3. Poincare Section Plot ##########################################################################
fig, ax = model.plot_poincare_section(x0, params, rk4, timestep=1e-3, sim_time=20,show = True)
filename_poincare = (f"poincareTEST_gamma_{np.rad2deg(gamma):.1f}_spokes_{N_spokes:02d}.png")
fig.savefig(output_folder / filename_poincare, dpi=300, bbox_inches="tight")

# Sanity Check 4. Phase Portrait ##########################################################################
dynamics_traj = np.zeros_like(state_traj)
for i, t in enumerate(time_traj):
    dynamics_traj[:, i] = model.dynamics(t, state_traj[:, i], params)
plt.figure()
plt.plot(state_traj[0, :], dynamics_traj[0, :],label="Theta Phase Portrait")
plt.plot(state_traj[1, :], dynamics_traj[1, :], label="Theta Dot Phase Portrait")
plt.xlabel("State")
plt.ylabel("Dynamics")
plt.legend()
plt.title("Phase Portrait")
plt.tight_layout()
filename_phase = (f"Phase_portrait_gamma_{np.rad2deg(gamma):.1f}_spokes_{N_spokes:02d}.png")
#plt.savefig(output_folder / filename_phase, dpi=300, bbox_inches="tight")
plt.show()


########################## GAMMA AND N LOOPS FOR ROA, POINCARE, MAIN STUFF HERE #####################################

# Initial Values
params = model.generate_params()
N_spokes = params["N_spokes"]
alpha = np.pi / N_spokes
gamma = params["gamma"]

gamma_values_degrees = np.array([0,5,10,20,30])
gamma_values = np.deg2rad(gamma_values_degrees)
N_values = np.array([6, 9, 10, 12])
x0 = np.array([0,0])

# Gamma and N Loop
for gamma in gamma_values:
    for N_spokes in N_values:
        params["gamma"] = gamma
        params["N_spokes"] = N_spokes

        filename1 = (f"poincare_gamma_{np.rad2deg(gamma):.1f}_spokes_{N_spokes:02d}.png")
        filename2 = (f"ROA_gamma_{np.rad2deg(gamma):.1f}_spokes_{N_spokes:02d}.png")
        filename3 = (f"ROA_ENERGY_gamma_{np.rad2deg(gamma):.1f}_spokes_{N_spokes:02d}.png")

        fig3, ax3, theta_values, theta_dot_values, energy_roa = model.plot_energy_roa(params, theta_dot_limits=(-3, 3), n_theta=200, n_theta_dot=200, show=False)
        fig3.savefig(output_folder / filename3, dpi=300, bbox_inches="tight")
        plt.close(fig3)

        fig1, ax1 = model.plot_poincare_section(x0, params, rk4, timestep=1e-3, sim_time=15,show = False)
        fig1.savefig(output_folder / filename1, dpi=300, bbox_inches="tight")
        plt.close(fig1)

        fig2, ax2, roa = model.plot_roa(params, rk4, theta_dot_limits=(-3, 3), n_theta=21, n_theta_dot=21, timestep=1e-3, sim_time=15, show=False)
        fig2.savefig(output_folder / filename2, dpi=300, bbox_inches="tight")
        plt.close(fig2)

################# OLD CODE BELOW ############################################

# n_gamma = len(gamma_values)
# n_N = len(N_values)

# fig, axes = plt.subplots(n_gamma, n_N, figsize=(18, 4 * n_gamma), squeeze=False)

# for row, gamma in enumerate(gamma_values):
#     for col, N_spokes in enumerate(N_values):
#         params = model.generate_params()
#         params["gamma"] = gamma
#         params["N_spokes"] = N_spokes

#         initial_state = np.array([-np.pi / params["N_spokes"], 0.0])

#         event_indices, theta_dot_k = model.collect_poincare_pairs(initial_state, params, rk4, timestep=1e-3, sim_time=3.0)

#         if theta_dot_k.size < 2:
#             x = np.array([0.0])
#             y = np.array([0.0])
#         else:
#             x = theta_dot_k[:-1]
#             y = theta_dot_k[1:]

#         ax = axes[row, col]

#         ax.scatter(x, y, label="Poincaré pairs")
#         ax.axline((0, 0), slope=1, color="black", linestyle="--", label="y = x")
#         ax.axvline(x[-1], color="red", linestyle=":", linewidth=1.5, label="Final x")

#         ax.set_xlabel(r"$\dot{\theta}_k$")
#         ax.set_ylabel(r"$P(\dot{\theta}_k)$")
#         ax.set_title(fr"$\gamma={gamma}$, $N={N_spokes}$")
#         ax.grid(True, alpha=0.3)
#         ax.legend()

# plt.tight_layout()
# plt.show()




# ###############

# #### GEMINI: 
# gamma_values = np.array([0, 5, 10, 15, 20])
# N_values = np.array([6, 8, 9, 10, 12])
# n_gamma = len(gamma_values)

# # Create figure with subplots: 2 metrics x 3 x-axes = 6 subplots per mu value
# fig = plt.figure(figsize=(18, 4*n_gamma))

# for idx, gamma in enumerate(gamma_values):
#     params = model.generate_params()
#     params["gamma"] = gamma
#     # Row for this mu value
#     base_idx = idx * 5

#     # --- First N Values
#     params["N_spokes"] = N_values[0]
#     initial_state = np.array([-np.pi/params["N_spokes"], 0.0])
#     event_indices, theta_dot_k = model.collect_poincare_pairs(initial_state, params, rk4, timestep=1e-3, sim_time=3.0)
#     if theta_dot_k.size < 2:
#         x = np.array([0.0])
#         y = np.array([0.0])
#     else:
#         x = theta_dot_k[:-1]
#         y = theta_dot_k[1:]

#     ax1 = plt.subplot(n_gamma, 2, base_idx + 1)
#     ax1.scatter(x, y)
#     ax1.axline((0, 0), slope=1, color="black", linestyle="--")
#     ax1.axvline(y[-1],color="red",linestyle=":",linewidth=1.5)
#     ax1.set_xlabel('Theta Dot K')
#     ax1.set_ylabel('P(Theta Dot K)')
#     ax1.set_title(f'{gamma}, {N_spokes}')
#     ax1.grid(True, alpha=0.3)
#     ax1.legend()

#     # --- First N Values
#     params["N_spokes"] = N_values[1]
#     initial_state = np.array([-np.pi/params["N_spokes"], 0.0])
#     event_indices, theta_dot_k = model.collect_poincare_pairs(initial_state, params, rk4, timestep=1e-3, sim_time=3.0)
#     if theta_dot_k.size < 2:
#         x = np.array([0.0])
#         y = np.array([0.0])
#     else:
#         x = theta_dot_k[:-1]
#         y = theta_dot_k[1:]

#     ax2 = plt.subplot(n_gamma, 6, base_idx + 1)
#     ax2.scatter(x, y)
#     ax2.plot("k--", linewidth=1)
#     ax2.axline((0, 0), slope=1, color="black", linestyle="--")
#     ax2.axvline(y[-1],color="red",linestyle=":",linewidth=1.5)
#     ax2.set_xlabel('Theta Dot K')
#     ax2.set_ylabel('P(Theta Dot K)')
#     ax2.set_title(f'{gamma}, {N_spokes}')
#     ax2.grid(True, alpha=0.3)
#     ax2.legend()

# plt.tight_layout()

# ##############
# gamma_values = np.array([0,5,10,15,20])
# N_values = np.array([6, 8, 9, 10, 12])

# fig, axes = plt.subplots(5, 5, figsize=(18, 18))

# for row, gamma in enumerate(gamma_values):
#     for col, N in enumerate(N_values):
#         params = model.generate_params()
#         params["gamma"] = gamma
#         params["N_spokes"] = N
#         alpha = np.pi / N
#         params["alpha"] = alpha
#         initial_state = np.array([-alpha, 0.0])
#         event_indices, theta_dot_k = model.collect_poincare_pairs(initial_state, params, rk4, timestep=1e-3, sim_time=3.0)

#         model.plot_poincare_map(theta_dot_k, ax=axes[row, col], show=False)

#         ax = axes[row, col]

#         ax.set_title(rf"$\gamma={gamma:.3f}$, $N={N}$", fontsize=7, pad=2)
#         ax.set_xticks([])
#         ax.set_yticks([])
#         ax.grid(False)
#         ax.set_xlabel(r"$\dot{\theta}_k$",fontsize=7)
#         ax.set_ylabel(r"$\dot{\theta}_{k+1}$",fontsize=7,)
        
# fig.suptitle("Poincaré Return Maps", fontsize=12)
# plt.tight_layout()
# plt.show()

# fig.savefig(
#     "poincare_parameter_sweep.png",
#     dpi=300,
#     bbox_inches="tight",
# )


# ############


# initial_state = np.array([-alpha, 0])

# theta_dot_k = model.collect_poincare_pairs(initial_state, params, rk4, timestep=1e-3, sim_time=5.0)

# model.plot_poincare_map(theta_dot_k)

# multiplier = model.estimate_floquet_multiplier(theta_dot_k[-1], params, rk4)

# print("Floquet multiplier:", multiplier)

# print("STOP HERE")


# #TEST
# slopes = np.deg2rad([10])
# spoke_counts = [10]

# model.plot_roa_sweep(slopes, spoke_counts, model, euler)


# # Basic simulation of the pendulum
# slopes = np.deg2rad([0, 5, 10, 20, 30])
# spoke_counts = [6, 8, 10, 12]
# model.plot_roa_sweep(slopes, spoke_counts, model, euler)

# params = model.generate_params()

# sim_time = 3
# timestep = 1e-5
# N_spokes = params["N_spokes"]
# two_alpha = 2*np.pi / N_spokes
# x0 = [two_alpha/2-0.01, 0] # starts right near impact

# theta_grid = np.linspace(-two_alpha/2,two_alpha/2,10)
# theta_dot_grid = np.linspace(-1,1,10)

# for theta_0 in theta_grid:
#     for theta_dot_0 in theta_dot_grid:
#         x0 = [theta_0,theta_dot_0]
#         time_traj, state_traj = euler(timestep, sim_time, x0, model.dynamics, params, model.check_event)








# # PLots
# time_traj, state_traj = euler(timestep, sim_time, x0, model.dynamics, params, model.check_event)
# length = params["length"]
# gamma = params["gamma"]

# theta = state_traj[0]
# time = np.linspace(0, sim_time, len(theta))
# x = length * np.sin(gamma + theta)
# y = length * np.cos(gamma + theta)

# plt.figure()
# plt.plot(time, x, label="x position")
# plt.plot(time, y, label="y position")
# plt.xlabel("Time (s)")
# plt.ylabel("Position (m)")
# plt.title("Wheel Hub Position")
# plt.legend()
# plt.grid()
# plt.tight_layout()
# plt.show()
















# # some set-up
# N = 1000
# initial_state = np.array([np.pi / 4, 0.0])
# kinetic_energy = np.zeros((2,N))
# potential_energy = np.zeros((2,N))
# dt_values = np.linspace(1e-5, 1, N)
# dt_largest = []

# sim_time = 5
# x0 = [np.pi / 4, 0]
# for k in range(2):
#     i=0
#     for timestep in dt_values:
#         if k==0:
#             name = "Euler"
#             time_traj, state_traj = euler(timestep, sim_time, x0, model.dynamics, params, model.check_event)
#         else:
#             name = "rk4"
#             time_traj, state_traj = rk4(timestep, sim_time, x0, model.dynamics, params, model.check_event)

#         # Calculate first and last potential energies for each time step
#         kinetic_energy1, potential_energy1 = model.calculate_energy(state_traj, params)
#         kinetic_energy[0,i] = kinetic_energy1[0]
#         potential_energy[0,i] = potential_energy1[0]
#         kinetic_energy[1,i] = kinetic_energy1[-1]
#         potential_energy[1,i] = potential_energy1[-1]
#         i+=1

#     # Calculate relative error between starting energy and final energy to see if its conserved
#     total_energy = potential_energy + kinetic_energy
#     energy_error = total_energy[1, :] - total_energy[0, :]
#     relative_error = np.abs(
#         (total_energy[1, :] - total_energy[0, :]) / total_energy[0, :]
#     )

#     # If energy error is larger than 1% then we say energy is not conserved and the integration is broken
#     exceeds = relative_error >= 0.01
#     if np.any(exceeds):
#         first_bad_idx = np.argmax(exceeds)
#         first_too_large = dt_values[first_bad_idx]

#         # print(f"{name} first timestep that exceeds 1% error: {first_too_large:.2e}")

#         if first_bad_idx > 0:
#             largest_acceptable_dt = dt_values[first_bad_idx - 1]
#             print(f"{name} largest timestep before exceeding 1% error tolerance: {largest_acceptable_dt:.2e}")
#             dt_largest.append(largest_acceptable_dt)
#         else:
#             dt_largest.append(1e-5)
#     else:
#         print(f"{name}: None of the tested timesteps exceeded the 1% error tolerance.")
#         largest_acceptable_dt = dt_values[-1]
#         dt_largest.append(largest_acceptable_dt)

# # plt.figure()
# # plt.plot(dt_values, relative_error*100, label = "Energy Error")
# # plt.xlabel("Timestep")
# # plt.ylabel("Energy Relative Error Percentage")
# # plt.legend()
# # plt.show()

# def time_integrator(integrator, timestep, n=1):
#     return timeit.timeit(
#         lambda: integrator(timestep, sim_time, x0, model.dynamics, params, model.check_event),
#         number=n) / n

# # Use SAME timestep
# timestep_same = 1e-5
# runtime_euler = time_integrator(euler, timestep_same)
# runtime_rk4 = time_integrator(rk4, timestep_same)

# print(f"Average Euler runtime (dt={timestep_same:.2e}): {runtime_euler:.4f} s")
# print(f"Average RK4 runtime (dt={timestep_same:.2e}): {runtime_rk4:.4f} s")

# # Use largest acceptable timestep for each integrator
# runtime_euler = time_integrator(euler, dt_largest[0])
# runtime_rk4 = time_integrator(rk4, dt_largest[1])

# print(f"Average Euler runtime (dt={dt_largest[0]:.2e}): {runtime_euler:.4f} s")
# print(f"Average RK4 runtime (dt={dt_largest[1]:.2e}): {runtime_rk4:.4f} s")
