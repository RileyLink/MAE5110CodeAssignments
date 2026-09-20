from pathlib import Path

import sys
import time, random
import matplotlib.pyplot as plt
import numpy as np
from integrators import rk4 as integrator
from integrators.rk4 import _rk4_step
from matplotlib.animation import FuncAnimation, PillowWriter
from models import inverted_pendulum_walker as model


# CALCULATE CONTROLLER ROA IF A DATA FILE IS NOT ALREADY SAVED ###########################################################
roa_path = Path("output/assignment_2/roa.npz")
if roa_path.exists():
    print("Extracting ROA data...")
    roa_data = np.load(roa_path, allow_pickle=False)
    classification_grid = roa_data["classification_grid"]
    theta_values = roa_data["theta_values"]
    theta_dot_values = roa_data["theta_dot_values"]
else:
    print("No ROA data found, generating now...")
    params = model.generate_params()
    params["K_p"] = 10
    params["K_d"] = 10
    # Calculate ROA
    fig,ax,classification_grid, theta_values, theta_dot_values = model.plot_controller_roa(
        (params["incline"]-params["angle_of_attack"], params["incline"]+params["angle_of_attack"]),
        theta_dot_limits=(-1.5,1.5),
        n_theta=250,n_theta_dot=250, 
        params = params, 
        sim_time = 20, 
        timestep = 5e-4, 
        show=True)
    np.savez(roa_path,classification_grid=classification_grid,theta_values=theta_values,theta_dot_values=theta_dot_values)
    fig.savefig("output/assignment_2/controller_roa.png",dpi=300,bbox_inches="tight")

# Run a specific trial #######################################################################################################
params = model.generate_params()
timestep = 1e-3
sim_time = 5
sim_steps = int(sim_time / timestep)
state_traj = np.zeros((sim_steps+1,2))
x0 = [0,3]
state_traj[0,:] = x0
current_state = x0.copy()
time_traj = np.linspace(0,sim_time,sim_steps+1)
completed_steps = 0
print(f"Running simulation starting at {x0}")
first_roa = True
for step in range(sim_steps):
    if model.state_is_in_roa(current_state, theta_values, theta_dot_values, classification_grid):
        if first_roa:
            print(f"Reached the ROA:\n Step: {step}\n State: {current_state}")
            first_roa = False
            ROA_state = current_state
        params["ankle_torque"] = model.calculate_torque(current_state,params)

    current_time = step * timestep
    next_state = _rk4_step(current_time,current_state,timestep,model.dynamics,params)

    if model.event_guard(current_state, next_state, params): # Collision occurs
        current_state = model.event_dynamics(next_state, params)
        state_traj[step+1, :] = current_state

        # Update Alpha
        completed_steps += 1
        new_angle_of_attack = np.pi/8
        #new_angle_of_attack = random.uniform(np.pi/8,np.pi/7)
        if new_angle_of_attack < np.pi/8 or new_angle_of_attack > np.pi/7:
            print("Angle of attack exceeded limits!")
            sys.exit()
        params["angle_of_attack"] = new_angle_of_attack
    else:
        current_state = next_state
        state_traj[step+1, :] = current_state

print("Simulation over, animating now...")
model.animate(state_traj.T,time_traj,params,timestep,completed_steps)
model.plot_phase_portrait(classification_grid, theta_values, theta_dot_values, state_traj, show=False)
plt.show()
plt.close()

# CREATE LOOKUP TABLE #######################################################################################################
base_params = model.generate_params()
gravity = base_params["gravity"]
length = base_params["length"]

n_theta_dot = 50
n_control_inputs = 3

control_inputs = np.linspace(np.pi / 8, np.pi / 7, n_control_inputs)
theta_dot_values_poincare = np.linspace(0, np.sqrt(2 * gravity / length), n_theta_dot)

lookup_table = np.full((n_theta_dot, n_control_inputs), np.nan)
reaches_roa = np.zeros((n_theta_dot, n_control_inputs), dtype=bool)
failed_transition = np.zeros((n_theta_dot, n_control_inputs),dtype=bool)

for velocity_index, theta_dot_0 in enumerate(theta_dot_values_poincare):
    for control_index, angle_of_attack in enumerate(control_inputs):
        next_velocity, reached_roa, failed = model.simulate_poincare_step(
            theta_dot_0,
            angle_of_attack,
            base_params,
            roa_theta_values=theta_values,
            roa_theta_dot_values=theta_dot_values,
            classification_grid=classification_grid,
            timestep=1e-3,
            sim_time=5.0,
        )

        lookup_table[velocity_index, control_index] = next_velocity
        reaches_roa[velocity_index, control_index] = reached_roa
        failed_transition[velocity_index, control_index] = failed

print("Next-velocity lookup table:")
print(lookup_table)
print("Lookup table legend: \n -100 Indicates ROA is reached \n nan indicates the poincare section was not hit again \n All other numbers indicate the next velocity")

# print(f"Reaches ROA: {reaches_roa}")

# print(f"Failed {failed_transition}")

sys.exit()



#########################################################################################################
# ALL CODE BELOW IS WORK IN PROGRESS

#
#
#
#
#
#
#
#
#
#
#
#

def control_to_label(control, control_inputs):
    return int(np.argmin(np.abs(control_inputs - control)))


def velocity_to_label(theta_dot, theta_dot_values):
    return int(np.argmin(np.abs(theta_dot_values - theta_dot)))


new_table = np.zeros_like(lookup_table)
for i,_ in enumerate(theta_dot_values_poincare):
    for j,__ in enumerate(control_inputs):
        if lookup_table[i,j] == -100:
            new_table[i,j] = -100
        else:
            new_table[i,j] = velocity_to_label(lookup_table[i,j],theta_dot_values_poincare)


ROA = n_theta_dot # Last state
n_states = n_theta_dot + 1

adjacency_matrix = np.zeros((n_states, n_states, n_control_inputs),dtype=bool)

for source in range(n_theta_dot):
    for control in range(n_control_inputs):
        destination = new_table[source, control]
        if destination == -100:
            adjacency_matrix[source, ROA, control] = True
        else:
            adjacency_matrix[source, int(destination), control] = True

print(adjacency_matrix)
for i in range(n_theta_dot):
    can_enter_roa = adjacency_matrix[i,ROA,:].any()
    print(f"From state theta_dot = {theta_dot_values_poincare[i]:.3f}, it is {can_enter_roa} that we can enter the ROA in one step.")


print(adjacency_matrix)

print(f"Lookup table: {lookup_table}")

print(f"New table: {new_table}")

# import numpy as np

# def find_paths_to_roa(
#     adjacency_matrix,
#     roa,
#     steps,
#     starts=None,
#     exact=True,
#     simple_paths=False,
# ):
#     """
#     adjacency_matrix[source, destination, control]

#     exact=True:
#         Only return paths that first reach the ROA on exactly `steps`.

#     exact=False:
#         Return paths that reach the ROA in at most `steps`.

#     simple_paths=True:
#         Do not revisit a state within the same path.
#     """
#     n_states, _, _ = adjacency_matrix.shape

#     if starts is None:
#         starts = [state for state in range(n_states) if state != roa]

#     paths = []

#     def search(current_state, remaining_steps, state_path, controls):
#         # Reaching the ROA ends the trajectory.
#         if current_state == roa:
#             if not exact or remaining_steps == 0:
#                 paths.append({
#                     "states": state_path.copy(),
#                     "controls": controls.copy(),
#                 })
#             return

#         # No transitions remain.
#         if remaining_steps == 0:
#             return

#         # Each row is [next_state, control].
#         outgoing_edges = np.argwhere(
#             adjacency_matrix[current_state]
#         )

#         for next_state, control in outgoing_edges:
#             next_state = int(next_state)
#             control = int(control)

#             if (
#                 simple_paths
#                 and next_state != roa
#                 and next_state in state_path
#             ):
#                 continue

#             search(
#                 current_state=next_state,
#                 remaining_steps=remaining_steps - 1,
#                 state_path=state_path + [next_state],
#                 controls=controls + [control],
#             )

#     for start in starts:
#         search(
#             current_state=start,
#             remaining_steps=steps,
#             state_path=[start],
#             controls=[],
#         )

#     return paths

# paths = find_paths_to_roa(
#     adjacency_matrix,
#     roa=ROA,
#     steps=3,
#     exact=False,
# )


# for path in paths:
#     states = path["states"]
#     controls = path["controls"]

#     pieces = [str(states[0])]

#     for control, destination in zip(controls, states[1:]):
#         destination_name = (
#             "ROA" if destination == ROA else str(destination)
#         )
#         pieces.append(f"--u={control}--> {destination_name}")

#     print(" ".join(pieces))

#     ##################

# sys.exit()




# STEPS = 2

# def find_two_step_paths(adjacency_matrix, roa):
#     paths = []
#     n_states, _, _ = adjacency_matrix.shape

#     for start in range(roa):
#         # Each result is [intermediate_state, first_control]
#         first_edges = np.argwhere(adjacency_matrix[start])

#         for intermediate, control_1 in first_edges:
#             # Exclude paths that reached the ROA on the first step
#             if intermediate == roa:
#                 continue

#             second_controls = np.flatnonzero(
#                 adjacency_matrix[intermediate, roa]
#             )

#             for control_2 in second_controls:
#                 paths.append({
#                     "start": start,
#                     "control_1": control_1,
#                     "intermediate": intermediate,
#                     "control_2": control_2,
#                     "destination": roa,
#                 })

#     return paths

# paths = find_two_step_paths(adjacency_matrix, ROA)

# for path in paths:
#     print(
#         f"{path['start']} "
#         f"--u={path['control_1']}--> {path['intermediate']} "
#         f"--u={path['control_2']}--> ROA"
#     )


# sys.exit()

# #############################3

# adjacency_matrix = np.zeros((n_theta_dot,n_theta_dot,n_control_inputs))
# for i in range(n_theta_dot):
#     for j in range(n_theta_dot):
#         for k in np.where((new_table[j,:] == i)):
#             adjacency_matrix[i,j,k] = 1



# adjacency_matrix = np.zeros((n_theta_dot, n_theta_dot, n_control_inputs), dtype=bool)

# for source in range(n_theta_dot):
#     for control in range(n_control_inputs):
#         destination = new_table[source, control]

#         if destination != -100:
#             adjacency_matrix[source, int(destination), control] = True

# # Example: 4x4 Adjacency Matrix



































# Poincare Section
# def control_to_label(control, control_inputs):
#     return np.argmin(np.abs(control_inputs - control))

# def velocity_to_label(current_theta_dot, theta_dot_values):
#     return np.argmin(np.abs(theta_dot_values - current_theta_dot))

# n_theta_dot = 25
# n_control_inputs = 3
# gravity = params["gravity"]
# length = params["length"]
# control_inputs = np.linspace(np.pi/8,np.pi/7,n_control_inputs)
# theta_dot_values_poincare = np.linspace(0,np.sqrt(2*gravity / length),n_theta_dot)
# lookup_table = np.full((n_theta_dot,n_control_inputs),np.nan) # Indexed by current_state, control_input, next_state

# total_ROA_flags = 0
# for theta_dot_0 in theta_dot_values_poincare:
#     ROA_flag = False
#     control_traj = np.zeros(sim_steps+1)
#     params = model.generate_params()
#     params["K_p"] = 10
#     params["K_d"] = 10
#     timestep = 1e-3
#     sim_time = 5
#     sim_steps = int(sim_time / timestep)
#     state_traj = np.zeros((sim_steps+1,2))
#     x0 = [0,theta_dot_0]
#     state_traj[0,:] = x0
#     current_state = x0.copy()
#     event_traj = np.zeros(sim_steps + 1, dtype=bool)
#     final_step = sim_steps + 1
#     time_traj = np.linspace(0,sim_time,sim_steps+1)
#     for step in range(sim_steps):
#         control_traj[step] = new_angle_of_attack
#         if model.state_is_in_roa(current_state, theta_values, theta_dot_values, classification_grid):
#             ROA_flag = True
#             final_step = step + 1
#             break

#         current_time = step * timestep
#         next_state = _rk4_step(current_time,current_state,timestep,model.dynamics,params)

#         collision = model.event_guard(current_state, next_state, params)
#         event_traj[step+1] = collision
#         if collision: # Collision occurs
#             current_state = model.event_dynamics(next_state, params)

#             # Update Alpha
#             new_angle_of_attack = np.pi/8
#             #new_angle_of_attack = random.uniform(np.pi/8,np.pi/7)
#             if new_angle_of_attack < np.pi/8 or new_angle_of_attack > np.pi/7:
#                 print("Angle of attack exceeded limits!")
#                 sys.exit()
#             params["angle_of_attack"] = new_angle_of_attack
#         else:
#             current_state = next_state

#         state_traj[step+1, :] = current_state    

#     #print("at second animations")
#     #model.plot_phase_portrait(classification_grid, theta_values, theta_dot_values, state_traj, show=True)
#     #model.animate(state_traj.T,time_traj,params,timestep,completed_steps)

#     state_traj = state_traj[:final_step]
#     control_traj = control_traj[:final_step]
#     event_traj = event_traj[:final_step]

#     theta_before = state_traj[:-1, 0]
#     theta_after = state_traj[1:, 0]

#     forward_crossing = ((theta_before < 0) & (theta_after >= 0))

#     continuous_interval = ~event_traj[1:]

#     zero_idx = np.flatnonzero(forward_crossing & continuous_interval) + 1
#     zero_idx = np.concatenate(([0], zero_idx))

#     for current_idx, next_idx in zip(zero_idx[:-1],zero_idx[1:],):
#         current_velocity = state_traj[current_idx, 1]
#         next_velocity = state_traj[next_idx, 1]

#         # The crossing occurred during the interval ending at current_idx.
#         current_control = control_traj[current_idx - 1]

#         velocity_label = velocity_to_label(current_velocity,theta_dot_values_poincare)
#         control_label = control_to_label(current_control,control_inputs)

#         lookup_table[velocity_label, control_label] = next_velocity

#     if ROA_flag and 'next_velocity' in locals():
#         lookup_table[velocity_to_label(next_velocity,theta_dot_values_poincare), control_to_label(next_velocity,control_inputs)] = -np.inf

# print(total_ROA_flags)
# print(lookup_table)
sys.exit()
###########################################################
for step in range(sim_steps):
    if model.state_is_in_roa(current_state, theta_values, theta_dot_values, classification_grid):
        # If we hit ROA, then we go to 0 at next state no matter control input so make new control input category
        lookup_table[velocity_to_label(current_state[1]),n_control_inputs+1] = 0
        break

    current_time = step * timestep
    next_state = _rk4_step(current_time,current_state,timestep,model.dynamics,params)
    if (current_state[0] >= 0 and next_state[0] <= 0) or (current_state[0] <= 0 and next_state[0] >= 0):
        action = np.pi/8
        lookup_table[velocity_to_label(current_state[1]),control_to_label(action)] = n


    if model.event_guard(current_state, next_state, params): # Collision occurs
        current_state = model.event_dynamics(next_state, params)
        state_traj[step+1, :] = current_state

        # Update Alpha
        completed_steps += 1
        new_angle_of_attack = np.pi/8
        #new_angle_of_attack = random.uniform(np.pi/8,np.pi/7)
        if new_angle_of_attack < np.pi/8 or new_angle_of_attack > np.pi/7:
            print("Angle of attack exceeded limits!")
            sys.exit()
        params["angle_of_attack"] = new_angle_of_attack
    else:
        current_state = next_state
        state_traj[step+1, :] = current_state
















dynamics_traj = np.zeros_like(state_traj)
for i, t in enumerate(time_traj):
    dynamics_traj[i, :] = model.dynamics(t, state_traj[i, :], params)
plt.figure()
plt.plot(state_traj[:, 0], dynamics_traj[:, 0],label="Theta Phase Portrait")
plt.axhline(ROA_state[1],color = "red",ls="dashed")
#plt.plot(state_traj[:, 1], dynamics_traj[:, 1], label="Theta Dot Phase Portrait")
plt.xlabel("State")
plt.ylabel("Dynamics")
plt.legend()
plt.title("Phase Portrait")
plt.tight_layout()
plt.show()

sys.exit()

event_indices = np.flatnonzero(events)
theta_dot_k = state_traj[1, event_indices] # Poincare section trajectories x_k

if theta_dot_k.size == 0:
    theta_dot_k = np.array([0])




sys.exit()


###################################################### VISUAL EXAMPLE BELOW #####################################################
# Fixed controls for this visualization example.
params = {
    "gravity": 9.81,  # m/s^2
    "length": 1.0,  # m
    "mass": 1.0,  # kg
    "incline": 0.06,  # rad
    "angle_of_attack": np.pi / 8,  # rad
    "ankle_torque": 0.0,  # N m
}

initial_state = np.array([0.0, 3.0])
timestep = 1e-4
sim_time = 3.0
desired_number_of_steps = 3

n_timesteps = round(sim_time / timestep) + 1
time_traj = np.arange(n_timesteps) * timestep
state_traj = np.zeros((2, n_timesteps))
state_traj[:, 0] = initial_state
completed_steps = 0

# Simulation loop. Replace this Euler step with your own integrator as needed.
for step, t in enumerate(time_traj[:-1]):
    state = state_traj[:, step]
    next_state = state + timestep * model.dynamics(t, state, params)

    if model.event_guard(state, next_state, params):
        next_state = model.event_dynamics(next_state, params)
        completed_steps += 1

    state_traj[:, step + 1] = next_state
    if completed_steps == desired_number_of_steps:
        break

time_traj = time_traj[: step + 2]
state_traj = state_traj[:, : step + 2]



fig, ax = plt.subplots(figsize=(8, 5), layout="constrained")


def draw_frame(index):
    # The massless swing leg is repositioned instantaneously at each impact.
    model.visualize(state_traj[:, index], params, ax=ax)
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
