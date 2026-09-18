from pathlib import Path

import sys
import time
import matplotlib.pyplot as plt
import numpy as np
from integrators import rk4 as integrator
from matplotlib.animation import FuncAnimation, PillowWriter
from models import inverted_pendulum_walker as model

# CALCULATE CONTROLLER ROA IF A DATA FILE IS NOT ALREADY SAVED
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
        n_theta=100,n_theta_dot=100, 
        params = params, 
        sim_time = 20, 
        timestep = 5e-4, 
        show=True)
    np.savez(roa_path,classification_grid=classification_grid,theta_values=theta_values,theta_dot_values=theta_dot_values)
    fig.savefig("output/assignment_2/controller_roa.png",dpi=300,bbox_inches="tight")

# Params
params = model.generate_params()
params["K_p"] = 10
params["K_d"] = 10

timestep = 1e-3
sim_time = 5
sim_steps = int(sim_time / timestep)
state_traj = np.zeros((sim_steps+1,2))
x0 = [0,30]
state_traj[0,:] = x0
current_state = x0.copy()
time_traj = np.linspace(0,sim_time,sim_steps+1)
event_traj = np.zeros(sim_steps + 1, dtype=bool)
completed_steps = 0
print(f"Running simulation starting at {x0}")
first_roa = True
for step in range(sim_steps):
    if model.state_is_in_roa(current_state, theta_values, theta_dot_values, classification_grid):
        if first_roa:
            print(f"Reached the ROA:\n Step: {step}\n State: {current_state}")
            first_roa = False
        params["ankle_torque"] = model.calculate_torque(current_state,params)
    
    local_events, local_time, local_state = integrator(timestep,timestep,current_state,model.dynamics,params,model.event_guard,model.event_dynamics)
    current_state = local_state[:,-1]
    state_traj[step+1, :] = current_state
    event_traj[step + 1] = local_events[-1]

    if local_events[-1]: # Collision occurs, update alpha
        completed_steps += 1
        new_angle_of_attack = np.pi/7
        if new_angle_of_attack < np.pi/8 or new_angle_of_attack > np.pi/7:
            print("Angle of attack exceeded limits!")
            sys.exit()
        params["angle_of_attack"] = new_angle_of_attack

print("Simulation over, animating now...")
model.animate(state_traj.T,time_traj,params,timestep,completed_steps)


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
