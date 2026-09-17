from pathlib import Path

import sys
import matplotlib.pyplot as plt
import numpy as np
from integrators import rk4 as integrator
from matplotlib.animation import FuncAnimation, PillowWriter
from models import inverted_pendulum_walker as model

params = model.generate_params()
mass = params["mass"]
gravity = params["gravity"]
length = params["length"]
K_p = 0.01
K_d = 0.01
tau_lower_bound = -0.1*mass*gravity*length
tau_upper_bound = 0.05*mass*gravity*length
timestep = 1e-4
one_time_step = timestep
sim_time = 3
x0 = [0.0,2.0]
current_state = x0.copy()
sim_steps = int(sim_time / timestep)
state_traj = np.zeros((sim_steps+1,2))
state_traj[0,:] = x0
time_traj = np.zeros(sim_steps)
completed_steps = 0
for step in range(sim_steps):
    events, local_time, local_state = integrator(timestep,one_time_step,current_state,model.dynamics,params,model.event_guard,model.event_dynamics)
    time_traj[step] = (step + 1) * timestep
    current_state = local_state[:,-1]
    state_traj[step+1, :] = current_state
    new_ankle_torque = -mass*gravity*length*np.sin(current_state[0])-K_p*current_state[0]-K_d*current_state[1]
    if new_ankle_torque < tau_lower_bound or new_ankle_torque > tau_upper_bound:
        if new_ankle_torque > 0:
            params["ankle_torque"] = tau_upper_bound
        else:
            params["ankle_torque"] = tau_lower_bound
    params["ankle_torque"] = new_ankle_torque

    if events[-1]: # Collision occurs, update alpha
        completed_steps += 1
        new_angle_of_attack = np.pi/7
        if new_angle_of_attack < np.pi/8 or new_angle_of_attack > np.pi/7:
            print("Angle of attack exceeded limits!")
            sys.exit()
        params["angle_of_attack"] = new_angle_of_attack



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
