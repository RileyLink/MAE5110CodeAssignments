import numpy as np
def rk4(timestep, sim_time, x0, dynamics, params, check_event = lambda state: state):
    n_timesteps = int(sim_time / timestep) + 1
    time_traj = np.arange(n_timesteps) * timestep
    state_traj = np.zeros((2, n_timesteps))
    state_traj[:, 0] = x0
    for step, t in enumerate(time_traj[:-1]):
        k1 = dynamics(t, state_traj[:, step], params)
        k2 = dynamics(t+timestep/2,state_traj[:,step]+k1*timestep/2, params)
        k3 = dynamics(t+timestep/2,state_traj[:,step]+k2*timestep/2, params)
        k4 = dynamics(t+timestep,state_traj[:,step]+k3*timestep, params)
        state_traj[:, step+1] = check_event(state_traj[:, step] + (timestep/6) * (k1+2*k2+2*k3+k4))

    return time_traj, state_traj 