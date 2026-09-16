import numpy as np
def explicit_euler(timestep, sim_time, x0, dynamics, params, check_event = lambda state: [False,state]):
    n_timesteps = int(sim_time / timestep) + 1
    time_traj = np.arange(n_timesteps) * timestep
    state_traj = np.zeros((2, n_timesteps))
    state_traj[:, 0] = x0
    events = np.zeros(n_timesteps, dtype=bool)
    # simulation loop
    for step, t in enumerate(time_traj[:-1]):
         events[step], state_traj[:, step + 1] = check_event(state_traj[:, step] + timestep * dynamics(
            t, state_traj[:, step], params),params)


    return time_traj, state_traj   