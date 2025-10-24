# Import necessary libraries
from Common import *  # Includes numpy import
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anim

# Robot definition
d = np.zeros(2)           # Displacement along Z-axis
q_init = np.array([0.2, 0.5])  # Initial joint angles
a = np.array([0.75, 0.5]) # Displacement along X-axis
alpha = np.zeros(2)       # Rotation around X-axis 
revolute = [True, True]
sigma_d = np.array([0.0, 1.0])  # Desired end-effector position
K = np.diag([1, 1])  # Control gain

# Simulation parameters
dt = 1.0 / 60.0

# Control methods to test
controllers = ["transpose", "pseudoinverse", "DLS"]

# Storage for error norms
error_storage = {ctrl: [] for ctrl in controllers}

# Define colors to plot path for different methods
controller_colors = {
    "transpose": 'b',         # Blue
    "pseudoinverse": 'orange',  # Orange
    "DLS": 'g'                # Green
}

# Drawing preparation
figs = {}
axes = {}
lines = {}
paths = {}
points = {}
PPxs = {}
PPys = {}
qs = {}  # Store independent q values for each controller

for ctrl in controllers:
    figs[ctrl] = plt.figure()
    axes[ctrl] = figs[ctrl].add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
    axes[ctrl].set_title(f'Simulation - {ctrl}')
    axes[ctrl].set_aspect('equal')
    axes[ctrl].grid()
    lines[ctrl], = axes[ctrl].plot([], [], 'o-', lw=2)
    paths[ctrl], = axes[ctrl].plot([], [], controller_colors[ctrl], lw=1)
    points[ctrl], = axes[ctrl].plot([], [], 'rx')
    PPxs[ctrl] = []
    PPys[ctrl] = []
    qs[ctrl] = np.array(q_init)  # Initialize independent q values for each controller

# Simulation initialization
def init():
    for ctrl in controllers:
        lines[ctrl].set_data([], [])
        paths[ctrl].set_data([], [])
        points[ctrl].set_data([], [])
    return sum([[lines[ctrl], paths[ctrl], points[ctrl]] for ctrl in controllers], [])

# Controller function
def controller(type, J):
    if type == "transpose":
        return J.T
    elif type == "pseudoinverse":
        return np.linalg.pinv(J)
    elif type == "DLS":
        return DLS(J, 0.1)  # Assuming DLS is defined elsewhere
    else:
        raise ValueError("Unknown controller method")

# Simulation loop (stores errors)
def simulate(t, current_controller):
    global d, a, alpha, revolute, sigma_d, error_storage
    
    q = qs[current_controller]  # independent q value per controller
    
    # Update robot
    T = kinematics(d, q, a, alpha)
    J = jacobian(T, revolute)[0:2, :]  # Extract first two rows for planar case
    P = robotPoints2D(T)  # Extract points

    # Compute error
    sigma = np.array([P[0, -1], P[1, -1]])  # Position of the end-effector
    err = sigma_d - sigma  # Control error
    dq = controller(current_controller, J) @ (K @ err)  # Compute control solution
    q += dt * dq  # Integrate to update joint angles
    qs[current_controller] = q  # Store updated q

    # Store error norm
    error_storage[current_controller].append(np.linalg.norm(err))

    # Update drawing
    lines[current_controller].set_data(P[0, :], P[1, :])
    PPxs[current_controller].append(P[0, -1])
    PPys[current_controller].append(P[1, -1])
    paths[current_controller].set_data(PPxs[current_controller], PPys[current_controller])
    points[current_controller].set_data(sigma_d[0], sigma_d[1])

    return lines[current_controller], paths[current_controller], points[current_controller]

# Run simulations for all controllers
animations = []
for ctrl in controllers:
    animation = anim.FuncAnimation(figs[ctrl], simulate, np.arange(0, 10, dt), 
                                   fargs=(ctrl,), interval=10, blit=True, init_func=init, repeat=False)
    animations.append(animation)

# Show animation plots
plt.show()

# **Plot Control Error Norms After Simulation Completes**
fig_error, ax_error = plt.subplots()
ax_error.set_title('Control Error Norm Evolution')
ax_error.set_xlabel('Time Steps')
ax_error.set_ylabel('Error Norm')
ax_error.grid(True)

for ctrl in controllers:
    ax_error.plot(error_storage[ctrl], label=ctrl, color=controller_colors[ctrl], lw=1.0)

ax_error.legend()
plt.show()

# Plot Path Comparison of path for All Controllers
fig_path, ax_path = plt.subplots()
ax_path.set_title('Path Comparison of Controllers')
ax_path.set_xlabel('X Position')
ax_path.set_ylabel('Y Position')
ax_path.grid(True)

# Plot the recorded paths for all controllers
for ctrl in controllers:
    ax_path.plot(PPxs[ctrl], PPys[ctrl], label=ctrl, color=controller_colors[ctrl], lw=2)

ax_path.legend()
plt.show()
