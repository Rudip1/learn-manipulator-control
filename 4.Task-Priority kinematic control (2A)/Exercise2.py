from lab5_robotics import * # Includes numpy import
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import matplotlib.patches as patch

# Robot model
d =  np.zeros(3)                           # displacement along Z-axis
theta = np.array([0.2, 0.2, 0.2]).reshape(3,1) # rotation around Z-axis
alpha = np.zeros(3)                       # rotation around X-axis
a =  np.array([0.6, 0.5, 0.4])            # displacement along X-axis
revolute = [True, True, True]             # flags specifying the type of joints
robot = Manipulator(d, theta, a, alpha, revolute) # Manipulator object


# Joint limit task parameters
velocity_limit = [-0.5, 0.5]  # Joint limits for joint 1
joint_limit_task = JointLimit("Joint limit", velocity_limit, [0.05, 0.15], link=1)

# End-effector task
sigma_endeff = np.zeros((2,1))
tasks = [
    joint_limit_task,
    Position2D("End-effector position", np.array([1.0, 0.5]).reshape(2,1))
]

# Simulation parameters
err_counter = 0
error_vector = np.zeros((0 ,len(tasks)))

# Simulation params
dt = 1.0/60.0

# Drawing preparation
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2,2))
ax.set_title('Simulation')
ax.set_aspect('equal')
ax.grid()
ax.set_xlabel('x[m]')
ax.set_ylabel('y[m]')

line, = ax.plot([], [], 'o-', lw=2)
path, = ax.plot([], [], 'c-', lw=1)
point, = ax.plot([], [], 'rx')
PPx, PPy = [], []

# Simulation initialization
def init():
    global sigma_endeff
    line.set_data([], [])
    path.set_data([], [])
    point.set_data([], [])
    sigma_endeff = np.random.uniform(-1.2, 1.2, (2, 1))
    for task in tasks:
        if task.name == "End-effector position":
            task.setDesired(sigma_endeff)
    return line, path, point

def simulate(t):
    global tasks, robot, PPx, PPy, err_counter, error_vector

    dof = robot.getDOF()
    P = np.eye(dof)
    err_acc = np.zeros((1,0))
    dq = np.zeros((dof,1))

    for index, task in enumerate(tasks):
        task.update(robot)
        J = task.getJacobian()
        if task.isTaskActive():
            Jbar = J @ P
            dq += DLS(Jbar, 0.1) @ (task.getFeedforwardVelocity() + task.getGain() @ task.getError() - J @ dq)
            P = P - np.linalg.pinv(Jbar) @ Jbar

        new_error = robot.getJointPos(0) if task.name == "Joint limit" else np.linalg.norm(task.getError())
        err_acc = np.block([err_acc, new_error])

    robot.update(dq, dt)
    error_vector = np.block([[error_vector], [err_acc]])
    err_counter += 1

    PP = robot.drawing()
    line.set_data(PP[0,:], PP[1,:])
    PPx.append(PP[0,-1])
    PPy.append(PP[1,-1])
    path.set_data(PPx, PPy)
    point.set_data(sigma_endeff[0], sigma_endeff[1])
    return line, path, point

# Run animation for up to 1 minute
animation = anim.FuncAnimation(fig, simulate, np.arange(0, 10, dt), 
                                interval=10, blit=True, init_func=init, repeat=True)
plt.show()

# Plot errors and joint limit state after simulation
if err_counter > 0:
    total_time = np.arange(0, err_counter * dt, dt)
    fig = plt.figure()
    ax2 = fig.add_subplot(111, autoscale_on=False, xlim=(0, err_counter * dt), 
                          ylim=(error_vector.min() - 0.1, error_vector.max() + 0.1))
    ax2.set_title(f'Task-Priority Control: {len(tasks)} Tasks, K={tasks[0].getGain()[0][0]}')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Value')
    ax2.grid()

    for i, task in enumerate(tasks):
        if task.name == "Joint limit":
            ax2.plot(total_time, error_vector[:, i], label="Joint 1 Position")
            ax2.axhline(y=velocity_limit[0], color='r', linestyle='dashed')
            ax2.axhline(y=velocity_limit[1], color='g', linestyle='dashed')
        else:
            ax2.plot(total_time, error_vector[:, i], label=f"{task.name} Error")

    ax2.legend()
    plt.show()
