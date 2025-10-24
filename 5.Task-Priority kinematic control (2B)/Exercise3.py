from lab6_robotics import * # Includes numpy import
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import matplotlib.animation as anim
import matplotlib.transforms as trans

# Robot model
d     =  np.zeros(3)     # displacement along Z-axis
theta =  np.array([0, 0., 0.]).reshape(3,1) # rotation around Z-axis
alpha =  np.zeros(3)                  # rotation around X-axis
a     =  np.array([0.5, 0.3 , 0.2])   # displacement along X-axis
revolute = [True, True , True]        # flags specifying the type of joints
robot = MobileManipulator(d, theta, a, alpha, revolute)

W = np.diag([1000,1000,1000,1000,1000])*0.001  # Weight matrix

# Predefined end-effector targets for consistent testing
sigma_targets = [
    np.array([[1.0], [0.5]]),
    np.array([[0.5], [-1.0]]),
    np.array([[-1.0], [0.2]]),
    np.array([[0.0], [1.2]]),
    np.array([[0.8], [-0.8]])
]
current_target_idx = 0
max_targets = len(sigma_targets)
target_reached_threshold = 0.05  # Acceptable error norm to consider target reached

# Initialize target
sigma_endeff = sigma_targets[current_target_idx]
sigma_config = np.block([[sigma_endeff], [0]]).reshape(3, 1)

velocity_limit = [-0.5, 0.2]
JointLimit_link = 3
obstacle_tasks = []


# Task list
tasks = [
    Position2D("End-effector position", sigma_endeff, 5),
    #Configuration2D("End-effector configuration", sigma_config, link=5),
]

# Logging and simulation state
num_obstacles = len(obstacle_tasks)
err_counter = 0
error_vector = np.zeros((0, len(tasks)))
ore_error = []
obstacle_dist = np.zeros((0, 0))
endeff_pose = np.zeros((2, 0))
base_pose = np.zeros((2, 0))
dt = 1.0 / 60.0

# Drawing preparation
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2,2))
ax.set_title('Simulation')
ax.set_aspect('equal')
ax.grid()
ax.set_xlabel('x[m]')
ax.set_ylabel('y[m]')
rectangle = patch.Rectangle((-0.25, -0.15), 0.5, 0.3, color='blue', alpha=0.3)
veh = ax.add_patch(rectangle)
line, = ax.plot([], [], 'o-', lw=2)
path, = ax.plot([], [], 'c-', lw=1)
point, = ax.plot([], [], 'rx')
PPx = []
PPy = []

# Initialization
def init():
    global tasks, sigma_endeff, error_vector
    line.set_data([], [])
    path.set_data([], [])
    point.set_data([], [])
    sigma_endeff = sigma_targets[current_target_idx]
    sigma_config = np.block([[sigma_endeff], [0]]).reshape(3,1)

    for task in tasks:
        if task.name == "End-effector position":
            task.setDesired(sigma_endeff)
        elif task.name == "End-effector configuration":
            task.setDesired(sigma_config)

    return line, path, point


# Simulation loop
def simulate(t):
    global current_target_idx, sigma_endeff, sigma_config
    global robot, err_counter, error_vector, ore_error, obstacle_dist, endeff_pose, base_pose
    global PPx, PPy

    if current_target_idx >= max_targets:
        return line, veh, path, point  # Stop simulation

    dof = robot.getDOF()
    P = np.eye(dof)
    dq = np.zeros((dof, 1))
    err_acc = np.zeros((1, 0))
    obs_dis = np.zeros((1, 0))

    for task in tasks:
        task.update(robot)
        J = task.getJacobian()

        if task.isTaskActive():
            Jbar = J @ P
            dq += W_DLS(Jbar, 0.1, W) @ (
                task.getFeedforwardVelocity() + task.getGain() @ task.getError() - J @ dq
            )
            P -= np.linalg.pinv(Jbar) @ Jbar

        err = np.linalg.norm(task.getError())
        err_acc = np.block([err_acc, err])

    error_vector = np.block([[error_vector], [err_acc]])
    obstacle_dist = np.block([[obstacle_dist], [obs_dis]])
    err_counter += 1
    # Check if target reached
    if np.linalg.norm(tasks[0].getError()) < target_reached_threshold:
        current_target_idx += 1
        if current_target_idx < max_targets:
            sigma_endeff = sigma_targets[current_target_idx]
            sigma_config = np.block([[sigma_endeff], [0]]).reshape(3, 1)
            tasks[0].setDesired(sigma_endeff)
        return line, veh, path, point

    robot.update(dq, dt)


    endeff_pose = np.hstack((endeff_pose, robot.getEETransform()[0:2,3].reshape(2,1)))
    base_pose = np.hstack((base_pose, robot.getBasePose()[0:2,0].reshape(2,1)))


    # Update drawing
    # -- Manipulator links
    PP = robot.drawing()[:, 2:]  # Skip the 3 base links
    line.set_data(PP[0,:], PP[1,:])
    PPx.append(PP[0,-1])
    PPy.append(PP[1,-1])
    path.set_data(PPx, PPy)
    point.set_data(sigma_endeff[0], sigma_endeff[1])
    # -- Mobile base
    eta = robot.getBasePose()
    veh.set_transform(
        trans.Affine2D().rotate(eta[2,0]) +
        trans.Affine2D().translate(eta[0,0], eta[1,0]) +
        ax.transData
    )

    return line, veh, path, point

# Run animation
animation = anim.FuncAnimation(fig, simulate, np.arange(0, 10, dt),
                               interval=10, blit=True, init_func=init, repeat=True)
plt.show()

# Plot error over time
total_time = np.arange(0, err_counter * dt, dt)
print(err_counter, error_vector.shape, obstacle_dist.shape)
ylim = (
    min(error_vector.min(), velocity_limit[0]) - 0.1,
    max(error_vector.max(), velocity_limit[1]) + 0.1
)
fig = plt.figure()
ax2 = fig.add_subplot(111, autoscale_on=False, xlim=(0, err_counter * dt), ylim=ylim)
ax2.set_title('Task-Priority -> ' + str(len(tasks)) + ' Tasks for K=' + str(tasks[0].getGain()[0][0]))
ax2.set_xlabel('Time(s)')
ax2.set_ylabel('Value[1]')
ax2.grid()

for i in range(num_obstacles):
    ax2.plot(total_time, obstacle_dist[:, i], label="d_" + str(i) + " `distance to obstacle` ")

for i, task in enumerate(tasks):
    if task.name == "End-effector configuration":
        ax2.plot(total_time, error_vector[:, i], label="position of end-effector")
        ax2.plot(total_time, ore_error, label="orientation of end-effector")
    elif task.name == "Joint limit":
        ax2.plot(total_time, error_vector[:, i], label="position of joint 1")
        ax2.axhline(y=velocity_limit[0], color='r', linestyle='dashed')
        ax2.axhline(y=velocity_limit[1], color='g', linestyle='dashed')
    elif task.name not in ["Obstacle avoidance"]:
        ax2.plot(total_time, error_vector[:, i], label=task.name + " error")

ax2.axhline(y=velocity_limit[0], color='r', linestyle='dashed')
ax2.axhline(y=velocity_limit[1], color='g', linestyle='dashed')
ax2.legend()
plt.show()

# Save simulation data
print("End-effector shape:", endeff_pose.shape)
print("Base pose shape:", base_pose.shape)

# 1. move forward then rotate
np.save('move forward then rotate.npy', np.hstack((endeff_pose.T, base_pose.T)))

# 2. rotate then move forward
#np.save('rotate then move forward.npy', np.hstack((endeff_pose.T, base_pose.T)))

# 3. rotate and move forward
#np.save('rotate and move forward.npy', np.hstack((endeff_pose.T, base_pose.T)))

