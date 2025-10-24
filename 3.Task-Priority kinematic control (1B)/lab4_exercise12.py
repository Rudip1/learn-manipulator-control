from lab4_robotics import *  # Includes numpy import
import matplotlib.pyplot as plt
import matplotlib.animation as anim

# Robot model - 3-link manipulator
d = np.zeros(3)  # Displacement along Z-axis
theta = np.array([np.pi/12, np.pi/6, np.pi/3]).reshape(3,1)  # Rotation around Z-axis
alpha = np.zeros(3)  # Rotation around X-axis
a = np.array([0.8, 0.6, 0.4])  # Displacement along X-axis
revolute = [True, True, True]  # Flags specifying the type of joints
robot = Manipulator(d, theta, a, alpha, revolute)  # Manipulator object
dt = 1.0 / 60.0  # Simulation time step

# Drawing preparation
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
ax.set_title('Simulation')
ax.set_aspect('equal')
ax.grid()
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
line, = ax.plot([], [], 'o-', lw=2)  # Robot structure
path, = ax.plot([], [], 'c-', lw=1)  # End-effector path
point, = ax.plot([], [], 'rx')  # Target
PPx = []
PPy = []

# Task hierarchy definition
sigma_endeff = np.zeros((2,1))  # Desired end-effector position
sigma_config = np.block([[sigma_endeff], [0]]).reshape(3,1)  # Desired end-effector configuration

# Exercise 1: Uncomment the appropriate task hierarchy (a, b, c or d)
##################################
# Task hierarchies (lists of tasks): uncomment one

#a. One task -> 1: end-effector position
#tasks = [Position2D("End-effector position", np.array([1, 1]).reshape(2,1), 3)]


#b. One task -> 1: end-effector con guration
#tasks = [Configuration2D("End-effector configuration", sigma_config.reshape(3,1), 3)]


#c. Two tasks -> 1: end-effector position, 2: end-effector orientation
#tasks = [
#         Position2D("End-effector position", np.array([1,1]).reshape(2,1), 3), 
#         Orientation2D("End-effector orientation", np.array([0]).reshape(1,1), 3)]


#d. Two tasks -> 1: end-effector position, 2: joint 1 position
#tasks = [
#     Position2D("End-effector position", np.array([1,1]).reshape(2,1), link=3),
#    JointPosition("Joint 1 position", np.array([0.0]).reshape(1,1), link=1)]  # Joint 1

##################################


# Exercise 2: Define tasks with tracking and gain matrices
tasks = [
    Position2D("End-effector position", np.array([1,1]).reshape(2,1), link=3, v_ff=np.array([[0.1], [0.1]]), K=np.diag([2,2])),
    Orientation2D("End-effector orientation", np.array([0]).reshape(1,1), link=2, v_ff=np.array([[0.05]]), K=np.array([[3]]))]

##################################



error_vector = np.zeros((0, len(tasks)))  # Error vector
err_counter = 0  # Error counter


# Simulation initialization
def init():
    global tasks, sigma_endeff, error_vector
    line.set_data([], [])
    path.set_data([], [])
    point.set_data([], [])
    sigma_endeff = np.random.uniform(-1, 1, (2, 1))  # Random end-effector goal
    sigma_config = np.block([[sigma_endeff], [np.pi / 12]]).reshape(3,1)  # Desired end-effector configuration

    for task in tasks:
        if task.name == "End-effector position":
            task.setDesired(sigma_endeff)
            task.SetFeedforwardVelocity(np.array([[0.1], [0.1]]))  # Tracking velocity
        elif task.name == "End-effector configuration":
            task.setDesired(sigma_config)
    
    return line, path, point

# Simulation loop
def simulate(t):
    global tasks, robot, PPx, PPy, err_counter, sigma_endeff, error_vector

    ### Recursive Task-Priority algorithm ###
    dof = robot.getDOF()  # Get robot degrees of freedom
    P = np.eye(dof)  # Initialize null-space projector
    err_acc = np.zeros((1, 0))  # Initialize accumulated error
    dq = np.zeros((dof, 1))  # Initialize joint velocity


    ##################################
    # Uncomment for Desired trajectory (Exercise 2 tracking)

    sigma_endeff = np.array([np.cos(t), np.sin(t)]).reshape(2,1)  # Circular trajectory
    feed_vel = np.array([-np.sin(t), np.cos(t)]).reshape(2,1)  # Corresponding velocity
    sigma_config = np.block([[sigma_endeff], [np.pi/2]]).reshape(3,1) # Desired end-effector configuration
    ##################################



    for task in tasks:  # Loop over tasks

        #####################################
        # Exercise 2: Enable tracking by setting dynamic desired values (uncomment for tracking)
        if task.name == "End-effector position":
            task.setDesired(sigma_endeff)
            task.SetFeedforwardVelocity(feed_vel)
        #####################################

        
        task.update(robot)  # Update task state
        J = task.getJacobian()  # Compute task Jacobian
        Jbar = J @ P  # Compute augmented Jacobian
        
        err_acc = np.block([err_acc, np.linalg.norm(task.getError())])  # Accumulate task error
        dq = dq + DLS(Jbar, 0.1) @ (task.getGain() @ task.getError() - J @ dq)  # Compute control update
        
        P = P - PseudoInverseJacob(Jbar) @ Jbar  # Update null-space projector

    # Update robot state
    robot.update(dq, dt)

    # Store error
    error_vector = np.block([[error_vector], [err_acc]])
    err_counter += 1

    # Update visualization
    PP = robot.drawing()
    line.set_data(PP[0, :], PP[1, :])
    PPx.append(PP[0, -1])
    PPy.append(PP[1, -1])
    path.set_data(PPx, PPy)
    point.set_data(sigma_endeff[0], sigma_endeff[1])

    return line, path, point

# Run simulation
animation = anim.FuncAnimation(fig, simulate, np.arange(0, 10, dt), 
                                interval=10, blit=True, init_func=init, repeat=True)

plt.show()

# Plot the evolution of task errors
total_time = np.arange(0, err_counter * dt, dt)
fig = plt.figure()
ax2 = fig.add_subplot(111, autoscale_on=False, xlim=(0, err_counter * dt), ylim=(0, error_vector.max()))
ax2.set_title('Task-Priority -> {} Tasks'.format(len(tasks)))
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Error')
ax2.grid()

for i, task in enumerate(tasks):
    ax2.plot(total_time, error_vector[:, i], label=task.name + " of link " + str(task.link))

ax2.legend()
plt.show()

