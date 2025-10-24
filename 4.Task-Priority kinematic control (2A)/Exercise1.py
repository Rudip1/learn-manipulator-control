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

# Task hierarchy definition
obstacle_pos = np.array([0.0, 1.0]).reshape(2,1)
obstacle_r = 0.5

obstacle_pos2 = np.array([-0.5, -0.75]).reshape(2,1)
obstacle_r2 = 0.25

obstacle_pos3 = np.array([0.75, -0.5]).reshape(2,1)
obstacle_r3 = 0.3

# obstacle task definition 
obstacle_tasks = [
    Obstacle2D("Obstacle avoidance", obstacle_pos, np.array([obstacle_r , obstacle_r+0.05])),
    Obstacle2D("Obstacle avoidance", obstacle_pos2, np.array([obstacle_r2 , obstacle_r2+0.05])),
    Obstacle2D("Obstacle avoidance", obstacle_pos3, np.array([obstacle_r3 , obstacle_r3+0.05])),   
]

# Desired end-effector position
sigma_endeff = np.zeros((2,1)) 

'''
tasks = [ 
          Obstacle2D("Obstacle avoidance", obstacle_pos, np.array([obstacle_r, obstacle_r+0.05])),
          Position2D("End-effector position", np.array([1.0, 0.5]).reshape(2,1)
        ] 
'''

# Task hierarchy definition
tasks = obstacle_tasks  + [ 
        # JointLimit("Joint limit", velocity_limit, velo_thershold = [0.05  , 0.15] ),
        Position2D("End-effector position", np.array([1.0, 0.5]).reshape(2,1))
    ] 

num_obstacles = len(obstacle_tasks)
err_counter = 0 # Error counter
error_vector = np.zeros((0 ,len(tasks))) # Error vector
obstacle_dist = np.zeros((0,num_obstacles))

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

ax.add_patch(patch.Circle(obstacle_pos.flatten(), obstacle_r, color='blue', alpha=0.3))
ax.add_patch(patch.Circle(obstacle_pos2.flatten(), obstacle_r2, color='orange', alpha=0.3))
ax.add_patch(patch.Circle(obstacle_pos3.flatten(), obstacle_r3, color='green', alpha=0.3))

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
    
    # Desired end-effector configuration
    sigma_config = np.block([[sigma_endeff], [np.pi/2]]).reshape(3,1)
    
    # endeffector task and configuration task 
    for task in tasks:
        if(task.name == "End-effector position"):
            task.setDesired(sigma_endeff)
        elif(task.name == "End-effector configuration"):
            task.setDesired(sigma_config)

    return line, path, point

# Simulation loop
def simulate(t):
    global tasks, robot, PPx, PPy, err_counter, obstacle_dist, sigma_endeff, error_vector
    
    ### Recursive Task-Priority algorithm (w/set-based tasks)
    # The algorithm works in the same way as in Lab4. 
    # The only difference is that it checks if a task is active.
    ###
    dof = robot.getDOF() # Get robot degrees of freedom
    P = np.eye(dof) ## Initialize null-space projector
    err_acc = np.zeros((1,0))
    obs_dis = np.zeros((1 , 0))
    dq = np.zeros((dof,1)) # Initialize joint velocity vector
    
    for index, task in enumerate(tasks): # Loop over tasks
        task.update(robot) # Update task state
        J = task.getJacobian()   # Compute augmented Jacobian
         # Compute task velocity  
        if(task.isTaskActive()):# check activation of the task
            Jbar= J @ P # Compute augmented Jacobian 
            dq = dq + DLS(Jbar,0.1)@( task.getFeedforwardVelocity() +
                        task.getGain()@task.getError() - J@dq)   
            P = P - np.linalg.pinv(Jbar)@Jbar # Update null-space projector
        
        new_error = np.linalg.norm(task.getError())
        if task.name == "Obstacle avoidance":
            obs_dis = np.block([obs_dis, task.distance_to_obstacle()])
        err_acc = np.block([err_acc, new_error])

    # Update robot
    robot.update(dq, dt)
    error_vector = np.block([[error_vector], [err_acc]])
    obstacle_dist = np.block([[obstacle_dist], [obs_dis]])
    err_counter += 1
    
    # Update drawing
    PP = robot.drawing()
    line.set_data(PP[0,:], PP[1,:])
    PPx.append(PP[0,-1])
    PPy.append(PP[1,-1])
    path.set_data(PPx, PPy)
    point.set_data(sigma_endeff[0], sigma_endeff[1])
    
    return line, path, point

# Run animation
animation = anim.FuncAnimation(fig, simulate, np.arange(0, 10, dt), 
                                interval=10, blit=True, init_func=init, repeat=True)
plt.show()

# Plot errors and distances after simulation
total_time = np.arange(0, err_counter * dt, dt)
fig = plt.figure()
ax2 = fig.add_subplot(111, autoscale_on=False, xlim=(0, err_counter * dt), 
                      ylim=(error_vector.min() - 0.1, error_vector.max() + 0.1))
ax2.set_title(f'Task-Priority Control: {len(tasks)} Tasks, K={tasks[0].getGain()[0][0]}')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Value')
ax2.grid()

for i in range(num_obstacles):
    ax2.plot(total_time, obstacle_dist[:, i], label=f"d_{i+1} Distance to Obstacle")

for i, task in enumerate(tasks):
    if task.name != "Obstacle avoidance":
        ax2.plot(total_time, error_vector[:, i], label=f"{task.name} Error")

ax2.legend()
plt.show()