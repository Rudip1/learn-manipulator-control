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

# Desired end-effector position
sigma_endeff = np.zeros((2,1)) 
# Desired end-effector configuration
sigma_config = np.block([[sigma_endeff], [0]]).reshape(3,1) 

velocity_limit = [-0.5,0.2] # Joint velocity limits
# Task definition
obstacle_tasks = []
JointLimit_link = 3
tasks = [ 
           JointLimit("Joint limit", velocity_limit, velo_thershold = [0.05  , 0.1] , link = JointLimit_link ),
           Position2D("End-effector position", np.array([1.0, 0.5]).reshape(2,1), 5)
        ] 
num_obstacles = len(obstacle_tasks)
err_counter = 0 # Error counter
error_vector = np.zeros((0 ,len(tasks))) # Error vector
obstacle_dist = np.zeros((0,0))
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
rectangle = patch.Rectangle((-0.25, -0.15), 0.5, 0.3, color='blue', alpha=0.3)
veh = ax.add_patch(rectangle)
line, = ax.plot([], [], 'o-', lw=2) # Robot structure
path, = ax.plot([], [], 'c-', lw=1) # End-effector path
point, = ax.plot([], [], 'rx') # Target
PPx = []
PPy = []

# Simulation initialization
def init():
    global tasks , sigma_endeff , error_vector 
    line.set_data([], [])
    path.set_data([], [])
    point.set_data([], [])
    sigma_endeff = np.random.uniform(-2, 2, (2, 1))
     # Desired end-effector configuration
    sigma_config = np.block([[sigma_endeff], [np.pi/2]]).reshape(3,1)
    # set desired values for each task in each iteration
    # endeffector task and configuration task 
    for task in tasks:
        if(task.name == "End-effector position"):
            task.setDesired(sigma_endeff)
        elif(task.name == "End-effector configuration"):
            task.setDesired(sigma_config)
    return line, path, point

# Simulation loop
def simulate(t):
    global tasks
    global robot , err_counter , obstacle_dist
    global PPx, PPy  , sigma_endeff , error_vector 
    
    ### Recursive Task-Priority algorithm
    
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
            Jbar= J@P # Compute augmented Jacobian 
            dq = dq + DLS(Jbar,0.1)@( task.getFeedforwardVelocity() +
                        task.getGain()@task.getError() - J@dq)   
            P = P - np.linalg.pinv(Jbar)@Jbar # Update null-space projector
        new_error = np.linalg.norm(task.getError()) #error accumulation

        if task.name == "Joint limit":
            new_error = robot.getJointPos(JointLimit_link-1) # get joint position and store 
        elif task.name == "Obstacle avoidance":
            # get distance to obstacle and store
            obs_dis = np.block([ obs_dis , task.distance_to_obstacle()])

        err_acc = np.block([ err_acc , new_error]) # Accumulate velocity 
     
    # Update robot
    robot.update(dq, dt)
    error_vector = np.block([[error_vector],[ err_acc]])
    obstacle_dist = np.block([[obstacle_dist], [obs_dis]])
    err_counter += 1


    # Update drawing
    # -- Manipulator links (skip base links)
    PP = robot.drawing()[:, 2:]  # Skip the 3 base links
    line.set_data(PP[0,:], PP[1,:])
    PPx.append(PP[0,-1])
    PPy.append(PP[1,-1])
    path.set_data(PPx, PPy)
    point.set_data(sigma_endeff[0], sigma_endeff[1])
    # -- Mobile base
    eta = robot.getBasePose()
    veh.set_transform(trans.Affine2D().rotate(eta[2,0]) + trans.Affine2D().translate(eta[0,0], eta[1,0]) + ax.transData)

    return line, veh, path, point

# Run simulation
animation = anim.FuncAnimation(fig, simulate, np.arange(0, 10, dt), 
                                interval=10, blit=True, init_func=init, repeat=True)
plt.show()


# Drawing The Error of each task with respect time
total_time = np.arange(0,err_counter*dt,dt)
print(err_counter , error_vector.shape , obstacle_dist.shape)
ylim =( min(error_vector.min() , error_vector.min() , velocity_limit[0] ) - 0.1 ,max(error_vector.max(),
         error_vector.max(), velocity_limit [1] ) + 0.1)
fig = plt.figure()
ax2 = fig.add_subplot(111, autoscale_on=False , xlim = (0,err_counter*dt), 
                      ylim=ylim)
ax2.set_title('Task-Priority -> '+ str(len(tasks)) + ' Tasks ' + "for  K= " 
              + str(tasks[0].getGain()[0][0]))
ax2.set_xlabel('Time(s)')   
ax2.set_ylabel('Value[1]')
ax2.grid()

# Drawing the error of each task with respect to time
for i in range(num_obstacles):
        ax2.plot(total_time, obstacle_dist[ : ,i ], label="d_" + str(i) 
                 +" `distance to obstacle` ")

for i,task in enumerate(tasks):
    if(task.name == "Joint limit"):
        # Add dashed lines for min and max velocity
        ax2.plot(total_time, error_vector[ : ,i ], label="position of joint 1 " )
        ax2.axhline( y=velocity_limit[0], color='r', linestyle='dashed', )
        ax2.axhline( y=velocity_limit[1],  color='g', linestyle='dashed', )
    elif(task.name not in ["Obstacle avoidance"]):
        ax2.plot(total_time, error_vector[ : ,i ], label=task.name +" error" )
    
# maximum and minimum velocity limits drawing dash
ax2.axhline( y=velocity_limit[0], color='r', linestyle='dashed', )
ax2.axhline( y=velocity_limit[1],  color='g', linestyle='dashed', )
ax2.legend()
plt.show()