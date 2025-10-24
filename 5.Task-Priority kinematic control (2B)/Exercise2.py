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

W = np.diag([1,1,1,1,1])*0.01# Weight matrix
velocity_vector = np.zeros((0,5))
# Desired end-effector position
sigma_endeff = np.zeros((2,1)) 
# Desired end-effector configuration
sigma_config = np.block([[sigma_endeff], [0]]).reshape(3,1) 

velocity_limit = [0,0] # Joint velocity limits
# Task definition
obstacle_tasks = []
JointLimit_link = 3
tasks = [ 
            #  JointLimit("Joint limit", velocity_limit, velo_thershold = [0.05  , 0.1] , link = JointLimit_link ),
            #  Position2D("End-effector position", np.array([1.0, 0.5]).reshape(2,1), 5),
           Configuration2D("End-effector configuration", sigma_config.reshape(3,1) , link = 5),
        ] 
num_obstacles = len(obstacle_tasks)
err_counter = 0 # Error counter
error_vector = np.zeros((0 ,len(tasks))) # Error vector
ore_error =[]
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
index = 0 
def init():
    global tasks , sigma_endeff , error_vector , index
    line.set_data([], [])
    path.set_data([], [])
    point.set_data([], [])

    sigma_endeff = np.random.uniform(-2, 2, (2, 1))
     # Desired end-effector configuration
    sigma_config = np.block([[sigma_endeff], [np.pi/2]]).reshape(3,1)
    # set desired values for each task in each iteration
    # endeffector task and configuration task 
    sigma_endeff_array = np.array([[1,1.2],[-1,-1.2],[1.8,0],[0,0.8],[1.8,1.8],[0,0],[0,0.9],[1,0],[-1.2,0],[0,-1.2]])
    sigma_endeff = sigma_endeff_array[index].reshape(2,1)
    sigma_config = np.block([[sigma_endeff], [0]]).reshape(3,1)
    for task in tasks:
        if(task.name == "End-effector position"):
            task.setDesired(sigma_endeff)
        elif(task.name == "End-effector configuration"):
            task.setDesired(sigma_config)
    index += 1
    if index == len(sigma_endeff_array):
        index = 0
    return line, path, point


# Simulation loop
def simulate(t):
    global tasks
    global robot , err_counter , obstacle_dist
    global PPx, PPy  , sigma_endeff , error_vector , ore_error , velocity_vector
    
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
            dq = dq + W_DLS(Jbar,0.1,W)@( task.getFeedforwardVelocity() +
                        task.getGain()@task.getError() - J@dq)   
            P = P - np.linalg.pinv(Jbar)@Jbar # Update null-space projector
        new_error = np.linalg.norm(task.getError()) #error accumulation

        if task.name == "Joint limit":
            new_error = robot.getJointPos(JointLimit_link-1) # get joint position and store
            err_acc = np.block([ err_acc , new_error]) # Accumulate velocity  
        elif task.name == "Obstacle avoidance":
            # get distance to obstacle and store
            obs_dis = np.block([ obs_dis , task.distance_to_obstacle()])
        elif task.name == "End-effector configuration":
            pose_error = np.linalg.norm(task.getError()[0:2,:]) #error accumulation    
            ore_error.append(np.linalg.norm(task.getError()[2,0]))  # Accumulate orientation error
            err_acc = np.block([ err_acc  ,  pose_error]) # Accumulate velocity 
        else:
            err_acc = np.block([ err_acc , new_error]) # Accumulate velocity 
        
    # Update robot
    robot.update(dq, dt)
    error_vector = np.block([[error_vector],[ err_acc]])
    obstacle_dist = np.block([[obstacle_dist], [obs_dis]])
    err_counter += 1

    velocity_vector = np.block([[velocity_vector],[dq.T]])

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
    veh.set_transform(trans.Affine2D().rotate(eta[2,0]) + trans.Affine2D().translate(eta[0,0], eta[1,0]) + ax.transData)

    return line, veh, path, point

# Run simulation
animation = anim.FuncAnimation(fig, simulate, np.arange(0, 10, dt), 
                                interval=10, blit=True, init_func=init, repeat=True)
plt.show()

# Drawing The Error of each task with respect time
total_time = np.arange(0,err_counter*dt,dt)

ylim =( min(error_vector.min() , error_vector.min() , velocity_limit[0] ) - 0.1 ,max(error_vector.max(),
         error_vector.max(), velocity_limit [1] ) + 0.1)
fig , ax = plt.subplots(1,2 , figsize=(10,5))

# ax[0] = fig.add_subplot(111, autoscale_on=False , xlim = (0,err_counter*dt), 
#                       ylim=ylim)
ax[0].set_autoscale_on(False)
ax[0].set_xlim(0,err_counter*dt)
ax[0].set_ylim(ylim)
ax[0].set_title('Task-Priority -> '+ str(len(tasks)) + ' Tasks ' + "for  K= " 
              + str(tasks[0].getGain()[0][0]))
ax[0].set_xlabel('Time(s)')   
ax[0].set_ylabel('error[1]')
ax[0].grid()

# Drawing the error of each task with respect to time
for i in range(num_obstacles):
        ax[0].plot(total_time, obstacle_dist[ : ,i ], label="d_" + str(i) 
                 +" `distance to obstacle` ")

for i,task in enumerate(tasks):

    if(task.name == "End-effector configuration"):
        ax[0].plot(total_time, error_vector[ : ,i ], label="position of end-effector " )
        ax[0].plot(total_time, ore_error, label="orientation of end-effector " )

    elif(task.name == "Joint limit"):
        # Add dashed lines for min and max velocity
        ax[0].plot(total_time, error_vector[ : ,i ], label="position of joint 1 " )
        ax[0].axhline( y=velocity_limit[0],  color='r', linestyle='dashed', )
        ax[0].axhline( y=velocity_limit[1],  color='g', linestyle='dashed', )

    elif(task.name not in ["Obstacle avoidance"]):
        ax[0].plot(total_time, error_vector[ : ,i ], label=task.name +" error" )
    
# maximum and minimum velocity limits drawing dash
# ax[0].axhline( y=velocity_limit[0],  color='r', linestyle='dashed', )
# ax[0].axhline( y=velocity_limit[1],  color='g', linestyle='dashed', )
# ax[0].legend()

# Drawing the velocity of each DOF with respect to time
ylim =( min(velocity_vector.min() , velocity_vector.min()  ) - 0.1 ,max(velocity_vector.max(),
         velocity_vector.max(), ) + 0.1)
ax[1].set_autoscale_on(False)
ax[1].set_xlim(0,err_counter*dt)
ax[1].set_ylim(ylim)
ax[1].set_title('Velocity of each DOF')
ax[1].set_xlabel('Time(s)')   
ax[1].set_ylabel('Velocity')
ax[1].grid()

for i in range(5):
    ax[1].plot(total_time, velocity_vector[ : ,i ], label="vel" + str(i+1) )

ax[1].legend()

plt.show()