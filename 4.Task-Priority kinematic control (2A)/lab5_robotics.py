from lab2_robotics import * # Includes numpy import

# Wraps angles into [0, 2pi)
def wrap_angle(angle):
    return angle % (2.0 * np.pi)

def jacobianLink(T, revolute, link): # Needed in Exercise 2
    '''
        Function builds a Jacobian for the end-effector of a robot,
        described by a list of kinematic transformations and a list of joint types.

        Arguments:
        T (list of Numpy array): list of transformations along the kinematic chain of the robot (from the base frame)
        revolute (list of Bool): list of flags specifying if the corresponding joint is a revolute joint
        link(integer): index of the link for which the Jacobian is computed

        Returns:
        (Numpy array): end-effector Jacobian
    '''
    # Code almost identical to the one from lab2_robotics...

    J = np.zeros((6, len(T)-1)) # Empty Jacobian (6 x N) where N - number of joints
    O = T[-1][:3, 3]# End-effector position   

    for i in range(link):
        z = T[i][0:3, 2]         # z-axis of joint i
        o = T[i][0:3, 3]         # position of joint i
        if revolute[i]:
            J[:3, i] = np.cross(z, (O - o))  # Linear velocity
            J[3:, i] = z                    # Angular velocity
        else:
            J[:3, i] = z
            J[3:, i] = np.zeros(3)
    return J

'''
    Class representing a robotic manipulator.
'''
class Manipulator:
    '''
        Constructor.

        Arguments:
        d (Numpy array): list of displacements along Z-axis
        theta (Numpy array): list of rotations around Z-axis
        a (Numpy array): list of displacements along X-axis
        alpha (Numpy array): list of rotations around X-axis
        revolute (list of Bool): list of flags specifying if the corresponding joint is a revolute joint
    '''
    def __init__(self, d, theta, a, alpha, revolute):
        self.d = d
        self.theta = theta
        self.a = a
        self.alpha = alpha
        self.revolute = revolute
        self.dof = len(revolute)  # Number of degrees of freedom
        self.q = np.zeros(self.dof).reshape(-1, 1)  # Joint variables initialized to zero
        self.update(0.0, 0.0)  # Initialize transformations


    def update(self, dq, dt):
        '''
        Method that updates the state of the robot.

        Arguments:
        dq (Numpy array): a column vector of joint velocities
        dt (double): sampling time
        '''
        self.q += dq * dt  # Integrates joint velocities over time step
        for i in range(len(self.revolute)):
            if self.revolute[i]:  
                self.theta[i] = self.q[i]  # Updates revolute joints
            else:
                self.d[i] = self.q[i]  # Updates prismatic joints
        self.T = kinematics(self.d, self.theta, self.a, self.alpha)  # Recompute forward kinematics


    def drawing(self):
        ''' 
        Method that returns the characteristic points of the robot.
        '''
        return robotPoints2D(self.T)


    def getEEJacobian(self):
        '''
        Method that returns the end-effector Jacobian.
        '''
        return jacobian(self.T, self.revolute)[:2, :]


    def getOrientationJacobian(self, link):
        '''
        Method that returns the orientation jacobian Jacobian.
        '''
        return self.getLinkJacobian(link)[5, :] # Orientation Jacobian


    def getConfigurationJacobian(self , link):
        '''
        Method that returns the configuration Jacobian.
        '''

        J = np.block([[self.getLinkJacobian(link)[0:2, :].reshape(2,3)], 
                      [self.getOrientationJacobian(link).reshape(1,3)]])
        return J
    

    def getEETransform(self):
        '''
        Method that returns the end-effector transformation.
        '''
        return self.T[-1]


    def getJointPos(self, joint):
        '''
        Method that returns the position of a selected joint.

        Argument:
        joint (integer): index of the joint

        Returns:
        (double): position of the joint
         '''
        return self.q[joint]
    


    def getLinkTransform(self, link):
        '''
        Method that returns the transformation of a selected link.

        Argument:
        link (integer): index of the link

        Returns:
        (Numpy array): transformation of the link
        '''
     
        return self.T[link]
    


    def getLinkJacobian(self, link):
        '''
        Method that returns the Jacobian of a selected link.

        Argument:
        link (integer): index of the link

        Returns:
        (Numpy array): Jacobian of the link
        '''
        return jacobianLink(self.T, self.revolute, link)


    def getDOF(self):
        '''
        Method that returns number of DOF of the manipulator.
        '''
        return self.dof

'''
    Base class representing an abstract Task.
'''
class Task:
    '''
        Constructor.

        Arguments:
        name (string): title of the task
        desired (Numpy array): desired sigma (goal)
    '''
    def __init__(self, name, desired , link , v_ff , K ):
        self.name = name # task title
        self.sigma_d = desired # desired sigma
        self.v_ff = v_ff # feedforward velocity
        self.K = K   # gain matrix
        self.link = link
        self.activate_flag = 1 # flag indicating if the task is active
        
    '''
        Method updating the task variables (abstract).

        Arguments:
        robot (object of class Manipulator): reference to the manipulator
    '''

    def isTaskActive(self):
        return True

    def getName(self):
        return self.name
    
    def update(self, robot):
        pass


    def setDesired(self, value):
        ''' 
        Method setting the desired sigma.

        Arguments:
        value(Numpy array): value of the desired sigma (goal)
        '''
        self.sigma_d = value


    def getDesired(self):
        '''
        Method returning the desired sigma.
      '''
        return self.sigma_d


    def getJacobian(self):
        '''
        Method returning the task Jacobian.
        '''
        return self.J

   
    def getError(self):
        '''
        Method returning the task error (tilde sigma).
        ''' 
        return self.err

    def setGain(self, K):
        '''
        Method setting the gain matrix K.

        Arguments:
        K(Numpy array): gain matrix
        '''
        self.K = K  
 
    def getGain(self):
        '''
        Method returning the gain matrix K.
        '''
        return self.K  

    def SetFeedforwardVelocity(self, value):
        ''' 
        Method setting the feedforward velocity.

        Arguments:
        value(Numpy array): value of the feedforward velocity
        '''
        self.v_ff = value

    def getFeedforwardVelocity(self):
        '''
        Method returning the feedforward velocity.
        '''
        return self.v_ff

class Position2D(Task):
    '''
    Subclass of Task, representing the 2D position task.
    '''
    def __init__(self, name, desired , link=3  , v_ff = np.zeros((2,1)) , K = np.eye(2) ):
        super().__init__(name, desired , link , v_ff , K)
        self.J = np.zeros((2,3)) # Initialize Jacobian
        self.err = np.zeros((2,1))  # Initialize error vector

    def update(self, robot):
        # self.J   = robot.getEEJacobian()
        # self.err = self.sigma_d - robot.get ()[0:2,3].reshape(2,1)

        self.J = robot.getLinkJacobian(self.link)[0:2, :]    # Update task Jacobian
        desired = self.getDesired().reshape(2,1)
        expected = robot.getLinkTransform(self.link)[0:2,-1].reshape(2,1)
        self.err = desired - expected # Update task error
     

class Orientation2D(Task):
    '''
    Subclass of Task, representing the 2D orientation task.
    '''
    def __init__(self, name, desired , link=3 , v_ff = np.zeros((1,1)) , K = np.eye(1) ):
        super().__init__(name, desired , link , v_ff , K)
        self.J = np.zeros((1,3))
        self.err = np.zeros((1,1)) # Initialize with proper dimensions
        
    def update(self, robot):

        self.J = robot.getOrientationJacobian(self.link).reshape(1,3) # Update task Jacobian
        T1 = robot.getLinkTransform(self.link)[1,0]
        T0 = robot.getLinkTransform(self.link)[0,0]
        sigma = np.arctan2(T1,T0).reshape(1,1)
        self.err = np.array(self.sigma_d - sigma ).reshape(1,1)# Update task error

class Configuration2D(Task):
    '''
    Subclass of Task, representing the 2D configuration task.
    '''
    def __init__(self, name, desired , link=3 , v_ff = np.zeros((3,1)) , K = np.eye(3)):
        super().__init__(name, desired , link , v_ff , K)
        self.J = np.zeros((3,3))
        self.err = np.zeros((1,1)) # Initialize with proper dimensions
        
    def update(self, robot):
        self.J = robot.getConfigurationJacobian(self.link).reshape(3,3)# Update task Jacobian
        T1 = robot.getLinkTransform(self.link)[1,0]
        T0 = robot.getLinkTransform(self.link)[0,0]
        sigma_or= np.arctan2(T1,T0).reshape(1,1)
        sigma_pos =robot.getLinkTransform(self.link)[0:2,-1].reshape(2,1)

        sigma = np.block([[sigma_pos], [sigma_or]]).reshape(3,1)
        self.err =  self.sigma_d - sigma # Update task error
    

class JointPosition(Task):
    ''' 
    Subclass of Task, representing the joint position task.
    '''
    def __init__(self, name, desired, link=1, v_ff=None, K=None):
        """
        Constructor for JointPosition Task.

        Arguments:
        name (string): Name of the task
        desired (numpy array): Desired joint position
        link (integer): Index of the joint (default is 1)
        v_ff (numpy array): Feedforward velocity vector (default is 0)
        K (numpy array): Gain matrix (default is identity)
        """
        # Ensure correct dimensions for feedforward velocity and gain matrix
        if v_ff is None:
            v_ff = np.zeros((1,1))  # Default feedforward velocity (1x1)
        if K is None:
            K = np.eye(1)  # Default gain matrix (1x1)

        super().__init__(name, desired, link, v_ff, K)  # Call parent constructor
        self.link = link  # Joint index
        self.J = np.zeros((1, 3))  # Initialize Jacobian (1x3)
        self.err = np.zeros((1, 1))  # Initialize error (1x1)

    def update(self, robot):
        """
        Update method to compute Jacobian and error for joint position control.

        Arguments:
        robot (Manipulator): The manipulator object
        """
        # Set Jacobian based on which joint is being controlled
        self.J = np.zeros((1, 3))  # Reset Jacobian
        if self.link < 3:  # Ensure within valid joint range
            self.J[0, self.link - 1] = 1  # Assign 1 to the correct joint

        # Compute error between desired and current joint position
        self.err = self.sigma_d - robot.getJointPos(self.link - 1)

class Obstacle2D(Task):

    def __init__(self, name, obstacle_pose , obstacle_th , link =3):
        #super().__init__(name , obstacle_pose )
        super().__init__(name, obstacle_pose, link, np.zeros((2,1)), np.eye(2))
        self.J =np.zeros((2,3))# Initialize with proper dimensions
        self.err =np.zeros((2,1)) # Initialize with proper dimensions
        self.obstacle_pose = obstacle_pose
        self.obstacle_th   = obstacle_th
        self.activation_th = obstacle_th[0]
        self.deactivation_th = obstacle_th[1]
        self.active_flag   = 0
        self.link = link 
        self.v_ff = np.zeros((2,1))
        self.K = np.eye(2)

    def update(self, robot):

        self.J = robot.getLinkJacobian(self.link)[0:2, :] # jacobian 
        end_effector_pose = robot.getEETransform()[0:2,3].reshape(2,1) # end effector pose
        self.distance = end_effector_pose  - self.obstacle_pose # distance between end effector and obstacle pose  
        dir_vector_len = np.linalg.norm(self.distance) # length of the distance vector
        self.err = self.distance / dir_vector_len # update task error 
     
        if(self.active_flag == 0 and dir_vector_len < self.obstacle_th[0]):
            self.active_flag = 1 # activate the task

        elif(self.active_flag == 1 and dir_vector_len > self.obstacle_th[1]):
            self.active_flag = 0 # deactivate the task
        
    def isTaskActive(self): 
        # check if the task is active
        if(self.active_flag == 0):
            return False
        else:
            return True
    def distance_to_obstacle(self):
        # get distance to obstacle
        self.dis_to_obs = np.linalg.norm(self.distance ) -  self.activation_th
        return self.dis_to_obs


class JointLimit(Task):
    def __init__(self, name, velo_limit, velo_threshold, link=1):
        v_ff = np.zeros((1,1))
        K = np.eye(1)
        desired = np.zeros((1,1))  # Placeholder; not used for inequality
        super().__init__(name, desired, link, v_ff, K)

        self.J = np.zeros((1, 3))
        self.err = np.zeros((1, 1))
        self.q_min = velo_limit[0]
        self.q_max = velo_limit[1]
        self.activation_th = velo_threshold[0]
        self.deactivation_th = velo_threshold[1]
        self.active_flag = 0
        self.link = link

    def update(self, robot):
        ''' update the task state '''
        self.J   =  np.array([1,0,0]).reshape((1,3))  # jacobian 
        self.err =  np.array([5]).reshape(1,1) # Initialize with proper dimensions

        qi =  robot.getJointPos(self.link-1) # joint position

        if(self.active_flag == 0 and qi >= self.q_max - self.activation_th):
            self.active_flag = -1
            
        elif(self.active_flag == 0 and qi <= self.q_min + self.activation_th):
            self.active_flag = 1

        elif(self.active_flag == -1 and qi <= self.q_max - self.deactivation_th):
            self.active_flag = 0

        elif(self.active_flag == 1 and qi >= self.q_min + self.deactivation_th):
            self.active_flag = 0

        self.err = self.err*self.active_flag # update task error

    def isTaskActive(self): 
        ''' check if the task is active '''
        if(self.active_flag == 0):
            return False
        else:
            return True