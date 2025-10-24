from lab2_robotics import * # Includes numpy import

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
        # a. Extract z and o.
        z = T[i][0:3, 2]
        o = T[i][0:3, 3]
        
        # b. Check joint type.
        if revolute[i]:
            J1_3 = np.cross(z, (O - o))
            J[:,i] = np.concatenate((J1_3, z), axis=0)
        else:
            J[:,i] = np.concatenate((z, np.zeros(3)), axis=0)
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
        self.dof = len(self.revolute)  # Number of degrees of freedom
        self.q = np.zeros(self.dof).reshape(-1, 1)  # Joint variables initialized to zero
        self.update(0.0, 0.0)  # Initialize transformations

    '''
        Method that updates the state of the robot.

        Arguments:
        dq (Numpy array): a column vector of joint velocities
        dt (double): sampling time
    '''
    def update(self, dq, dt):
        self.q += dq * dt  # Integrates joint velocities over time step
        for i in range(len(self.revolute)):
            if self.revolute[i]:  
                self.theta[i] = self.q[i]  # Updates revolute joints
            else:
                self.d[i] = self.q[i]  # Updates prismatic joints
        self.T = kinematics(self.d, self.theta, self.a, self.alpha)  # Recompute forward kinematics


    ''' 
        Method that returns the characteristic points of the robot.
    '''
    def drawing(self):
        return robotPoints2D(self.T)

    '''
        Method that returns the end-effector Jacobian.
    '''
    def getEEJacobian(self):
        return jacobian(self.T, self.revolute)[:2, :]

    '''
        Method that returns the orientation jacobian Jacobian.
    '''
    def getOrientationJacobian(self, link):
        return self.getLinkJacobian(link)[5, :] # Orientation Jacobian
    '''
        Method that returns the configuration Jacobian.
    '''
    def getConfigurationJacobian(self , link):

        J = np.block([[self.getLinkJacobian(link)[0:2, :].reshape(2,3)], 
                      [self.getOrientationJacobian(link).reshape(1,3)]])
        return J
    '''
        Method that returns the end-effector transformation.
    '''
    def getEETransform(self):
        return self.T[-1]

    '''
        Method that returns the position of a selected joint.

        Argument:
        joint (integer): index of the joint

        Returns:
        (double): position of the joint
    '''
    def getJointPos(self, joint):
        return self.q[joint]
    '''
        Method that returns the transformation of a selected link.

        Argument:
        link (integer): index of the link

        Returns:
        (Numpy array): transformation of the link'''
    def getLinkTransform(self, link):
     
        return self.T[link]
    '''
        Method that returns the Jacobian of a selected link.

        Argument:
        link (integer): index of the link

        Returns:
        (Numpy array): Jacobian of the link'''
    def getLinkJacobian(self, link):
        return jacobianLink(self.T, self.revolute, link)

    '''
        Method that returns number of DOF of the manipulator.
    '''
    def getDOF(self):
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
        
    '''
        Method updating the task variables (abstract).

        Arguments:
        robot (object of class Manipulator): reference to the manipulator
    '''
    def getNmae(self):
        return self.name
    
    def update(self, robot):
        pass

    ''' 
        Method setting the desired sigma.

        Arguments:
        value(Numpy array): value of the desired sigma (goal)
    '''
    def setDesired(self, value):
        self.sigma_d = value

    '''
        Method returning the desired sigma.
    '''
    def getDesired(self):
        return self.sigma_d

    '''
        Method returning the task Jacobian.
    '''
    def getJacobian(self):
        return self.J

    '''
        Method returning the task error (tilde sigma).
    '''    
    def getError(self):
        return self.err
    '''
        Method setting the gain matrix K.

        Arguments:
        K(Numpy array): gain matrix
    '''
    def setGian(self, K):
        self.K = K  
    '''
        Method returning the gain matrix K.
    ''' 
    def getGain(self):
        return self.K  
    ''' 
        Method setting the feedforward velocity.

        Arguments:
        value(Numpy array): value of the feedforward velocity
    '''
    def SetFeedforwardVelocity(self, value):
        self.v_ff = value
    '''
        Method returning the feedforward velocity.
    '''
    def getFeedforwardVelocity(self):
        return self.v_ff
'''
    Subclass of Task, representing the 2D position task.
'''
class Position2D(Task):
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
     
'''
    Subclass of Task, representing the 2D orientation task.
'''
class Orientation2D(Task):
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
'''
    Subclass of Task, representing the 2D configuration task.
'''
class Configuration2D(Task):
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
    
''' 
    Subclass of Task, representing the joint position task.
'''
class JointPosition(Task):
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