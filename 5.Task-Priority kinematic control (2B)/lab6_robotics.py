from lab5_robotics import *
import math
deg90 = np.pi / 2
class MobileManipulator:
    """
    Constructor.

    Arguments:
    d (Numpy array): list of displacements along Z-axis
    theta (Numpy array): list of rotations around Z-axis
    a (Numpy array): list of displacements along X-axis
    alpha (Numpy array): list of rotations around X-axis
    revolute (list of Bool): list of flags specifying if the corresponding joint is a revolute joint
    """

    def __init__(self, d, theta, a, alpha, revolute):
        self.d = d
        self.theta = theta
        self.a = a
        self.alpha = alpha
        self.revolute = revolute

        # List of joint types extended with base joints
        self.revoluteExt = [True, False] + self.revolute

        self.r = 0.2  # Distance from robot centre to manipulator base
        self.dof = len(self.revoluteExt)  # Number of DOF of the system

        # Vector of joint positions (manipulator)
        self.q = np.zeros((len(self.revolute), 1))

        # Vector of base pose (position & orientation)
        self.eta = np.zeros((3, 1))

        # Initialise robot state
        self.update(np.zeros((self.dof, 1)), 0.0)

    """
        Method that updates the state of the robot.

        Arguments:
        dQ (Numpy array): a column vector of quasi velocities
        dt (double): sampling time
    """

    def update(self, dQ, dt):
        # Update manipulator
        self.q += dQ[2:, 0].reshape(-1, 1) * dt
        for i in range(len(self.revolute)):
            if self.revolute[i]:
                self.theta[i] = self.q[i]
            else:
                self.d[i] = self.q[i]

        # Update mobile base pose
        forward_vel = dQ[1, 0]
        angular_vel = dQ[4, 0]
        yaw = self.eta[2, 0]

        # Use one at a time:
        self.forward_rotate(dQ, dt)           # 1. Move forward then rotate
        #self.rotate_forward(dQ, dt)         # 2. Rotate then move forward
        #self.rotateandmove_forward(dQ, dt)  # 3. Rotate and move forward (arc)


    # Base kinematics
        x, y, yaw = self.eta.flatten()
        Tb = self.translation(x, y) @ self.rotation_z(yaw)  # Transformation of the mobile base
        # Tb = self.rotation_z(yaw) @ self.translation(x, y)

        ### Additional rotations performed, to align the axis:
        # Rotate Z +90 (using the theta of the first base joint)
        # Rotate X +90 (using the alpha of the first base joint)
        ## Z now aligns with the forward velocity of the base
        # Rotate X -90 (using the alpha of the second base joint)
        ## Z is now back to vertical position
        # Rotate Z -90 (using the theta of the first manipulator joint)

        # Modify the theta of the base joint, to account for an additional Z rotation
        self.theta[0] -= deg90

        # Combined system kinematics (DH parameters extended with base DOF)
        dExt = np.concatenate([np.array([0, self.r]), self.d])
        thetaExt = np.concatenate([np.array([deg90, 0]), self.theta.reshape(3,)])
        aExt = np.concatenate([np.array([0, 0 ]), self.a])
        alphaExt = np.concatenate([np.array([deg90, -deg90]), self.alpha])

        self.T = kinematics(dExt, thetaExt, aExt, alphaExt, Tb)

    """ 
        Method that returns the characteristic points of the robot.
    """

    def drawing(self):
        return robotPoints2D(self.T)

    """
        Method that returns the end-effector Jacobian.
    """

    def getEEJacobian(self):
        return jacobian(self.T, self.revoluteExt)

    """
        Method that returns the end-effector transformation.
    """

    def getEETransform(self):
        return self.T[-1]

    """
        Method that returns the position of a selected joint.

        Argument:
        joint (integer): index of the joint

        Returns:
        (double): position of the joint
    """

    def getJointPos(self, joint):
        return self.q[joint - 2]

    def getBasePose(self):
        return self.eta

    """
        Method that returns number of DOF of the manipulator.
    """

    def getDOF(self):
        return self.dof

    ###
    def getLinkJacobian(self, link):
        return jacobianLink(self.T, self.revoluteExt, link)

    def getLinkTransform(self, link):
        return self.T[link]
    def getOrientationJacobian(self, link):
        return self.getLinkJacobian(link)[5, :] # Orientation Jacobian
    '''
        Method that returns the configuration Jacobian.
    '''
    def getConfigurationJacobian(self , link):

        J = np.block([[self.getLinkJacobian(link)[0:2, :].reshape(2,5)], 
                      [self.getOrientationJacobian(link).reshape(1,5)]])
        return J
    def get2dbaseTransformation(self):
        x, y, yaw = self.eta.flatten()
        return np.array([[np.cos(yaw), -np.sin(yaw), x],
                         [np.sin(yaw), np.cos(yaw), y],
                         [0, 0, 1]])

    def translation(self, x, y):
        return np.array([[1, 0, 0 , x],
                         [0, 1, 0 , y],
                         [0, 0, 1 , 0],
                         [0, 0, 0 , 1]])
    def rotation_z(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                         [np.sin(theta), np.cos(theta), 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
    
    def rotate_forward(self,dQ , dt): 
     #Update mobile base pose ( rotate then move forward)
        self.eta[2,0] += dQ[0, 0]* dt
        self.eta[0,0] += dQ[1, 0]*np.cos(self.eta[2,0])*dt
        self.eta[1,0] += dQ[1, 0]*np.sin(self.eta[2,0])*dt
        
    def rotateandmove_forward(self,dQ , dt):
        if math.isclose(dQ[0,0], 0.0):
            pass

        else:
            #calculate radius
            self.radius = dQ[1,0]/dQ[0,0]
            self.eta[0,0] += -self.radius*np.sin(self.eta[2,0])  + self.radius*np.sin(dQ[0,0]*dt+(self.eta[2,0]))
            self.eta[1,0] +=  self.radius*np.cos(self.eta[2,0])  - self.radius*np.cos(dQ[0,0]*dt+(self.eta[2,0]))
            self.eta[2,0] +=  dQ[0,0]*dt
        
    def forward_rotate(self,dQ,dt):
         # Update mobile base pose (move forward then rotate)
        self.eta[0,0] += dQ[1, 0]*np.cos(self.eta[2,0])*dt
        self.eta[1,0] += dQ[1, 0]*np.sin(self.eta[2,0])*dt
        self.eta[2,0] += dQ[0, 0]* dt
        
