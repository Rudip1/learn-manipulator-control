import numpy as np # Import Numpy

# Denavit-Hartenberg (DH) Transformation Matrix
def DH(d, theta, a, alpha):
    '''
        Function builds elementary Denavit-Hartenberg transformation matrices 
        and returns the transformation matrix resulting from their multiplication.

        Arguments:
        d (double): displacement along Z-axis
        theta (double): rotation around Z-axis
        a (double): displacement along X-axis
        alpha (double): rotation around X-axis

        Returns:
        (Numpy array): composition of elementary DH transformations
    '''
    # 1. Build matrices representing elementary transformations (based on input parameters).
    T_d=np.array([[1,0,0,0],
                  [0,1,0,0],
                  [0,0,1,d],
                  [0,0,0,1]])
    
    T_theta=np.array([[np.cos(theta),-np.sin(theta),0,0],
                      [np.sin(theta),np.cos(theta),0,0],
                      [0,0,1,0],
                      [0,0,0,1]])
    
    T_a=np.array([[1,0,0,a],
                  [0,1,0,0],
                  [0,0,1,0],
                  [0,0,0,1]])
    
    T_alpha=np.array([[1,0,0,0],
                      [0,np.cos(alpha),-np.sin(alpha),0],
                      [0,np.sin(alpha),np.cos(alpha),0],
                      [0,0,0,1]])
    

    # 2. Multiply matrices in the correct order (result in T).
    T=T_d @ T_theta @ T_a @ T_alpha

    return T

# Forward Kinematics Computation
def kinematics(d, theta, a, alpha):
    '''
        Functions builds a list of transformation matrices, for a kinematic chain,
        descried by a given set of Denavit-Hartenberg parameters. 
        All transformations are computed from the base frame.

        Arguments:
        d (list of double): list of displacements along Z-axis
        theta (list of double): list of rotations around Z-axis
        a (list of double): list of displacements along X-axis
        alpha (list of double): list of rotations around X-axis

        Returns:
        (list of Numpy array): list of transformations along the kinematic chain (from the base frame)
    '''

    #Initialize the base transformation matrix (Identity matrix)
    T = [np.eye(4)]

    # For each set of DH parameters:
    # 1. Compute the DH transformation matrix.
    # 2. Compute the resulting accumulated transformation from the base frame.
    # 3. Append the computed transformation to T.

    for i in range(len(d)):
        Ti = DH(d[i], theta[i], a[i], alpha[i])  # Compute DH transformation
        T.append(T[-1] @ Ti)  # Compute cumulative transformation
    return T

# Compute Jacobian Matrix, Inverse kinematics
def jacobian(T, revolute):
    '''
        Function builds a Jacobian for the end-effector of a robot,
        described by a list of kinematic transformations and a list of joint types.

        Arguments:
        T (list of Numpy array): list of transformations along the kinematic chain of the robot (from the base frame)
        revolute (list of Bool): list of flags specifying if the corresponding joint is a revolute joint

        Returns:
        (Numpy array): end-effector Jacobian
    '''
    # 1. Initialize J and O.
    # 2. For each joint of the robot
    #   a. Extract z and o.
    #   b. Check joint type.
    #   c. Modify corresponding column of J.

    # 1. Initialize J and O.
    J = np.zeros((6,len(T)-1))

    # 2. For each joint of the robot
    for i in range (len(T)-1):
    #   a. Extract z and o.
        if i == 0:
            z = np.array([0,0,1]) #Base Z-axis

        else:
            z = T[i][0:3,2] # Extract the Z-axis from the transformation matrix
        
        o = T[-1][0:3, 3] - T[i][0:3, 3] # Compute position vector

        zero = np.zeros(3)

    #   b. Check joint type.
        if revolute[i]:

    #   c. Modify corresponding column of J.
            J[:,i] = np.concatenate((np.cross(z, o), z)).reshape((6,))
        else:
            J[:,i] = np.concatenate((z,zero)).reshape((6,))
    return J        

# Damped Least-Squares Inverse of Jacobian
def DLS(J, damping):
    '''
        Function computes the damped least-squares (DLS) solution to the matrix inverse problem.

        Arguments:
        A (Numpy array): matrix to be inverted
        damping (double): damping factor

        Returns:
        (Numpy array): inversion of the input matrix
    '''
    # Zeta = J(𝔮).T (J(𝔮)*J(𝔮).T + λ**2 * I)−1 ·xE
    DLS = J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(J.shape[0]))

    return DLS # Implement the formula to compute the DLS of matrix A.

# Extract characteristic points of a robot projected on X-Y plane
def robotPoints2D(T):
    '''
        Function extracts the characteristic points of a kinematic chain on a 2D plane,
        based on the list of transformations that describe it.

        Arguments:
        T (list of Numpy array): list of transformations along the kinematic chain of the robot (from the base frame)
    
        Returns:
        (Numpy array): an array of 2D points
    '''
    P = np.zeros((2,len(T))) # Create an empty array for 2D coordinates

    for i in range(len(T)):
        P[:,i] = T[i][0:2,3] # Extract X and Y positions
    return P