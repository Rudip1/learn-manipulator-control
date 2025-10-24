import numpy as np
import matplotlib.pyplot as plt
import os

# Always use the path where this script is located
base_path = os.path.dirname(os.path.abspath(__file__))

# Load from same folder as this script
a_load = np.load(os.path.join(base_path, 'rotate and move forward.npy'))
b_load = np.load(os.path.join(base_path, 'rotate then move forward.npy'))
c_load = np.load(os.path.join(base_path, 'move forward then rotate.npy'))

print("'rotate and move forward.npy:",a_load.shape)
print("rotate then move forward.npy:", b_load.shape)
print("move forward then rotate.npy:", c_load.shape)

fig = plt.figure()
ax = fig.add_subplot(111)

ax.plot(a_load[:,0], a_load[:,1], label='rotate and move(EE)' , color='r' , linestyle='dashed' , lw=1)
ax.plot(a_load[:,2], a_load[:,3],label='rotate and move(Base)' , color='r', lw=1)

ax.plot(b_load[:,0], b_load[:,1] , label='rotate then move(EE)' , color='g' , linestyle='dashed', lw=1)
ax.plot(b_load[:,2], b_load[:,3],label='rotate then move(Base)', color='g', lw=1)

ax.plot(c_load[:,0], c_load[:,1] , label='move forward(EE)' , color='b' , linestyle='dashed', lw=1)
ax.plot(c_load[:,2], c_load[:,3],label='move forward(Base)',color='b', lw=1 )

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('End-effector and Base Position')
ax.grid()
ax.legend()

plt.show()
