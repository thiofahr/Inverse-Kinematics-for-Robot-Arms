import numpy as np
import math as mt
import matplotlib.pyplot as plt
import matplotlib.animation as anim

# coordinate of end effector
def E(theta1, theta2):
    """Calculate the position of end-effector based on the joint angle"""
    x = L1 * np.cos(np.radians(theta1)) + L2 * np.cos(np.radians(theta1 + theta2))
    y = L1 * np.sin(np.radians(theta1)) + L2 * np.sin(np.radians(theta1 + theta2))
    return x, y

# numerical jacobian
def jacobian(theta1, theta2, dtheta, T):
    """Calculate the change in joint angle based on Jacobian method"""
    # jacobian for theta1
    E_plus = E(theta1+dtheta,theta2)
    E_minus = E(theta1-dtheta,theta2)
    J1 = mt.dist(T,E_minus) - mt.dist(T,E_plus)

    # jacobian for theta2
    E_plus = E(theta1, theta2+dtheta)
    E_minus = E(theta1, theta2-dtheta)
    J2 =  mt.dist(T,E_minus) - mt.dist(T,E_plus)
    return J1, J2

# inverse kinematics
def invkinematic(theta1, theta2, dtheta, target):
    """
    Applying inverse kinematics process for 
    finding the target coordinate
    """   
    # initial position of q2
    q2 = list(E(theta1, theta2))
    array_q2 = [q2]

    # initial position of q1
    q1 = [L1*np.cos(np.radians(theta1)), L1*np.sin(np.radians(theta1))]
    array_q1 = [q1]

    while mt.dist(target,q2) > tol:
        # get the angle change based on jacobian method
        J1, J2 = jacobian(theta1, theta2, dtheta, target)

        # update the joint angle
        theta1 += J1
        theta2 += J2

        # get the new coordinate of q1 and q2
        q1 = [L1*np.cos(np.radians(theta1)), L1*np.sin(np.radians(theta1))]
        q2 = E(theta1, theta2)

        # add the new coordinate to the list
        array_q1.append(q1)
        array_q2.append(q2)
    
    return array_q1,array_q2


# main code
# target coordinate
target = [-20,20]

# arms length
res = round(np.sqrt(target[0]**2 + target[1]**2), 1) # resultant of the target coordinate
L1 = res / 2 
L2 = res / 2

# tolerance  
tol = 0.5

# initial joints angle
theta1 = 0
theta2 = 0

# change of angles
dtheta = 1

# initial position of q1
q1 = [L1*np.cos(np.radians(theta1)), L1*np.sin(np.radians(theta1))]
array_q1 = [q1]

# initial position of q2
q2 = list(E(theta1, theta2))
array_q2 = [q2]

while mt.dist(target,q2) > tol:
    # get the angle change based on jacobian method
    J1, J2 = jacobian(theta1, theta2, dtheta, target)

    # update the joint angle
    theta1 += J1
    theta2 += J2

    # update the q1 and q2 coordinate
    q1 = [L1*np.cos(np.radians(theta1)), L1*np.sin(np.radians(theta1))]
    q2 = E(theta1, theta2)

    # add the new coordinate to the list
    array_q1.append(q1)
    array_q2.append(q2)

# setting the figure
fig, ax = plt.subplots()
ax.set_aspect('equal')

# plot configuration
line1, = ax.plot([], [], 'o-', color="C0" , lw=2)           # arm 1
line2, = ax.plot([], [], 'o-', color="C0", lw=2)            # arm 2
path, = ax.plot([], [], '--', color="C0", lw=2, alpha=0.5)  # trajectory of end effector

# axis configuration
ax.set_xlim([-30, 30])
ax.set_ylim([-10, 30])
ax.set_ylabel("y [cm]")
ax.set_xlabel("x [cm]")

# plotting the target
ax.plot(target[0], target[1], "rx")                                         # plot
ax.text(target[0],target[1] + 2, f"({target[0]},{target[1]})", ha="center") # add text showing the coordinate

# animation
def init():
    line1.set_data([], [])
    line2.set_data([], [])
    path.set_data([], [])
    return line1, line2, path

def update(frame):
    # accessing the coordinate for each joint
    x0, y0 = 0, 0
    x1, y1 = array_q1[frame]
    x2, y2 = array_q2[frame]

    # plotting the arms position
    line1.set_data([x0, x1], [y0, y1])
    line2.set_data([x1, x2], [y1, y2])
    path.set_data([coord[0] for coord in array_q2[:frame+1]], [coord[1] for coord in array_q2[:frame+1]])
    
    # set the title
    ax.set_title(f"Two-Arms Robot (loop {frame})")
    return line1, line2, path

ani = anim.FuncAnimation(fig, update, frames=len(array_q1), init_func=init, blit=True, repeat=False)
plt.grid(True)
ani.save('two-arms-robot.gif', writer='pillow', fps=24)
plt.show()