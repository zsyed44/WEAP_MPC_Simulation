# ===============================================================
# Zain Syed, February 2025
# MPC Controller for a Car Following a given path, for WEAP
# I HIGHLY RECOMMEND READING THEORY.md BEFORE READING THIS CODE
# ===============================================================


import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Rectangle

# Simulation parameters
timeStep = 0.1 # time step (seconds), how often our simulation will update
totalSteps = 100 # total simulation steps, how long the simulation will run
horizonLength = 10 # MPC horizon (number of steps), how far ahead the controller plans

# System: double integrator in 2D (point mass model)
# State: [x, y, vx, vy], Input: [ax, ay]
A = np.array([[1, 0, timeStep, 0],
              [0, 1, 0, timeStep],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])
B = np.array([[0.5*timeStep**2, 0],
              [0, 0.5*timeStep**2],
              [timeStep, 0],
              [0, timeStep]])

# MPC cost weights, penalizing deviations from the reference trajectory
stateCost = np.diag([1, 1, 0.6, 0.6])
inputCost = np.diag([0.01, 0.01])
terminalCost = stateCost 

# Predefine a circular reference path.
R0 = 10.0           # radius of the circle
v_des = 1.75         # desired constant speed along the path

# Precompute a reference trajectory that covers the full simulation (plus horizon)
# Each reference state is [x_ref, y_ref, vx_ref, vy_ref].
t_vec = np.array([i * timeStep for i in range(totalSteps + horizonLength + 1)])
ref_traj = np.zeros((totalSteps + horizonLength + 1, 4))
for i, t in enumerate(t_vec):
    # For a circle, arc length s = v_des*t, angle theta = s/R0
    theta = v_des * t / R0
    x_ref = R0 * np.cos(theta)
    y_ref = R0 * np.sin(theta)
    # Tangential velocity (derivatives)
    vx_ref = -v_des * np.sin(theta)
    vy_ref = v_des * np.cos(theta)
    ref_traj[i, :] = np.array([x_ref, y_ref, vx_ref, vy_ref])

# Input constraints (acceleration limits)
u_min = np.array([-1.0, -1.0])
u_max = np.array([1.0, 1.0])

# Simulation history storage
state_history = []
# Start the simulation with an initial state that is offset from the circle
x_current = np.array([R0 + 5, -5, 0, 0])
state_history.append(x_current)

for t in range(totalSteps):
    # Define cvxpy variables for the state and control over the horizon
    x = cp.Variable((4, horizonLength+1))
    u = cp.Variable((2, horizonLength))
    
    cost = 0
    constraints = []
    
    # Initial condition
    constraints += [x[:, 0] == x_current]
    
    # Build cost and dynamics constraints for each step in the horizon
    for k in range(horizonLength):
        ref_state = ref_traj[t + k]  # reference state at time t+k
        cost += cp.quad_form(x[:, k] - ref_state, stateCost) + cp.quad_form(u[:, k], inputCost)
        constraints += [x[:, k+1] == A @ x[:, k] + B @ u[:, k]]
        constraints += [u[:, k] <= u_max, u[:, k] >= u_min]
    
    # Terminal cost at time t+N
    ref_state_terminal = ref_traj[t + horizonLength]
    cost += cp.quad_form(x[:, horizonLength] - ref_state_terminal, terminalCost)
    
    # Solve the MPC optimization problem
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP, warm_start=True)
    
    if prob.status not in ["optimal", "optimal_inaccurate"]:
        print(f"MPC problem at time step {t} did not solve optimally.")
        break
    
    # Extract and apply the first control input
    u_apply = u[:, 0].value
    if u_apply is None:
        u_apply = np.zeros(2)
    
    # Update the state with the applied control input
    x_current = A @ x_current + B @ u_apply
    state_history.append(x_current)

# Convert history to a NumPy array for easy indexing
state_history = np.array(state_history)  # shape: (sim_time+1, 4)

fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-R0 - 5, R0 + 5)
ax.set_ylim(-R0 - 5, R0 + 5)
ax.set_aspect('equal')
ax.set_title("Car Following a Circular Path with MPC")

# Plot the reference path (as a dashed red line)
ax.plot(ref_traj[:, 0], ref_traj[:, 1], 'r--', label="Reference Path")
ax.legend()

# Car drawing parameters (the car will be drawn as a rectangle)
car_length = 1.0
car_width = 0.5

# Create a rectangle patch to represent the car.
# (We will update its position and rotation at each frame.)
car_patch = Rectangle((0, 0), car_length, car_width, fc='blue', ec='black')
ax.add_patch(car_patch)

# Number of frames for the animation (one per simulation step)
sim_steps = state_history.shape[0]

def init():
    car_patch.set_visible(True)
    return car_patch,

def animate(i):
    # Get the current state
    x = state_history[i, 0]
    y = state_history[i, 1]
    vx = state_history[i, 2]
    vy = state_history[i, 3]
    # Estimate orientation from the velocity vector.
    # (If the car is nearly stationary, default to 0.)
    theta = np.arctan2(vy, vx) if np.linalg.norm([vx, vy]) > 1e-3 else 0
    
    # We want to draw the rectangle so that its center is at (x, y).
    # Matplotlib's Rectangle by default is positioned by its lower-left corner.
    # Here we use an affine transform to rotate around the center.
    trans = (plt.matplotlib.transforms.Affine2D()
             .rotate_around(0, 0, theta)
             .translate(x, y)
             + ax.transData)
    car_patch.set_transform(trans)
    # Reset the rectangle's position so that it is centered at (0,0) before the transform.
    car_patch.set_xy((-car_length/2, -car_width/2))
    car_patch.set_width(car_length)
    car_patch.set_height(car_width)
    
    return car_patch,

# Create the animation (adjust the interval as needed; here, 100 ms per frame)
ani = animation.FuncAnimation(fig, animate, frames=sim_steps,
                              init_func=init, blit=True, interval=100, repeat=True)

plt.show()