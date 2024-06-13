import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D

# Parameters
ball_diameter = 2.5  # cm
ball_mass = 20.8 # g
rod_length = 40.0  # cm
# angle = 0  # angle between rods at pivot in degrees
gravity = 9.81  # m/s^2
rod_separation = 20.0 # cm
pivot_gap = 2.0 # cm
angle = 2.0 # degrees

# Initial conditions
position = np.array([0.0, 0.0, 0.0])  # meters
velocity = np.array([0.0, 0.0, 0.0])  # m/s
time = 0.0  # seconds
dt = 0.01  # time step for the simulation

# Create a new figure and axis
fig, ax = plt.subplots()

# Draw the rods
top_rod = Line2D([0, rod_length], [pivot_gap, rod_separation / 2])
bottom_rod = Line2D([0, rod_length], [-pivot_gap, -rod_separation / 2])
ax.add_line(top_rod)
ax.add_line(bottom_rod)

# Create the ball
ball = patches.Circle((0, 0), ball_diameter / 2, fc='b')
ax.add_patch(ball)

# Set the x and y limits to fix the animation frame
screen_diameter=50
ax.set_xlim(-screen_diameter, screen_diameter)
ax.set_ylim(-screen_diameter, screen_diameter)

# Set the aspect ratio to be equal
ax.set_aspect('equal')

def update(frame):
    global position, velocity
    global angle
    global ball_diameter, ball_mass, rod_length, rod_separation, pivot_gap
    global gravity
    global ball, ani

    # Calculate forces
    ratio = (np.sin(np.deg2rad(angle)) * max(position[0], 1))
    grad = ratio + 1
    fg = np.array([-gravity * ball_mass, -gravity * ball_mass, 0.0])
    fr = np.array([grad * gravity * ball_mass, gravity * ball_mass, 0.0])

    # Calculate acceleration
    acceleration = fg + fr

    # Update velocity and position
    velocity += acceleration * dt
    position += velocity * dt

    # hard-code X position of ball to 0 as a simulation of the backboard
    position[0] = 0.0 if position[0] < 0 else position[0]

    # Update the position of the ball
    ball.center = position

    # Calculate the separation between the rods at the position of the ball
    ball_separation = ratio * 2

    # Redraw the rods at the new separation
    # ax.plot([rod_endpoints[0][0], rod_endpoints[1][0]], [rod_endpoints[0][1], rod_endpoints[1][1]], 'k')  # top rod
    # ax.plot([rod_endpoints[0][0], rod_endpoints[1][0]], [-rod_y, -rod_y], 'k')  # bottom rod

    # Check if the ball has fallen off the track
    if ball_separation > ball_diameter:
        print("The ball has fallen off the track.")
        ani.event_source.stop()
        sys.exit()

# Create the animation
ani = FuncAnimation(fig, update, frames=range(100), interval=100)

# Show the plot
plt.show()