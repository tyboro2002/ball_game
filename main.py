import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Define constants
WIDTH, HEIGHT = 8, 8
RADIUS = 3.5
BALL_RADIUS = 0.1
INITIAL_VELOCITY = 0.1
GRAVITY = -0.002  # Adjust this value for realistic gravity
DAMPING = 0.9  # Energy loss on bounce

# Setup the figure and axis with a black background
fig, ax = plt.subplots()
fig.patch.set_facecolor('black')
ax.set_xlim(-WIDTH / 2, WIDTH / 2)
ax.set_ylim(-HEIGHT / 2, HEIGHT / 2)
ax.set_facecolor('black')
circle = plt.Circle((0, 0), RADIUS, fill=False, color='orange')
ax.add_artist(circle)
ax.set_xticks([]), ax.set_yticks([])
ax.axis('off')  # Remove axes

# Ball properties
ball1 = {'x': 0, 'y': -RADIUS + BALL_RADIUS, 'vx': INITIAL_VELOCITY, 'vy': INITIAL_VELOCITY, 'radius': BALL_RADIUS}
ball2 = {'x': 0, 'y': RADIUS - BALL_RADIUS, 'vx': -INITIAL_VELOCITY, 'vy': -INITIAL_VELOCITY, 'radius': BALL_RADIUS}

ball1_patch = plt.Circle((ball1['x'], ball1['y']), ball1['radius'], color='red')
ball2_patch = plt.Circle((ball2['x'], ball2['y']), ball2['radius'], color='blue')
ax.add_patch(ball1_patch)
ax.add_patch(ball2_patch)


# Function to check collision with circle and adjust velocity
def check_collision(ball):
    # Check collision with the circle
    dx = ball['x']
    dy = ball['y']
    distance_to_center = np.sqrt(dx ** 2 + dy ** 2)

    if distance_to_center + ball['radius'] >= RADIUS:
        normal = np.array([dx, dy]) / distance_to_center
        velocity = np.array([ball['vx'], ball['vy']])
        velocity_normal = np.dot(velocity, normal)
        new_velocity = velocity - 2 * velocity_normal * normal
        ball['vx'], ball['vy'] = new_velocity * DAMPING
        overlap = ball['radius'] - (distance_to_center - RADIUS)
        ball['x'] += overlap * normal[0]
        ball['y'] += overlap * normal[1]

    # Check collision with the other ball
    dx = ball1['x'] - ball2['x']
    dy = ball1['y'] - ball2['y']
    distance_between_centers = np.sqrt(dx ** 2 + dy ** 2)

    if distance_between_centers < ball1['radius'] + ball2['radius']:
        # Normal vector along the line connecting the centers
        normal = np.array([dx, dy]) / distance_between_centers

        # Relative velocity
        rel_velocity = np.array([ball1['vx'] - ball2['vx'], ball1['vy'] - ball2['vy']])
        rel_vel_along_normal = np.dot(rel_velocity, normal)

        # Elastic collision response
        if rel_vel_along_normal < 0:  # Check if balls are moving towards each other
            # Calculate impulse
            impulse = (1 + DAMPING) * rel_vel_along_normal / (1 / ball1['radius'] + 1 / ball2['radius'])

            # Update velocities
            ball1['vx'] -= impulse * normal[0] / ball1['radius']
            ball1['vy'] -= impulse * normal[1] / ball1['radius']
            ball2['vx'] += impulse * normal[0] / ball2['radius']
            ball2['vy'] += impulse * normal[1] / ball2['radius']


def update(frame):
    # Apply gravity
    ball1['vy'] += GRAVITY
    ball2['vy'] += GRAVITY

    # Move the balls
    ball1['x'] += ball1['vx']
    ball1['y'] += ball1['vy']
    ball2['x'] += ball2['vx']
    ball2['y'] += ball2['vy']

    # Check collisions
    check_collision(ball1)
    check_collision(ball2)

    # Cap ball positions to stay within the circle
    cap_position(ball1)
    cap_position(ball2)

    # Update ball positions and sizes
    ball1_patch.set_center((ball1['x'], ball1['y']))
    ball2_patch.set_center((ball2['x'], ball2['y']))

    return ball1_patch, ball2_patch


def cap_position(ball):
    # Calculate distance of ball from origin
    distance_from_origin = np.sqrt(ball['x'] ** 2 + ball['y'] ** 2)

    # Maximum allowed distance from origin
    max_distance = RADIUS - ball['radius']

    # If ball is beyond max distance, cap its position
    if distance_from_origin > max_distance:
        # Calculate normalized direction vector towards origin
        direction = np.array([ball['x'], ball['y']]) / distance_from_origin

        # Move ball to the boundary
        ball['x'] = direction[0] * max_distance
        ball['y'] = direction[1] * max_distance


# Create animation
ani = animation.FuncAnimation(fig, update, frames=3000, interval=30, blit=True)

# Save animation
ani.save('bouncing_balls_with_gravity.mp4', writer='ffmpeg')
