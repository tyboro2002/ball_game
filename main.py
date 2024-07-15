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
SIZE_INCREMENT = 0.02  # Increase in ball radius on collision

# Number of balls
NUM_BALLS = 1

# Initialize balls
balls = []
for _ in range(NUM_BALLS):
    ball = {
        'x': np.random.uniform(-WIDTH / 2 + RADIUS, WIDTH / 2 - RADIUS),
        'y': np.random.uniform(-HEIGHT / 2 + RADIUS, HEIGHT / 2 - RADIUS),
        'vx': np.random.uniform(-INITIAL_VELOCITY, INITIAL_VELOCITY),
        'vy': np.random.uniform(-INITIAL_VELOCITY, INITIAL_VELOCITY),
        'radius': BALL_RADIUS
    }
    balls.append(ball)

# Setup the figure and axis with a black background
fig, ax = plt.subplots()
fig.patch.set_facecolor('black')
ax.set_xlim(-WIDTH / 2, WIDTH / 2)
ax.set_ylim(-HEIGHT / 2, HEIGHT / 2)
ax.set_facecolor('black')

# Draw circle
circle = plt.Circle((0, 0), RADIUS, fill=False, color='orange')
ax.add_artist(circle)

# Draw balls
ball_patches = []
for ball in balls:
    ball_patch = plt.Circle((ball['x'], ball['y']), ball['radius'], color='red')
    ax.add_patch(ball_patch)
    ball_patches.append(ball_patch)

ax.set_xticks([]), ax.set_yticks([])
ax.axis('off')  # Remove axes


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

    # Check collision with other balls
    for other_ball in balls:
        if other_ball != ball:
            dx = ball['x'] - other_ball['x']
            dy = ball['y'] - other_ball['y']
            distance_between_centers = np.sqrt(dx ** 2 + dy ** 2)

            if distance_between_centers < ball['radius'] + other_ball['radius']:
                # Normal vector along the line connecting the centers
                normal = np.array([dx, dy]) / distance_between_centers

                # Relative velocity
                rel_velocity = np.array([ball['vx'] - other_ball['vx'], ball['vy'] - other_ball['vy']])
                rel_vel_along_normal = np.dot(rel_velocity, normal)

                # Elastic collision response
                if rel_vel_along_normal < 0:  # Check if balls are moving towards each other
                    # Calculate impulse
                    impulse = (1 + DAMPING) * rel_vel_along_normal / (1 / ball['radius'] + 1 / other_ball['radius'])

                    # Update velocities
                    ball['vx'] -= impulse * normal[0] / ball['radius']
                    ball['vy'] -= impulse * normal[1] / ball['radius']
                    other_ball['vx'] += impulse * normal[0] / other_ball['radius']
                    other_ball['vy'] += impulse * normal[1] / other_ball['radius']


def update(frame):
    # Apply gravity
    for ball in balls:
        ball['vy'] += GRAVITY

    # Move the balls
    for ball in balls:
        ball['x'] += ball['vx']
        ball['y'] += ball['vy']

    # Check collisions
    for ball in balls:
        check_collision(ball)

    # Cap ball positions to stay within the circle
    for ball in balls:
        distance_from_origin = np.sqrt(ball['x'] ** 2 + ball['y'] ** 2)
        max_distance = RADIUS - ball['radius']
        if distance_from_origin > max_distance:
            direction = np.array([ball['x'], ball['y']]) / distance_from_origin
            ball['x'] = direction[0] * max_distance
            ball['y'] = direction[1] * max_distance

    # Update ball positions
    for ball, patch in zip(balls, ball_patches):
        patch.set_center((ball['x'], ball['y']))

    return ball_patches


# Create animation
ani = animation.FuncAnimation(fig, update, frames=3000, interval=30, blit=True)

# Save animation
ani.save('bouncing_balls_with_gravity_and_collision.mp4', writer='ffmpeg')
