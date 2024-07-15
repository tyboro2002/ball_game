import random

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import pygame
import os

from moviepy.audio.io.AudioFileClip import AudioFileClip
from moviepy.editor import ImageSequenceClip, concatenate_audioclips, VideoFileClip
from moviepy.video.VideoClip import VideoClip
from moviepy.video.compositing.concatenate import concatenate_videoclips

# Initialize pygame
pygame.mixer.init()

# Define constants
WIDTH, HEIGHT = 8, 8
RADIUS = 3.5
BALL_RADIUS = 0.1
INITIAL_VELOCITY = 0.1
GRAVITY = -0.002  # Adjust this value for realistic gravity
DAMPING = 0.9  # Energy loss on bounce
SIZE_INCREMENT = 0.1  # Increase in ball radius on collision
FRAMES = 300
SOUND_PATH = 'assets/sounds/chime-sound.mp3'
SOUND_DEVIATION = 0

# Number of balls
NUM_BALLS = 1

def random_color():
    return (random.random(), random.random(), random.random())

# Initialize balls
balls = []
for _ in range(NUM_BALLS):
    ball = {
        'x': np.random.uniform(-WIDTH / 2 + RADIUS, WIDTH / 2 - RADIUS),
        'y': np.random.uniform(-HEIGHT / 2 + RADIUS, HEIGHT / 2 - RADIUS),
        'vx': np.random.uniform(-INITIAL_VELOCITY, INITIAL_VELOCITY),
        'vy': np.random.uniform(-INITIAL_VELOCITY, INITIAL_VELOCITY),
        'radius': BALL_RADIUS,
        'color': random_color()
    }
    balls.append(ball)

# Setup the figure and axis with a black background
fig, ax = plt.subplots()
fig.patch.set_facecolor('black')
ax.set_xlim(-WIDTH / 2, WIDTH / 2)
ax.set_ylim(-HEIGHT / 2, HEIGHT / 2)
ax.set_facecolor('black')
frame_number = 0

# Draw circle
circle = plt.Circle((0, 0), RADIUS, fill=False, color='orange')
ax.add_artist(circle)

# Draw balls
ball_patches = []
for ball in balls:
    ball_patch = plt.Circle((ball['x'], ball['y']), ball['radius'], color=ball['color'])
    ax.add_patch(ball_patch)
    ball_patches.append(ball_patch)

ax.set_xticks([]), ax.set_yticks([])
ax.axis('off')  # Remove axes

# Load collision sound
collision_sound = pygame.mixer.Sound(os.path.join(os.getcwd(), SOUND_PATH))

# Initialize lists to store frame timestamps or indices where collisions occur
collision_frames = []


def check_collision(ball, frame):
    global collision_frames
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

        # Increase radius on collision
        ball['radius'] += SIZE_INCREMENT

        # Play collision sound
        # collision_sound.play()
        collision_frames.append(frame)

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

                    # Play collision sound
                    # collision_sound.play()
                    collision_frames.append(frame)


def update(frame):
    global frame_number
    # Apply gravity
    for ball in balls:
        ball['vy'] += GRAVITY

    # Move the balls
    for ball in balls:
        ball['x'] += ball['vx']
        ball['y'] += ball['vy']

    # Check collisions
    for ball in balls:
        check_collision(ball, frame)

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
        patch.set_radius(ball['radius'])

    # Save frame as PNG file
    # filename = f"temp/frame_{frame_number:03d}.png"
    # fig.savefig(filename, facecolor=fig.get_facecolor())
    # print(frame_number)
    # frame_number += 1

    return ball_patches


# Function to save frames and return their filenames
def save_frames(num_frames):
    for frame in range(num_frames):
        update(frame)
        filename = f"temp/frame_{frame:03d}.png"
        fig.savefig(filename, facecolor=fig.get_facecolor())
        plt.close(fig)  # Close the figure to release resources


# Create animation
# ani = animation.FuncAnimation(fig, update, frames=FRAMES, interval=30, blit=True)

# # Save animation
# ani.save('bouncing_balls_with_gravity_and_collision.mp4', writer='ffmpeg')

save_frames(FRAMES)

print(collision_frames)

# Load frame filenames into moviepy
frame_filenames = [f"temp/frame_{frame:03d}.png" for frame in range(FRAMES)]

# Create VideoClip from image sequence
clips = ImageSequenceClip(frame_filenames, fps=30)

# Load collision sound clip (if necessary)
audio_clip = AudioFileClip(SOUND_PATH)

# Create a list to hold final clips
final_clips = []

# Iterate over each frame and decide whether to add audio or not
for i, frame in enumerate(frame_filenames):
    if i - SOUND_DEVIATION in collision_frames:
        final_clip = clips.subclip(i / 30, (i + 1) / 30).set_audio(audio_clip)
    else:
        final_clip = clips.subclip(i / 30, (i + 1) / 30)
    final_clips.append(final_clip)

# Concatenate clips into final video
final_video = concatenate_videoclips(final_clips)

# Export the final video with synchronized audio
final_video.write_videofile('bouncing_balls_with_sound.mp4', fps=30)

# Clean up: remove temporary PNG files
for filename in frame_filenames:
    os.remove(filename)
