"""
Example Genesis Simulation Script

This script can be loaded from the web UI to start a simulation.
It creates a simple scene with a robot and camera.
"""

import genesis as gs
import numpy as np

# Create a headless scene
scene = gs.Scene(show_viewer=False)

# Add ground plane
scene.add_entity(gs.morphs.Plane())

# Add a simple box robot
robot = scene.add_entity(
    gs.morphs.Box(
        size=[0.5, 0.5, 1.0],
        pos=[0.0, 0.0, 0.5],
        quat=[1.0, 0.0, 0.0, 0.0]
    )
)

# Add camera for rendering
camera = scene.add_camera(
    res=(1280, 720),
    pos=[2.0, 2.0, 1.5],
    lookat=[0.0, 0.0, 0.5],
    fov=40,
    GUI=False
)

# Set camera to follow robot
camera.follow_entity(robot, smoothing=0.9)

# Build the scene
scene.build()

print("Genesis simulation scene created successfully!")
print(f"Scene has {len(scene.entities)} entities")
print(f"Camera resolution: {camera.res}")
