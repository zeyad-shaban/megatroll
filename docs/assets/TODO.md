# MegaJaw Asset Checklist

Add media files here with these names so `readme.md` renders cleanly.

## Required README Images

- `hardware-overview.jpg`
  - A clear photo of the real MegaJaw robot.
  - Show the chassis, wheels, phone camera, gripper, and electronics if possible.

- `gazebo-simulation.jpg`
  - A screenshot of the Gazebo world with MegaJaw and the red box bottle stand-ins visible.

- `web-dashboard.jpg`
  - A screenshot of `src/web_interface/index.html` connected to either Gazebo or the real robot.
  - Ideally show camera feed, joystick, target telemetry, and connection status.

- `circuit-cad-overview.jpg`
  - A combined image or collage showing Proteus/circuit simulation and CAD/mechanical design.
  - If this becomes too crowded, split it into separate files and update the README table.

## Recommended Additional Media

- `system-architecture.png`
  - A clean diagram of the Raspberry Pi, STM32, motors, H-bridge, phone camera, gripper, and ROS 2 software flow.

- `bottle-detection-real.jpg`
  - Real camera frame showing the bottle detector locking onto a bottle.

- `proteus-circuit.jpg`
  - Proteus or electronics simulation screenshot.

- `cad-chassis-gripper.jpg`
  - CAD render or screenshot of the chassis and gripper design.

- `demo-cycle.gif`
  - Short GIF of MegaJaw finding a bottle, gripping it, returning, dropping it, and continuing.

## Video

The README currently links to:

```text
https://www.youtube.com/watch?v=nNZg3iLsRV4
```

When the final project walkthrough is published, replace the YouTube video ID in `readme.md` with the real one. The thumbnail image is generated from the YouTube video ID, so updating the link updates the displayed thumbnail too.

## Folder Guidance

Keep non-ROS portfolio assets outside `src/` when possible:

- `docs/assets/` for README screenshots and diagrams.
- `docs/cad/` for CAD source files and renders.
- `docs/circuit_sim/` for Proteus or circuit simulation files.

This keeps `src/` focused on ROS packages and project code.
