# KUKA iiwa Robot Arm Simulation & Contact Force Sensing

A PyBullet simulation of a KUKA iiwa robot arm performing inverse-kinematics-based
trajectory tracking and a contact-based manipulation task with real-time force sensing.

## What it does

- **Environment setup**: Loads a KUKA iiwa robot arm, a table, and a Lego brick into a
  PyBullet physics simulation (URDF-based).
- **Trajectory tracking**: Generates a 400-point circular trajectory in the robot's
  workspace and drives the end-effector along it using inverse kinematics
  (`calculateInverseKinematics`), visualized as a live debug trail.
- **Contact manipulation**: Commands the robot to push a Lego brick along a linear path,
  while continuously measuring the contact (normal) force between the end-effector and
  the object via `getContactPoints`.
- **Data visualization**: Plots the measured contact force over time using
  NumPy and Matplotlib to analyze the interaction dynamics.

## Tech stack

- Python
- [PyBullet](https://pybullet.org/) (physics simulation, inverse kinematics, contact sensing)
- NumPy
- Matplotlib

## Requirements

```bash
pip install pybullet numpy matplotlib
```

## Assets

This project uses the standard `kuka_iiwa`, `table_square`, and `lego` URDF/mesh assets
that ship with `pybullet_data` / common PyBullet course exercises. Update the paths in
`main()` (`urdf_path_robot`, `urdf_path_table`, `urdf_path_lego`) to match your local
asset locations before running.

## Run

```bash
python robot_force_sensing.py
```

This opens a PyBullet GUI window, runs the circular trajectory, performs the push task,
and displays a force-vs-time plot once the simulation completes.

## Background

Developed as part of coursework in robotics simulation and control
(B.Sc. Autonomy Technologies, FAU Erlangen-Nürnberg).
