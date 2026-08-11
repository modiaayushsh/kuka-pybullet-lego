import pybullet as p
import pybullet_data
import time
import math
import matplotlib.pyplot as plt
import numpy as np

"""Before running the script, make sure the file is sotred in the correct folder and the paths to the URDF files are adjusted accordingly."""

def init_simulation():
    """Initializes the PyBullet simulation."""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")


def init_environment(urdf_path_robot, urdf_path_table, urdf_path_lego):
    """Loads the robot and returns (id, joints, tcp-index)."""
    robot_id = p.loadURDF(urdf_path_robot, [0, 0, 0], useFixedBase=True)

    # Consider only movable joints
    joint_indices = [i for i in range(p.getNumJoints(robot_id)) if p.getJointInfo(robot_id, i)[2] != p.JOINT_FIXED]
    tcp_link_index = joint_indices[-1]  # last active link as TCP

    table_id = p.loadURDF(urdf_path_table, [0.6, 0.1, 0.01], useFixedBase=True)
    lego_id = p.loadURDF(urdf_path_lego, [0.6, 0, 0.8])

    return robot_id, joint_indices, tcp_link_index, table_id, lego_id


def generate_circle_trajectory(center, radius, num_points):
    """Generates points on a circle in the XY-plane."""
    trajectory = []
    for i in range(num_points + 1):
        angle = 2 * math.pi * i / num_points
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        z = center[2]
        trajectory.append([x, y, z])
    return trajectory


def move_robot_along_trajectory(robot_id, joint_indices, tcp_link_index, trajectory, sleep_dt=1.0 / 240):
    """Moves the robot along the given trajectory."""
    last_pos = None
    target_ori = p.getQuaternionFromEuler([math.pi, 0, 0])  # TCP points downward

    for target_pos in trajectory:
        joint_angles = p.calculateInverseKinematics(
            robot_id,
            tcp_link_index,
            target_pos,
            targetOrientation=target_ori,
            maxNumIterations=200
        )

        for j, angle in zip(joint_indices, joint_angles):
            p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, targetPosition=angle, force=200)

        p.stepSimulation()
        time.sleep(sleep_dt)

        # Draw line
        state = p.getLinkState(robot_id, tcp_link_index)
        tcp_pos = state[4]
        if last_pos is not None:
            p.addUserDebugLine(last_pos, tcp_pos, [1, 0, 0], 2, 0)
        last_pos = tcp_pos


def push_lego(robot_id,
              joint_indices,
              tcp_link_index,
              lego_id,
              dt=1.0 / 240):
    """Pushes the lego brick and measures contact forces between TCP and lego brick and returns the contact forces at each timestep."""

    lego_pos, _ = p.getBasePositionAndOrientation(lego_id)
    tcp_push_start = [lego_pos[0], lego_pos[1] - 0.08, lego_pos[2] + 0.003]
    lego_pos_end = [lego_pos[0], lego_pos[1] + 0.3, lego_pos[2]]

    tcp_pos, _ = p.getLinkState(robot_id, tcp_link_index)[4:6]
    print("tcp_pos start", tcp_pos)

    trajectory = []
    trajectory += linear_trajectory(tcp_pos, tcp_push_start, 100)
    trajectory += linear_trajectory(tcp_push_start, lego_pos_end, 100)

    target_ori = p.getQuaternionFromEuler([math.pi, 0, 0])  # TCP points downward
    forces = []
    last_tcp = None

    for pos in trajectory:
        q = p.calculateInverseKinematics(
            robot_id,
            tcp_link_index,
            pos,
            targetOrientation=target_ori,
            maxNumIterations=200
        )

        # Move joints
        for j, angle in zip(joint_indices, q):
            p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL,
                                    targetPosition=angle, force=300)

        p.stepSimulation()
        time.sleep(dt)

        # Contact force between TCP link & Lego
        contacts = p.getContactPoints(bodyA=robot_id,
                                      bodyB=lego_id,
                                      linkIndexA=tcp_link_index)
        normal_force = sum(c[9] for c in contacts) if contacts else 0.0
        forces.append(normal_force)

        # Visualize path (blue)
        tcp_world = p.getLinkState(robot_id, tcp_link_index)[4]
        if last_tcp is not None:
            p.addUserDebugLine(last_tcp, tcp_world, [0, 0, 1], 1, 0)
        last_tcp = tcp_world


    return forces


def linear_trajectory(start, end, steps):
    """
    Returns 'steps + 1' points that uniformly sample the line from
    'start' to 'end'.

    Parameters
    ----------
    start : [x, y, z]
    end   : [x, y, z]
    steps : int
        Number of segments (points = steps + 1)

    Returns
    -------
    list[list[float]]  # [[x0,y0,z0], [x1,y1,z1], ...]
    """
    return [
        [
            start[0] + (end[0] - start[0]) * t / steps,
            start[1] + (end[1] - start[1]) * t / steps,
            start[2] + (end[2] - start[2]) * t / steps,
        ]
        for t in range(steps + 1)
    ]


def plot_forces(forces, timestep=1.0 / 240):
    """
    Plots contact force over time.

    Parameters
    ----------
    forces : list[float]
        Force values measured in push_lego()
    timestep : float
        Time between simulation steps (default: 1/240 s)
    """
    time_array = np.linspace(0, timestep * len(forces), len(forces))

    plt.figure(figsize=(10, 4))
    plt.plot(time_array, forces, label="Contact Force (N)", color="red")
    plt.xlabel("Time (s)")
    plt.ylabel("Force (N)")
    plt.title("Contact Force between TCP and Lego over Time")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


def main():
    #init
    init_simulation()

    robot_id, joint_indices, tcp_link_index, table_id, lego_id = init_environment("kuka_iiwa\model.urdf",
                                                                                  "table_square\\table_square.urdf",
                                                                                  "lego\\lego.urdf")
    #draw circle
    center = [0.4, 0, 0.7]
    radius = 0.1
    num_points = 400

    trajectory = generate_circle_trajectory(center, radius, num_points)
    move_robot_along_trajectory(robot_id, joint_indices, tcp_link_index, trajectory)


    # push lego

    forces = push_lego(robot_id,
                       joint_indices,
                       tcp_link_index,
                       lego_id,
                       dt=1.0 / 240)

    #plot forces
    plot_forces(forces)

    while p.isConnected():
        p.stepSimulation()
        time.sleep(1.0 / 240)


if __name__ == "__main__":
    main()
