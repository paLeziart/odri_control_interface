#! /usr/bin/env python

# Run a PD controller to keep the robot at desired position.

import libodri_control_interface_pywrap as oci
import numpy as np
import pathlib
import threading

np.set_printoptions(suppress=True, precision=2)


def get_input():
    keystrk = input()
    # thread doesn't continue until key is pressed
    # and so it remains alive


"""from example_robot_data import load

# Load robot model and data
robot = load("sassa")

# Init Gepetto viewer
robot.initViewer(loadModel=True)
if "viewer" in robot.viz.__dict__:
    # robot.viewer.gui.addFloor("world/floor")
    robot.viewer.gui.setRefreshIsSynchronous(False)
viewer = robot"""

# Create the robot object from yaml.
robot = oci.robot_from_yaml_file(
    "/home/paleziart/git/quadruped-replay/configs/config_sassa.yaml"
)

# Store initial position data.
des_pos = np.array([0.0, -0.6, 1.2] * 2 + [0.0, 0.6, -1.2] * 2)
fix_pos = np.zeros_like(des_pos)

# Initialize the communication, session, joints, wait for motors to be ready
# and run the joint calibration.
robot.initialize(des_pos)

i = threading.Thread(target=get_input)
i.start()
print("Correct joint offsets and press Enter")

c = 0
while not robot.is_timeout and i.is_alive():
    robot.parse_sensor_data()

    imu_attitude = robot.imu.attitude_euler
    positions = robot.joints.positions
    velocities = robot.joints.velocities

    # Correct offsets when joints are manually moved far from their target position
    fix_pos += np.pi * ((des_pos + fix_pos - positions) > (3 * np.pi / 2))
    fix_pos -= np.pi * ((des_pos + fix_pos - positions) < (3 * np.pi / 2))

    # Compute the PD control.
    torques = 15.0 * (des_pos + fix_pos - positions) - 0.5 * velocities
    robot.joints.set_torques(torques)

    """viewer.display(
        np.hstack(
            (
                np.array([0.0, 0.0, 0.0]),
                robot.imu.attitude_quaternion,
                positions,
            )
        )
    )"""

    robot.send_command_and_wait_end_of_cycle(0.001)
    c += 1

    if c % 500 == 0:
        print("joint pos:   ", positions)
        print("joint vel:   ", velocities)
        print("torques:     ", torques)
        robot.robot_interface.PrintStats()
