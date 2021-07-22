#! /usr/bin/env python

import numpy as np

np.set_printoptions(suppress=True, precision=2)

import libodri_control_interface_pywrap as oci

robot = oci.robot_from_yaml_file("config_solo12.yaml")
joint_calibrator = oci.joint_calibrator_from_yaml_file(
    "config_solo12.yaml", robot.joints
)

# Initialize the communication and the session.
robot.start()
robot.wait_until_ready()

# Calibrate the robot if needed and get into initial configuration
initial_configuration = np.array([0.0, 0.7, -1.4, -0.0, 0.7, -1.4, 0.0, -0.7, +1.4, -0.0, -0.7, +1.4])
robot.run_calibration(joint_calibrator, initial_configuration)

robot.joints.set_zero_gains()
robot.joints.set_zero_commands()

c = 0
while not robot.is_timeout:
    robot.parse_sensor_data()

    if c % 100 == 0:
        print("IMU attitude:", robot.imu.attitude_euler)
        print("joint pos   :", robot.joints.positions)
        print("joint vel   :", robot.joints.velocities)
        robot.robot_interface.PrintStats()

    # Waiting in initial configuration
    robot.joints.set_torques(np.zeros(12))
    robot.joints.set_desired_positions(initial_configuration)
    robot.joints.set_desired_velocities(np.zeros(12))
    robot.joints.set_position_gains(3.0 * np.ones(12))
    robot.joints.set_velocity_gains(0.05 * np.ones(12))

    robot.send_command_and_wait_end_of_cycle()
    c += 1
