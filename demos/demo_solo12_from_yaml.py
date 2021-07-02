#! /usr/bin/env python

import numpy as np

np.set_printoptions(suppress=True, precision=2)

import time

import libmaster_board_sdk_pywrap as mbs
import libodri_control_interface_pywrap as oci

import pathlib

robot = oci.robot_from_yaml_file("config_solo12.yaml")
joint_calibrator = oci.joint_calibrator_from_yaml_file(
    "config_solo12.yaml", robot.joints
)

# Initialize the communication and the session.
robot.start()
robot.wait_until_ready()

# Calibrate the robot if needed.
robot.run_calibration(joint_calibrator)

robot.parse_sensor_data()


init_imu_attitude = robot.imu.attitude_euler.copy()

des_pos = np.zeros(12)

c = 0
dt = 0.001
calibration_done = False
next_tick = time.time()
while not robot.is_timeout:
    robot.parse_sensor_data()

    imu_attitude = robot.imu.attitude_euler
    positions = robot.joints.positions
    velocities = robot.joints.velocities

    if c % 2000 == 0:
        print("IMU attitude:", imu_attitude)
        print("joint pos   :", positions)
        print("joint vel   :", velocities)
        robot.robot_interface.PrintStats()

    des_pos[:] = imu_attitude[2] - init_imu_attitude[2]
    torques = 5.0 * (des_pos - positions) - 0.1 * velocities
    robot.joints.set_torques(0.0 * torques)

    robot.send_command_and_wait_end_of_cycle()
    c += 1
