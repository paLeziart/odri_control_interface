# coding: utf8

import threading
import numpy as np
import argparse
import libodri_control_interface_pywrap as oci

DT = 0.0020

key_pressed = False


def get_input():
    keystrk = input()
    # thread doesn't continue until key is pressed
    key_pressed = True


def put_on_the_floor(device, q_init):
    """Make the robot go to the default initial position and wait for the user
    to press the Enter key to start the main control loop

    Args:
        device (robot wrapper): a wrapper to communicate with the robot
        q_init (array): the default position of the robot
    """

    np.set_printoptions(linewidth=100, precision=3)

    print("PUT ON THE FLOOR.")

    key_pressed = False
    Kp_pos = 3.
    Kd_pos = 0.01
    imax = 3.0
    T = 3.0
    t = 0.0
    dt = 0.002
    pos = np.zeros(q_init.size)

    device.parse_sensor_data()
    pos_init = np.copy(device.joints.positions)

    i = threading.Thread(target=get_input)
    i.start()

    print("Put the robot on the floor and press Enter")
    k = 0
    while i.is_alive():
        #print("IsAlive")

        device.parse_sensor_data()
        if t <= T:
            x = (t / T)
            pos = x * q_init.ravel() + (1 - x) * pos_init
            t += dt
        torques = Kp_pos * (pos - device.joints.positions) - Kd_pos * device.joints.velocities
        if (k < 20):
            print(k)
            print("Target:    ", pos)
            print("Joint pos: ", device.joints.positions)
            # print("Torques: ", torques)
        k = k+1
        device.joints.set_torques(np.zeros(12))#0.0 * torques)
        device.send_command_and_wait_end_of_cycle()
    
    print("Start the motion.")


def control_loop(name_interface):
    """Main function that calibrates the robot, get it into a default waiting position then launch
    the main control loop once the user has pressed the Enter key

    Args:
        name_interface (string): name of the interface that is used to communicate with the robot
    """

    # Default position after calibration
    q_init = np.array([0.2, 0.7, -1.4, -0.2, 0.7, -1.4, 0.2, -0.7, +1.4, -0.2, -0.7, +1.4])

    # Communication interface    
    device = oci.robot_from_yaml_file('config_solo12.yaml')
    joint_calibrator = oci.joint_calibrator_from_yaml_file('config_solo12.yaml', device.joints)

    # Number of motors
    nb_motors = 12

    # Initialize the communication and the session.
    device.start()
    device.wait_until_ready()

    # Calibrate the robot if needed.
    device.run_calibration(joint_calibrator)

    device.parse_sensor_data()

    # Wait for Enter input before starting the control loop
    put_on_the_floor(device, q_init)

    # Main control loop once Enter has been pressed
    print("Entering infinite loop")
    while True:
        device.parse_sensor_data()
        torques = - 0.1 * device.joints.velocities
        device.joints.set_torques(0.0 * torques)
        device.send_command_and_wait_end_of_cycle()
        pass

    # Whatever happened we send 0 torques to the motors.
    device.joints.set_torques(np.zeros(nb_motors))
    device.send_command_and_wait_end_of_cycle()

    quit()


def main():
    """Main function
    """

    parser = argparse.ArgumentParser(description='Playback trajectory to show the extent of solo12 workspace.')
    parser.add_argument('-i',
                        '--interface',
                        required=True,
                        help='Name of the interface (use ifconfig in a terminal), for instance "enp1s0"')

    control_loop(parser.parse_args().interface)


if __name__ == "__main__":
    main()
