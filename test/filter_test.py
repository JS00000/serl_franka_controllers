import sys
import rospy
import numpy as np
import geometry_msgs.msg as geom_msg
import time
import subprocess
from dynamic_reconfigure.client import Client
from scipy.spatial.transform import Rotation as R

if __name__ == "__main__":
    eepub = rospy.Publisher('/cartesian_impedance_controller/equilibrium_pose', geom_msg.PoseStamped, queue_size=10)
    rospy.init_node('franka_control_api')
    client = Client("/cartesian_impedance_controllerdynamic_reconfigure_compliance_param_node")

    # Setting the impedance_controller params through ros dynamic reconfigure
    client.update_configuration({"filter_params": 0.0005})
    client.update_configuration({"translational_stiffness": 2000.0})
    client.update_configuration({"translational_damping": 200.0})
    client.update_configuration({"rotational_stiffness": 150.0})
    client.update_configuration({"rotational_damping": 7.0})
    client.update_configuration({"nullspace_stiffness": 0.2})
    client.update_configuration({"joint1_nullspace_stiffness": 100.0})
    for direction in ['x', 'y', 'z', 'neg_x', 'neg_y', 'neg_z']:
        client.update_configuration({"translational_clip_" + direction: 0.02})
        client.update_configuration({"rotational_clip_" + direction: 0.05})
    time.sleep(1)
    print("\nNew reference limiting values has been set")

    init_z = 0.4
    # Reset the arm
    msg = geom_msg.PoseStamped()
    msg.header.frame_id = "0"
    msg.header.stamp = rospy.Time.now()
    msg.pose.position = geom_msg.Point(0.5, 0, init_z)
    quat = R.from_euler('xyz', [np.pi, 0, np.pi/2]).as_quat()
    msg.pose.orientation = geom_msg.Quaternion(quat[0], quat[1], quat[2], quat[3])
    input("\033[33m\nObserve the surroundings. Press enter to move the robot to the initial position.\033[0m")
    eepub.publish(msg)
    time.sleep(1)


    input("\033[33mPress enter to begin sin loop. \033[0m")

    amplitude = 0.1         # 正弦波振幅
    amplitude_yaw = 1.0     # 正弦波振幅
    frequency = 0.125       # 正弦波频率
    loop_rate = 1.0         # 发布频率
    rate = rospy.Rate(loop_rate)
    start_time = rospy.get_time()
    cnt = 0

    input_str = ''
    while not rospy.is_shutdown():
        cnt = cnt + 1

        # 手动发送
        # input_str = input()

        # 自动发送
        rate.sleep()

        dz = amplitude * np.sin(2 * np.pi * frequency * cnt / loop_rate)
        dyaw = amplitude_yaw * np.sin(2 * np.pi * frequency * cnt / loop_rate)

        msg = geom_msg.PoseStamped()
        msg.header.frame_id = "0"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position = geom_msg.Point(0.5, 0, init_z+dz)
        quat = R.from_euler('xyz', [np.pi, 0, np.pi/2 + dyaw]).as_quat()
        msg.pose.orientation = geom_msg.Quaternion(quat[0], quat[1], quat[2], quat[3])
        eepub.publish(msg)

