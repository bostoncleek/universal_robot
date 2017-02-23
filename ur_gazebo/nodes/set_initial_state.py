#!/usr/bin/env python

import sys
import rospy
import time
from gazebo_msgs.srv import SetModelConfiguration
from std_srvs.srv import Empty


if __name__ == "__main__":
    rospy.init_node("set_initial_state")

    rospy.get_rostime()
    time.sleep(8.0) # let the robot load for a second or 5 in gazebo (slower machines may need to increase this)
    rospy.loginfo("setting initial state for robot")

    model_config = SetModelConfiguration()

    model_config.model_name = "robot"
    model_config.urdf_param_name = "robot_description"

    model_config.joint_names = ["shoulder_lift_joint", "wrist_1_joint"]
    model_config.joint_positions = [-1.5070, -1.5707]

    rospy.wait_for_service('/gazebo/set_model_configuration', timeout=10.0)
    # allow plenty of time for gazebo to load the robot first
    set_initial_state = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
    response = set_initial_state(model_config.model_name, model_config.urdf_param_name, model_config.joint_names, model_config.joint_positions)

    if response.success:
        rospy.loginfo("function call success")
    else:
        rospy.loginfo("function call FAIL")

    rospy.wait_for_service('/gazebo/unpause_physics', timeout=10.0)
    resume_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    resume_gazebo()
