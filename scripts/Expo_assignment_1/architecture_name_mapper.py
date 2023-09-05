#!/usr/bin/env python3

"""
.. module::architecture_name_mapper.py
   :platform: ROS
   :synopsis:: Class for the Name Mapper module
.. moduleauthor:: Raja Taha

This scripts is used in the package to define all the names of the variables used. It is a way to use a variable instead of the total name thus avoiding spelling problems while writing it.
It is divided as follows:
* name of the state if the Finite State Machine and of the Sub Finite State Machine;
* name of all the transition used: there are also some parameters that are not properly transition but they are used to modify the value of the shared variable of the helper to guarantee the correct flow in the program;
* name of the action, planner and controller, used in the program;
* parameters for the robot behaviour:
    * battery threshold: value after which the robot has to be recharged;
    * busy paramter: value to define the busy waiting that the robot has to perform before starting the plan to another location;
    * number of points: value that define the number of points the planner module has to compute for each path required;
    * recharging room: string name of the recharging room. It is needed in particular when the robot has to check the availability of this particualr location when it has to go to the recharge state.
* rooms: name of all locations in the ontology;
* coordinates: x and y coordinate of each location. Here it is needed a  match one to one with the names above.
"""

import rospy

# The name of the parameter to define the environment size.
# It should be a list [max_x, max_y] such that x:[0, max_x) and y:[0, max_y).
PARAM_ENVIRONMENT_SIZE = 'config/environment_size'

# The name of parameter to set the initial robot position.
PARAM_INITIAL_POSE = 'state/initial_pose'
# ---------------------------------------------------------
PARAM_RANDOM_ACTIVE = 'test/random_sense/active'

# The name of the node that sets/gets the pose of the robot and manages its battery.
NODE_ROBOTS_CONDITION = 'robots_condition'

# The name of the server to get the current robot pose.
SERVER_GET_POSE = 'state/get_pose'

# The name of the server to set the current robot pose. 
SERVER_SET_POSE = 'state/set_pose'

# The name of the topic where the battery state is published.
TOPIC_BATTERY_LOW = 'state/battery_low'
# ---------------------------------------------------------




# Parameter indicating the sleep time [s]
SLEEP_TIME = 0.3

# Parameter indicating the battery time [s]
PARAM_BATTERY_TIME = 'test/random_sense/battery_time'
# ---------------------------------------------------------
MONITOR_TIME = 5

# The name of the planner node.
NODE_PLANNER = 'planner'

# The name of the action server solving the motion planning problem.
ACTION_PLANNER = 'motion/planner'

# The number of points in the plan. It should be a list [min_n, max_n],
# Where the number of points is a random value in the interval [min_n, max_n).
PARAM_PLANNER_POINTS = 'test/random_plan_points'

# The delay between the computation of the next via points.
# It should be a list `[min_time, max_time]`, and the computation will 
# last for a random number of seconds in such an interval.
PARAM_PLANNER_TIME = 'test/random_plan_time'
# -------------------------------------------------


# The name of the controller node.
NODE_CONTROLLER = 'controller'

# The name of the action server solving the motion control problem.
ACTION_CONTROLLER = 'motion/controller'

# The time required to reach a via points.
# It should be a list `[min_time, max_time]`, and the time to reach a
# via point will be a random number of seconds in such an interval.
PARAM_CONTROLLER_TIME = 'test/random_motion_time'
# -------------------------------------------------


def tag_log(msg, producer_tag):
    """
	Function used to label each log with a producer tag.
	
    Args:
        msg(Str): message that will be visualized
        producer_tag(Str): tag identifying the log producer
            
    Returns:
        log_msg(Str): message for the log
    """

    return f'@{producer_tag}>> {msg}'
