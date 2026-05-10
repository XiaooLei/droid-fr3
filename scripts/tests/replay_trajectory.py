from droid.robot_env import RobotEnv
from droid.trajectory_utils.misc import replay_trajectory
import numpy as np

trajectory_folderpath = "/home/fnlp/dev/droid/data/success/2026-05-07/Thu_May__7_23:28:36_2026"
action_space = "joint_position"

env = RobotEnv(action_space=action_space)

h5_filepath = trajectory_folderpath + "/trajectory.h5"
replay_trajectory(env, filepath=h5_filepath)



