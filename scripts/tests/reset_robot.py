from droid.robot_env import RobotEnv
import numpy as np

env = RobotEnv(action_space="joint_position")
env.reset()
print("Reset complete")
