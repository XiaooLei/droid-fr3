import zerorpc

from droid.franka.franky_robot import FrankyRobot

if __name__ == "__main__":
    robot_client = FrankyRobot()
    robot_client.launch_robot()  # Initialize franky connection
    s = zerorpc.Server(robot_client, heartbeat=120)  # 120s heartbeat for long motions
    s.bind("tcp://0.0.0.0:4242")
    s.run()
