import sys
import time
import zerorpc

from droid.franka.franky_robot import FrankyRobot

if __name__ == "__main__":
    print("[SERVER] Starting DROID robot server...")

    robot_client = FrankyRobot()

    retries = 3
    for attempt in range(1, retries + 1):
        try:
            robot_client.launch_robot()
            break
        except Exception as e:
            print(f"[SERVER] launch_robot attempt {attempt}/{retries} failed: {e}")
            if attempt == retries:
                print("[SERVER] Could not connect to robot. Check FR3 power and IP.")
                sys.exit(1)
            time.sleep(3)

    s = zerorpc.Server(robot_client, heartbeat=120)
    s.bind("tcp://0.0.0.0:4242")
    print("[SERVER] Listening on tcp://0.0.0.0:4242")
    s.run()
