import os
from cv2 import aruco

# Robot Params #
nuc_ip = "172.16.0.2"
robot_ip = "172.16.0.3"
laptop_ip = "172.16.0.1"
sudo_password = ""
robot_type = "fr3"  # 'panda' or 'fr3'
robot_serial_number = ""

# Camera ID's #
hand_camera_id = "19006932"       # 腕部相机
varied_camera_1_id = "37322041"  # 左侧第三方相机
varied_camera_2_id = "37818728"   # 右侧第三方相机

# Camera Names #
camera_name_dict = {
    hand_camera_id: "Hand Camera",
    varied_camera_1_id: "Left Third-Person Camera",
    varied_camera_2_id: "Right Third-Person Camera",
}

# Charuco Board Params #
CHARUCOBOARD_ROWCOUNT = 8
CHARUCOBOARD_COLCOUNT = 11
CHARUCOBOARD_CHECKER_SIZE = 0.0248
CHARUCOBOARD_MARKER_SIZE = 0.01488
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)

# Ubuntu Pro Token (RT PATCH) #
ubuntu_pro_token = ""

# Code Version [DONT CHANGE] #
droid_version = "1.3"

