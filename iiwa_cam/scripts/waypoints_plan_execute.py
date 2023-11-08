import rospy

import numpy as np

from geometry_msgs.msg import Pose, Point, Quaternion

from iiwa_cam.msg import GeneralControl
from iiwa_cam.srv import GeneralExecution, GeneralPlan

waypoints = [
    "0.32; 0.027; 0.36; 0.99993; -0.01142; -0.002606; -0.00045451",
    "0.626; 0.16; 0.36; 0.99993; -0.01142; -0.002606; -0.00045451",
    "0.626; 0.16; 0.05; 0.99993; -0.01142; -0.002606; -0.00045451",
    "0.626; 0.003; 0.05; 0.99993; -0.01142; -0.002606; -0.00045451",
    "0.626; 0.003; 0.36; 0.99993; -0.01142; -0.002606; -0.00045451",
    "0.32; 0.027; 0.36; 0.99993; -0.01142; -0.002606; -0.00045451",
]


def get_traj_exec(request):
    rospy.wait_for_service("general_execution")

    node = rospy.ServiceProxy("general_execution", GeneralExecution)
    result = node(request)

    return result


def get_traj_plan(poses, speed=0.1, stiffness=1000):
    rospy.wait_for_service("general_plan")

    request = GeneralControl("iiwa_green", poses, stiffness, speed, True, True)

    node = rospy.ServiceProxy("general_plan", GeneralPlan)
    result = node(request)

    return result


def generate_poses(waypoints=[]):
    poses = []

    for points_str in waypoints:
        points = np.array(points_str.split("; "))
        points = np.asarray(points, float)

        pose = Pose(
            Point(points[0], points[1], points[2]),
            Quaternion(
                points[3],
                points[4],
                points[5],
                points[6],
            ),
        )

        poses.append(pose)

    return poses


if __name__ == "__main__":
    rospy.init_node("waypoints_plan_execute", anonymous=True)

    # gripper_control(pub, pos=255, grasp=False)

    poses = generate_poses(waypoints)
    res = get_traj_plan(poses)
    while res.success:
        replan = input("replan? [y/n]")
        if replan == "y":
            res = get_traj_plan(poses)
        elif replan == "n" or replan == "":
            break

    if res.success:
        execute = input("execute? [y/n]")
        if execute == "y":
            get_traj_exec(res.general_traj)
