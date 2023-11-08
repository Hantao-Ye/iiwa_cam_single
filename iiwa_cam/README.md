# Kuka ROS Tutorial - iiwa_cam Package & CAM::KUKA Class

## Content  

1. [Kuka Class](#kuka-class)
2. [Kuka with MoveIt!](./wiki/KukaMoveIt.md)
3. [Microservices](./wiki/KukaMicroservices.md)
4. [Dependencies](#dependencies)

## Kuka Class

The Kuka class implements common functions for controlling the KUKA manipulator through ROS C++

**APIs of the cam::Kuka Class:**

- [API Reference](./wiki/KukaClassReference.md)

**Source File:**

- [iiwa.hpp](./include/iiwa_cam/iiwa.hpp): definition and implementation of the cam::Kuka class

**Demo Files:**

- [cartesian_lin_drop_pose_control_node.cpp](./src/demo/cartesian_lin_drop_pose_control_node.cpp):  move kuka linearly assigned by a droppable cartesian space goal  
- [cartesian_lin_pose_control_node.cpp](./src/demo/cartesian_lin_pose_control_node.cpp):  move kuka linearly assigned by a cartesian space goal  
- [cartesian_ptp_drop_pose_control_node.cpp](./src/demo/cartesian_ptp_drop_pose_control_node.cpp):  move kuka point to point assigned by a droppable cartesian space goal  
- [cartesian_ptp_pose_control_node.cpp](./src/demo/cartesian_ptp_pose_control_node.cpp):  move kuka point to point assigned by a cartesian space goal  
- [cartesian_spline_node.cpp](./src/demo/cartesian_spline_node.cpp):  move robot along a trajectory in cartesian space  
- [get_frames_node.cpp](./src/demo/get_frames_node.cpp): move robot to pre-set points using cam::Kuka::get_recorded_frames
- [joint_pose_control_node.cpp](./src/demo/joint_pose_control_node.cpp):  move kuka point to point assigned by a joint space goal

## Microservice List

- [MoveIt! - RViz Support Services](./wiki/KukaMicroservices.md#moveit---rviz-support-services)
- [End Effector State Services](./wiki/KukaMicroservices.md#end-effector-state-services)
- [Special Services of iiwa_stack_cam](./wiki/KukaMicroservices.md#special-services-of-iiwastackcam)
- [Services from Original iiwa_stack](./wiki/KukaMicroservices.md#services-from-original-iiwastack)

## Dependencies

- [iiwa_stack_cam](https://github.com/RROS-Lab/iiwa_stack_cam) (need installation)
- [csv2](https://github.com/p-ranav/csv2) (included already)
