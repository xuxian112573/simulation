#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs

moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('move_group_grasp', anonymous=True)
robot = moveit_commander.robot.RobotCommander()

arm_group = moveit_commander.move_group.MoveGroupCommander("arm")
hand_group = moveit_commander.move_group.MoveGroupCommander("gripper")
arm_group.set_named_target("home_j")
plan = arm_group.go()
hand_group.set_named_target("open")
plan = hand_group.go()

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.5
pose_target.orientation.x = -0.5
pose_target.orientation.y = 0.5
pose_target.orientation.z = -0.5
pose_target.position.x = 0.15
pose_target.position.y = 0
pose_target.position.z = 1.25
arm_group.set_pose_target(pose_target)
plan = arm_group.go()

pose_target.position.z = 1.2 #1.08
arm_group.set_pose_target(pose_target)
plan = arm_group.go()
#rospy.sleep(5)

pose_target.position.z = 1.081 #1.08
arm_group.set_pose_target(pose_target)
plan = arm_group.go()
rospy.sleep(1)

hand_group.set_named_target("close")
plan = hand_group.go()

pose_target.position.z = 1.5
arm_group.set_pose_target(pose_target)
plan = arm_group.go()

pose_target2 = geometry_msgs.msg.Pose()
pose_target2.orientation.w = 0.5
pose_target2.orientation.x = -0.5
pose_target2.orientation.y = 0.5
pose_target2.orientation.z = -0.5
pose_target2.position.x = 0.10
pose_target2.position.y = 0.2
pose_target2.position.z = 1.09
arm_group.set_pose_target(pose_target2)
plan = arm_group.go()
rospy.sleep(1)
hand_group.set_named_target("open")
plan = hand_group.go()
pose_target.position.z = 1.5
arm_group.set_pose_target(pose_target)
plan = arm_group.go()
rospy.sleep(1)

pose_target3 = geometry_msgs.msg.Pose()
pose_target3.orientation.w = 0.5
pose_target3.orientation.x = -0.5
pose_target3.orientation.y = 0.5
pose_target3.orientation.z = -0.5
pose_target3.position.x = -0.15
pose_target3.position.y = 0.0
pose_target3.position.z = 1.25
arm_group.set_pose_target(pose_target3)
plan = arm_group.go()
rospy.sleep(1)
pose_target3.position.z = 1.081
arm_group.set_pose_target(pose_target3)
plan = arm_group.go()
rospy.sleep(1)
hand_group.set_named_target("close")
plan = hand_group.go()
rospy.sleep(1)
pose_target3.position.z = 1.5
arm_group.set_pose_target(pose_target3)
plan = arm_group.go()

pose_target2.position.z = 1.13
arm_group.set_pose_target(pose_target2)
plan = arm_group.go()
rospy.sleep(1)
hand_group.set_named_target("open")
plan = hand_group.go()
pose_target.position.z = 1.5
arm_group.set_pose_target(pose_target)
plan = arm_group.go()
rospy.sleep(1)

pose_target4 = geometry_msgs.msg.Pose()
pose_target4.orientation.w = 0.5
pose_target4.orientation.x = -0.5
pose_target4.orientation.y = 0.5
pose_target4.orientation.z = -0.5
pose_target4.position.x = -0.15
pose_target4.position.y = 0.1
pose_target4.position.z = 1.25
arm_group.set_pose_target(pose_target4)
plan = arm_group.go()
rospy.sleep(1)
pose_target4.position.z = 1.081
arm_group.set_pose_target(pose_target4)
plan = arm_group.go()
rospy.sleep(1)
hand_group.set_named_target("close")
plan = hand_group.go()
rospy.sleep(1)
pose_target4.position.z = 1.5
arm_group.set_pose_target(pose_target4)
plan = arm_group.go()

pose_target2.position.z = 1.17
arm_group.set_pose_target(pose_target2)
plan = arm_group.go()
rospy.sleep(1)
hand_group.set_named_target("open")
plan = hand_group.go()
pose_target.position.z = 1.5
arm_group.set_pose_target(pose_target)
plan = arm_group.go()
rospy.sleep(1)

pose_target5 = geometry_msgs.msg.Pose()
pose_target5.orientation.w = 0.5
pose_target5.orientation.x = -0.5
pose_target5.orientation.y = 0.5
pose_target5.orientation.z = -0.5
pose_target5.position.x = 0.15
pose_target5.position.y = 0.1
pose_target5.position.z = 1.25
arm_group.set_pose_target(pose_target5)
plan = arm_group.go()
rospy.sleep(1)
pose_target5.position.z = 1.081
arm_group.set_pose_target(pose_target5)
plan = arm_group.go()
rospy.sleep(1)
hand_group.set_named_target("close")
plan = hand_group.go()
rospy.sleep(1)
pose_target5.position.z = 1.5
arm_group.set_pose_target(pose_target5)
plan = arm_group.go()

pose_target2.position.z = 1.21
arm_group.set_pose_target(pose_target2)
plan = arm_group.go()
rospy.sleep(2)
hand_group.set_named_target("open")
plan = hand_group.go()
pose_target.position.z = 1.5
arm_group.set_pose_target(pose_target)
plan = arm_group.go()
rospy.sleep(2)

pose_target6 = geometry_msgs.msg.Pose()
pose_target6.orientation.w = 0.5
pose_target6.orientation.x = -0.5
pose_target6.orientation.y = 0.5
pose_target6.orientation.z = -0.5
pose_target6.position.x = 0.0
pose_target6.position.y = 0.1
pose_target6.position.z = 1.25
arm_group.set_pose_target(pose_target6)
plan = arm_group.go()
rospy.sleep(1)
pose_target6.position.z = 1.08
arm_group.set_pose_target(pose_target6)
plan = arm_group.go()
rospy.sleep(1)
hand_group.set_named_target("close")
plan = hand_group.go()
rospy.sleep(1)
pose_target6.position.z = 1.5
arm_group.set_pose_target(pose_target6)
plan = arm_group.go()

pose_target2.position.z = 1.25
arm_group.set_pose_target(pose_target2)
plan = arm_group.go()
rospy.sleep(2)
hand_group.set_named_target("open")
plan = hand_group.go()
pose_target.position.z = 1.5
arm_group.set_pose_target(pose_target)
plan = arm_group.go()
rospy.sleep(2)

pose_target7 = geometry_msgs.msg.Pose()
pose_target7.orientation.w = 0.5
pose_target7.orientation.x = -0.5
pose_target7.orientation.y = 0.5
pose_target7.orientation.z = -0.5
pose_target7.position.x = 0.0
pose_target7.position.y = 0.0
pose_target7.position.z = 1.25
arm_group.set_pose_target(pose_target7)
plan = arm_group.go()
rospy.sleep(1)
pose_target7.position.z = 1.08
arm_group.set_pose_target(pose_target7)
plan = arm_group.go()
rospy.sleep(1)
hand_group.set_named_target("close")
plan = hand_group.go()
rospy.sleep(1)
pose_target7.position.z = 1.5
arm_group.set_pose_target(pose_target7)
plan = arm_group.go()

pose_target2.position.z = 1.29
arm_group.set_pose_target(pose_target2)
plan = arm_group.go()
rospy.sleep(2)
hand_group.set_named_target("open")
plan = hand_group.go()
pose_target.position.z = 1.5
arm_group.set_pose_target(pose_target)
plan = arm_group.go()
rospy.sleep(2)

pose_target8 = geometry_msgs.msg.Pose()
pose_target8.orientation.w = 0.5
pose_target8.orientation.x = -0.5
pose_target8.orientation.y = 0.5
pose_target8.orientation.z = -0.5
pose_target8.position.x = 0.0
pose_target8.position.y = -0.1
pose_target8.position.z = 1.25
arm_group.set_pose_target(pose_target8)
plan = arm_group.go()
rospy.sleep(1)
pose_target8.position.z = 1.08
arm_group.set_pose_target(pose_target8)
plan = arm_group.go()
rospy.sleep(1)
hand_group.set_named_target("close")
plan = hand_group.go()
rospy.sleep(1)
pose_target8.position.z = 1.5
arm_group.set_pose_target(pose_target8)
plan = arm_group.go()

pose_target2.position.z = 1.33
arm_group.set_pose_target(pose_target2)
plan = arm_group.go()
rospy.sleep(2)
hand_group.set_named_target("open")
plan = hand_group.go()
pose_target.position.z = 1.5
arm_group.set_pose_target(pose_target)
plan = arm_group.go()
rospy.sleep(2)

moveit_commander.roscpp_initializer.roscpp_shutdown()
