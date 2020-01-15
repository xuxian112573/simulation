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

#home pose
arm_group.set_named_target("home_j")
plan = arm_group.go()
#rospy.sleep(0.5)
#hand open
arm_group.set_max_acceleration_scaling_factor(0.1)#o.o1
arm_group.set_max_velocity_scaling_factor(0.05)#.04
hand_group.set_named_target("open")
plan = hand_group.go()
#rospy.sleep(0.5)
#pickup 01 pose
pickup = geometry_msgs.msg.Pose()
pickup.orientation.w = 0.5
pickup.orientation.x = -0.5
pickup.orientation.y = 0.5
pickup.orientation.z = -0.5
pickup.position.x = -0.05
pickup.position.y = -0.0
pickup.position.z = 1.5
arm_group.set_pose_target(pickup)
plan = arm_group.go()
#rospy.sleep(0.5)
#hight-l pose
#pickup.position.z = 1.2 #1.08
#arm_group.set_pose_target(pickup)
#plan = arm_group.go()
#rospy.sleep(0.5)


#rospy.sleep(5)
#hight-l pose
pickup.position.z = 1.081 #1.08
arm_group.set_pose_target(pickup)
plan = arm_group.go()
rospy.sleep(1)
#hand close
hand_group.set_named_target("close")
plan = hand_group.go()
rospy.sleep(1)
# hight 
pickup.position.z = 1.5
arm_group.set_pose_target(pickup)
plan = arm_group.go()
#rospy.sleep(0.5)
#place -01 pose
plac = geometry_msgs.msg.Pose()
plac.orientation.w = 0.5
plac.orientation.x = -0.5
plac.orientation.y = 0.5
plac.orientation.z = -0.5
plac.position.x = 0.7  #0.6 0.5
plac.position.y = 0.0 #0.15 0.1
plac.position.z = 1.5 #1.0  1.2

arm_group.set_pose_target(plac)
plan = arm_group.go()



plac.position.z = 0.805  #1.0  1.2
arm_group.set_pose_target(plac)
plan = arm_group.go()
rospy.sleep(1)
hand_group.set_named_target("open")
plan = hand_group.go()
rospy.sleep(1)

plac.position.z = 1.5
arm_group.set_pose_target(plac)
plan = arm_group.go()
rospy.sleep(1)
#arm_group.set_max_acceleration_scaling_factor(0.1)#o.o1
#arm_group.set_max_velocity_scaling_factor(0.1)#.04
#plac.position.z = 1.5
#arm_group.set_pose_target(plac)
#plan = arm_group.go()
#arm_group.set_named_target("home_j")
#plan = arm_group.go()



