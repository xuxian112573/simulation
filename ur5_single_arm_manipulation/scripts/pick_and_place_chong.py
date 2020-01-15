#! /usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import rospkg

from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown

from actionlib import SimpleActionClient, GoalStatus

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
from moveit_msgs.msg import PickupAction, PickupGoal
from moveit_msgs.msg import PlaceAction, PlaceGoal
from moveit_msgs.msg import PlaceLocation
from moveit_msgs.msg import MoveItErrorCodes
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal, GraspGeneratorOptions

from tf.transformations import quaternion_from_euler

import sys
import copy
import numpy
import os

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)


moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


class Pick_Place:
    def __init__(self):
        # 检索参数:
        self._table_object_name = rospy.get_param('~table_object_name', 'Grasp_Table')
        self._table2_object_name = rospy.get_param('~table_object_name', 'Grasp_Table2')
        self._grasp_object_name = rospy.get_param('~grasp_object_name', 'Grasp_Object')
        self._grasp_object2_name = rospy.get_param('~grasp_object_name', 'Grasp_Object2')

        self._grasp_object_width = rospy.get_param('~grasp_object_width', 0.01)

        self._arm_group     = rospy.get_param('~manipulator', 'manipulator')
        self._gripper_group = rospy.get_param('~gripper', 'gripper')

        self._approach_retreat_desired_dist = rospy.get_param('~approach_retreat_desired_dist', 0.2)
        self._approach_retreat_min_dist = rospy.get_param('~approach_retreat_min_dist', 0.1)

        # 创建（调试）发布者:
        self._grasps_pub = rospy.Publisher('grasps', PoseArray, queue_size=1, latch=True)
        self._places_pub = rospy.Publisher('places', PoseArray, queue_size=1, latch=True)

        # 创建规划现场，机器人指挥官:
        self._scene = PlanningSceneInterface()
        self._robot = RobotCommander()

        rospy.sleep(1.0)

        # 清理现场:
        self._scene.remove_world_object(self._table_object_name)
        self._scene.remove_world_object(self._table2_object_name)
        self._scene.remove_world_object(self._grasp_object_name)
        self._scene.remove_world_object(self._grasp_object2_name)

        # 将表和可乐罐对象添加到计划场景:
        self._pose_table    = self._add_table(self._table_object_name)
        self._pose_table    = self._add_table2(self._table2_object_name)
        self._pose_coke_can = self._add_grasp_block_(self._grasp_object_name)
        self._pose_coke_can2 = self._add_grasp_block2_(self._grasp_object2_name)

        rospy.sleep(1.0)

        # 定义目标位置姿势:
        self._pose_place = Pose()

        self._pose_place.position.x = self._pose_coke_can.position.x-0.5
        self._pose_place.position.y = self._pose_coke_can.position.y-0.85
        self._pose_place.position.z = self._pose_coke_can.position.z-0.3
        self._pose_place.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, 0.0))

        self._pose_coke_can.position.z = self._pose_coke_can.position.z - 0.9 # base_link is 0.9 above
        self._pose_place.position.z = self._pose_place.position.z + 0.05 # target pose a little above

        # 检索组 (arm and gripper):
        self._arm     = self._robot.get_group(self._arm_group)
        self._gripper = self._robot.get_group(self._gripper_group)

        self._arm.set_named_target('pickup')
        self._arm.go(wait=True)
        print("Pickup pose")

        # 创建抓取生成器“生成”动作客户端:
        self._grasps_ac = SimpleActionClient('/moveit_simple_grasps_server/generate', GenerateGraspsAction)
        if not self._grasps_ac.wait_for_server(rospy.Duration(1.0)):
            rospy.logerr('Grasp generator action client not available!')
            rospy.signal_shutdown('Grasp generator action client not available!')
            return

        # 创建移动组“zhua取”操作客户端:
        self._pickup_ac = SimpleActionClient('/pickup', PickupAction)
        if not self._pickup_ac.wait_for_server(rospy.Duration(1.0)):
            rospy.logerr('Pick up action client not available!')
            rospy.signal_shutdown('Pick up action client not available!')
            return

        # 创建移动组“放置”动作客户端:
        self._place_ac = SimpleActionClient('/place', PlaceAction)
        if not self._place_ac.wait_for_server(rospy.Duration(1.0)):
            rospy.logerr('Place action client not available!')
            rospy.signal_shutdown('Place action client not available!')
            return

        # Pick Coke can object:
        while not self._pickup(self._arm_group, self._grasp_object_name, self._grasp_object_width):
            rospy.logwarn('Pick up failed! Retrying ...')
            rospy.sleep(1.0)

        rospy.loginfo('Pick up successfully')
        print("pose_place: ", self._pose_place)


        # 放置可乐可能会在支撑表面（桌子）上的其他地方产生异物:
        while not self._place(self._arm_group, self._grasp_object_name, self._pose_place):
            rospy.logwarn('Place failed! Retrying ...')
            rospy.sleep(1.0)

        rospy.loginfo('Place successfully')
        print "-------------test1--------------"
        rospy.sleep(1.0)

    def __del__(self):
        # Clean the scene:
        self._scene.remove_world_object(self._grasp_object_name)
        self._scene.remove_world_object(self._table_object_name)
        self._scene.remove_world_object(self._table2_object_name)
        print "-------------test2------------------"

    def _add_table(self, name):
        """
        创建表并将其添加到场景
        """
        p = PoseStamped()
        p.header.frame_id = self._robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = 0.85
        p.pose.position.y = 0.0
        p.pose.position.z = 0.70

        q = quaternion_from_euler(0.0, 0.0, numpy.deg2rad(90.0))
        p.pose.orientation = Quaternion(*q)

        # Table size from ~/.gazebo/models/table/model.sdf, using the values
        # for the surface link.
        self._scene.add_box(name, p, (1, 1, 0.05))
        print "-------------test3------------------"
        return p.pose
        
    def _add_table2(self, name):
        """
        创建表并将其添加到场景
        """
        p = PoseStamped()
        p.header.frame_id = self._robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = 0.0
        p.pose.position.y = 0.85
        p.pose.position.z = 0.40

        q = quaternion_from_euler(0.0, 0.0, numpy.deg2rad(90.0))
        p.pose.orientation = Quaternion(*q)

        # Table size from ~/.gazebo/models/table/model.sdf, using the values
        # for the surface link.
        self._scene.add_box(name, p, (0.5, 0.5, 0.05))
        print "-------------test4------------------"
        return p.pose
        
        
    def _add_grasp_block_(self, name):
        """
        创建场景并将其添加到场景
        """
        p = PoseStamped()
        p.header.frame_id = self._robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = 0.5
        p.pose.position.y = 0.0
        p.pose.position.z = 0.74

        q = quaternion_from_euler(0.0, 0.0, 0.0)
        p.pose.orientation = Quaternion(*q)

        # Coke can size from ~/.gazebo/models/coke_can/meshes/coke_can.dae,
        # using the measure tape tool from meshlab.
        # The box is the bounding box of the coke cylinder.
        # The values are taken from the cylinder base diameter and height.
        self._scene.add_box(name, p, (0.045, 0.045, 0.045))
        print "-------------test5------------------"
        return p.pose
        
    def _add_grasp_block2_(self, name):
        """
        创建场景并将其添加到场景
        """
        p = PoseStamped()
        p.header.frame_id = self._robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = 0.5
        p.pose.position.y = 0.3
        p.pose.position.z = 0.74

        q = quaternion_from_euler(0.0, 0.0, 0.0)
        p.pose.orientation = Quaternion(*q)

        # Coke can size from ~/.gazebo/models/coke_can/meshes/coke_can.dae,
        # using the measure tape tool from meshlab.
        # The box is the bounding box of the coke cylinder.
        # The values are taken from the cylinder base diameter and height.
        self._scene.add_box(name, p, (0.045, 0.045, 0.045))
        print "-------------test6------------------"
        return p.pose
        
    def _generate_grasps(self, pose, width):
        """
        Generate grasps by using the grasp generator generate action; based on
        server_test.py example on moveit_simple_grasps pkg.
        使用抓握生成器生成动作来生成抓握； 基于moveit_simple_grasps pkg上的server_test.py示例

        Generate the grasp Pose Array data for visualization, 
        and then send the grasp goal to the grasp server.
        生成抓取姿势阵列数据以进行可视化，然后将抓取目标发送到抓取服务器。
        """

        # Create goal:
        goal = GenerateGraspsGoal()

        goal.pose  = pose
        goal.width = width

        options = GraspGeneratorOptions()
        # simple_graps.cpp doesn't implement GRASP_AXIS_Z!
        #options.grasp_axis      = GraspGeneratorOptions.GRASP_AXIS_Z
        options.grasp_direction = GraspGeneratorOptions.GRASP_DIRECTION_UP
        options.grasp_rotation  = GraspGeneratorOptions.GRASP_ROTATION_FULL

        # @todo disabled because it works better with the default options
        #goal.options.append(options)

        # 发送目标并等待结果:
        # 将目标发送到ActionServer，等待目标完成，并且必须抢占目标.
        state = self._grasps_ac.send_goal_and_wait(goal)
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr('Grasp goal failed!: %s' % self._grasps_ac.get_goal_status_text())
            return None

        grasps = self._grasps_ac.get_result().grasps

        # 发布掌握（用于调试/可视化目的）:
        self._publish_grasps(grasps)
        print "-------------test7------------------"
        return grasps

    def _generate_places(self, target):
        """
        Generate places (place locations), based on
        https://github.com/davetcoleman/baxter_cpp/blob/hydro-devel/
        baxter_pick_place/src/block_pick_place.cpp

        为该位置的姿势创建姿势数组数据
        """

        # Generate places:
        places = []
        now = rospy.Time.now()
        for angle in numpy.arange(0.0, numpy.deg2rad(360.0), numpy.deg2rad(1.0)):
            # Create place location:
            place = PlaceLocation()

            place.place_pose.header.stamp = now
            place.place_pose.header.frame_id = self._robot.get_planning_frame()

            # Set target position:
            place.place_pose.pose = copy.deepcopy(target)

            # Generate orientation (wrt Z axis):
            q = quaternion_from_euler(0.0, 0.0, angle )
            place.place_pose.pose.orientation = Quaternion(*q)

            # 生成预安置方法:
            place.pre_place_approach.desired_distance = self._approach_retreat_desired_dist
            place.pre_place_approach.min_distance = self._approach_retreat_min_dist

            place.pre_place_approach.direction.header.stamp = now
            place.pre_place_approach.direction.header.frame_id = self._robot.get_planning_frame()

            place.pre_place_approach.direction.vector.x =  0
            place.pre_place_approach.direction.vector.y =  0
            place.pre_place_approach.direction.vector.z = 0.1

            # Generate post place approach:
            place.post_place_retreat.direction.header.stamp = now
            place.post_place_retreat.direction.header.frame_id = self._robot.get_planning_frame()

            place.post_place_retreat.desired_distance = self._approach_retreat_desired_dist
            place.post_place_retreat.min_distance = self._approach_retreat_min_dist

            place.post_place_retreat.direction.vector.x = 0
            place.post_place_retreat.direction.vector.y = 0
            place.post_place_retreat.direction.vector.z = 0.1

            # Add place:
            places.append(place)

        # Publish places (for debugging/visualization purposes):
        self._publish_places(places)
        print "-------------test8------------------"
        return places

    def _create_pickup_goal(self, group, target, grasps):
        """
        创建一个MoveIt！ 接送目标
         创建一个捡起抓取物体的目标
        """

        # Create goal:
        goal = PickupGoal()

        goal.group_name  = group
        goal.target_name = target

        goal.possible_grasps.extend(grasps)

        goal.allowed_touch_objects.append(target)

        goal.support_surface_name = self._table_object_name
        

        # 配置目标计划选项:
        goal.allowed_planning_time = 7.0

        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 20
        print "-------------test9------------------"
        return goal

    def _create_place_goal(self, group, target, places):
        """
        Create a MoveIt! PlaceGoal
        Create a place goal for MoveIt!
        """

        # Create goal:
        goal = PlaceGoal()

        goal.group_name           = group
        goal.attached_object_name = target

        goal.place_locations.extend(places)

        # Configure goal planning options:
        goal.allowed_planning_time = 7.0

        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 20
        print "-------------test10------------------"
        return goal

    def _pickup(self, group, target, width):
        """
        使用计划小组选择目标
        """

        # 从掌握生成器服务器获取可能的掌握:
        grasps = self._generate_grasps(self._pose_coke_can, width)

        # 创建并发送提货目标:
        goal = self._create_pickup_goal(group, target, grasps)

        state = self._pickup_ac.send_goal_and_wait(goal)
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr('Pick up goal failed!: %s' % self._pickup_ac.get_goal_status_text())
            return None

        result = self._pickup_ac.get_result()

        # Check for error:
        err = result.error_code.val
        if err != MoveItErrorCodes.SUCCESS:
            rospy.logwarn('Group %s cannot pick up target %s!: %s' % (group, target, str(moveit_error_dict[err])))

            return False
        print "-------------test11------------------"
        return True

    def _place(self, group, target, place):
        """
        使用计划组放置目标
        """

        # Obtain possible places:
        places = self._generate_places(place)

        # Create and send Place goal:
        goal = self._create_place_goal(group, target, places)

        state = self._place_ac.send_goal_and_wait(goal)
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr('Place goal failed!: %s' % self._place_ac.get_goal_status_text())
            return None

        result = self._place_ac.get_result()

        # Check for error:
        err = result.error_code.val
        if err != MoveItErrorCodes.SUCCESS:
            rospy.logwarn('Group %s cannot place target %s!: %s' % (group, target, str(moveit_error_dict[err])))

            return False
        print "-------------test12------------------"
        return True

    def _publish_grasps(self, grasps):
        """
        使用PoseArray消息将抓取发布为姿势
        """

        if self._grasps_pub.get_num_connections() > 0:
            msg = PoseArray()
            msg.header.frame_id = self._robot.get_planning_frame()
            msg.header.stamp = rospy.Time.now()

            for grasp in grasps:
                p = grasp.grasp_pose.pose

                msg.poses.append(Pose(p.position, p.orientation))

            self._grasps_pub.publish(msg)
        print "-------------test13------------------"
    def _publish_places(self, places):
        """
        使用PoseArray消息将位置发布为姿势
        """

        if self._places_pub.get_num_connections() > 0:
            msg = PoseArray()
            msg.header.frame_id = self._robot.get_planning_frame()
            msg.header.stamp = rospy.Time.now()

            for place in places:
                msg.poses.append(place.place_pose.pose)

            self._places_pub.publish(msg)
        print "-------------test14------------------"

def main():
    p = Pick_Place()
    print "-------------test15------------------"
    rospy.spin()


def spawn_gazebo_model(model_path, model_name, model_pose, reference_frame="world"):
  """
  Spawn model in gazebo
  """
  model_xml = ''
  with open(model_path, "r") as model_file:
    model_xml = model_file.read().replace('\n', '')
  rospy.wait_for_service('/gazebo/spawn_urdf_model')
  try:
    spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    resp_urdf = spawn_urdf(model_name, model_xml, "/", model_pose, reference_frame)
  except rospy.ServiceException, e:
    rospy.logerr("Spawn URDF service call failed: {0}".format(e))
  print "-------------test16------------------"
def delete_gazebo_model(models):
  """
  Delete model in gazebo
  """
  try:
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    for a_model in models:
      resp_delete = delete_model(a_model)
  except rospy.ServiceException, e:
    rospy.loginfo("Delete Model service call failed: {0}".format(e))
  print "-------------test17------------------"
if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('pick_and_place')
    print "-------------test18------------------"
    rospack = rospkg.RosPack()
    pack_path = rospack.get_path('ur5_single_arm_tufts')

    """
    rosrun gazebo_ros spawn_model -file $(rospack find ur5_single_arm_tufts)/urdf/objects/table.urdf -urdf -x 0.85 -y 0.0 -z 0.73 -model my_object
    rosrun gazebo_ros spawn_model -file $(rospack find ur5_single_arm_tufts)/urdf/objects/block.urdf -urdf -x 0.5 -y -0.0 -z 0.77 -model block
    """
    while(1):
        table_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'table.urdf'
        table_name = 'table'
        table_pose = Pose(position=Point(x=0.85, y=0.0, z=0.70+0.03)) # increase z by 0.03 to make gripper reach block

        table2_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'table2.urdf'
        table2 = 'table2'
        table2_pose = Pose(position=Point(x=0.0, y=0.85, z=0.4+0.03))
    
        block_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'block.urdf'
        block_name = 'block'
        block_pose = Pose(position=Point(x=0.5, y=0.0, z=0.74+0.03))
    
        dropbox_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'dropbox.urdf'
        dropbox = 'dropbox'
        dropbox_pose = Pose(position=Point(x=0, y=-0.85, z=0.24+0.03))

        delete_gazebo_model([table_name,table2,block_name,dropbox])
        spawn_gazebo_model(table_path, table_name, table_pose)
        spawn_gazebo_model(table2_path, table2, table2_pose)
        spawn_gazebo_model(block_path, block_name, block_pose)
        spawn_gazebo_model(dropbox_path, dropbox, dropbox_pose)
    
        main()
    print" ------yyyy----"
    #rospy.on_shutdown(self.shutdown)
    #roscpp_shutdown()
    
