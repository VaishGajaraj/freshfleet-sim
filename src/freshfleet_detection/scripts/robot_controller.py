#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import MoveItErrorCodes
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        
        # Initialize MoveIt commander
        self.move_group = MoveGroupCommander("manipulator")
        self.scene = PlanningSceneInterface()
        
        # Set planning parameters
        self.move_group.set_planning_time(5.0)
        self.move_group.set_max_velocity_scaling_factor(0.3)
        self.move_group.set_max_acceleration_scaling_factor(0.3)
        
        # Publishers
        self.status_pub = rospy.Publisher('/robot_status', String, queue_size=10)
        
        # Subscribers
        self.detection_sub = rospy.Subscriber('/detected_objects', PoseStamped, self.detection_callback)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        # TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Robot state
        self.current_joint_state = None
        self.is_moving = False
        
        # Home position (adjust based on your setup)
        self.home_position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        
        # Pick and place positions
        self.pick_offset = 0.05  # Offset above object for picking
        self.place_position = Pose()
        self.place_position.position.x = 0.6
        self.place_position.position.y = 0.4
        self.place_position.position.z = 0.3
        self.place_position.orientation.w = 1.0
        
        rospy.loginfo("Robot controller initialized")
        self.publish_status("Ready")
        
        # Move to home position
        self.go_home()
    
    def joint_state_callback(self, msg):
        """Update current joint state"""
        self.current_joint_state = msg
    
    def detection_callback(self, msg):
        """Handle object detection and plan pick operation"""
        if self.is_moving:
            rospy.logwarn("Robot is currently moving, ignoring new detection")
            return
        
        rospy.loginfo(f"Received detection at position: ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})")
        
        # Plan and execute pick operation
        self.pick_object(msg.pose)
    
    def go_home(self):
        """Move robot to home position"""
        try:
            self.is_moving = True
            self.publish_status("Moving to home")
            
            self.move_group.set_joint_value_target(self.home_position)
            plan = self.move_group.plan()
            
            if plan[0]:
                success = self.move_group.execute(plan[1])
                if success:
                    rospy.loginfo("Successfully moved to home position")
                    self.publish_status("At home")
                else:
                    rospy.logerr("Failed to execute home movement")
                    self.publish_status("Home movement failed")
            else:
                rospy.logerr("Failed to plan home movement")
                self.publish_status("Home planning failed")
                
        except Exception as e:
            rospy.logerr(f"Error in go_home: {e}")
            self.publish_status("Home error")
        finally:
            self.is_moving = False
    
    def pick_object(self, object_pose):
        """Execute pick operation for detected object"""
        try:
            self.is_moving = True
            self.publish_status("Planning pick")
            
            # Create pick pose (slightly above object)
            pick_pose = Pose()
            pick_pose.position.x = object_pose.position.x
            pick_pose.position.y = object_pose.position.y
            pick_pose.position.z = object_pose.position.z + self.pick_offset
            pick_pose.orientation = object_pose.orientation
            
            # Plan approach movement
            self.move_group.set_pose_target(pick_pose)
            plan = self.move_group.plan()
            
            if not plan[0]:
                rospy.logerr("Failed to plan pick approach")
                self.publish_status("Pick planning failed")
                return
            
            # Execute approach
            self.publish_status("Approaching object")
            success = self.move_group.execute(plan[1])
            if not success:
                rospy.logerr("Failed to execute pick approach")
                self.publish_status("Pick approach failed")
                return
            
            # Move down to object (simple linear movement)
            rospy.sleep(1.0)  # Wait for approach to complete
            
            # Plan grasp movement (move down to object)
            grasp_pose = Pose()
            grasp_pose.position.x = object_pose.position.x
            grasp_pose.position.y = object_pose.position.y
            grasp_pose.position.z = object_pose.position.z
            grasp_pose.orientation = object_pose.orientation
            
            self.move_group.set_pose_target(grasp_pose)
            plan = self.move_group.plan()
            
            if plan[0]:
                self.publish_status("Grasping object")
                success = self.move_group.execute(plan[1])
                if success:
                    rospy.loginfo("Successfully grasped object")
                    self.publish_status("Object grasped")
                    
                    # Move to place position
                    self.place_object()
                else:
                    rospy.logerr("Failed to execute grasp")
                    self.publish_status("Grasp failed")
            else:
                rospy.logerr("Failed to plan grasp")
                self.publish_status("Grasp planning failed")
                
        except Exception as e:
            rospy.logerr(f"Error in pick_object: {e}")
            self.publish_status("Pick error")
        finally:
            self.is_moving = False
    
    def place_object(self):
        """Execute place operation"""
        try:
            self.publish_status("Planning place")
            
            # Plan place movement
            self.move_group.set_pose_target(self.place_position)
            plan = self.move_group.plan()
            
            if not plan[0]:
                rospy.logerr("Failed to plan place movement")
                self.publish_status("Place planning failed")
                return
            
            # Execute place
            self.publish_status("Placing object")
            success = self.move_group.execute(plan[1])
            if success:
                rospy.loginfo("Successfully placed object")
                self.publish_status("Object placed")
                
                # Return to home
                rospy.sleep(1.0)
                self.go_home()
            else:
                rospy.logerr("Failed to execute place")
                self.publish_status("Place failed")
                
        except Exception as e:
            rospy.logerr(f"Error in place_object: {e}")
            self.publish_status("Place error")
    
    def publish_status(self, status):
        """Publish robot status"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
        rospy.loginfo(f"Robot status: {status}")
    
    def get_current_pose(self):
        """Get current end effector pose"""
        return self.move_group.get_current_pose()
    
    def get_current_joint_values(self):
        """Get current joint values"""
        return self.move_group.get_current_joint_values()

if __name__ == '__main__':
    try:
        controller = RobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 