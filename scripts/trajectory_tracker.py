#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from quadrotor_msgs.msg import PositionCommand

class TrajectoryTracker:
    def __init__(self):
        rospy.init_node('trajectory_tracker')

        self.current_state = State()
        self.target_pos = None
        self.hover_pose = None
        
        # Initial pose capture flag
        self.initial_pose_captured = False

        # Subscribers
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_cb)
        self.local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_pos_cb)
        
        # Subscribe to ego_planner's PositionCommand
        self.pos_cmd_sub = rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.pos_cmd_cb)

        # Publishers
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # Services
        rospy.wait_for_service('mavros/cmd/arming')
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        
        rospy.wait_for_service('mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        self.rate = rospy.Rate(50) 

    def state_cb(self, msg):
        self.current_state = msg

    def local_pos_cb(self, msg):
        # Capture initial position once, to set as hover point
        if not self.initial_pose_captured:
            self.hover_pose = msg
            self.hover_pose.pose.position.z += 1.0 # Takeoff 1m above current
            self.initial_pose_captured = True
            rospy.loginfo(f"Initial hover pose set to: x={self.hover_pose.pose.position.x:.2f}, y={self.hover_pose.pose.position.y:.2f}, z={self.hover_pose.pose.position.z:.2f}")

    def pos_cmd_cb(self, msg):
        self.target_pos = msg

    def run(self):
        # Wait for Flight Controller connection
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

        # Wait for initial position
        while not rospy.is_shutdown() and not self.initial_pose_captured:
            rospy.loginfo_throttle(1.0, "Waiting for initial position...")
            self.rate.sleep()

        pose = self.hover_pose

        # Send a few setpoints before starting
        for i in range(100):
            if rospy.is_shutdown():
                break
            pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(pose)
            self.rate.sleep()

        last_req = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client(base_mode=0, custom_mode="OFFBOARD").mode_sent:
                    rospy.loginfo("Offboard enabled")
                last_req = rospy.Time.now()
            else:
                if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if self.arming_client(True).success:
                        rospy.loginfo("Vehicle armed")
                    last_req = rospy.Time.now()

            # Calculate setpoint
            pose = self.calculate_setpoint()
            
            pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(pose)

            self.rate.sleep()

    def calculate_setpoint(self):
        # If no target from planner (stopped or idle), hold hover or last position
        if self.target_pos is None:
             return self.hover_pose
        
        # Construct PoseStamped from PositionCommand
        p = PoseStamped()
        p.header = self.target_pos.header
        p.header.frame_id = "map" # Ensure frame is map

        p.pose.position = self.target_pos.position
        
        # Convert yaw to quaternion
        q = tf.transformations.quaternion_from_euler(0, 0, self.target_pos.yaw)
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        
        # update hover pose to last known valid pose
        self.hover_pose = p
        
        return p

if __name__ == "__main__":
    try:
        tracker = TrajectoryTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
