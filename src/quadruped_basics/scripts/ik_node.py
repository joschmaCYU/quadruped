#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class GazeboQuadrupedNode(Node):
    def __init__(self):
        super().__init__('gazebo_quadruped_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_group_position_controller/commands', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.dashboard_override = False
        self.override_sub = self.create_subscription(Bool, '/dashboard_override', self.override_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.cmd_x = 0.0 
        self.cmd_w = 0.0
            
        self.walk_time = 0.0     
        self.dt = 0.05           
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info("Spider Brain Online! Waiting for keyboard commands...")

    def override_callback(self, msg):
        self.dashboard_override = msg.data

    def cmd_vel_callback(self, msg):
        self.cmd_x = msg.linear.x
        self.cmd_w = msg.angular.z

    # --- THE NEW SPIDER MATH ---
    def get_spider_gait(self, t, phase_offset):
        T = 2 # time to complet the movement
        duty_factor = 0.60    # 60% of the time on the ground
        cycle_progress = ((t / T) + phase_offset) % 1.0
        
        swing_amplitude = 0.85 # How far the shoulder sweeps forward/backward
        lift_amplitude = 0.85  # How high the knee bends to clear the floor
        
        if cycle_progress < duty_factor:
            # STANCE PHASE: Foot on the floor, pushing the robot forward
            stance_p = cycle_progress / duty_factor 
            # Sweep from +amplitude (Forward) to -amplitude (Backward)
            shoulder_move = swing_amplitude - (2 * swing_amplitude * stance_p)
            knee_move = 0.0 # Keep knee pointing straight down at the floor
        else:
            # SWING PHASE: Foot in the air, reaching forward for the next step
            swing_p = (cycle_progress - duty_factor) / (1.0 - duty_factor) 
            # Sweep from -amplitude (Backward) to +amplitude (Forward)
            shoulder_move = -swing_amplitude + (2 * swing_amplitude * swing_p)
            # Use a sine wave to lift the knee into the air and put it back down
            knee_move = lift_amplitude * math.sin(swing_p * math.pi)

        return shoulder_move, knee_move

    def timer_callback(self):
        if self.cmd_x != 0.0 or self.cmd_w != 0.0:
            self.walk_time += self.dt
            # --- CALCULATE SMOOTH ODOMETRY ---
            # These multipliers match the physical speed of your Gazebo robot
            speed_multiplier = 0.03 
            turn_multiplier = 0.03

            # Calculate our new X, Y, and rotation based on keyboard inputs!
            self.odom_yaw += (self.cmd_w * turn_multiplier) * self.dt
            self.odom_x += (self.cmd_x * speed_multiplier * math.cos(self.odom_yaw)) * self.dt
            self.odom_y += (self.cmd_x * speed_multiplier * math.sin(self.odom_yaw)) * self.dt

        # --- 2. PUBLISH THE ODOMETRY TO SLAM ---
        # We publish this constantly so SLAM never loses track of the robot
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0

        # Convert our flat rotation into a 3D Quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.odom_yaw / 2.0)
        t.transform.rotation.w = math.cos(self.odom_yaw / 2.0)

        self.tf_broadcaster.sendTransform(t)
            
        # Pair A (Front Left, Back Right) and Pair B (Front Right, Back Left)
        sweep_A, knee_A = self.get_spider_gait(self.walk_time, 0.0)
        sweep_B, knee_B = self.get_spider_gait(self.walk_time, 0.5)

        amp_FL = (self.cmd_x - (self.cmd_w * 1.5))
        amp_FR = (self.cmd_x + (self.cmd_w * 1.5))
        amp_BL = (self.cmd_x - (self.cmd_w * 1.5))
        amp_BR = (self.cmd_x + (self.cmd_w * 1.5))

        # Build the final array. 
        # +/- 1.57 centers the thighs so they point straight forward/backward
        msg = Float64MultiArray()
        msg.data = [
            float(amp_FL * sweep_A),   float(knee_A),   # Front Left
            float(-(amp_BR * sweep_A)), float(knee_A),   # Back Right (Mirrored)
            float(-(amp_FR * sweep_B)), float(knee_B),   # Front Right (Mirrored)
            float(amp_BL * sweep_B),   float(knee_B)    # Back Left
        ]
        
        if not self.dashboard_override:
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboQuadrupedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
