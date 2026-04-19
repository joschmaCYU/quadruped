#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
import math

class GazeboQuadrupedNode(Node):
    def __init__(self):
        super().__init__('gazebo_quadruped_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_group_position_controller/commands', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

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
    # def get_spider_gait(self, t, phase_offset):
    #     T = 2 # time to complet the movement
    #     duty_factor = 0.60    # 60% of the time on the ground
    #     cycle_progress = ((t / T) + phase_offset) % 1.0
    #
    #     swing_amplitude = 0.85 # How far the shoulder sweeps forward/backward
    #     lift_amplitude = 0.85  # How high the knee bends to clear the floor
    #
    #     if cycle_progress < duty_factor:
    #         # STANCE PHASE: Foot on the floor, pushing the robot forward
    #         stance_p = cycle_progress / duty_factor 
    #         # Sweep from +amplitude (Forward) to -amplitude (Backward)
    #         shoulder_move = swing_amplitude - (2 * swing_amplitude * stance_p)
    #         knee_move = 0.0 # Keep knee pointing straight down at the floor
    #     else:
    #         # SWING PHASE: Foot in the air, reaching forward for the next step
    #         swing_p = (cycle_progress - duty_factor) / (1.0 - duty_factor) 
    #         # Sweep from -amplitude (Backward) to +amplitude (Forward)
    #         shoulder_move = -swing_amplitude + (2 * swing_amplitude * swing_p)
    #         # Use a sine wave to lift the knee into the air and put it back down
    #         knee_move = lift_amplitude * math.sin(swing_p * math.pi)
    #
    #     return shoulder_move, knee_move

    def calculate_ik(self, x, z):
        # 1. Physical Leg Lengths
        L1 = 0.206  # Pink thigh (horizontal sweep)
        L2 = 0.250  # Yellow calf (vertical lift)

        # 2. Safety Cap! Prevent the math from crashing if we ask it to stretch too far.
        z = max(z, -0.24) 
        z = min(z, 0.0)

        # 3. KNEE MATH (Controls Height)
        # Because your URDF draws the calf pointing straight down when angle is 0:
        # A 0 angle means straight down. A larger angle bends it outwards.
        cos_knee = abs(z) / L2
        cos_knee = max(0.0, min(1.0, cos_knee))
        knee_angle = math.acos(cos_knee) 

        # 4. SHOULDER MATH (Controls Forward Stride)
        # Calculate how far out the foot currently is due to the knee bend
        horizontal_reach = L1 + (L2 * math.sin(knee_angle))
        
        # Calculate the sweep angle required to move 'x' meters forward
        step_reach = x / horizontal_reach
        step_reach = max(-1.0, min(1.0, step_reach))
        shoulder_angle = math.asin(step_reach)

        return shoulder_angle, knee_angle

    def get_ik_gait(self, t, phase_offset, step_scale):
        T = 0.80
        duty_factor = 0.60
        cycle_progress = ((t / T) + phase_offset) % 1.0
        
        # --- SPIDER GAIT SETTINGS ---
        stride_length = 0.25  # 15cm max steps
        step_height = 0.08    # Lift foot 8cm into the air
        stand_height = -0.25  # Keep the hip 20cm off the floor (Safe for L2=25cm)
        
        if cycle_progress < duty_factor:
            # STANCE PHASE
            stance_p = cycle_progress / duty_factor 
            target_x = (stride_length / 2.0) - (stride_length * stance_p)
            target_z = stand_height 
        else:
            # SWING PHASE
            swing_p = (cycle_progress - duty_factor) / (1.0 - duty_factor) 
            target_x = -(stride_length / 2.0) + (stride_length * swing_p)
            target_z = stand_height + (step_height * math.sin(swing_p * math.pi))
            
        # SCALE THE PHYSICAL STEP (Not the joint angle!)
        target_x = target_x * step_scale
        
        return self.calculate_ik(target_x, target_z)

    def timer_callback(self):
        # 1. Deadband filter to ignore tiny noise from Nav2
        actual_cmd_x = self.cmd_x if abs(self.cmd_x) > 0.01 else 0.0
        actual_cmd_w = self.cmd_w if abs(self.cmd_w) > 0.01 else 0.0

        # 2. Apply trim ONLY to the legs, NOT the odometry
        leg_cmd_w = actual_cmd_w
        trim_value = -0.20 # to counter the drift 
        
        if actual_cmd_x != 0.0 and actual_cmd_w == 0.0:
            leg_cmd_w = trim_value

        if actual_cmd_x != 0.0 or actual_cmd_w != 0.0:
            self.walk_time += self.dt

        speed_multiplier = 0.08247
        turn_multiplier = 0.188

        # 3. Odometry uses the PURE command (actual_cmd), ignoring the trim
        self.odom_yaw += (actual_cmd_w * turn_multiplier) * self.dt
        self.odom_x += (actual_cmd_x * speed_multiplier * math.cos(self.odom_yaw)) * self.dt
        self.odom_y += (actual_cmd_x * speed_multiplier * math.sin(self.odom_yaw)) * self.dt

        # --- TF BROADCASTER ---
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.odom_yaw / 2.0)
        t.transform.rotation.w = math.cos(self.odom_yaw / 2.0)

        self.tf_broadcaster.sendTransform(t)

        # --- ODOMETRY PUBLISHER ---
        odom_msg = Odometry()
        odom_msg.header.stamp = t.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.orientation = t.transform.rotation
        
        # Give Nav2 the PURE speed, ignoring trim
        odom_msg.twist.twist.linear.x = actual_cmd_x * speed_multiplier
        odom_msg.twist.twist.angular.z = actual_cmd_w * turn_multiplier
        
        self.odom_pub.publish(odom_msg)

        # --- KINEMATICS ---
        is_moving = (actual_cmd_x != 0.0 or actual_cmd_w != 0.0)
        
        msg = Float64MultiArray()
            
        if not is_moving:
            # IDLE STATE: Snap to perfect standing pose
            stand_height = -0.20
            idle_shoulder, idle_knee = self.calculate_ik(0.0, stand_height)
            msg.data = [
                float(idle_shoulder),   float(idle_knee),   # FL
                float(-idle_shoulder),  float(idle_knee),   # BR (Mirrored)
                float(-idle_shoulder),  float(idle_knee),   # FR (Mirrored)
                float(idle_shoulder),   float(idle_knee)    # BL
            ]
        else:
            # WALKING STATE: Use leg_cmd_w which contains the mechanical trim
            amp_FL = (actual_cmd_x - (leg_cmd_w * 1.5))
            amp_FR = (actual_cmd_x + (leg_cmd_w * 1.5))
            amp_BL = (actual_cmd_x - (leg_cmd_w * 1.5))
            amp_BR = (actual_cmd_x + (leg_cmd_w * 1.5))

            shoulder_FL, knee_FL = self.get_ik_gait(self.walk_time, 0.0, amp_FL)
            shoulder_BR, knee_BR = self.get_ik_gait(self.walk_time, 0.0, amp_BR)
            
            shoulder_FR, knee_FR = self.get_ik_gait(self.walk_time, 0.5, amp_FR)
            shoulder_BL, knee_BL = self.get_ik_gait(self.walk_time, 0.5, amp_BL)

            msg.data = [
                float(shoulder_FL),   float(knee_FL),   # Front Left
                float(-shoulder_BR),  float(knee_BR),   # Back Right
                float(-shoulder_FR),  float(knee_FR),   # Front Right
                float(shoulder_BL),   float(knee_BL)    # Back Left
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
