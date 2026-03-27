#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import math

class GazeboQuadrupedNode(Node):
    def __init__(self):
        super().__init__('gazebo_quadruped_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_group_position_controller/commands', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.cmd_x = 0.0 
        self.cmd_w = 0.0
            
        self.walk_time = 0.0     
        self.dt = 0.05           
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info("Spider Brain Online! Waiting for keyboard commands...")

    def cmd_vel_callback(self, msg):
        self.cmd_x = msg.linear.x
        self.cmd_w = msg.angular.z

    # --- THE NEW SPIDER MATH ---
    def get_spider_gait(self, t, phase_offset):
        T = 2.5 # time to complet the movement
        duty_factor = 0.60    # 60% of the time on the ground
        cycle_progress = ((t / T) + phase_offset) % 1.0
        
        swing_amplitude = 0.4 # How far the shoulder sweeps forward/backward
        lift_amplitude = 0.6  # How high the knee bends to clear the floor
        
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
            
        # Pair A (Front Left, Back Right) and Pair B (Front Right, Back Left)
        sweep_A, knee_A = self.get_spider_gait(self.walk_time, 0.0)
        sweep_B, knee_B = self.get_spider_gait(self.walk_time, 0.5)

        amp_FL = (self.cmd_x - (self.cmd_w * 0.5))
        amp_FR = (self.cmd_x + (self.cmd_w * 0.5))
        amp_BL = (self.cmd_x - (self.cmd_w * 0.5))
        amp_BR = (self.cmd_x + (self.cmd_w * 0.5))

        # Build the final array. 
        # +/- 1.57 centers the thighs so they point straight forward/backward
        msg = Float64MultiArray()
        msg.data = [
            float(amp_FL * sweep_A),   float(knee_A),   # Front Left
            float(-(amp_BR * sweep_A)), float(knee_A),   # Back Right (Mirrored)
            float(-(amp_FR * sweep_B)), float(knee_B),   # Front Right (Mirrored)
            float(amp_BL * sweep_B),   float(knee_B)    # Back Left
        ]
        
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
