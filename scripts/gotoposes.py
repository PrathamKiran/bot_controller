#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
import tf2_ros
import math

class PoseControl(Node):
    def __init__(self):
        super().__init__('goto_pose')


        self.max_linear_vel = 0.2
        self.max_angular_vel = 2

        # Minimum angular velocity is the negative of the maximum angular velocity
        self.min_angular_vel = -self.max_angular_vel

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.offset_length = 0.1
        self.waypoints = [(1.8, 0.3), (1.2, 0.3), (0.9, 0.15), (1.2, 0.0), (1.5, -0.15), (1.2, -0.3), (0.6, -0.3)]
        self.current_waypoint = 0

        self.control_timer = self.create_timer(0.1, self.control_loop)

    def set_target(self, x, y):
        self.waypoints.append((x, y))

    def control_loop(self):
        if self.current_waypoint >= len(self.waypoints):
            self.stop_robot()
            return

        target_x, target_y = self.waypoints[self.current_waypoint]
        self.set_target(target_x, target_y)

        try:
            trans = self.tf_buffer.lookup_transform('world', 'aruco_marker_0', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().info(f'Transform not found: {e}')
            return

        aruco_x = trans.transform.translation.x
        aruco_y = trans.transform.translation.y
        current_theta = self.quaternion_to_euler(trans.transform.rotation) + math.pi/2

        current_x = aruco_x + self.offset_length * math.sin(current_theta) 
        current_y = aruco_y - self.offset_length * math.cos(current_theta) 

        # print(current_x,current_y)
        target_x, target_y = self.waypoints[self.current_waypoint]

        error_x = target_x - current_x
        error_y = target_y - current_y

        distance = math.sqrt(error_x**2 + error_y**2)
        angle_to_target = math.atan2(error_y, error_x)
        angle_error = self.normalize_angle(angle_to_target - current_theta)
        print(angle_error)

        kp = 2 # Proportional control for linear velocity
        kt = 2 # Proportinal control for angular vel
        if distance > 0.04:
            linear_vel = min(self.max_linear_vel, kp*distance*((math.pi-abs(angle_error))/math.pi))
            if angle_error>=0:
                angular_vel = min(self.max_angular_vel, kt * angle_error)
            else:
                angular_vel = max(self.min_angular_vel, kt * angle_error)
        else:
            linear_vel = 0.0
            angular_vel = 0.0
            self.current_waypoint += 1

        twist_msg = Twist()
        twist_msg.linear.x = float(linear_vel)
        twist_msg.angular.z = float(angular_vel)
        self.get_logger().info(f"Going to position:{target_x,target_y}Linear Velocity: {linear_vel}, Angular Velocity: {angular_vel}")
        self.cmd_vel_publisher.publish(twist_msg)


    def quaternion_to_euler(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        normalized_angle = angle
        while normalized_angle > math.pi:
            normalized_angle -= 2 * math.pi
        while normalized_angle < -math.pi:
            normalized_angle += 2 * math.pi
        return normalized_angle

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info('Stopping the robot.')

def main(args=None):
    rclpy.init(args=args)
    controller = PoseControl()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.stop_robot()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
