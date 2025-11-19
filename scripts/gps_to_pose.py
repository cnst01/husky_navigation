#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler


class GpsToPose(Node):
    def __init__(self):
        super().__init__('gps_to_pose')
        
        # Obter namespace dos parâmetros
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('namespace', 'a200_0000')
        
        namespace = self.get_parameter('namespace').value
        
        # Parâmetros do GPS de referência
        self.declare_parameter('reference_latitude', -22.001)
        self.declare_parameter('reference_longitude', -47.001)
        
        self.reference_lat = self.get_parameter('reference_latitude').value
        self.reference_lon = self.get_parameter('reference_longitude').value
        
        # Fatores de conversão
        self.lat_to_meters = 111320.0
        self.lon_to_meters = 111320.0 * 0.788
        
        # Tópicos com namespace
        gps_topic = f'/{namespace}/sensors/gps_0/fix'
        initialpose_topic = '/initialpose'  # AMCL geralmente escuta neste tópico global
        
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            initialpose_topic,
            10
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            gps_topic,
            self.gps_callback,
            10
        )
        
        self.get_logger().info(f'GPS to Pose node initialized for namespace: {namespace}')
        self.get_logger().info(f'Listening to: {gps_topic}')
        self.get_logger().info(f'Publishing to: {initialpose_topic}')

    def gps_callback(self, msg):
        if msg.status.status >= 0:  # GPS fix válido
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            
            # Converter coordenadas GPS para coordenadas do mapa
            x = (msg.longitude - self.reference_lon) * self.lon_to_meters
            y = (msg.latitude - self.reference_lat) * self.lat_to_meters
            
            pose_msg.pose.pose.position.x = x
            pose_msg.pose.pose.position.y = y
            pose_msg.pose.pose.position.z = 0.0
            
            # Orientação
            q = quaternion_from_euler(0, 0, 0)
            pose_msg.pose.pose.orientation.x = q[0]
            pose_msg.pose.pose.orientation.y = q[1]
            pose_msg.pose.pose.orientation.z = q[2]
            pose_msg.pose.pose.orientation.w = q[3]
            
            # Covariância
            covariance = [0.0] * 36
            covariance[0] = 2.0  # x
            covariance[7] = 2.0  # y  
            covariance[35] = 0.1  # yaw
            pose_msg.pose.covariance = covariance
            
            self.pose_pub.publish(pose_msg)
            
            self.get_logger().info(f'Published initial pose from GPS: x={x:.2f}, y={y:.2f}')

def main():
    rclpy.init()
    node = GpsToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()