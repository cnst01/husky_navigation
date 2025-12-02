#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler
import os


class GpsToPose(Node):
    def __init__(self):
        super().__init__('gps_to_pose')
        
        # Obter parâmetros
        self.declare_parameter('use_sim_time', True)
        # namespace pode ser fornecido por parâmetro ROS ou pela variável de ambiente ROBOT_NAMESPACE
        self.declare_parameter('namespace', os.environ.get('ROBOT_NAMESPACE', 'a200_0000'))
        self.declare_parameter('publish_initial_pose', True)
        self.declare_parameter('reference_latitude', -22.001)
        self.declare_parameter('reference_longitude', -47.001)
        
        namespace = self.get_parameter('namespace').value
        self.publish_initial_pose = self.get_parameter('publish_initial_pose').value
        self.reference_lat = self.get_parameter('reference_latitude').value
        self.reference_lon = self.get_parameter('reference_longitude').value
        
        # Fatores de conversão
        self.lat_to_meters = 111320.0
        self.lon_to_meters = 111320.0 * 0.788
        
        # Tópicos
        gps_topic = f'/{namespace}/sensors/gps_0/fix'
        initialpose_topic = '/initialpose'
        
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
        
        # Controlar publicação única
        self.initial_pose_published = False
        self.gps_fix_count = 0
        
        self.get_logger().info(f'GPS to Pose node initialized for namespace: {namespace}')
        self.get_logger().info(f'Reference coordinates: lat={self.reference_lat}, lon={self.reference_lon}')

    def gps_callback(self, msg):
        self.gps_fix_count += 1
        
        # Esperar alguns fixes antes de publicar para garantir qualidade
        if (msg.status.status >= 0 and 
            self.publish_initial_pose and 
            not self.initial_pose_published and
            self.gps_fix_count >= 3):  # Esperar pelo menos 3 fixes
            
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            
            # Converter coordenadas GPS para coordenadas do mapa
            x = (msg.longitude - self.reference_lon) * self.lon_to_meters
            y = (msg.latitude - self.reference_lat) * self.lat_to_meters
            
            pose_msg.pose.pose.position.x = x
            pose_msg.pose.pose.position.y = y
            pose_msg.pose.pose.position.z = 0.0
            
            # Orientação neutra
            q = quaternion_from_euler(0, 0, 0)
            pose_msg.pose.pose.orientation.x = q[0]
            pose_msg.pose.pose.orientation.y = q[1]
            pose_msg.pose.pose.orientation.z = q[2]
            pose_msg.pose.pose.orientation.w = q[3]
            
            # Covariância - alta incerteza inicial
            covariance = [0.0] * 36
            covariance[0] = 5.0  # x
            covariance[7] = 5.0  # y  
            covariance[35] = 1.57  # yaw (90 graus de incerteza)
            pose_msg.pose.covariance = covariance
            
            self.pose_pub.publish(pose_msg)
            self.initial_pose_published = True
            
            self.get_logger().info(f'Published initial pose from GPS: x={x:.2f}, y={y:.2f}')
            self.get_logger().info('SLAM will now use this as starting point')

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