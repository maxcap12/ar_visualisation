#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs 

class MarkerTransformer(Node):
    def __init__(self):
        super().__init__('marker_transformer')

        self.target_frame = 'map'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscriber = self.create_subscription(
            MarkerArray,
            '/s_graphs/markers',
            self.marker_callback,
            10
        )

        self.publisher = self.create_publisher(MarkerArray, '/transformed_marker_array', 10)

    def marker_callback(self, msg: MarkerArray):
        transformed_array = MarkerArray()

        for marker in msg.markers:
            new_marker = Marker()
            try:
                # Set basic info
                new_marker.header.frame_id = self.target_frame
                new_marker.header.stamp = self.get_clock().now().to_msg()
                new_marker.ns = marker.ns
                new_marker.id = marker.id
                new_marker.type = marker.type
                new_marker.action = marker.action
                new_marker.scale = marker.scale
                new_marker.color = marker.color
                new_marker.lifetime = marker.lifetime
                new_marker.frame_locked = False  # Usually best to disable

                # Transform pose (for markers that use it)
                if marker.type in [Marker.ARROW, Marker.CUBE, Marker.SPHERE, Marker.CYLINDER, Marker.TEXT_VIEW_FACING, Marker.MESH_RESOURCE]:
                    pose_stamped = PoseStamped()
                    pose_stamped.header = marker.header
                    pose_stamped.pose = marker.pose

                    transformed_pose = self.tf_buffer.transform(
                        pose_stamped,
                        self.target_frame,
                        timeout=Duration(seconds=1.0)
                    )
                    new_marker.pose = transformed_pose.pose

                else:
                    new_marker.pose = marker.pose  # Optional, keep as fallback

                # Transform points (for LINE_LIST, LINE_STRIP, SPHERE_LIST, etc.)
                new_marker.points = []
                for point in marker.points:
                    point_stamped = PointStamped()
                    point_stamped.header = marker.header
                    point_stamped.point = point

                    transformed_point = self.tf_buffer.transform(
                        point_stamped,
                        self.target_frame,
                        timeout=Duration(seconds=1.0)
                    )
                    new_marker.points.append(transformed_point.point)

                # Copy other fields
                new_marker.text = marker.text
                new_marker.mesh_resource = marker.mesh_resource
                new_marker.mesh_use_embedded_materials = marker.mesh_use_embedded_materials

                transformed_array.markers.append(new_marker)

            except Exception as e:
                self.get_logger().warn(f'Could not transform marker {marker.id}: {e}')

        self.publisher.publish(transformed_array)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()