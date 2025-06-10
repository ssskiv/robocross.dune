import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np
from scipy.spatial.distance import euclidean

class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('scan_filter_node')

        # Parameters
        self.declare_parameter('input_topic', '/scan/raw')
        self.declare_parameter('output_topic', '/scan/filtered')
        self.declare_parameter('distance_threshold', 0.40)  # clustering
        self.declare_parameter('max_perimeter', 3.0)        # metersz
        self.key = 4568.0

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.distance_threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value
        self.max_perimeter = self.get_parameter('max_perimeter').get_parameter_value().double_value

        self.sub = self.create_subscription(LaserScan, self.input_topic, self.scan_callback, 10)
        self.pub = self.create_publisher(LaserScan, self.output_topic, 10)

        self.get_logger().info(f"Filtering scan from '{self.input_topic}' to '{self.output_topic}'")

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        # angles =  msg.angle_min + np.arange(len(ranges)) * msg.angle_increment


        # Convert polar to cartesian
        # x = ranges * np.cos(angles)
        # y = ranges * np.sin(angles)
        points = ranges #list(zip(x, y))
        # self.get_logger().info(f'{msg.angle_increment}')

        # Cluster points
        clusters = []
        current_cluster = []
        s=0
        for i in range(len(points)):
            # self.get_logger().info(f'{points[i]}')
            if not np.isfinite(ranges[i]):
                current_cluster.append(self.key)
                s +=1
                continue
            if current_cluster:
                if abs(ranges[i]-current_cluster[-1])>self.distance_threshold:
                    clusters.append(current_cluster)
                    current_cluster = [] 
            current_cluster.append(points[i])
        if current_cluster:
            clusters.append(current_cluster)
        # self.get_logger().info(f'{s}')

        # Filter by perimeter
        def perimeter(cluster):
            per = 0
            if len(cluster)>1:
                for i in range(1,len(cluster)-1):
                    if cluster[i-1]!=self.key and cluster[i]!=self.key:
                        d=abs(cluster[i-1]-cluster[i])
                        per+=d
            # self.get_logger().info(f'{per}')
                return per
            else:
                return 0    
            # return sum(euclidean(cluster[i], cluster[i+1]) for i in range(len(cluster)-1))
        # clusters = [clusters[0]]
        filtered_indices = set()
        index = 0
        for cluster in clusters:
            # self.get_logger().info(f'{perimeter(cluster)}')
            if perimeter(cluster) <= self.max_perimeter and len(cluster)>5:
                # self.get_logger().info('Got cluster in bounds')
                for _ in cluster:
                    filtered_indices.add(index)
                    index += 1
            else:
                # self.get_logger().info(f'{index}')
                index += len(cluster)

        # Construct filtered scan
        filtered_ranges = list(msg.ranges)
        # self.get_logger().info(f'{index}')
        # self.get_logger().info(f'{len(clusters)}')
        for i in range(len(filtered_ranges)):
            if i not in filtered_indices:
                filtered_ranges[i] = float(np.inf)

        filtered_msg = LaserScan(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id=msg.header.frame_id
            ),
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            time_increment=msg.time_increment,
            scan_time=msg.scan_time,
            range_min=msg.range_min,
            range_max=msg.range_max,
            ranges=filtered_ranges,
            intensities=msg.intensities
        )

        self.pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
