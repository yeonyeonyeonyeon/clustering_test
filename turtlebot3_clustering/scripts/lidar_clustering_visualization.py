import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from sklearn.cluster import DBSCAN
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray

# 미리 정의된 색상 목록 (r, g, b) 값으로 구성
colors = [
    (1, 0, 0),        # Red
    (0, 1, 0),        # Green
    (0, 0, 1),        # Blue
    (1, 1, 0),        # Yellow
    (1, 0, 1),        # Magenta
    (0, 1, 1),        # Cyan
    (1, 0.5, 0),      # Orange
    (0.5, 1, 0),      # Lime
    (0.5, 0.5, 1),    # Sky blue
    (0.5, 0, 0.5),    # Purple
    (1, 0.5, 0.5),    # Pink
    (0.6, 0.3, 0),    # Brown
    (0.5, 0.5, 0.5),  # Gray
    (0, 0, 0.5),      # Navy
    (1, 0.8, 0)       # Gold
]

def compute_cluster_center(points):
    """Compute the center of a cluster."""
    return np.mean(points, axis=0)

def laser_callback(msg):
    t1 = rospy.Time.now()
    # LaserScan data to (x, y) points
    angle = msg.angle_min
    points = []
    for r in msg.ranges:
        if msg.range_min < r < msg.range_max:
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            points.append([x, y])
        angle += msg.angle_increment

    # Convert to numpy array for clustering
    np_points = np.array(points)

    # DBSCAN clustering
    clustering = DBSCAN(eps=0.1, min_samples=2).fit(np_points)
    labels = clustering.labels_

    # Visualization using MarkerArray
    marker_array = MarkerArray()

    unique_labels = list(set(labels))
    unique_labels.sort()  # Make sure the order of labels is consistent

    # Compute centers of each cluster
    cluster_centers = {}
    for label in unique_labels:
        if label == -1:  # noise
            continue
        class_member_mask = (labels == label)
        cluster_points = np_points[class_member_mask]
        center = compute_cluster_center(cluster_points)
        cluster_centers[label] = center

    # Sort clusters based on their centers (for example, by their x-coordinate)
    sorted_labels = sorted(cluster_centers.keys(), key=lambda x: cluster_centers[x][0])


    for idx, label in enumerate(sorted_labels):  # sorted_labels로 변경
        if label == -1:  # noise
            continue

        marker = Marker()
        marker.header.frame_id = "base_scan"
        marker.type = Marker.POINTS
        marker.id = idx  # ID는 idx로 설정 (정렬된 순서대로)
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.a = 1.0

        # Assign predefined colors for each cluster
        marker.color.r = colors[idx % len(colors)][0]  # len(colors)로 변경
        marker.color.g = colors[idx % len(colors)][1]
        marker.color.b = colors[idx % len(colors)][2]

        class_member_mask = (labels == label)
        cluster_points = np_points[class_member_mask]

        for point in cluster_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            marker.points.append(p)

        marker_array.markers.append(marker)

    pub.publish(marker_array)
    t2 = rospy.Time.now()
    rospy.logwarn("[] cycle time : %s"%(t2-t1).to_sec())

rospy.init_node('lidar_clustering_visualization')
sub = rospy.Subscriber('/scan', LaserScan, laser_callback)
pub = rospy.Publisher('/lidar_clusters', MarkerArray, queue_size=10)
rospy.spin()