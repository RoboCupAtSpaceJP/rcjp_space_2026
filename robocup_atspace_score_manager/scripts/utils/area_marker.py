#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray

def create_marker(name, ranges, marker_id):
    marker = Marker()
    marker.header.frame_id = "iss_body"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "areas"
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    x_min, x_max = ranges['x_range']
    y_min, y_max = ranges['y_range']
    z_min, z_max = ranges['z_range']

    marker.pose.position.x = (x_min + x_max) / 2.0
    marker.pose.position.y = (y_min + y_max) / 2.0
    marker.pose.position.z = (z_min + z_max) / 2.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = abs(x_max - x_min)
    marker.scale.y = abs(y_max - y_min)
    marker.scale.z = abs(z_max - z_min)

    marker.color.a = 0.5
    if "docking" in name:
        marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
    elif "navigation" in name:
        marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
    elif "search" in name:
        marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
    return marker

def main():
    rospy.init_node('area_marker_publisher')

    pub = rospy.Publisher('area_marker_array', MarkerArray, queue_size=10, latch=True)
    
    rules = rospy.get_param('/rules')

    target_areas = rules.get('areas').get('name')
    marker_array = MarkerArray()
    marker_id = 0
    
    for area_name in target_areas:
        ranges = rules.get('areas').get(area_name)
        if ranges is None:
            rospy.logwarn(f"Area '{area_name}' not found in rules. Skipping marker creation.")
            continue
        marker = create_marker(area_name, ranges, marker_id)
        marker_array.markers.append(marker)
        marker_id += 1

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        for m in marker_array.markers:
            m.header.stamp = current_time
        pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass