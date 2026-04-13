#!/usr/bin/env python3

import rospy
import tf

def publish_search_points():
    rospy.init_node('search_point_tf_publisher')
    tf_broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        rules = rospy.get_param('/rules')
        search_points = rules.get('search_points')
        
        point_names = search_points.get('fixed', {}).get('name', [])

        for name in point_names:
            point_data = search_points.get('fixed').get(name)
            
            if point_data is None:
                rospy.logwarn(f"Search point '{name}' not found in rules. Skipping.")
                continue
            pos = point_data.get('position')
            rot = point_data.get('rotation')

            if pos and rot:
                tf_broadcaster.sendTransform(
                    (pos[0], pos[1], pos[2]),
                    rotation_to_quaternion(*rot),
                    rospy.Time.now(),
                    name,
                    "iss_body"
                )

        rate.sleep()

def rotation_to_quaternion(roll, pitch, yaw):
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return q

if __name__ == '__main__':
    try:
        publish_search_points()
    except rospy.ROSInterruptException:
        pass