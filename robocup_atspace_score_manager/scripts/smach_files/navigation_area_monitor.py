#!/usr/bin/env python3

import rospy
import tf2_ros

class NavigationAreaMonitor:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rules = rospy.get_param('~rules', {})
        self.dock_area = rules.get('docking_area', {})
        self.search_area = rules.get('search_area', {})
        
        self.target_frame = "body"
        self.source_frame = "iss_body"

    def _check_area(self, pos, area_cfg):
        x_r = area_cfg.get('x_range', [0, 0])
        y_r = area_cfg.get('y_range', [0, 0])
        z_r = area_cfg.get('z_range', [0, 0])
        
        return (x_r[0] <= pos.x <= x_r[1] and
                y_r[0] <= pos.y <= y_r[1] and
                z_r[0] <= pos.z <= z_r[1])

    def monitor_transition(self):
        rate = rospy.Rate(10)
        rospy.loginfo("Monitoring navigation transition...")
        
        while not rospy.is_shutdown():
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.source_frame, 
                    self.target_frame, 
                    rospy.Time(0), 
                    rospy.Duration(0.5)
                )
                pos = trans.transform.translation

                if self._check_area(pos, self.search_area):
                    rospy.loginfo("Reached search_area.")
                    return "search_area_reached"

                if self._check_area(pos, self.dock_area):
                    rospy.loginfo("Reached docking_area.")
                    return "docking_area_reached"

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('navigation_monitor_node')
    nav_monitor = NavigationMonitor()
    result = nav_monitor.monitor_transition()
    rospy.loginfo(f"Transition result: {result}")