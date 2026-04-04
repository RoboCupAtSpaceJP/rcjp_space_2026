#!/usr/bin/env python3

import rospy
import tf2_ros

class AreaMonitor:
    def __init__(self, area_name="docking_area"):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        areas = rospy.get_param('/rules/areas')
        self.area_name = area_name
        self.area_config = areas.get(area_name)
        
        self.x_range = self.area_config.get('x_range', [0, 0])
        self.y_range = self.area_config.get('y_range', [0, 0])
        self.z_range = self.area_config.get('z_range', [0, 0])
        
        self.target_frame = "body"
        self.source_frame = "iss_body"

    def is_inside_dock(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.source_frame, 
                self.target_frame, 
                rospy.Time(0), 
                rospy.Duration(0.5)
            )
            pos = trans.transform.translation
            
            inside = (self.x_range[0] <= pos.x <= self.x_range[1] and
                      self.y_range[0] <= pos.y <= self.y_range[1] and
                      self.z_range[0] <= pos.z <= self.z_range[1])
            return inside
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
        
    def wait_until_departed(self):
        rate = rospy.Rate(10)
        rospy.loginfo("Waiting for departure from %s...", self.area_name)
        while not rospy.is_shutdown():
            status = self.is_inside_dock()
            if status is False:
                rospy.loginfo("Departure confirmed.")
                break
            rate.sleep()
            
    def wait_until_reached(self):
        rate = rospy.Rate(10)
        rospy.loginfo("Waiting for return to %s...", self.area_name)
        while not rospy.is_shutdown():
            status = self.is_inside_dock()
            if status is True:
                rospy.loginfo("Return confirmed.")
                break
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('area_monitor')
    
    docking_area_monitor = AreaMonitor(area_name="docking_area")
    docking_area_monitor.wait_until_departed()
    
    rospy.loginfo("Executing mission...")
    rospy.sleep(5.0)
    
    docking_area_monitor.wait_until_reached()