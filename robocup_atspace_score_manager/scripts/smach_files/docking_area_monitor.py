#!/usr/bin/env python3

import rospy
import tf2_ros

class DockingAreaMonitor:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rules = rospy.get_param('~rules', {})
        area = rules.get('docking_area', {})
        
        self.x_range = area.get('x_range', [0, 0])
        self.y_range = area.get('y_range', [0, 0])
        self.z_range = area.get('z_range', [0, 0])
        
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

class DockingDepartureMonitor(DockingAreaMonitor):
    def wait_until_departed(self):
        rate = rospy.Rate(10)
        rospy.loginfo("Waiting for departure from docking_area...")
        while not rospy.is_shutdown():
            status = self.is_inside_dock()
            if status is False:
                rospy.loginfo("Departure confirmed.")
                break
            rate.sleep()

class DockingReturnMonitor(DockingAreaMonitor):
    def wait_until_returned(self):
        rate = rospy.Rate(10)
        rospy.loginfo("Waiting for return to docking_area...")
        while not rospy.is_shutdown():
            status = self.is_inside_dock()
            if status is True:
                rospy.loginfo("Return confirmed.")
                break
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('docking_task_monitor')
    
    departure_task = DockingDepartureMonitor()
    departure_task.wait_until_departed()
    
    rospy.loginfo("Executing mission...")
    rospy.sleep(5.0)
    
    return_task = DockingReturnMonitor()
    return_task.wait_until_returned()