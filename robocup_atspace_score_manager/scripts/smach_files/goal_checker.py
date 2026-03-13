#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import geometry_msgs.msg

class GoalChecker:
    def __init__(self, x_range, y_range, z_range):
        rospy.init_node('goal_detector_node')

        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.score     = rospy.get_param('~initial_score', 0)
        self.is_inside = False

        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_position)

    def check_position(self, event):
        try:
            trans = self.tf_buffer.lookup_transform('iss_body', 'body', rospy.Time(0))

            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y
            curr_z = trans.transform.translation.z
            

            in_x = self.x_range[0] < curr_x < self.x_range[1]
            in_y = self.y_range[0] < curr_y < self.y_range[1]
            in_z = self.z_range[0] < curr_z < self.z_range[1]


            # rospy.loginfo("Current Position: (%.2f, %.2f, %.2f) | In Box: X:%s Y:%s Z:%s", 
            #         curr_x, curr_y, curr_z, in_x, in_y, in_z)

            if in_x and in_y and in_z:
                if not self.is_inside:
                    self.score += 10
                    self.is_inside = True
                    rospy.loginfo("First Goal reached! Score: %d", self.score)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

if __name__ == '__main__':
    try:
        checker = GoalChecker([10.0, 12.0], [-10.0, -8.0], [4.0, 6.0])
        rospy.spin()
    except rospy.ROSInterruptException:
        pass