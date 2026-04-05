#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from robocup_atspace_score_manager.srv import CaptureReport, CaptureReportResponse

from tf.transformations import quaternion_matrix

class CaptureDetector:
    def __init__(self, target_names=["airlock", "laptop"]):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.target_names = target_names

        rules = rospy.get_param('/rules', {})
        conditions = rules.get('capture_conditions', {})
        self.dist_threshold = conditions.get('distance_threshold', 1.0)
        self.dot_threshold = conditions.get('dot_product_threshold', 0.5)

        self.robot_frame = "body"
        self.iss_frame = "iss_body"
        
        self.result = None
        self.srv = rospy.Service('/capture_report', CaptureReport, self.handle_capture_report)

    def get_unit_vector(self, transform):
        q = [transform.transform.rotation.x, transform.transform.rotation.y,
             transform.transform.rotation.z, transform.transform.rotation.w]
        return quaternion_matrix(q)[0:3, 0]

    def handle_capture_report(self, req):
        res = CaptureReportResponse()
        if not req.target_object_name:
            res.success = False
            res.message = "capture_failed"
            return res

        try:
            r_t = self.tf_buffer.lookup_transform(self.iss_frame, self.robot_frame, rospy.Time(0), rospy.Duration(1.0))
            r_pos = np.array([r_t.transform.translation.x, r_t.transform.translation.y, r_t.transform.translation.z])
            forward = self.get_unit_vector(r_t)

            success_object = False

            o_t = self.tf_buffer.lookup_transform(self.iss_frame, req.target_object_name, rospy.Time(0), rospy.Duration(1.0))
            o_pos = np.array([o_t.transform.translation.x, o_t.transform.translation.y, o_t.transform.translation.z])
            
            diff = o_pos - r_pos
            dist = np.linalg.norm(diff)
            dot = np.dot(forward, diff / dist) if dist > 0 else 1.0
            if dist <= self.dist_threshold and dot >= self.dot_threshold:
                success_object = True
            
            self.last_captured_name = req.target_object_name
            self.result = "capture_failed"
            if success_object:
                res.success = True
                res.message = f"capture_succeeded: {success_object}"
                rospy.loginfo(f"Capture Succeeded for: {success_object}")
            else:
                res.success = False
                res.message = "capture_failed: no target in range"
                rospy.logwarn("Capture Failed: out of range or wrong direction")

        except Exception as e:
            self.result = "error"
            res.success = False
            res.message = f"error: {str(e)}"
            rospy.logerr(f"Capture Detector Error: {e}")
        
        return res

    def wait_for_result(self):
        self.result = None
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.result is None:
            rate.sleep()
        return self.last_captured_name, self.result