#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from std_srvs.srv import SetBool, SetBoolResponse
from tf.transformations import quaternion_matrix

class CaptureDetector:
    def __init__(self, object_name="airlock"):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.target_object = object_name

        rules = rospy.get_param('/rules', {})
        conditions = rules.get('capture_conditions', {})
        self.dist_threshold = conditions.get('distance_threshold', 1.0)
        self.dot_threshold = conditions.get('dot_product_threshold', 0.5)

        self.robot_frame = "body"
        self.iss_frame = "iss_body"
        
        self.result = None
        self.srv = rospy.Service('/capture_report', SetBool, self.handle_capture_report)

    def get_unit_vector(self, transform):
        q = [transform.transform.rotation.x, transform.transform.rotation.y,
             transform.transform.rotation.z, transform.transform.rotation.w]
        return quaternion_matrix(q)[0:3, 0]

    def handle_capture_report(self, req):
        res = SetBoolResponse()
        if not req.data:
            res.success = False
            res.message = "capture_failed"
            return res

        try:
            r_t = self.tf_buffer.lookup_transform(self.iss_frame, self.robot_frame, rospy.Time(0), rospy.Duration(1.0))
            o_t = self.tf_buffer.lookup_transform(self.iss_frame, self.target_object, rospy.Time(0), rospy.Duration(1.0))

            r_pos = np.array([r_t.transform.translation.x, r_t.transform.translation.y, r_t.transform.translation.z])
            o_pos = np.array([o_t.transform.translation.x, o_t.transform.translation.y, o_t.transform.translation.z])
            
            forward = self.get_unit_vector(r_t)
            diff = o_pos - r_pos
            dist = np.linalg.norm(diff)
            dot = np.dot(forward, diff / dist) if dist > 0 else 1.0

            if dist <= self.dist_threshold and dot >= self.dot_threshold:
                self.result = "capture_succeed"
                res.success = True
                res.message = "capture_succeed"
            else:
                self.result = "capture_failed"
                res.success = False
                res.message = "capture_failed"

        except Exception as e:
            self.result = "error"
            res.success = False
            res.message = f"error: {str(e)}"
        
        return res

    def wait_for_result(self):
        self.result = None
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.result is None:
            rate.sleep()
        return self.result