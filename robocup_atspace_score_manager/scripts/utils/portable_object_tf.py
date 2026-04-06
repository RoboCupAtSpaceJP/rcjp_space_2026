#!/usr/bin/env python3

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel, DeleteModel


class PortableObjectTF:
    def __init__(self):
        self.model_name = rospy.get_param('~model_name')
        self.world_frame = rospy.get_param('~world_frame', 'base')
        self.base_frame = rospy.get_param('~base_frame')
        self.spawn_enabled = rospy.get_param('~spawn_enabled', True)
        rate = rospy.get_param('~updateFreqHz', 10)

        self.urdf_param = rospy.get_param('~urdf_param', 'portable_object_description')

        # Read position/rotation from rules.yaml search_points.portable
        portable_config = rospy.get_param('/rules/search_points/portable', {})
        obj_config = portable_config.get(self.model_name, {})
        pos = obj_config.get('position', [0.0, 0.0, 0.0])
        rot = obj_config.get('rotation', [0.0, 0.0, 0.0])
        self.init_x, self.init_y, self.init_z = pos
        self.init_roll, self.init_pitch, self.init_yaw = rot

        self.spawned = False
        self.latest_pose = None
        self.br = tf.TransformBroadcaster()

        if self.spawn_enabled:
            self.spawn_model()
            rospy.on_shutdown(self.delete_model)

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        rospy.Timer(rospy.Duration(1.0 / rate), self.publish)

    def spawn_model(self):
        urdf_xml = rospy.get_param(self.urdf_param, '')
        if not urdf_xml:
            rospy.logerr('URDF param [%s] is empty', self.urdf_param)
            return

        from geometry_msgs.msg import Pose
        from tf.transformations import quaternion_from_euler
        pose = Pose()
        pose.position.x = self.init_x
        pose.position.y = self.init_y
        pose.position.z = self.init_z
        q = quaternion_from_euler(self.init_roll, self.init_pitch, self.init_yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        try:
            rospy.wait_for_service('/gazebo/spawn_urdf_model', timeout=30.0)
            spawn = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp = spawn(self.model_name, urdf_xml, '', pose, 'world')
            if resp.success:
                rospy.loginfo('Spawned portable object [%s] at (%.1f, %.1f, %.1f)',
                              self.model_name, self.init_x, self.init_y, self.init_z)
                self.spawned = True
            else:
                rospy.logwarn('Spawn failed: %s', resp.status_message)
        except Exception as e:
            rospy.logerr('Spawn service error: %s', str(e))

    def delete_model(self):
        if not self.spawned:
            return
        try:
            rospy.wait_for_service('/gazebo/delete_model', timeout=5.0)
            delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp = delete(self.model_name)
            if resp.success:
                rospy.loginfo('Deleted portable object [%s]', self.model_name)
            else:
                rospy.logwarn('Delete failed: %s', resp.status_message)
        except Exception as e:
            rospy.logwarn('Delete service error: %s', str(e))

    def callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == self.model_name:
                self.latest_pose = msg.pose[i]
                return

    def publish(self, event):
        if self.latest_pose is None:
            return
        p = self.latest_pose
        self.br.sendTransform(
            (p.position.x, p.position.y, p.position.z),
            (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
            rospy.Time.now(),
            self.base_frame,
            self.world_frame
        )


if __name__ == '__main__':
    rospy.init_node('portable_object_tf')
    PortableObjectTF()
    rospy.spin()
