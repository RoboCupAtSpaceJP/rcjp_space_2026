#!/usr/bin/env python3
import rospy
import tf
import numpy as np
import tf.transformations as tft
import argparse
import sys
import os
import rospkg
import re
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class SDFSpawner:
    def __init__(self, args):
        rospy.init_node('sdf_spawner', anonymous=True)

        # 1. パラメータ設定
        self.category = args.category
        self.mesh_filename = args.mesh
        self.obs_name = args.name
        self.target_frame = "iss_body"  # 親となるTFフレーム名
        
        if args.offset is None:
            if self.category == "human_obstacles":
                self.offset = np.array([11.0, -6.0, 5.0])
            else:
                self.offset = np.array([11.5, -8.5, 5.3])
        else:
            self.offset = np.array(args.offset)

        self.rpy = args.rpy if args.rpy is not None else [0.0, 0.0, 0.0]
            
        self.iss_model_name = 'iss'
        self.spawned = False
        self.latest_obj_pose = None
        self.latest_iss_pose = None

        self.br = tf.TransformBroadcaster()

        # 2. パス構成
        rospack = rospkg.RosPack()
        try:
            self.pkg_path = rospack.get_path('robocup_atspace_score_manager')
        except rospkg.ResourceNotFound:
            rospy.logerr("Package 'robocup_atspace_score_manager' not found.")
            sys.exit(1)
            
        template_file = "human_obstacle.sdf" if self.category == "human_obstacles" else "portable_object.sdf"
        self.template_path = os.path.join(self.pkg_path, 'models', 'templates', template_file)

        # 3. Gazeboサービスの待機
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.wait_for_service('/gazebo/delete_model')
        self.spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        # 4. スポーン実行
        try:
            self.do_spawn()
        except Exception as e:
            rospy.logerr(f"Spawn process failed: {e}")

        # 5. 常に監視（TF計算のためにISSと物体の両方の位置が必要）
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        if self.category == "portable_objects":
            rospy.Timer(rospy.Duration(0.1), self.publish_tf)

        rospy.on_shutdown(self.cleanup)
        rospy.loginfo(f"Node Initialized. Monitoring: {self.obs_name}")

    def gazebo_callback(self, msg):
        """ISSと物体の両方の位置を更新"""
        if self.obs_name in msg.name:
            self.latest_obj_pose = msg.pose[msg.name.index(self.obs_name)]
        if self.iss_model_name in msg.name:
            self.latest_iss_pose = msg.pose[msg.name.index(self.iss_model_name)]

    def publish_tf(self, event):
        """ISS基準の相対座標に変換してTF配信"""
        if self.latest_obj_pose is None or self.latest_iss_pose is None:
            return

        # 1. ワールド座標系での行列を作成
        # ISSの行列
        q_iss = [self.latest_iss_pose.orientation.x, self.latest_iss_pose.orientation.y, 
                 self.latest_iss_pose.orientation.z, self.latest_iss_pose.orientation.w]
        t_iss = [self.latest_iss_pose.position.x, self.latest_iss_pose.position.y, self.latest_iss_pose.position.z]
        m_iss = tft.concatenate_matrices(tft.translation_matrix(t_iss), tft.quaternion_matrix(q_iss))

        # 物体の行列
        q_obj = [self.latest_obj_pose.orientation.x, self.latest_obj_pose.orientation.y, 
                 self.latest_obj_pose.orientation.z, self.latest_obj_pose.orientation.w]
        t_obj = [self.latest_obj_pose.position.x, self.latest_obj_pose.position.y, self.latest_obj_pose.position.z]
        m_obj = tft.concatenate_matrices(tft.translation_matrix(t_obj), tft.quaternion_matrix(q_obj))

        # 2. ISSから見た物体の相対行列を計算 ( M_rel = M_iss^-1 * M_obj )
        m_rel = np.dot(tft.inverse_matrix(m_iss), m_obj)

        # 3. 行列から並進と回転を取り出す
        rel_pos = tft.translation_from_matrix(m_rel)
        rel_ori = tft.quaternion_from_matrix(m_rel)

        # 4. iss_body 直下に配信
        self.br.sendTransform(
            rel_pos,
            rel_ori,
            rospy.Time.now(),
            self.obs_name,    # child
            self.target_frame # parent (iss_body)
        )

    def wait_for_iss(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5.0)
                if self.iss_model_name in msg.name:
                    idx = msg.name.index(self.iss_model_name)
                    return msg.pose[idx]
            except: pass
            rate.sleep()
        return None

    def calculate_pose(self, iss_pose):
        q_iss = [iss_pose.orientation.x, iss_pose.orientation.y, 
                 iss_pose.orientation.z, iss_pose.orientation.w]
        q_rot = tft.quaternion_from_euler(np.radians(self.rpy[0]), np.radians(self.rpy[1]), np.radians(self.rpy[2]))
        q_final = tft.quaternion_multiply(q_iss, q_rot)
        rot_mat = tft.quaternion_matrix(q_iss)[:3, :3]
        world_offset = np.dot(rot_mat, self.offset)

        p = Pose()
        p.position.x = iss_pose.position.x + world_offset[0]
        p.position.y = iss_pose.position.y + world_offset[1]
        p.position.z = iss_pose.position.z + world_offset[2]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q_final
        return p

    def do_spawn(self):
        base_meshes_dir = os.path.join(self.pkg_path, 'models', self.category, 'meshes')
        mesh_abs_path = os.path.join(base_meshes_dir, self.mesh_filename)
        mesh_dir = os.path.dirname(mesh_abs_path)
        
        model_sdf_path = None
        for d in [mesh_dir, os.path.dirname(mesh_dir)]:
            p = os.path.join(d, 'model.sdf')
            if os.path.exists(p):
                model_sdf_path = p
                current_model_dir = d
                break

        if model_sdf_path:
            with open(model_sdf_path, 'r') as f: sdf_xml = f.read()
            sdf_xml = re.sub(r'model://[^/\s<]+', f"file://{current_model_dir}", sdf_xml)
            sdf_xml = re.sub(r'<model name=[\'\"](.*?)[\'\"]>', f'<model name="{self.obs_name}">', sdf_xml, count=1)
        else:
            if not os.path.exists(self.template_path): return
            with open(self.template_path, 'r') as f: sdf_xml = f.read()
            collision_name = "human_collision" if self.category == "human_obstacles" else "body_collision"
            sdf_xml = sdf_xml.replace("__MODEL_NAME__", self.obs_name).replace("__MESH_URI__", f"file://{mesh_abs_path}")
            sdf_xml = sdf_xml.replace("__COLLISION_NAME__", collision_name).replace("__TARGET_ID__", f"{self.obs_name}::link::{collision_name}")
            sdf_xml = sdf_xml.replace("__TOPIC_NAME__", f"/{self.obs_name}_contact")

        iss_pose = self.wait_for_iss()
        if iss_pose is None: return
        spawn_pose = self.calculate_pose(iss_pose)
        
        res = self.spawn_proxy(model_name=self.obs_name, model_xml=sdf_xml, initial_pose=spawn_pose, reference_frame="world")
        if res.success: self.spawned = True

    def cleanup(self):
        if self.spawned:
            try: self.delete_proxy(model_name=self.obs_name)
            except: pass

if __name__ == '__main__':
    my_argv = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('--category', type=str, default='portable_objects')
    parser.add_argument('--mesh', type=str, required=True)
    parser.add_argument('--name', type=str, default='spawned_model')
    parser.add_argument('--offset', type=float, nargs=3, default=None)
    parser.add_argument('--rpy', type=float, nargs=3, default=None)
    args = parser.parse_args(my_argv[1:])

    try:
        spawner = SDFSpawner(args)
        rospy.spin()
    except rospy.ROSInterruptException: pass