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
import time
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class SDFSpawner:
    def __init__(self, args):
        rospy.init_node('sdf_spawner', anonymous=True)

        # --- 1. 基本変数の初期化 (エラー回避のため最優先で実行) ---
        self.category = rospy.get_param('~category', args.category)
        self.obs_name = rospy.get_param('~name', args.name)
        self.target_frame = "iss_body"
        self.iss_model_name = 'iss'
        self.spawned = False
        self.publish_tf_enabled = True
        self.latest_obj_pose = None
        self.latest_iss_pose = None
        self.br = tf.TransformBroadcaster()
        
        # --- 2. 起動直後のパラメータサーバー反映待ち ---
        timeout = 10.0
        start_time = time.time()
        rospy.loginfo(f"[{rospy.get_name()}] Waiting for parameters...")
        
        while not rospy.is_shutdown():
            # rulesは共通で必須
            has_rules = rospy.has_param(f"/rules/search_points/{self.category}")
            # 人間の場合は割り当て、ポータブルの場合は名前決定のために competition が必要
            has_comp = rospy.has_param("/competition")
            
            if has_rules and has_comp:
                break
                
            if (time.time() - start_time) > timeout:
                rospy.logwarn(f"[{self.obs_name}] Parameter wait timeout!")
                break
            time.sleep(0.2)

        # --- 3. competition.yaml からの割り当て/設定取得 ---
        assignments = rospy.get_param("/competition/human_assignments", {})
        assigned_mesh_key = assignments.get(self.obs_name)

        # --- 4. 自動名前決定ロジック (Portable Objects用) ---
        if self.category == "portable_objects" and self.obs_name == "auto":
            self.obs_name = rospy.get_param("/competition/portable_object_name", "laptop")
            rospy.loginfo(f"[{rospy.get_name()}] Auto-selected: {self.obs_name}")
        
        # --- 5. 割り当てチェックロジック (Human Obstacles用) ---
        if self.category == "human_obstacles":
            if not assigned_mesh_key:
                rospy.loginfo(f"[{self.obs_name}] No assignment found (empty string). Skipping spawn.")
                # self.spawned = False 等は初期化済みなのでそのまま監視へ
                self._setup_monitoring_only()
                return
        
        # --- 6. rules.yaml から詳細設定を読み込み ---
        rules_param_path = f"/rules/search_points/{self.category}/{self.obs_name}"
        if not rospy.has_param(rules_param_path):
             rospy.logerr(f"[{self.obs_name}] Parameter NOT FOUND: {rules_param_path}")
        
        obj_config = rospy.get_param(rules_param_path, {})

        # TF publish制御 (human_obstaclesのみ、デフォルトTrue)
        self.publish_tf_enabled = rospy.get_param(
            f"/rules/search_points/{self.category}/publish_tf", True
        )

        # offsetの決定 (YAML優先)
        if args.offset is not None:
            self.offset = np.array(args.offset)
        else:
            yaml_pos = obj_config.get('position', [11.0, -4.5, 5.0]) 
            self.offset = np.array(yaml_pos)

        rospy.loginfo(f"[{self.obs_name}] Final offset used: {self.offset}")

        # rpyの決定
        if args.rpy is not None:
            self.rpy = args.rpy
        else:
            yaml_rpy = obj_config.get('rotation', [0.0, 0.0, 0.0])
            self.rpy = [np.degrees(val) for val in yaml_rpy]

        # meshの決定
        if args.mesh:
            self.mesh_filename = args.mesh
        elif self.category == "human_obstacles":
            templates = rospy.get_param("/rules/search_points/human_templates", {})
            template_info = templates.get(assigned_mesh_key, {})
            self.mesh_filename = template_info.get('model_folder', assigned_mesh_key)
        else:
            self.mesh_filename = obj_config.get('model_folder', self.obs_name)
            
        # --- 7. パス構成 ---
        rospack = rospkg.RosPack()
        try:
            self.pkg_path = rospack.get_path('robocup_atspace_score_manager')
        except rospkg.ResourceNotFound:
            rospy.logerr("Package 'robocup_atspace_score_manager' not found.")
            sys.exit(1)
            
        template_file = "human_obstacle.sdf" if self.category == "human_obstacles" else "portable_object.sdf"
        self.template_path = os.path.join(self.pkg_path, 'models', 'templates', template_file)

        # --- 8. Gazeboサービスの待機 ---
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.wait_for_service('/gazebo/delete_model')
        self.spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        # --- 9. スポーン実行 ---
        try:
            self.do_spawn()
        except Exception as e:
            rospy.logerr(f"Spawn process failed: {e}")

        self._setup_monitoring_only()

    def _setup_monitoring_only(self):
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        if self.category in ["portable_objects", "portable"] or (self.category == "human_obstacles" and self.spawned and self.publish_tf_enabled):
            rospy.Timer(rospy.Duration(0.1), self.publish_tf)
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo(f"Node Initialized: {self.obs_name} (Spawned: {self.spawned})")

    def gazebo_callback(self, msg):
        # 名前リストの中から正確に自分の名前を探す
        try:
            idx = msg.name.index(self.obs_name)
            self.latest_obj_pose = msg.pose[idx]
        except ValueError:
            # 自分の名前がリストにない場合は何もしない
            pass

        if self.iss_model_name in msg.name:
            self.latest_iss_pose = msg.pose[msg.name.index(self.iss_model_name)]

    def publish_tf(self, event):
        if self.latest_obj_pose is None or self.latest_iss_pose is None:
            return
        q_iss = [self.latest_iss_pose.orientation.x, self.latest_iss_pose.orientation.y, 
                 self.latest_iss_pose.orientation.z, self.latest_iss_pose.orientation.w]
        t_iss = [self.latest_iss_pose.position.x, self.latest_iss_pose.position.y, self.latest_iss_pose.position.z]
        m_iss = tft.concatenate_matrices(tft.translation_matrix(t_iss), tft.quaternion_matrix(q_iss))
        q_obj = [self.latest_obj_pose.orientation.x, self.latest_obj_pose.orientation.y, 
                 self.latest_obj_pose.orientation.z, self.latest_obj_pose.orientation.w]
        t_obj = [self.latest_obj_pose.position.x, self.latest_obj_pose.position.y, self.latest_obj_pose.position.z]
        m_obj = tft.concatenate_matrices(tft.translation_matrix(t_obj), tft.quaternion_matrix(q_obj))
        m_rel = np.dot(tft.inverse_matrix(m_iss), m_obj)
        rel_pos = tft.translation_from_matrix(m_rel)
        rel_ori = tft.quaternion_from_matrix(m_rel)
        self.br.sendTransform(rel_pos, rel_ori, rospy.Time.now(), self.obs_name, self.target_frame)

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
        for d in [mesh_abs_path, mesh_dir, os.path.dirname(mesh_dir)]:
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
    parser.add_argument('--mesh', type=str, default=None)
    parser.add_argument('--name', type=str, default=None) 
    parser.add_argument('--offset', type=float, nargs=3, default=None)
    parser.add_argument('--rpy', type=float, nargs=3, default=None)
    args = parser.parse_args(my_argv[1:])

    try:
        spawner = SDFSpawner(args)
        rospy.spin()
    except rospy.ROSInterruptException: pass