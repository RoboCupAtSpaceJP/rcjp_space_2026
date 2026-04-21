#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import re
import threading
import sys
import math
import rospy
import tf

class CollisionMonitor:
    def __init__(self, robot_prefix="ib2"):
        self.robot_prefix = robot_prefix
        
        # 1. 監視対象リストの構築 (rosparamから動的に取得)
        self.watch_targets = ["iss"]  # 壁(iss)は常に監視
        
        # 人間の名前をパラメータから取得（空文字でない場合のみリストに追加）
        h1 = rospy.get_param('/competition/human_assignments/human_pos_1', "")
        h2 = rospy.get_param('/competition/human_assignments/human_pos_2', "")
        
        if h1: self.watch_targets.append("human_pos_1")
        if h2: self.watch_targets.append("human_pos_2")

        # 実際にぶつかった相手の名前を記録するセット
        self.collided_names = set()
        
        self.process = None
        self._stop_event = threading.Event()
        self._thread = None

    def _monitor_loop(self):
        """別スレッドで実行される監視メインループ"""
        command = ["gz", "topic", "-e", "/gazebo/default/physics/contacts"]
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE, text=True)

        try:
            c1, c2 = None, None
            is_in_contact = False

            for line in self.process.stdout:
                if self._stop_event.is_set():
                    break

                if "contact {" in line:
                    is_in_contact = True
                    continue
                
                if is_in_contact:
                    m1 = re.search(r'collision1:\s+"([^"]+)"', line)
                    if m1: c1 = m1.group(1)
                    m2 = re.search(r'collision2:\s+"([^"]+)"', line)
                    if m2: c2 = m2.group(1)
                    
                    if c1 and c2:
                        is_c1_robot = c1.startswith(self.robot_prefix)
                        is_c2_robot = c2.startswith(self.robot_prefix)

                        if is_c1_robot or is_c2_robot:
                            opponent = c2 if is_c1_robot else c1
                            for target in self.watch_targets:
                                if target in opponent:
                                    self.collided_names.add(target)
                        
                        c1, c2 = None, None
                        is_in_contact = False

                if "}" in line:
                    c1, c2 = None, None
                    is_in_contact = False
        finally:
            if self.process:
                self.process.terminate()

    def start_monitoring(self):
        """監視を開始する"""
        self.collided_names.clear()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._monitor_loop)
        self._thread.daemon = True
        self._thread.start()
        rospy.loginfo(f"[CollisionMonitor] Started. Watching: {self.watch_targets}")

    def stop_monitoring(self):
        """監視を終了し、『一度もぶつからなかった安全な対象』のリストを返す"""
        self._stop_event.set()
        if self.process:
            self.process.terminate()
        
        safe_targets = [t for t in self.watch_targets if t not in self.collided_names]
        rospy.loginfo(f"[CollisionMonitor] Stopped. Collided: {list(self.collided_names)}")
        return safe_targets

class SafetyDistanceMonitor:
    """ナビゲーション中にIB2が人のTF中心から一定距離以内に入ったか監視する"""

    def __init__(self, robot_frame="body", ref_frame="iss_body"):
        self.robot_frame = robot_frame
        self.ref_frame = ref_frame
        self.safety_threshold = rospy.get_param('/rules/safety_conditions/distance_threshold', 0.4)

        self.human_frames = []
        h1 = rospy.get_param('/competition/human_assignments/human_pos_1', "")
        h2 = rospy.get_param('/competition/human_assignments/human_pos_2', "")
        if h1: self.human_frames.append(rospy.get_param('/competition/human_pos_1_tf', 'human_pos_1'))
        if h2: self.human_frames.append(rospy.get_param('/competition/human_pos_2_tf', 'human_pos_2'))

        self.too_close_set = set()
        self._stop_event = threading.Event()
        self._thread = None

    def _monitor_loop(self):
        listener = tf.TransformListener()
        rate = rospy.Rate(10)
        while not self._stop_event.is_set() and not rospy.is_shutdown():
            try:
                listener.waitForTransform(self.ref_frame, self.robot_frame, rospy.Time(0), rospy.Duration(0.5))
                (robot_pos, _) = listener.lookupTransform(self.ref_frame, self.robot_frame, rospy.Time(0))
                for human_frame in self.human_frames:
                    if human_frame in self.too_close_set:
                        continue
                    try:
                        (human_pos, _) = listener.lookupTransform(self.ref_frame, human_frame, rospy.Time(0))
                        dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(robot_pos, human_pos)))
                        if dist < self.safety_threshold:
                            self.too_close_set.add(human_frame)
                            rospy.logwarn(f"[SafetyDistanceMonitor] Too close to {human_frame}: {dist:.3f}m")
                    except Exception:
                        pass
            except Exception:
                pass
            rate.sleep()

    def start_monitoring(self):
        self.too_close_set.clear()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._monitor_loop)
        self._thread.daemon = True
        self._thread.start()
        rospy.loginfo(f"[SafetyDistanceMonitor] Started. threshold={self.safety_threshold}m, watching: {self.human_frames}")

    def stop_monitoring(self):
        """監視終了。安全距離を保てた場合True、40cm以内に入った人がいればFalseを返す"""
        self._stop_event.set()
        kept_safe = len(self.too_close_set) == 0
        rospy.loginfo(f"[SafetyDistanceMonitor] Stopped. Too close: {list(self.too_close_set)}, safe={kept_safe}")
        return kept_safe


# ==========================================
# 単体テスト用メインルーチン
# ==========================================
if __name__ == "__main__":
    rospy.init_node('test_collision_detector', anonymous=True)
    monitor = CollisionMonitor(robot_prefix="ib2")
    
    print("\n" + "="*50)
    print(" COLLISION MONITOR - STANDALONE TEST MODE")
    print(f" Watching for: {monitor.watch_targets}")
    print("="*50)
    print(">> MONITORING... (Press Ctrl+C to stop and see results)")
    print(">> Move the robot in Gazebo and test collisions.")

    monitor.start_monitoring()
    
    try:
        # Ctrl+C が押されるまでここで待機
        rospy.spin() 
    except KeyboardInterrupt:
        # Ctrl+C を検知
        print("\n\n>> Ctrl+C detected. Finishing test...")
    finally:
        # SMACH統合時と同様、どんな終わり方でも必ず stop して結果を回収する
        safe_list = monitor.stop_monitoring()
        
        print("\n" + "#"*50)
        print(" [FINAL TEST RESULT]")
        print(f" Safe Targets (No Collision): {safe_list}")
        
        # 判定のまとめ
        collided = [t for t in monitor.watch_targets if t not in safe_list]
        if collided:
            print(f" Collided with: {collided}")
        else:
            print(" Result: PERFECT! No collisions detected.")
        print("#"*50 + "\n")

    sys.exit(0)