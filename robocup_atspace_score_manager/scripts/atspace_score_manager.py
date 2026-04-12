#!/usr/bin/env python3
import sys
sys.dont_write_bytecode = True

import datetime
import time
import rospy
import smach
import smach_ros
from smach_files.area_monitor import AreaMonitor
from smach_files.capture_detector import CaptureDetector
from std_srvs.srv import Trigger, TriggerResponse
from utils.score_writer import write_scores
from utils.gz_collision_detector import CollisionMonitor
import os
import rospkg

def child_term_cb(outcome_map):
        return True

class TimerState(smach.State):
    def __init__(self, duration):
        smach.State.__init__(self, outcomes=['timeout'])
        self.duration = duration

    def execute(self, userdata):
        start_time = time.time()
        while (time.time() - start_time) < self.duration:
            if self.preempt_requested():
                self.service_preempt()
                return 'timeout'
            time.sleep(0.1)
        return 'timeout'

class InitialState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             output_keys=['scores_dict', 'log_file_path', 'trial_id'])

    def execute(self, userdata):
        rospy.loginfo('=== Initial State ===')
        try:
            trial_number = rospy.get_param('/competition/trial_number')
            team_name = rospy.get_param('/competition/team_name')
            userdata.scores_dict = {
                            'trial_number': trial_number,
                            'start_task': 0,
                            'navigation_outbound': 0,
                            'navigation_return': 0,
                            'obstacle_avoidance_outbound': 0,
                            'obstacle_avoidance_return': 0,
                            'search_task': 0,
                            'docking_task': 0,
                            'time_bonus': 0,
                            'elapsed_time': '00:00:00',
                            'timestamp': None,
                        }
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('robocup_atspace_score_manager')
            score_dir = os.path.join(pkg_path, 'scores', team_name)
            if not os.path.exists(score_dir):
                os.makedirs(score_dir)
            userdata.log_file_path = os.path.join(score_dir, 'score.yaml')
            userdata.trial_id = None
            return 'success'
        except Exception as e:
            rospy.logerr('System initialization error: %s', str(e))
            return 'fail'


class CompetitionStartState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['scores_dict'],
                             output_keys=['start_time', 'scores_dict'])

        self.fixed_object_name = rospy.get_param('/competition/fixed_object_name')
        self.portable_object_name = rospy.get_param('/competition/portable_object_name')
        self.phrase = rospy.get_param('/rules/instruction_phrase')

        self.instruction_msg = f"{self.phrase}{self.fixed_object_name} and {self.portable_object_name}."
        self.start_requested = False

    def handle_start_call(self, req):
        res = TriggerResponse()
        if req:
            self.start_requested = True
            res.success = True
            res.message = self.instruction_msg
        else:
            res.success = False
            res.message = "Start request was false."
        return res

    def wait_for_result(self, timeout=5.0):
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and not self.start_requested:
            if time.time() >= deadline:
                return False
            time.sleep(0.1)
        return True

    def execute(self, userdata):
        rospy.loginfo('=== Competition Start State ===')
        self.start_requested = False
        srv = rospy.Service('/competition_start', Trigger, self.handle_start_call)
        try:
            # /competition_start 待ち（タイマー外なので preempt 対応不要）
            while not self.start_requested and not rospy.is_shutdown():
                self.wait_for_result(timeout=5.0)

            userdata.start_time = time.time()
            userdata.scores_dict['timestamp'] = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            rospy.loginfo('Competition started. Timer will begin now.')
            srv.shutdown()
            return 'success'
        except Exception as e:
            rospy.logerr('Competition start error: %s', str(e))
            srv.shutdown()
            return 'fail'


class DepartureTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['scores_dict', 'log_file_path', 'trial_id'],
                             output_keys=['scores_dict', 'trial_id'])

    def execute(self, userdata):
        rospy.loginfo('=== Departure Task State ===')
        try:
            docking_area_monitor = AreaMonitor(area_name="docking_area")
            while not rospy.is_shutdown():
                if self.preempt_requested():
                    self.service_preempt()
                    rospy.loginfo('[DEBUG] Preempt detected in departure task')
                    userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
                    return 'success'
                if docking_area_monitor.wait_until_departed(timeout=5.0):
                    break

            if self.preempt_requested():
                self.service_preempt()
                rospy.loginfo('[DEBUG] Preempt detected in departure task (post-break)')
                userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
                return 'success'

            userdata.scores_dict['start_task'] = rospy.get_param('/rules/scoring/start_task/departure')
            userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
            return 'success'
        except Exception as e:
            rospy.logerr('Departure task error: %s', str(e))
            return 'fail'

class NavigationTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['searchtask', 'dockingtask', 'fail'],
                            input_keys=['scores_dict', 'searched', 'log_file_path', 'trial_id'],
                            output_keys=['scores_dict', 'searched', 'trial_id'])
        self.collision_monitor = CollisionMonitor()

    def _calc_avoidance_score(self, safe_targets):
        """safe_targets を受け取り、障害物回避の加点を計算して返す。
        壁（ISS）に衝突していた場合は 0 を返す。"""
        if "iss" not in safe_targets:
            return 0
        human_safe = [t for t in safe_targets if t != "iss"]
        avoid_point = rospy.get_param('/rules/scoring/navigation_task/avoid_obstacle')
        return len(human_safe) * avoid_point

    def execute(self, userdata):
        rospy.loginfo('=== Navigation Task State ===')
        try:
            if not userdata.searched:
                # 往路：search_area への到達を待つ
                self.collision_monitor.start_monitoring()
                try:
                    search_area_monitor = AreaMonitor(area_name="search_area")
                    while not rospy.is_shutdown():
                        if self.preempt_requested():
                            rospy.loginfo('[DEBUG] Preempt detected in forward navigation')
                            safe_targets = self.collision_monitor.stop_monitoring()
                            userdata.scores_dict['obstacle_avoidance_outbound'] = self._calc_avoidance_score(safe_targets)
                            userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
                            return 'searchtask'
                        if search_area_monitor.wait_until_reached(timeout=5.0):
                            break

                    if self.preempt_requested():
                        self.service_preempt()
                        rospy.loginfo('[DEBUG] Preempt detected in forward navigation (post-break)')
                        safe_targets = self.collision_monitor.stop_monitoring()
                        userdata.scores_dict['obstacle_avoidance_outbound'] = self._calc_avoidance_score(safe_targets)
                        userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
                        return 'searchtask'

                    safe_targets = self.collision_monitor.stop_monitoring()
                    userdata.scores_dict['obstacle_avoidance_outbound'] = self._calc_avoidance_score(safe_targets)
                    userdata.scores_dict['navigation_outbound'] = rospy.get_param('/rules/scoring/navigation_task/reach_goal')
                    userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
                    return 'searchtask'
                except Exception:
                    self.collision_monitor.stop_monitoring()
                    raise
            else:
                # 復路：search_area 離脱 → navigation_area 到達 → navigation_area 離脱で加点
                # search_area 再侵入時はステップ1に戻る
                self.collision_monitor.start_monitoring()
                try:
                    search_area_monitor = AreaMonitor(area_name="search_area")
                    nav_area_monitor = AreaMonitor(area_name="navigation_area")

                    while not rospy.is_shutdown():
                        # ステップ1: search_area 離脱を待つ
                        while not rospy.is_shutdown():
                            if self.preempt_requested():
                                self.service_preempt()
                                rospy.loginfo('[DEBUG] Preempt detected in return navigation (search departure)')
                                safe_targets = self.collision_monitor.stop_monitoring()
                                userdata.scores_dict['obstacle_avoidance_return'] = self._calc_avoidance_score(safe_targets)
                                userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
                                return 'dockingtask'
                            if search_area_monitor.wait_until_departed(timeout=5.0):
                                break

                        # ステップ2: navigation_area 到達を待つ（search_area 再侵入チェック付き）
                        nav_reached = False
                        while not rospy.is_shutdown():
                            if self.preempt_requested():
                                self.service_preempt()
                                rospy.loginfo('[DEBUG] Preempt detected in return navigation (nav reach)')
                                safe_targets = self.collision_monitor.stop_monitoring()
                                userdata.scores_dict['obstacle_avoidance_return'] = self._calc_avoidance_score(safe_targets)
                                userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
                                return 'dockingtask'
                            if search_area_monitor.is_inside_dock() is True:
                                rospy.loginfo('[DEBUG] Re-entered search_area, resetting to step 1')
                                break
                            if nav_area_monitor.wait_until_reached(timeout=5.0):
                                nav_reached = True
                                break
                        if not nav_reached:
                            continue  # search_area 再侵入 → ステップ1に戻る

                        # ステップ3: navigation_area 離脱を待つ（search_area 再侵入チェック付き）
                        nav_departed = False
                        while not rospy.is_shutdown():
                            if self.preempt_requested():
                                self.service_preempt()
                                rospy.loginfo('[DEBUG] Preempt detected in return navigation (nav departure)')
                                safe_targets = self.collision_monitor.stop_monitoring()
                                userdata.scores_dict['obstacle_avoidance_return'] = self._calc_avoidance_score(safe_targets)
                                userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
                                return 'dockingtask'
                            if search_area_monitor.is_inside_dock() is True:
                                rospy.loginfo('[DEBUG] Re-entered search_area, resetting to step 1')
                                break
                            if nav_area_monitor.wait_until_departed(timeout=5.0):
                                nav_departed = True
                                break
                        if not nav_departed:
                            continue  # search_area 再侵入 → ステップ1に戻る

                        # navigation_area 離脱完了 → 加点
                        break

                    safe_targets = self.collision_monitor.stop_monitoring()
                    userdata.scores_dict['obstacle_avoidance_return'] = self._calc_avoidance_score(safe_targets)
                    userdata.scores_dict['navigation_return'] += rospy.get_param('/rules/scoring/navigation_task/reach_goal')
                    userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
                    return 'dockingtask'
                except Exception:
                    self.collision_monitor.stop_monitoring()
                    raise
        except Exception as e:
            rospy.logerr('Navigation task error: %s', str(e))
            return 'fail'

class SearchTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['scores_dict', 'log_file_path', 'trial_id'],
                             output_keys=['scores_dict', 'searched', 'trial_id'])
        self.fixed_captured = False
        self.portable_captured = False
    def execute(self, userdata):
            rospy.loginfo('=== Search Task State ===')
            self.fixed_captured = False
            self.portable_captured = False
            try:
                fixed_object_name = rospy.get_param('/competition/fixed_object_name')
                portable_object_name = rospy.get_param('/competition/portable_object_name')
                search_area_monitor = AreaMonitor(area_name="search_area")
                detector = CaptureDetector(target_names=[fixed_object_name, portable_object_name])

                # TF バッファのウォームアップ完了後、エリア内を一度確認してから離脱チェックを有効化
                entry_confirmed = False
                while not entry_confirmed and not rospy.is_shutdown():
                    if self.preempt_requested():
                        rospy.loginfo('[DEBUG] Preempt detected in search task')
                        userdata.searched = True
                        userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
                        return 'success'
                    if search_area_monitor.is_inside_dock() is True:
                        entry_confirmed = True
                    else:
                        time.sleep(0.1)

                while not (self.fixed_captured and self.portable_captured) and not rospy.is_shutdown():
                    # preempt チェック
                    if self.preempt_requested():
                        rospy.loginfo('[DEBUG] Preempt detected in search task')
                        userdata.searched = True
                        userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
                        return 'success'

                    # エリア離脱チェック
                    if search_area_monitor.is_inside_dock() is False:
                        rospy.loginfo('Left search area, finishing search task with partial score')
                        userdata.searched = True
                        userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
                        return 'success'

                    result_name, result_status = detector.wait_for_result(timeout=5.0)

                    if result_status == "capture_succeed":
                        if result_name == fixed_object_name and not self.fixed_captured:
                            rospy.loginfo(f'Fixed object [{fixed_object_name}] capture succeeded!')
                            userdata.scores_dict['search_task'] += rospy.get_param('/rules/scoring/search_task/fixed_object_capture')
                            self.fixed_captured = True
                            userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
                        elif result_name == portable_object_name and not self.portable_captured:
                            rospy.loginfo(f'Portable object [{portable_object_name}] capture succeeded!')
                            userdata.scores_dict['search_task'] += rospy.get_param('/rules/scoring/search_task/portable_object_capture')
                            self.portable_captured = True
                            userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
                    elif result_status != "timeout":
                        rospy.logwarn(f'Capture attempt for {result_name} failed: {result_status}')

                userdata.searched = True
                userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
                return 'success'

            except Exception as e:
                rospy.logerr(f'Search task error: {str(e)}')
                return 'fail'

class DockingTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['scores_dict', 'log_file_path', 'trial_id'],
                             output_keys=['scores_dict', 'trial_id'])

    def execute(self, userdata):
        rospy.loginfo('=== Docking Task State ===')
        try:
            docking_area_monitor = AreaMonitor(area_name="docking_area")
            while not rospy.is_shutdown():
                if self.preempt_requested():
                    self.service_preempt()
                    rospy.loginfo('[DEBUG] Preempt detected in docking task')
                    userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
                    return 'success'
                if docking_area_monitor.wait_until_reached(timeout=5.0):
                    break

            userdata.scores_dict['docking_task'] += rospy.get_param('/rules/scoring/docking_task/docking')
            userdata.trial_id = write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
            return 'success'
        except Exception as e:
            rospy.logerr('Docking task error: %s', str(e))
            return 'fail'

class FinishState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['scores_dict', 'log_file_path', 'trial_id', 'start_time'])
    def execute(self, userdata):
        rospy.loginfo('=== Finish State ===')
        try:
            if userdata.start_time is not None:
                elapsed_sec = int(time.time() - userdata.start_time)
                h = elapsed_sec // 3600
                m = (elapsed_sec % 3600) // 60
                s = elapsed_sec % 60
                userdata.scores_dict['elapsed_time'] = f"{h:02d}:{m:02d}:{s:02d}"
            write_scores(userdata.scores_dict, userdata.log_file_path, userdata.trial_id)
            return 'success'
        except Exception as e:
            rospy.logerr('Finish state error: %s', str(e))
            return 'fail'


def build_sm(time_limit):
    task_sm = smach.StateMachine(outcomes=['task_all_finished', 'task_failed'],
                                 input_keys=['scores_dict', 'searched', 'log_file_path', 'trial_id'],
                                 output_keys=['scores_dict', 'searched', 'trial_id'])
    task_sm.userdata.searched = False

    with task_sm:
        smach.StateMachine.add('DEPARTURE_TASK', DepartureTaskState(),
                               transitions={'success': 'NAVIGATIONTASK',
                                            'fail': 'task_failed'})
        smach.StateMachine.add('NAVIGATIONTASK', NavigationTaskState(),
                               transitions={'searchtask': 'SEARCHTASK',
                                            'dockingtask': 'DOCKINGTASK',
                                            'fail': 'task_failed'})
        smach.StateMachine.add('SEARCHTASK', SearchTaskState(),
                               transitions={'success': 'NAVIGATIONTASK',
                                            'fail': 'task_failed'})
        smach.StateMachine.add('DOCKINGTASK', DockingTaskState(),
                               transitions={'success': 'task_all_finished',
                                            'fail': 'task_failed'})

    concurrence = smach.Concurrence(
        outcomes=['finished_on_time', 'global_timeout', 'fail'],
        default_outcome='fail',
        input_keys=['scores_dict', 'log_file_path', 'searched', 'trial_id'],
        output_keys=['scores_dict', 'log_file_path', 'searched', 'trial_id'],
        child_termination_cb=child_term_cb,
        outcome_map={
            'global_timeout': {'GLOBAL_TIMER': 'timeout'},
            'finished_on_time': {'TASK_CHAIN': 'task_all_finished'},
            'fail': {'TASK_CHAIN': 'task_failed'}
        }
    )
    with concurrence:
        smach.Concurrence.add('TASK_CHAIN', task_sm)
        smach.Concurrence.add('GLOBAL_TIMER', TimerState(duration=time_limit))

    root_sm = smach.StateMachine(outcomes=['SUCCESS', 'FAIL'])
    root_sm.userdata.scores_dict = {}
    root_sm.userdata.log_file_path = ""
    root_sm.userdata.searched = False
    root_sm.userdata.trial_id = None
    root_sm.userdata.start_time = None
    with root_sm:
        smach.StateMachine.add('INITIAL', InitialState(),
                               transitions={'success': 'COMPETITION_START',
                                            'fail': 'FAIL'})
        smach.StateMachine.add('COMPETITION_START', CompetitionStartState(),
                               transitions={'success': 'MAIN_COMPETITION',
                                            'fail': 'FAIL'})
        smach.StateMachine.add('MAIN_COMPETITION', concurrence,
                               transitions={'finished_on_time': 'FINISH',
                                            'global_timeout': 'FINISH',
                                            'fail': 'FAIL'})
        smach.StateMachine.add('FINISH', FinishState(),
                               transitions={'success': 'SUCCESS',
                                            'fail': 'FAIL'})
    return root_sm


def main():
    time_limit = rospy.get_param('/competition/time_limit')
    root_sm = build_sm(time_limit)

    sis = smach_ros.IntrospectionServer('robocup_atspace_score_manager_smach', root_sm, '/robocup_atspace_score_manager')
    sis.start()

    rospy.loginfo('RoboCup@Space Score Manager is running...')
    outcome = root_sm.execute()

    rospy.loginfo('Score Manager finished: %s', outcome)
    sis.stop()


if __name__ == '__main__':
    try:
        rospy.init_node('robocup_atspace_score_manager_node')
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('Program interrupted by user')
    except Exception as e:
        rospy.logerr('Unexpected error: %s', str(e))
