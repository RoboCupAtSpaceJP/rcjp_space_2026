#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from smach_files.area_monitor import AreaMonitor
from smach_files.capture_detector import CaptureDetector

import os
import rospkg

def child_term_cb(outcome_map):
        return True

class TimerState(smach.State):
    def __init__(self, duration):
        smach.State.__init__(self, outcomes=['timeout'])
        self.duration = duration

    def execute(self, userdata):
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < self.duration:
            if self.preempt_requested():
                self.service_preempt()
                return 'timeout'
            rospy.sleep(0.1)
            
        return 'timeout'

class InitialState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             output_keys=['scores_dict', 'log_file_path'])
    
    def execute(self, userdata):
        rospy.loginfo('=== Initial State ===')
        try:
            trial_number = rospy.get_param('/competition/trial_number')
            team_name = rospy.get_param('/competition/team_name')
            userdata.scores_dict = {
                            'trial_number': trial_number,
                            'start_task': 0,
                            'navigation_task': 0,
                            'search_task': 0,
                            'docking_task': 0,
                            'time_bonus': 0
                        }
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('robocup_atspace_score_manager')
            score_dir = os.path.join(pkg_path, 'scores', team_name)
            if not os.path.exists(score_dir):
                os.makedirs(score_dir)
            userdata.log_file_path = os.path.join(score_dir, 'score.txt')
            return 'success'
        except Exception as e:
            rospy.logerr('System initialization error: %s', str(e))
            return 'fail'


class StartTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['scores_dict'],
                             output_keys=['scores_dict'])
    
    def execute(self, userdata):
        rospy.loginfo('=== Start Task State ===')
        try:
            docking_area_monitor = AreaMonitor(area_name="docking_area")
            docking_area_monitor.wait_until_departed()
            userdata.scores_dict['start_task'] = rospy.get_param('/rules/scoring/start_task/departure')
            return 'success'
        except Exception as e:
            rospy.logerr('Start task error: %s', str(e))
            return 'fail'

class NavigationTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['searchtask', 'dockingtask', 'fail'], 
                            input_keys=['scores_dict', 'searched'],
                            output_keys=['scores_dict', 'searched'])

    def execute(self, userdata):
        rospy.loginfo('=== Navigation Task State ===')
        try:
            if not userdata.searched:
                search_area_monitor = AreaMonitor(area_name="search_area")
                search_area_monitor.wait_until_reached()
                userdata.scores_dict['navigation_task'] = rospy.get_param('/rules/scoring/navigation_task/reach_goal')
                return 'searchtask'
            else:
                search_area_monitor = AreaMonitor(area_name="search_area")
                search_area_monitor.wait_until_departed()
                docking_area_monitor = AreaMonitor(area_name="docking_area")
                docking_area_monitor.wait_until_reached()
                userdata.scores_dict['navigation_task'] += rospy.get_param('/rules/scoring/navigation_task/reach_goal')
                return 'dockingtask'
        except Exception as e:
            rospy.logerr('Navigation task error: %s', str(e))
            return 'fail'

class SearchTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['scores_dict'],
                             output_keys=['scores_dict', 'searched'])
        self.detector = None

    def execute(self, userdata):
            rospy.loginfo('=== Search Task State ===')
            try:
                target_object_type = rospy.get_param('/competition/target_object_type')
                target_object_name = rospy.get_param('/competition/target_object_name')
                if self.detector is None:
                    self.detector = CaptureDetector(object_name=target_object_name)
                else:
                    self.detector.target_object = target_object_name

                userdata.searched = True

                result = self.detector.wait_for_result()

                if result == "capture_succeed":
                    rospy.loginfo('Target captured successfully!')
                    param_path = f'/rules/scoring/search_task/{target_object_type}_object_capture'
                    userdata.scores_dict['search_task'] = rospy.get_param(param_path)
                    userdata.searched = True
                    return 'success'
                else:
                    rospy.logwarn(f'Target capture failed: {result}')
                    userdata.searched = True
                    return 'success'

            except Exception as e:
                rospy.logerr(f'Search task error: {str(e)}')
                return 'fail'

class DockingTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['scores_dict'],
                             output_keys=['scores_dict'])

    def execute(self, userdata):
        rospy.loginfo('=== Docking Task State ===')
        try:
            # userdata.scores_dict['docking_task'] += rospy.get_param('/rules/scoring/docking_task/docking') #TODO
            return 'success'
        except Exception as e:
            rospy.logerr('Docking task error: %s', str(e))
            return 'fail'

class FinishState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['scores_dict', 'log_file_path'])
    def execute(self, userdata):
        rospy.loginfo( '=== Finish State ===')
        try:
            sd = userdata.scores_dict
            total = sum([sd['start_task'], sd['navigation_task'], sd['search_task'], sd['docking_task'], sd['time_bonus']])
            
            with open(userdata.log_file_path, 'a') as f:
                f.write(f"trial_number: {sd['trial_number']}\n")
                f.write(f"    start_task: {sd['start_task']}\n")
                f.write(f"    navigation_task: {sd['navigation_task']}\n")
                f.write(f"    search_task: {sd['search_task']}\n")
                f.write(f"    docking_task: {sd['docking_task']}\n")
                f.write(f"    time_bonus: {sd['time_bonus']}\n")
                f.write(f"    final_score: {total}\n")
            return 'success'
        except Exception as e:
            rospy.logerr('Finish state error: %s', str(e))
            return 'fail'

def main():    
    
    task_sm = smach.StateMachine(outcomes=['task_all_finished', 'task_failed'],
                                 input_keys=['scores_dict', 'searched'], output_keys=['scores_dict', 'searched'])

    task_sm.userdata.searched = False

    with task_sm:
        
        smach.StateMachine.add('STARTTASK', StartTaskState(),
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
        input_keys=['scores_dict', 'log_file_path', 'searched'],
        output_keys=['scores_dict', 'log_file_path', 'searched'],
        child_termination_cb = child_term_cb,
        outcome_map={
            'finished_on_time': {'TASK_CHAIN': 'task_all_finished'},
            'global_timeout': {'GLOBAL_TIMER': 'timeout'},
            'fail': {'TASK_CHAIN': 'task_failed'}
        }
    )

    time_limit = rospy.get_param('/competition/time_limit')
    with concurrence:
        smach.Concurrence.add('TASK_CHAIN', task_sm)
        smach.Concurrence.add('GLOBAL_TIMER', TimerState(duration=time_limit))

    root_sm = smach.StateMachine(outcomes=['SUCCESS', 'FAIL'])
    root_sm.userdata.scores_dict = {} 
    root_sm.userdata.log_file_path = ""
    root_sm.userdata.searched = False
    with root_sm:
        smach.StateMachine.add('INITIAL', InitialState(),
                               transitions={'success': 'MAIN_COMPETITION',
                                           'fail': 'FAIL'})
        smach.StateMachine.add('MAIN_COMPETITION', concurrence,
                               transitions={'finished_on_time': 'FINISH',
                                            'global_timeout': 'FINISH',
                                            'fail': 'FAIL'})
        smach.StateMachine.add('FINISH', FinishState(),
                               transitions={'success': 'SUCCESS',
                                           'fail': 'FAIL'})
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
