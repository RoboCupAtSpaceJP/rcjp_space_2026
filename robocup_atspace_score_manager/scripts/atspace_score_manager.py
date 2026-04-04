#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import tf2_ros
from smach_files.area_monitor import AreaMonitor
from smach_files.capture_detector import CaptureDetector

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
        smach.State.__init__(self, outcomes=['success', 'fail'], output_keys=['initial_score'])
    
    def execute(self, userdata):
        rospy.loginfo('=== Initial State ===')
        try:
            return 'success'
        except Exception as e:
            rospy.logerr('System initialization error: %s', str(e))
            return 'fail'


class StartTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'], input_keys=['start_task_score'], output_keys=['start_task_score'])
    
    def execute(self, userdata):
        rospy.loginfo('=== Start Task State ===')
        try:
            # docking_area_monitor = AreaMonitor(area_name="docking_area")
            # docking_area_monitor.wait_until_departed()
            # userdata.start_task_score += 10
            return 'success'
        except Exception as e:
            rospy.logerr('Start task error: %s', str(e))
            return 'fail'

class NavigationTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['searchtask', 'dockingtask', 'fail'], 
                            input_keys=['navigation_task_score', 'searched'],
                            output_keys=['navigation_task_score'])

    def execute(self, userdata):
        rospy.loginfo('=== Navigation Task State ===')
        try:
            if not userdata.searched:
                # search_area_monitor = AreaMonitor(area_name="search_area")
                # search_area_monitor.wait_until_reached()
                # userdata.navigation_task_score += 10
                return 'searchtask'
            else:
                docking_area_monitor = AreaMonitor(area_name="docking_area")
                docking_area_monitor.wait_until_reached()
                userdata.navigation_task_score += 20
                return 'dockingtask'
        except Exception as e:
            rospy.logerr('Navigation task error: %s', str(e))
            return 'fail'

class SearchTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['search_task_score'],
                             output_keys=['search_task_score', 'searched'])
    def execute(self, userdata):
            rospy.loginfo('=== Search Task State ===')
            try:
                target_object = rospy.get_param('/competition/target_object_name')
                detector = CaptureDetector(object_name=target_object)
                
                result = detector.wait_for_result()

                if result == "capture_succeed":
                    rospy.loginfo('Target captured successfully!')
                    userdata.search_task_score += 50
                    userdata.searched = True
                    return 'success'
                else:
                    rospy.logwarn(f'Capture failed or error occurred: {result}')
                    userdata.searched = False
                    return 'fail'

            except Exception as e:
                rospy.logerr(f'Search task critical error: {str(e)}')
                return 'fail'

class DockingTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'], input_keys=['docking_task_score'], output_keys=['docking_task_score'])

    def execute(self, userdata):
        rospy.loginfo('=== Docking Task State ===')
        try:
            docking_area_monitor = AreaMonitor(area_name="docking_area")
            docking_area_monitor.wait_until_reached()
            userdata.docking_task_score += 20
            return 'success'
        except Exception as e:
            rospy.logerr('Docking task error: %s', str(e))
            return 'fail'

class FinishState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'], input_keys=['finish_score'])
    def execute(self, userdata):
        rospy.loginfo( '=== Finish State ===')
        try:
            return 'success'
        except Exception as e:
            rospy.logerr('Finish state error: %s', str(e))
            return 'fail'

def main():    
    
    task_sm = smach.StateMachine(outcomes=['task_all_finished', 'task_failed'],
                                 input_keys=['score'], output_keys=['score'])

    task_sm.userdata.searched = False

    with task_sm:
        
        smach.StateMachine.add('STARTTASK', StartTaskState(),
                               transitions={'success': 'NAVIGATIONTASK',
                                            'fail': 'task_failed'},
                               remapping={'start_task_score': 'score'})

        smach.StateMachine.add('NAVIGATIONTASK', NavigationTaskState(),
                               transitions={'searchtask': 'SEARCHTASK',
                                            'dockingtask': 'DOCKINGTASK',
                                            'fail': 'task_failed'},
                               remapping={'navigation_task_score': 'score',
                                          'searched': 'searched'})

        smach.StateMachine.add('SEARCHTASK', SearchTaskState(),
                               transitions={'success': 'DOCKINGTASK',
                                            'fail': 'task_failed'},
                               remapping={'search_task_score': 'score',
                                          'searched': 'searched'})

        smach.StateMachine.add('DOCKINGTASK', DockingTaskState(),
                               transitions={'success': 'task_all_finished',
                                           'fail': 'task_failed'},
                               remapping={'docking_task_score': 'score'})
        
    
    concurrence = smach.Concurrence(
        outcomes=['finished_on_time', 'global_timeout', 'fail'],
        default_outcome='fail',
        input_keys=['score'],
        output_keys=['score'],
        child_termination_cb = child_term_cb,
        outcome_map={
            'finished_on_time': {'TASK_CHAIN': 'task_all_finished'},
            'global_timeout': {'GLOBAL_TIMER': 'timeout'},
            'fail': {'TASK_CHAIN': 'task_failed'}
        }
    )

    root_sm = smach.StateMachine(outcomes=['SUCCESS', 'FAIL'])
    root_sm.userdata.score = rospy.get_param('~competition/initial_score', 0)
    time_limit = rospy.get_param('~competition/time_limit', 300)

    with root_sm:
        smach.StateMachine.add('INITIAL', InitialState(),
                               transitions={'success': 'MAIN_COMPETITION',
                                           'fail': 'FAIL'},
                               remapping={'initial_score': 'score'})

        with concurrence:
            smach.Concurrence.add('TASK_CHAIN', task_sm)
            smach.Concurrence.add('GLOBAL_TIMER', TimerState(duration=time_limit))
        smach.StateMachine.add('MAIN_COMPETITION', concurrence,
                               transitions={'finished_on_time': 'FINISH',
                                            'global_timeout': 'FINISH',
                                            'fail': 'FAIL'})
        smach.StateMachine.add('FINISH', FinishState(),
                               transitions={'success': 'SUCCESS',
                                           'fail': 'FAIL'},
                               remapping={'finish_score': 'score'})
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
