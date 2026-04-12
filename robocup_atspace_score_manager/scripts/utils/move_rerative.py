#!/usr/bin/python3
# -*- coding:utf-8 -*-
import argparse
import math
import rospy
import threading
import numpy
from actionlib import (
    GoalStatus,
    SimpleActionClient,
)
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
)
from ib2_msgs.msg import (
    CtlCommandAction,
    CtlCommandGoal,
    CtlStatus,
    CtlStatusType,
    NavigationStartUpAction,
    NavigationStartUpGoal,
    NavigationStartUpResult,
)
from platform_msgs.msg import UserNodeStatus
from std_msgs.msg import Header, Time

# --- クォータニオン変換関数 ---
_AXES2TUPLE = {'sxyz': (0, 0, 0, 0)}
_NEXT_AXIS = [1, 2, 0, 1]

def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]
    ai /= 2.0; aj /= 2.0; ak /= 2.0
    ci = math.cos(ai); si = math.sin(ai)
    cj = math.cos(aj); sj = math.sin(aj)
    ck = math.cos(ak); sk = math.sin(ak)
    cc = ci*ck; cs = ci*sk; sc = si*ck; ss = si*sk
    quaternion = numpy.empty((4, ), dtype=numpy.float64)
    quaternion[i] = cj*sc - sj*cs
    quaternion[j] = cj*ss + sj*cc
    quaternion[k] = cj*cs - sj*sc
    quaternion[3] = cj*cc + sj*ss
    return quaternion

class SimpleMoveNode:
    MAX_MSG_SIZE = 800

    def __init__(self, move_params):
        rospy.init_node('simple_move_node')
        self.__status = 'Initializing'
        self.__stop_requested = False
        self.__rate = rospy.Rate(1)
        self.__executed = False 
        self.__move_params = move_params

        self.__status_publisher = rospy.Publisher('/ib2_user/status', UserNodeStatus, queue_size=1)
        self.__complete_publisher = rospy.Publisher('/ib2_user/complete', Time, queue_size=1)
        self.__ctl_command_client = SimpleActionClient('/ctl/command', CtlCommandAction)
        self.__navigation_start_up_client = SimpleActionClient(
            '/sensor_fusion/navigation_start_up', NavigationStartUpAction)

    def __generate_ctl_command_goal(self, goal_type, position, orientation):
        return CtlCommandGoal(
            type=CtlStatus(type=goal_type),
            target=PoseStamped(
                header=Header(stamp=rospy.Time.now()),
                pose=Pose(position=position, orientation=orientation)
            )
        )

    def __wait_for_ctl_command(self):
        wait_result = False
        while not self.__stop_requested and not wait_result:
            wait_result = self.__ctl_command_client.wait_for_server(rospy.Duration(1.0))
        return bool(not self.__stop_requested)

    def __start_navigation(self):
        if self.__navigation_start_up_client.wait_for_server(rospy.Duration(20)):
            nav_goal = NavigationStartUpGoal(command=NavigationStartUpGoal.ON)
            self.__navigation_start_up_client.send_goal_and_wait(nav_goal)
            result = self.__navigation_start_up_client.get_result()
            if result and result.type == NavigationStartUpResult.ON_READY:
                return True
        return False

    def __main_logic(self):
        """メインの動作シーケンス"""
        if not self.__start_navigation():
            rospy.signal_shutdown("Navigation failed")
            return

        if not self.__wait_for_ctl_command():
            rospy.signal_shutdown("Control server not found")
            return

        self.__status = "Step 1: Moving with CLI relative parameters"
        rospy.loginfo(self.__status)
        rospy.loginfo(
            "Effective params: x=%.3f y=%.3f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f",
            self.__move_params['x'], self.__move_params['y'], self.__move_params['z'],
            self.__move_params['roll'], self.__move_params['pitch'], self.__move_params['yaw']
        )

        pos_target = Point(
            x=self.__move_params['x'],
            y=self.__move_params['y'],
            z=self.__move_params['z']
        )
        q = quaternion_from_euler(
            self.__move_params['roll'],
            self.__move_params['pitch'],
            self.__move_params['yaw']
        )
        ori_rot = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.__ctl_command_client.send_goal_and_wait(
            self.__generate_ctl_command_goal(CtlStatusType.MOVE_TO_RELATIVE_TARGET, pos_target, ori_rot)
        )

        # 終了処理
        self.__status = "Done"
        rospy.loginfo("Sequence completed. Shutting down...")
        self.__complete_publisher.publish(Time(data=rospy.Time.now()))
        rospy.sleep(2.0)
        rospy.signal_shutdown("Normal completion")

    def run(self):
        while not rospy.is_shutdown():
            if not self.__executed:
                self.__executed = True
                thread = threading.Thread(target=self.__main_logic)
                thread.start()

            try:
                msg_data = list(self.__status.encode('utf-8')[:self.MAX_MSG_SIZE])
                if len(msg_data) < self.MAX_MSG_SIZE:
                    msg_data.extend([0] * (self.MAX_MSG_SIZE - len(msg_data)))
                self.__status_publisher.publish(UserNodeStatus(stamp=rospy.Time.now(), msg=msg_data))
            except:
                break
            self.__rate.sleep()


def parse_args():
    parser = argparse.ArgumentParser(
        description='Send a relative move command using CLI parameters.'
    )
    parser.add_argument('-x', '--x', type=float, default=0.0, help='relative move x [m] (default: 0.0)')
    parser.add_argument('-y', '--y', type=float, default=0.0, help='relative move y [m] (default: 0.0)')
    parser.add_argument('-z', '--z', type=float, default=0.0, help='relative move z [m] (default: 0.0)')
    parser.add_argument('-r', '--roll', type=float, default=0.0, help='relative roll [rad] (default: 0.0)')
    parser.add_argument('-p', '--pitch', type=float, default=0.0, help='relative pitch [rad] (default: 0.0)')
    parser.add_argument('-w', '--yaw', type=float, default=0.0, help='relative yaw [rad] (default: 0.0)')

    args = parser.parse_args()
    return {
        'x': float(args.x),
        'y': float(args.y),
        'z': float(args.z),
        'roll': float(args.roll),
        'pitch': float(args.pitch),
        'yaw': float(args.yaw),
    }

if __name__ == '__main__':
    try:
        SimpleMoveNode(parse_args()).run()
    except rospy.ROSInterruptException:
        pass