#!/usr/bin/env python3
import sys, os

import rospy
from mavros_msgs.msg import State

import subprocess, shlex, psutil

mav_state_initialized = False
mav_state = State()
mav_state_prev = State()

class RosbagRecord:
    def __init__(self) -> None:
        self.is_record = False

    def start(self):
        if not self.is_record:
            prefix = "rosbag_"
            filelist = os.listdir(os.getcwd())
            idx = len(filelist)
            while os.path.exists(os.getcwd() + "/" + prefix + self.num2str(idx) + ".bag"):
                idx += 1

            command = "rosbag record -a" + " -O " + prefix + self.num2str(idx)
            self.rosbag_proc = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True,
                                                executable='/bin/bash')
            self.is_record = True
        else:
            print("self.is_record is True!")

    def stop(self):
        if self.is_record:
            self.rosbag_proc.send_signal(subprocess.signal.SIGINT)
            self.is_record = False
        else:
            print("self.is_record is False!")

    def num2str(self, num):
        if num < 10:
            return "0" + str(num)
        else:
            return str(num)

recorder = RosbagRecord()

# node_rc_mode_change.py
def state_callback(msg: State):
    global mav_state_initialized
    global mav_state

    mav_state_initialized = True
    mav_state = msg

def rosbag_manager(event):
    global mav_state_initialized
    global mav_state
    global mav_state_prev

    if not mav_state_initialized:
        return

    if mav_state.armed and not mav_state_prev.armed:
        print("armed!")
        recorder.start()
        
    if not mav_state.armed and mav_state_prev.armed:
        print("disarmed!")
        recorder.stop()

    mav_state_prev = mav_state

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)

    rospy.init_node("rosbag_record_node", anonymous=True)

    rc_sub = rospy.Subscriber("/mavros/state", State, state_callback)

    rosbag_manager_timer = rospy.Timer(rospy.Duration(0.5), rosbag_manager)

    rospy.spin()
