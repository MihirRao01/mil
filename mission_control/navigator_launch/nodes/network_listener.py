#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sub8_alarm import single_alarm
from twisted.internet import reactor
import time


class NetworkCheck(object):
    '''
    Meant to only run on the boat. When the network is dropped, it triggers a kill.
    '''
    def __init__(self, timeout=2.0):
        self.timeout = rospy.Duration(timeout)
        self.last_time = rospy.Time.now()
        self.last_msg = ''
        self.sub = rospy.Subscriber('/keep_alive', String, self.got_network_msg, queue_size=1)

        self.alarm_broadcaster, self.alarm = single_alarm('kill', severity=3, problem_description="Network loss")
        rospy.Timer(rospy.Duration(0.1), self.check)

    def check(self, *args):
        if self.need_kill() and self.last_msg != '':
            self.alarm.raise_alarm()
            rospy.logerr("NETWORK LOSS: KILLING BOAT")
        else:
            self.alarm.clear_alarm()

    def got_network_msg(self, msg):
        self.last_msg = msg.data
        self.last_time = rospy.Time.now()

    def need_kill(self):
        return ((rospy.Time.now() - self.last_time) > self.timeout)

if __name__ == '__main__':
    rospy.init_node('network_kill')
    NetworkCheck()
    rospy.spin()
