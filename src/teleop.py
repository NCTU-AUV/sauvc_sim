#!/usr/bin/env python3
import sys
import rospy
from gazebo_msgs.msg import ModelState
from select import select
import termios
import tty


class KeyboardController:
    
    def __init__(self):
        self.pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
        self.model_name = "orca"
        self.LIN_VEL_STEP = 0.1
        self.ANG_VEL_STEP = 0.1

        self.settings = termios.tcgetattr(sys.stdin)
        self.key_timeout = 0.5

        self.state = ModelState()
        self.state.model_name = self.model_name
        self.state.reference_frame = self.model_name

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], self.key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        key = self.get_key()
        if key == 'w':
            self.state.twist.linear.x = 0
            self.state.twist.linear.y = self.LIN_VEL_STEP
            self.state.twist.linear.z = 0
            rospy.loginfo("front")
        elif key == 'a':
            self.state.twist.linear.x = -self.LIN_VEL_STEP
            self.state.twist.linear.y = 0
            self.state.twist.linear.z = 0
            rospy.loginfo("left")
        elif key == 's':
            self.state.twist.linear.x = 0
            self.state.twist.linear.y = -self.LIN_VEL_STEP
            self.state.twist.linear.z = 0
            rospy.loginfo("back")
        elif key == 'd':
            self.state.twist.linear.x = self.LIN_VEL_STEP
            self.state.twist.linear.y = 0
            self.state.twist.linear.z = 0
            rospy.loginfo("right")
        elif key == 'k':
            self.state.twist.linear.x = 0
            self.state.twist.linear.y = 0
            self.state.twist.linear.z = self.LIN_VEL_STEP
            rospy.loginfo("up")
        elif key == 'j':
            self.state.twist.linear.x = 0
            self.state.twist.linear.y = 0
            self.state.twist.linear.z = -self.LIN_VEL_STEP
            rospy.loginfo("down")
        elif key == 'u':
            self.state.twist.angular.z = self.ANG_VEL_STEP
            rospy.loginfo("yaw_left")
        elif key == 'i':
            self.state.twist.angular.z = -self.ANG_VEL_STEP
            rospy.loginfo("yaw_right")
        elif key == 'q':
            self.state.twist.linear.x = 0
            self.state.twist.linear.y = 0
            self.state.twist.linear.z = 0
            rospy.loginfo("stop")
        else:
            self.state.twist.angular.x = 0
            self.state.twist.angular.y = 0
            self.state.twist.angular.z = 0

        self.pub.publish(self.state)
        
if __name__ == "__main__":
    try:
        rospy.init_node('keyboard_ctrl')
        kc = KeyboardController()
        rate = rospy.Rate(10) # hz
        while not rospy.is_shutdown():
            kc.run()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
