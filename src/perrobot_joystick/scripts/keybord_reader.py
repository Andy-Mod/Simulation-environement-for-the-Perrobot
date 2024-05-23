#!/bin/python3

import rospy
import curses
from std_msgs.msg import String

class TKey:
    def __init__(self, stdscr):
        self.stdscr = stdscr
        self.init_curses()
        self.running = True

    def init_curses(self):

        curses.curs_set(0)
        self.stdscr.nodelay(1)
        self.stdscr.timeout(100)

    def process_key(self, key):

        if key == ord('q'):
            self.running = False
            rospy.loginfo("Quitting teleop...")
            return None
        elif key != -1:
            key = chr(key)
            rospy.loginfo(f"Key pressed : {key}")  # Log only once
            return key

    def run(self):

        try:
            pub = rospy.Publisher('key_pressed', String, queue_size=10)
            rospy.init_node('T_key')
            rate = rospy.Rate(10)
            message = String()
            rospy.loginfo("TeleopKey node started. Press 'q' to quit.")
            
            while not rospy.is_shutdown() and self.running:
                key = self.stdscr.getch()
                if key is not None:  
                    processed_key = self.process_key(key)
                    if processed_key is not None:
                        pub.publish(processed_key)
                rate.sleep()
                
        except rospy.ROSInterruptException:
            print("ROS node error!")

def main(stdscr):
    teleop = TKey(stdscr)
    teleop.run()

if __name__ == '__main__':
    curses.wrapper(main)
