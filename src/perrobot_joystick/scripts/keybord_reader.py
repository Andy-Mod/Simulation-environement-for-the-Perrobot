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
        self.stdscr.clear()
        self.display_quit_message()

    def display_quit_message(self):
        height, width = self.stdscr.getmaxyx()
        message = [
            "Press 'q' to quit :",
            "    - Four legs sequences : l",
            "    - Go to position x : x",
            "    - Go to position cc : <",
            "    - Go to position ↃↃ : >",
            "    - Go to position >< : n",
            "    - walk forward : ↑",
            "    - walk backward : ↓",
            "    - from x to nx and back : v",
            "    - fall recovery : f",
            "    - sit : s"
        ]
        
        y = height // 2 - len(message) // 2
        for i, line in enumerate(message):
            x = width // 2 - len(line) // 2
            self.stdscr.addstr(y + i, x, line)
        
        self.stdscr.refresh()

    def process_key(self, key):
        if key == ord('q'):
            self.running = False
            rospy.loginfo("Quitting teleop...")
            return None
        elif key != -1:
            if key == curses.KEY_UP:
                key = "Up"
            elif key == curses.KEY_DOWN:
                key = "Down"
            elif key == curses.KEY_LEFT:
                key = "Left"
            elif key == curses.KEY_RIGHT:
                key = "Right"
            else:
                key = chr(key)
            return key

    def run(self):
        try:
            pub = rospy.Publisher('key_pressed', String, queue_size=1)
            rospy.init_node('Teleop_key')
            rate = rospy.Rate(10)
            message = String()

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
