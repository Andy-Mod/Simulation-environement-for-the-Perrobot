#!/bin/python3

import rospy
from std_msgs.msg import String
from moves import Moves

def action_sender(data):
    key = data.data
    if key == 'Up':
        print("walk Foward")
        Moves.one_step_foward()
        
    elif key == 'Down':
        print("walk backward")
        Moves.one_step_backward()
        
    elif key == 'l':
        print("Four legs sequences")
        Moves.four_legs_sequence()
        
    elif key == 'x':
        print("Go to position x")
        Moves.stand_x()
        
    elif key == '<':
        print("Go to position cc")
        Moves.stand_cc()
        
    elif key == '>':
        print("Go to position ↃↃ")
        Moves.stand_ncc()
        
    elif key == 'n':
        print("Go to position ><")
        Moves.stand_nx()
        
    elif key == 'v':
        print("from x to nx and back")
        Moves.from_x_to_nx_and_back()
        
    elif key == 'f':
        print("fall recovery")
        Moves.fall_recovery()
        
    elif key == 's':
        print("sit")
        Moves.sit_and_return_to_x()
    

def listener():
    
    rospy.init_node("teleop_key_listener")
    rospy.Subscriber("key_pressed", String, action_sender)
    rospy.spin()


if __name__ == '__main__':
    listener()
