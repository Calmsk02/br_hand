#!/usr/bin/python
#
# (Copyright) Bluerobin inc.
# Author: Minsu Kim < minsu309@bluerobin.co.kr, minsu309@snu.ac.kr >
# Description:

import rospy
import subprocess

def main():
    node1 = subprocess.Popen(["rosrun", "br_hand", "hand_node.py"])
    node2 = subprocess.Popen(["rosrun", "br_hand", "gui_node.py"])
    
    node2.wait()
    print("gui node closed")
    node1.terminate()
    node1.wait()
    print("hand node closed")
    
if __name__ == "__main__":
    rospy.init_node("node_manager")
    main()
    