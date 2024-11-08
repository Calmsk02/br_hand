# br_hand
bluerobin inc. hand device controller

# Dependencies
1. python3.8
  
2. Dynamixel SDK  
sudo apt-get install ros-[ROS Distribution]-dynamixel-sdk

3. PyQt5  
pip3 install PyQt5

4. json (if not installed)  
pip3 install json

5. serial (if not installed)  
pip3 install pyserial

# Usage
1. run br_hand and gui  
rosrun br_hand node_manager.py

2. run br_hand or gui  
rosrun br_hand hand_node.py  
rosrun br_hand gui_node.py
