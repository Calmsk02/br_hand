#!/usr/bin/python
#
# (Copyright) Bluerobin inc.
# Author: Minsu Kim < minsu309@bluerobin.co.kr, minsu309@snu.ac.kr >
# Description: GUI-based controller for BRHand

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from br_hand.gui.template import Ui_MainWindow
import rospy, rosparam
from br_hand.msg import MotorInfo
from br_hand.srv import MotorCMD
from br_hand.utils.utilities import get_rosparam_name, dict2list, BRLogger
from br_hand.hand.br_dxl_protocol import BR_CMD_TYPE
import sys, json, os

ADVANCED = False

class RosListner(QThread):
    message_received = pyqtSignal(MotorInfo)
    def __init__(self, topic):
        super(RosListner, self).__init__()
        self.subscriber = None
        self.topic = topic
        rospy.init_node("br_hand_gui_listener_node", anonymous=False)
        
    def callback(self, msg):
        self.message_received.emit(msg)
        
    def start(self):
        if self.subscriber:
            return
        self.subscriber = rospy.Subscriber(self.topic, MotorInfo, self.callback)
        
    def stop(self):
        if self.subscriber is not None:
            self.subscriber.unregister()
            self.subscriber = None
        
        
class BrGui(Ui_MainWindow):
    def __init__(self, MainWindow):
        super().setupUi(MainWindow)
        # initial parmeters
        file = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'setting.json')
        with open(file, 'r') as f:
            data = json.load(f)
        self.motor_ids = data["motor_id"][0]
        self.min_max_radian = dict2list(data["min_max_radian"])
        self.num_motors = len(self.motor_ids)
        if self.num_motors > 8:
            print("The number of motors exceeds 8. Please check \'setting.json\' file")
            return
        
        # member variables
        self.ros_connected = False
        self.is_running = False
        self.is_initialized = [False]*self.num_motors
        self.scroll_txt = ""
        
        # ros listener setup
        self.ros_listener_thread = RosListner('/motor')
        self.motor_data = MotorInfo()
        
        # buttons setup
        self._current_strength_level = 0
        self._current_strength = 50
        self.button_setup()
        
    def start_ros_listener(self):
        if not self.is_running:
            self.ros_listener_thread.start()
            self.is_running = True
            
    def stop_ros_listener(self):
        if self.is_running:
            self.ros_listener_thread.stop()
            self.is_running = False  
            self.is_initialized = [False]*self.num_motors
        
    def ros_listener_callback(self, msg):
        self.motor_data = msg
        for i, item in enumerate(self.cur_vals):
            if self.combo_modes[i].currentText() == "Current":
                val = round(msg.current[i],2)
                item.setText(f"{val}")
            elif self.combo_modes[i].currentText() == "Position" or self.combo_modes[i].currentText() == "PD":
                val = round(msg.position[i],2)
                item.setText(f"{val}")
            elif self.combo_modes[i].currentText() == "Velocity":
                val = round(msg.velocity[i],2)
                item.setText(f"{round(msg.velocity[i],2)}")
            if not self.is_initialized[i]:
                self.update_txt_and_slider(i)
                self.is_initialized[i] = True
                    
        self.label_real1.setText(self.cur_val1.text())
        self.label_real2.setText(self.cur_val3.text())
        self.label_real3.setText(self.cur_val5.text())
        self.label_real4.setText(self.cur_val7.text())
        
        
    def button_setup(self):
        self.ros_listener_thread.message_received.connect(self.ros_listener_callback)
        self.push_connect.clicked.connect(self.f_push_connect)
        self.push_enableall.clicked.connect(self.f_push_enableall)
        self.push_disableall.clicked.connect(self.f_push_disableall)
        self.radio_enables = []
        self.radio_enables.append(self.radio_enable1)
        self.radio_enables.append(self.radio_enable2)
        self.radio_enables.append(self.radio_enable3)
        self.radio_enables.append(self.radio_enable4)
        self.radio_enables.append(self.radio_enable5)
        self.radio_enables.append(self.radio_enable6)
        self.radio_enables.append(self.radio_enable7)
        self.radio_enables.append(self.radio_enable8)
        self.radio_enables[0].toggled.connect(lambda: self.f_radio_enable(0))
        self.radio_enables[1].toggled.connect(lambda: self.f_radio_enable(1))
        self.radio_enables[2].toggled.connect(lambda: self.f_radio_enable(2))
        self.radio_enables[3].toggled.connect(lambda: self.f_radio_enable(3))
        self.radio_enables[4].toggled.connect(lambda: self.f_radio_enable(4))
        self.radio_enables[5].toggled.connect(lambda: self.f_radio_enable(5))
        self.radio_enables[6].toggled.connect(lambda: self.f_radio_enable(6))
        self.radio_enables[7].toggled.connect(lambda: self.f_radio_enable(7))
        self.combo_modes = []
        self.combo_modes.append(self.combo_mode1)
        self.combo_modes.append(self.combo_mode2)
        self.combo_modes.append(self.combo_mode3)
        self.combo_modes.append(self.combo_mode4)
        self.combo_modes.append(self.combo_mode5)
        self.combo_modes.append(self.combo_mode6)
        self.combo_modes.append(self.combo_mode7)
        self.combo_modes.append(self.combo_mode8)
        self.combo_modes[0].currentIndexChanged.connect(lambda: self.f_combo_modeChange(0))
        self.combo_modes[1].currentIndexChanged.connect(lambda: self.f_combo_modeChange(1))
        self.combo_modes[2].currentIndexChanged.connect(lambda: self.f_combo_modeChange(2))
        self.combo_modes[3].currentIndexChanged.connect(lambda: self.f_combo_modeChange(3))
        self.combo_modes[4].currentIndexChanged.connect(lambda: self.f_combo_modeChange(4))
        self.combo_modes[5].currentIndexChanged.connect(lambda: self.f_combo_modeChange(5))
        self.combo_modes[6].currentIndexChanged.connect(lambda: self.f_combo_modeChange(6))
        self.combo_modes[7].currentIndexChanged.connect(lambda: self.f_combo_modeChange(7))
        self.gains = []
        self.gains.append([self.gain_kp1, self.gain_kv1])
        self.gains.append([self.gain_kp2, self.gain_kv2])
        self.gains.append([self.gain_kp3, self.gain_kv3])
        self.gains.append([self.gain_kp4, self.gain_kv4])
        self.gains.append([self.gain_kp5, self.gain_kv5])
        self.gains.append([self.gain_kp6, self.gain_kv6])
        self.gains.append([self.gain_kp7, self.gain_kv7])
        self.gains.append([self.gain_kp8, self.gain_kv8])
        self.push_gains = []
        self.push_gains.append(self.push_gain1)
        self.push_gains.append(self.push_gain2)
        self.push_gains.append(self.push_gain3)
        self.push_gains.append(self.push_gain4)
        self.push_gains.append(self.push_gain5)
        self.push_gains.append(self.push_gain6)
        self.push_gains.append(self.push_gain7)
        self.push_gains.append(self.push_gain8)
        self.push_gains[0].clicked.connect(lambda: self.f_gainChange(0))
        self.push_gains[1].clicked.connect(lambda: self.f_gainChange(1))
        self.push_gains[2].clicked.connect(lambda: self.f_gainChange(2))
        self.push_gains[3].clicked.connect(lambda: self.f_gainChange(3))
        self.push_gains[4].clicked.connect(lambda: self.f_gainChange(4))
        self.push_gains[5].clicked.connect(lambda: self.f_gainChange(5))
        self.push_gains[6].clicked.connect(lambda: self.f_gainChange(6))
        self.push_gains[7].clicked.connect(lambda: self.f_gainChange(7))
        self.sliders = []
        self.sliders.append(self.slider1)
        self.sliders.append(self.slider2)
        self.sliders.append(self.slider3)
        self.sliders.append(self.slider4)
        self.sliders.append(self.slider5)
        self.sliders.append(self.slider6)
        self.sliders.append(self.slider7)
        self.sliders.append(self.slider8)
        self.sliders[0].valueChanged.connect(lambda: self.f_sliderChange(0))
        self.sliders[1].valueChanged.connect(lambda: self.f_sliderChange(1))
        self.sliders[2].valueChanged.connect(lambda: self.f_sliderChange(2))
        self.sliders[3].valueChanged.connect(lambda: self.f_sliderChange(3))
        self.sliders[4].valueChanged.connect(lambda: self.f_sliderChange(4))
        self.sliders[5].valueChanged.connect(lambda: self.f_sliderChange(5))
        self.sliders[6].valueChanged.connect(lambda: self.f_sliderChange(6))
        self.sliders[7].valueChanged.connect(lambda: self.f_sliderChange(7))
        self.slider_abd1.valueChanged.connect(self.slider1.setValue)
        self.slider_abd2.valueChanged.connect(self.slider3.setValue)
        self.slider_abd3.valueChanged.connect(self.slider5.setValue)
        self.slider_abd4.valueChanged.connect(self.slider7.setValue)
        self.slider1.valueChanged.connect(self.slider_abd1.setValue)
        self.slider3.valueChanged.connect(self.slider_abd2.setValue)
        self.slider5.valueChanged.connect(self.slider_abd3.setValue)
        self.slider7.valueChanged.connect(self.slider_abd4.setValue)
        self.cur_vals = []
        self.cur_vals.append(self.cur_val1)
        self.cur_vals.append(self.cur_val2)
        self.cur_vals.append(self.cur_val3)
        self.cur_vals.append(self.cur_val4)
        self.cur_vals.append(self.cur_val5)
        self.cur_vals.append(self.cur_val6)
        self.cur_vals.append(self.cur_val7)
        self.cur_vals.append(self.cur_val8)
        self.zero_vals = []
        self.zero_vals.append(self.zero_val1)
        self.zero_vals.append(self.zero_val2)
        self.zero_vals.append(self.zero_val3)
        self.zero_vals.append(self.zero_val4)
        self.zero_vals.append(self.zero_val5)
        self.zero_vals.append(self.zero_val6)
        self.zero_vals.append(self.zero_val7)
        self.zero_vals.append(self.zero_val8)
        self.min_vals = []
        self.min_vals.append(self.min_val1)
        self.min_vals.append(self.min_val2)
        self.min_vals.append(self.min_val3)
        self.min_vals.append(self.min_val4)
        self.min_vals.append(self.min_val5)
        self.min_vals.append(self.min_val6)
        self.min_vals.append(self.min_val7)
        self.min_vals.append(self.min_val8)
        self.max_vals = []
        self.max_vals.append(self.max_val1)
        self.max_vals.append(self.max_val2)
        self.max_vals.append(self.max_val3)
        self.max_vals.append(self.max_val4)
        self.max_vals.append(self.max_val5)
        self.max_vals.append(self.max_val6)
        self.max_vals.append(self.max_val7)
        self.max_vals.append(self.max_val8)
        self.label_cmds = []
        self.label_cmds.append(self.label_cmd1)
        self.label_cmds.append(self.label_cmd2)
        self.label_cmds.append(self.label_cmd3)
        self.label_cmds.append(self.label_cmd4)
        self.push_strengths = []
        self.push_strengths.append(self.push_strength1)
        self.push_strengths.append(self.push_strength2)
        self.push_strengths.append(self.push_strength3)
        self.push_strength1.clicked.connect(lambda: self.f_push_strength(0))
        self.push_strength2.clicked.connect(lambda: self.f_push_strength(1))
        self.push_strength3.clicked.connect(lambda: self.f_push_strength(2))
        self.push_minus.clicked.connect(lambda: self.f_push_strength(3))
        self.push_plus.clicked.connect(lambda: self.f_push_strength(4))
        self.push_grip.pressed.connect(self.f_push_grip)
        self.push_release.pressed.connect(self.f_push_release)
        self.push_zero.pressed.connect(self.f_push_zero)
        self.push_reset.clicked.connect(self.f_push_reset)
        
        self.tabWidget.setTabEnabled(1, ADVANCED)
        
    def f_push_connect(self):
        if not self.ros_connected:
            self.label_status.setText(f"ROS disconnected")
            self.label_status.setAlignment(Qt.AlignHCenter)
            self.combo_modes[0].setCurrentIndex(1)
            self.combo_modes[2].setCurrentIndex(1)
            self.combo_modes[4].setCurrentIndex(1)
            self.combo_modes[6].setCurrentIndex(1)
            self.combo_modes[1].setCurrentIndex(0)
            self.combo_modes[3].setCurrentIndex(0)
            self.combo_modes[5].setCurrentIndex(0)
            self.combo_modes[7].setCurrentIndex(0)
            self._current_strength = 0
            self.label_8.setText(str(self._current_strength))
            rospy.wait_for_service("motor_command", 1)
            try:
                service = rospy.ServiceProxy("motor_command", MotorCMD)
                res = service(-1, int(BR_CMD_TYPE.CONNECT.value), 0)
                if res.res == True:
                    # button enables
                    self.push_enableall.setEnabled(True)
                    self.push_disableall.setEnabled(True)
                    self.combo_reset.setEnabled(True)
                    self.push_reset.setEnabled(True)
                    for i in range(self.num_motors):
                        self.radio_enables[i].setEnabled(True)
                        self.combo_modes[i].setEnabled(True)
                    # text show
                    self.label_status.setText(f"ROS connected")
                    self.label_status.setAlignment(Qt.AlignHCenter)
                    # ros listener
                    self.start_ros_listener()
                    
            except rospy.ServiceException as e:
                self.log(f"Service call failed: {e}")
        else:
            self.f_push_disableall()
            self.push_enableall.setEnabled(False)
            self.push_disableall.setEnabled(False)
            self.combo_reset.setEnabled(False)
            self.push_reset.setEnabled(False)
            for i in range(self.num_motors):
                self.radio_enables[i].setEnabled(False)
            self.label_status.setText(f"ROS disconnected")
            self.label_status.setAlignment(Qt.AlignHCenter)
            self.stop_ros_listener()
        self.ros_connected = not self.ros_connected
        
    def f_push_enableall(self):
        for i in range(self.num_motors):
            if not self.radio_enables[i].isChecked():
                self.radio_enables[i].toggle()
        # self.push_cmd1.setEnabled(True)
        # self.push_cmd2.setEnabled(True)
        # self.push_cmd3.setEnabled(True)
        self.groupBox_2.setEnabled(True)
        self.groupBox_3.setEnabled(True)
        self.groupBox_7.setEnabled(True)
        self.groupBox_8.setEnabled(True)
        self.groupBox_9.setEnabled(True)
        self.groupBox_10.setEnabled(True)
        self.push_grip.setEnabled(True)
        self.push_release.setEnabled(True)
        self.push_zero.setEnabled(True)
        self.push_strengths[self._current_strength_level].setStyleSheet("background-color: red; color: black")
        self._current_strength = 50
        self.label_8.setText(str(self._current_strength))
        
    def f_push_disableall(self):
        self.f_push_zero()
        for i in range(self.num_motors):
            if self.radio_enables[i].isChecked():
                self.radio_enables[i].toggle()
        self.push_cmd1.setEnabled(False)
        self.push_cmd2.setEnabled(False)
        self.push_cmd3.setEnabled(False)
        self.groupBox_2.setEnabled(False)
        self.groupBox_3.setEnabled(False)
        self.groupBox_7.setEnabled(False)
        self.groupBox_8.setEnabled(False)
        self.groupBox_9.setEnabled(False)
        self.groupBox_10.setEnabled(False)
        for i in range(self.num_motors):
            self.combo_modes[i].setEnabled(True)
        self.push_grip.setEnabled(False)
        self.push_release.setEnabled(False)
        self.push_zero.setEnabled(False)
        self.push_strength1.setStyleSheet("")
        self.push_strength2.setStyleSheet("")
        self.push_strength3.setStyleSheet("")
        self._current_strength_level = 0
        self.check_target1.setChecked(False)
        self.check_target2.setChecked(False)
        self.check_target3.setChecked(False)
        self.check_target4.setChecked(False)
    
    def f_push_reset(self):
        i = self.combo_reset.currentIndex()-1
        if i>=0:
            rospy.wait_for_service("motor_command")
            try:
                service = rospy.ServiceProxy("motor_command", MotorCMD)
                res = service(i, int(BR_CMD_TYPE.REBOOT.value), True)
                if res.res:
                    self.log(f"motor {i} enabled!\n") 
            except rospy.ServiceException as e:
                self.log(f"Service call failed: {e}")

        
    def f_push_strength(self, val):
        if val < 3:
            for i in range(3):
                self.push_strengths[i].setStyleSheet("")
            self.push_strengths[val].setStyleSheet("background-color: red; color: black")
        if val == 0:
            self._current_strength = 50
        elif val == 1:
            self._current_strength = 100
        elif val == 2:
            self._current_strength = 150
        elif val == 3:
            self._current_strength -= 10
            if self._current_strength < 0:
                self._current_strength = 0
        elif val == 4:
            self._current_strength += 10
            if self._current_strength > 200:
                self._current_strength = 200
        self.label_8.setText(str(self._current_strength))
            
    def gripCheck(self):
        checked_list = []
        if self.check_target1.isChecked():
            checked_list.append(1)
        if self.check_target2.isChecked():
            checked_list.append(3)
        if self.check_target3.isChecked():
            checked_list.append(5)
        if self.check_target4.isChecked():
            checked_list.append(7)    
        return checked_list
            
    def f_push_grip(self):
        checked_list = self.gripCheck()
        for i in checked_list:
            min_val = float(self.min_vals[i].text())
            interval = float(self.max_vals[i].text())-min_val
            value = (-self._current_strength-min_val)*100/interval
            self.sliders[i].setValue(int(value))
                
    def f_push_zero(self):
        checked_list = self.gripCheck()   
        for i in checked_list:
            min_val = float(self.min_vals[i].text())
            interval = float(self.max_vals[i].text())-min_val
            value = (0-min_val)*100/interval
            self.sliders[i].setValue(int(value))
    
    def f_push_release(self):
        checked_list = self.gripCheck()   
        for i in checked_list:
            min_val = float(self.min_vals[i].text())
            interval = float(self.max_vals[i].text())-min_val
            value = (self._current_strength-min_val)*100/interval
            self.sliders[i].setValue(int(value))
        
    def f_radio_enable(self, i):
        if self.radio_enables[i].isChecked():
            self.update_txt_and_slider(i)
            self.combo_modes[i].setEnabled(False)
            self.gains[i][0].setEnabled(True)
            self.gains[i][1].setEnabled(True)
            self.push_gains[i].setEnabled(True)
            self.sliders[i].setEnabled(True)
            rospy.wait_for_service("motor_command")
            try:
                service = rospy.ServiceProxy("motor_command", MotorCMD)
                res = service(i, int(BR_CMD_TYPE.ENABLE.value), True)
                if res.res:
                    self.log(f"motor {i} enabled!\n") 
            except rospy.ServiceException as e:
                self.log(f"Service call failed: {e}")
        else:
            self.combo_modes[i].setEnabled(True)
            self.gains[i][0].setEnabled(False)
            self.gains[i][1].setEnabled(False)
            self.push_gains[i].setEnabled(False)
            self.sliders[i].setEnabled(False)
            rospy.wait_for_service("motor_command")
            try:
                service = rospy.ServiceProxy("motor_command", MotorCMD)
                res = service(i, int(BR_CMD_TYPE.ENABLE.value), False)
                if res.res:
                    self.log(f"motor {i} disabled!\n")
            except rospy.ServiceException as e:
                self.log(f"Service call failed: {e}")
            
    def f_combo_modeChange(self, i):
        txt = self.combo_modes[i].currentText()
        mode_val = {"Current" : 0,
                    "PD"      : 0,
                    "Velocity": 1,
                    "Position": 5}
        rospy.wait_for_service("motor_command")
        try:
            service = rospy.ServiceProxy("motor_command", MotorCMD)
            res = service(i, int(BR_CMD_TYPE.CHANGEMODE.value), mode_val[txt])
            if res.res:
                self.log(f"motor {i} changed to {txt} mode!\n")
                target_param_list = get_rosparam_name("control")
                if rospy.has_param(target_param_list[2*i]):
                    rosparam.set_param(target_param_list[2*i], str(mode_val[txt]))
                if txt == "Position" or txt == "PD":
                    self.min_vals[i].setText(str(self.min_max_radian[2*i]))
                    self.max_vals[i].setText(str(self.min_max_radian[2*i+1]))
                elif txt == "Current":
                    self.min_vals[i].setText(str(-200))
                    self.max_vals[i].setText(str(200))
                self.min_vals[i].setAlignment(Qt.AlignHCenter)
                self.max_vals[i].setAlignment(Qt.AlignHCenter)
                self.is_initialized[i] = False
        except rospy.ServiceException as e:
            self.log(f"Service call failed: {e}")
        
    def f_gainChange(self, i):
        kp_value = self.gains[i][0].value()
        kv_value = self.gains[i][1].value()
        self.log(f"gain for motor {i} changed to kp:{kp_value:.2f} kv:{kv_value:.2f}\n")
        param_list = get_rosparam_name("gain")
        if rospy.has_param(param_list[2*i]):
            rosparam.set_param(param_list[2*i],    f"{kp_value:.2f}")
        if rospy.has_param(param_list[2*i+1]):
            rosparam.set_param(param_list[2*i+1],  f"{kv_value:.2f}")
        
    def f_sliderChange(self, i):
        val = self.sliders[i].value()
        min_val = float(self.min_vals[i].text())
        interval = float(self.max_vals[i].text())-min_val
        val = val/100*interval+min_val

        self.zero_vals[i].setText(f"{val:.2f}")
        self.zero_vals[i].setAlignment(Qt.AlignHCenter)
        if i%2 == 0:
            self.label_cmds[int((i)/2)].setText(f"{val:.2f}")
            self.label_cmds[int((i)/2)].setAlignment(Qt.AlignHCenter)
        
        target_param_list = get_rosparam_name("control")
        if rospy.has_param(target_param_list[2*i+1]):
            rosparam.set_param(target_param_list[2*i+1], f"{val:.2f}")
            
    def update_txt_and_slider(self, i):
        if self.ros_connected:
            min_val = float(self.min_vals[i].text())
            interval = float(self.max_vals[i].text())-min_val
            txt = self.combo_modes[i].currentText()
            if txt == "Current":
                value = self.motor_data.current[i]
            elif txt == "Position" or txt == "PD":
                value = self.motor_data.position[i]
            value = (value-min_val)*100/interval
            self.sliders[i].setValue(int(value))
            self.is_initialized[i] = True
            
    def log(self, msg):
        self.scroll_txt += msg
        self.label_log.setText(self.scroll_txt)
        scroll_bar_height = self.scrollArea.verticalScrollBar().maximum()
        self.scrollArea.verticalScrollBar().setValue(scroll_bar_height)
        
    
def main():
    app = QApplication(sys.argv)
    MainWindow = QMainWindow()
    ui = BrGui(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    main()