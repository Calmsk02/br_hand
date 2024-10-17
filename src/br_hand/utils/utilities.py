#!/usr/bin/python
import rospy, os, time

def get_rosparam_name(param_name):
    ret = []
    if rospy.has_param(param_name):
        param = rospy.get_param(param_name)
        if isinstance(param, dict):
            keys = list(param.keys())
            for key in keys:
                if isinstance(param[key], dict):
                    second_keys = list(param[key].keys())
                    for second_key in second_keys:
                        if isinstance(param[key][second_key], dict):
                            third_keys = list(param[key][second_key].keys())
                            for third_key in third_keys:
                                name = "/" + param_name + "/" + key + "/" + second_key + "/" + third_key
                                ret.append(name)   
                        else:
                            name = "/" + param_name + "/" + key + "/" + second_key
                            ret.append(name)
                else:
                    name = "/" + param_name + "/" + key
                    ret.append(name)
        else:
            ret = param
    return ret

def dict2list(param):
    result = []
    for key, value in param.items():
        if isinstance(value, dict):
            result.extend(dict2list(value))
        else:
            result.append(value)
    return result

def get_rosparam_value(param_name):
    if rospy.has_param(param_name):
        param = rospy.get_param(param_name)
        return dict2list(param)
        
def squeeze_list(input_list):
    squeezed =[]
    for l in input_list:
        squeezed.extend(l)
    return squeezed

class BRLogger():
    def __init__(self, device):
        filepath = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))), ".log")
        if not os.path.exists(filepath):
            os.mkdir(filepath)
        filename = time.strftime("%Y-%m-%d %H:%M:%S."+str(device)+".brlog", time.localtime())
        filename = os.path.join(filepath, filename)
        self.f = open(filename, "w")
    
    def log(self, _str):
        t = time.strftime("%H:%M:%S ", time.localtime())
        print(_str)
        self.f.write(t+_str+"\n")