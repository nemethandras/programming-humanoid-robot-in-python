'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
import time
import numpy as np
from SimpleXMLRPCServer import SimpleXMLRPCServer
import xmlrpclib
server = SimpleXMLRPCServer(("localhost", 8000))
import threading

print ("Connecting to port 8000")
server.register_introspection_functions()

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        val = False
        print ("Incoming request: get_angle")
        if joint_name in self.perception.joint:
            val = self.perception.joint[joint_name]
            print ("Return value \"", str(val),"\" for joint",joint_name)
        else:
            print ("Error: \"", joint_name)

        return val
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        if joint_name in self.perception.joint:
            self.target_joints[joint_name] = angle
            print ('Set angle', joint_name, 'to ', str(angle))
        else:
            print ("Error")

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        print ("Robot has posture \"", self.posture,"\"")
        return str(self.posture)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.start_time = None
        self.keyframes = keyframes

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        self.forward_kinematics(self.perception.joint)

        transf = False
        if name in self.transforms:
            transf = self.from_trans(self.transforms[name])
            print ("Return value", transf ,"for joint "+name)
        else :
            print ("Error: \"", name, "\"")
            return False
        
        py_transf = []

        for i in range(len(transf)):
            py_transf.append(np.float(transf[i]).item())
        return py_transf

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.transforms[effector_name] = transform

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

