import numpy as np

class Model():
    def __init__(self, kinematic_model) :
        self.kinematic_model = kinematic_model

    def forward_kinematics(self, wheel_vel):
        return np.dot(self.kinematic_model, wheel_vel)
        

class Diff2Wheels(Model):
    def __init__(self, b, r ):
        kinematic_model = np.array([[
            
        ]])
        super().__init__(kinematic_model)