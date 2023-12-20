import numpy as np

class kinematicModel:
    def __init__ (self, R = 0, b = 0):
        

        self.R = R
        self.b = b

        self.mecanum4_kinematics = np.array([
            [R/4,  R/4, R/4, R/4],
            [R/4, -R/4, -R/4, R/4],
            [R/(4*b), -R/(4*b),R/(4*b), -R/(4*b)]
        ])

        self.inv_kinematics = np.linalg.pinv(self.mecanum4_kinematics)


    
    def diff_wheels_forward(self, wheel_vels):
        pass


    def mecanum4_wheels_inverse(self, robot_vels):
        return self.inv_kinematics @ robot_vels.T


   
    
if __name__ == '__main__':
    model = kinematicModel(R=0.06, b=0.3)
    robot_vel = np.array([[10,0,0]])
    print(model.diff_wheels_inverse(robot_vels=robot_vel))