import emm
from ar4_configs import *

ROBOT_CONFIG = AR4_CFG



class Joint:
    def __init__(self) -> None:
    
        self.id = 1 
        self.driver = emm.EMMV5_MOROT()
    
        self.axis_pos_limit_angle = 0
        self.axis_neg_limit_angle = 0
        self.axis_nominal_angle = 0    
        self.step_master_ = 0
        self.gear_ratio = 0
        
        self.steps_pre_degre = 0
        self.micro_steps = 0
        
        
        self.isStalled = False
        
        # self.dir
    
    
    
    
    pass




class Robot:
    def __init__(self) -> None:
        self.
        
        pass
    