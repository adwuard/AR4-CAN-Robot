import emm
from ar4_configs import *

ROBOT_CONFIG = AR4_CFG



class Joint:
    def __init__(self, joint_cfg) -> None:
    
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
    
    
    
    def init_joint(self):
        # setup zeroing
        
        pass
    
    def move_joint_schedule(self, angle):
        # always use pos mode and abs on fixed direction
        
        # calc acceleration pulse count
        
        pass
    
    def move_joint_nonblocking(self, angle):
        pass
    
    def check_arrived_flag(self):
        pass
    
    def check_collision_flag(self):
        pass


class Joints:
    def __init__(self, cfg) -> None:
        self.homeing_sequence = cfg.homeing_sequence
        self.joints = cfg.joints
        
        pass

    def stop_all_joints(self):
        pass
    
    def run_scheduled_task(self):
        pass
    
    def check_all_arrived(self):
        pass
    
    def homeing(self):
        for s in self.homeing_sequence:
            for j in s:
                
                pass
        pass
    
        
    



class Robot:
    def __init__(self) -> None:
        self.
        
        pass
    
    
    
""" 
Joint Initialization:

- 


"""