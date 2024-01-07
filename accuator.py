from emm import *
from ar4_configs import *

import time
import struct
import serial


class Joint:
    def __init__(self, motor_driver, joint_cfg) -> None:
        self.driver = motor_driver
        self.cfg = joint_cfg
        
        self.id = joint_cfg['motor_addr']
        self.name = joint_cfg['name']
        self.enable = joint_cfg['enable']
        self.axis_pos_limit_angle = joint_cfg['max_angle']
        self.axis_neg_limit_angle = joint_cfg['min_angle']

        self.step_master_ = 0
        self.gear_ratio = joint_cfg['gear_ratio']
        self.steps_pre_degree = joint_cfg['steps_per_degree']
        self.micro_steps = joint_cfg['microstep']
        self.isStalled = False
        self.dir = joint_cfg['motor_direction']
        self.speed = joint_cfg['max_speed_rpm']
        self.acc = joint_cfg['acc_and_dec']
        
        self.offset_angle = joint_cfg['calibration_zero_offset_angle']
    
        if self.enable:
            self.init_joint()
    
    def joint_error_handler(self):    
        pass
    
    def init_joint(self):
        # setup zeroing
        
        """ Used for homing """
        self.home_direction = self.cfg['home_direction']
        self.home_speed_rpm = self.cfg['home_speed_rpm']
        self.home_timeout_ms = self.cfg['home_timeout_ms']
        self.home_method = self.cfg['home_method']
    
    
        pass
    
    def move_joint_schedule(self, angle):
        
        new_neg=abs(self.axis_neg_limit_angle) + self.cfg['calibration_zero_offset_angle']
        
        # always use pos mode and abs on fixed direction
        total_angle_limit = abs(new_neg) + abs(self.axis_pos_limit_angle)
        self.legal_full_steps = self.steps_pre_degree * total_angle_limit
        self.t_pose_zero_steps = abs(new_neg) * self.steps_pre_degree
        
        # print(self.legal_full_steps, self.t_pose_zero_steps, total_angle_limit)
        
        # self.acc = 10   
        s = self.t_pose_zero_steps + (angle * self.steps_pre_degree)
        self.driver.set_position_control(self.dir, self.speed, self.acc, s*self.micro_steps, absolute_mode=True, wait_broadcast_signal=True)
        
        
        
        
        
        # calc acceleration pulse count
        
        pass
    
    def move_joint_nonblocking(self, angle):
        pass
    
    
    def check_motor_status(self):
        pass
    
    def on_arrived_flag(self, message):
        pass
    
    def check_collision_flag(self, message):
        pass


class Joints:
    def __init__(self, cfg) -> None:
        self.homeing_sequence = cfg.homeing_sequence
        self.joints = []
        
        self.broadcast_channel = 0x00
        
        self.serial_backend = serial.Serial(COM_PORT, 115200, timeout=0.001)
        self.hw_interface = SerialPort(self.serial_backend)
        self.broadcast = ZDT_EMMV5_MOTOR(self.hw_interface, self.broadcast_channel)
        
        for cfg in cfg.joints:
            driver = ZDT_EMMV5_MOTOR(self.hw_interface, cfg['motor_addr'])
            self.joints.append(Joint(driver, cfg))
        
        
        self.broadcast.enable_motor()
        self.broadcast.clear_stall_error()
        self.broadcast.trigger_sensor_zeroing()
        time.sleep(10)
        # while 1:
            
        self.move_joints_joint_move_motion([0,0,0,0,0,0,0,0])
        # time.sleep(1)
        self.move_joints_joint_move_motion([20,20,3,20,34,40,0,0])
        self.move_joints_joint_move_motion([-20,60,30,20,-10,-34,0,0])
        self.move_joints_joint_move_motion([0,0,0,0,0,0,0,0])
        
        # self.broadcast.disable_motor()
        # time.sleep(5)
        
        return

    def stop_all_joints(self):
        self.broadcast.disable_motor()
        pass
    
    def run_scheduled_task(self):
        self.broadcast.broadcast_run()
    
    def check_all_arrived(self):
        pass
    
    def homeing(self):
        self.broadcast.trigger_sensor_zeroing()
        for s in self.homeing_sequence:
            for j in s:
                pass
        pass
    
    
    def make_angles(j_1, j_2, j_3, j_4, j_5, j_6, j_7, j_8):
        pass
    
    def move_joints_joint_move_motion(self, angles):    
        i = 0
        for j in self.joints:
            j.move_joint_schedule(angles[i])
            i+=1
        self.run_scheduled_task()        

        while 1:
            arrived = True        
            for j in self.joints:
                if not arrived:
                    continue
                # j.check_motor_status()                
                status = j.driver.read_motor_status_flags()
                # print("j", j.name, status)
                if status is None:
                    continue
                if status['MOTOR_ARRIVED_TARGET'] == False:
                    arrived = False
                    
            if arrived:
                break
                
                
        print("angle done")
        
    
        
    



class Robot:
    def __init__(self) -> None:
        # self.
        
        pass
    
    


""""
# 注意： 加速度档位为 0 表示不使用曲线加减速， 直接按照设定的速度运行。 曲线加
# 减速时间计算公式： t2 - t1 = (256 - acc) * 50(us)， Vt2 = Vt1 + 1(RPM)；
# （注： acc 为加速度档位， Vt1为 t1 时刻的转速， ， Vt2 为 t2 时刻的转速）
"""
if __name__ == "__main__":
    j = Joints(AR4_CFG)    