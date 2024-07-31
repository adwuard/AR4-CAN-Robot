from zdt_emmv5 import *
from ar4_configs import *
import serial
import numpy as np



class Joint:
    def __init__(self, motor_driver, joint_cfg) -> None:
        self.driver = motor_driver
        self.cfg = joint_cfg

        self.id = joint_cfg["motor_addr"]
        self.name = joint_cfg["name"]
        self.enable = joint_cfg["enable"]
        self.axis_pos_limit_angle = joint_cfg["max_angle"]
        self.axis_neg_limit_angle = joint_cfg["min_angle"]

        self.step_master_ = 0
        self.gear_ratio = joint_cfg["gear_ratio"]
        self.steps_per_degree = joint_cfg["steps_per_degree"]
        self.steps_per_rev = joint_cfg["steps_per_rev"]
        self.micro_steps = joint_cfg["microstep"]
        self.isStalled = False
        self.dir = joint_cfg["motor_direction"]
        self.speed = joint_cfg["max_speed_rpm"]
        self.acc = joint_cfg["acc_and_dec"]

        self.offset_angle = joint_cfg["calibration_zero_offset_angle"]

        self.is_joint_dir_flipped = joint_cfg["flip_joint_direction"]

        

        

        if self.enable:
            self.init_joint()

    def joint_error_handler(self):
        pass

    def init_joint(self):
        # setup zeroing

        """Used for homing"""
        self.home_direction = self.cfg["home_direction"]
        self.home_speed_rpm = self.cfg["home_speed_rpm"]
        self.home_timeout_ms = self.cfg["home_timeout_ms"]
        self.home_method = self.cfg["home_method"]

        pass

    def clamp_angle(self, angle, neg_limit, pos_limit):
        angle = float(angle)
        neg_limit = float(neg_limit)
        pos_limit = float(pos_limit)
        if angle < neg_limit:
            return neg_limit
        elif angle > pos_limit:
            return pos_limit
        else:
            return angle


    def move_joint_schedule(self, angle):
        
        
        
        angle = self.clamp_angle(angle, self.axis_neg_limit_angle, self.axis_pos_limit_angle)
        
        if self.is_joint_dir_flipped:
            angle = angle * -1
            new_neg = (
                abs(self.axis_pos_limit_angle) + self.cfg["calibration_zero_offset_angle"]
            )
            total_angle_limit = abs(new_neg) + abs(self.axis_neg_limit_angle)
        else:
            new_neg = (
                abs(self.axis_neg_limit_angle) + self.cfg["calibration_zero_offset_angle"]
            )
            total_angle_limit = abs(new_neg) + abs(self.axis_pos_limit_angle)

        self.legal_full_steps = self.steps_per_degree * total_angle_limit
        self.t_pose_zero_steps = abs(new_neg) * self.steps_per_degree

        # print(self.legal_full_steps, self.t_pose_zero_steps, total_angle_limit)

        # self.acc = 10
        s = int(self.t_pose_zero_steps + (angle * self.steps_per_degree))
        if s < 0:
            s = 0
        if not self.enable:
            return
        self.driver.set_position_control(
            self.dir,
            self.speed,
            self.acc,
            s * self.micro_steps,
            absolute_mode=True,
            wait_broadcast_signal=True,
        )



    def move_joint_linear_schedule(self, angle, speed_rpm, accel):
        if self.is_joint_dir_flipped:
            angle = angle * -1
            new_neg = (
                abs(self.axis_pos_limit_angle) + self.cfg["calibration_zero_offset_angle"]
            )
            total_angle_limit = abs(new_neg) + abs(self.axis_neg_limit_angle)
        else:
            new_neg = (
                abs(self.axis_neg_limit_angle) + self.cfg["calibration_zero_offset_angle"]
            )
            total_angle_limit = abs(new_neg) + abs(self.axis_pos_limit_angle)


        # always use pos mode and abs on fixed direction
        total_angle_limit = abs(new_neg) + abs(self.axis_pos_limit_angle)
        self.legal_full_steps = self.steps_per_degree * total_angle_limit
        self.t_pose_zero_steps = abs(new_neg) * self.steps_per_degree

        # print(self.legal_full_steps, self.t_pose_zero_steps, total_angle_limit)        
        
        
        
        # self.acc = 10
        s = int(self.t_pose_zero_steps + (angle * self.steps_per_degree))
        if s < 0:
            s = 0
        if not self.enable:
            return
        self.driver.set_position_control(
            self.dir,
            speed_rpm,
            accel,
            s * self.micro_steps,
            absolute_mode=True,
            wait_broadcast_signal=True,
        )
        
        

    def get_joint_angle(self):
        new_neg = (
            abs(self.axis_neg_limit_angle) + self.cfg["calibration_zero_offset_angle"]
        )
    
        pulse_cnt = abs(self.driver.read_pulse_count() / self.micro_steps)
        
        angle = pulse_cnt / self.steps_per_degree
        
        if angle > new_neg:
            return (angle - new_neg)
        else:
            return -(abs(new_neg) - angle)

    def move_joint_nonblocking(self, angle):
        pass

    def check_motor_status(self):
        pass

    def on_arrived_flag(self, message):
        pass

    def check_collision_flag(self, message):
        pass


import can

class Joints:
    def __init__(self, cfg) -> None:
        self.homing_sequence = cfg.homing_sequence
        self.joints = []
        self.cur_joint_angle = []

        self.broadcast_channel = 0x00

        # self.serial_backend = serial.Serial(COM_PORT, 115200, timeout=0.001)
        # self.hw_interface = SerialPort(self.serial_backend)
        
        
        interface = can.interface.Bus(bustype='slcan', channel='COM3', bitrate=100000)
        self.hw_interface = SLCANPort(interface)
    
        
        self.kAcc = 80
        self.kMaxRpm = 400
        
        self.broadcast = ZDT_EMMV5_MOTOR(self.hw_interface, self.broadcast_channel)


        accel_delay_us_hop = (256 - self.kAcc) * 50 # takes 8800us time delta to increment and decrement rpm by 1 
        
        self.curve_rps = []
        for i in range(0,500):
            self.curve_rps.append(i/60)
            
        # print(self.curve_rps[100])


        for cfg in cfg.joints:
            driver = ZDT_EMMV5_MOTOR(self.hw_interface, cfg["motor_addr"])
            self.joints.append(Joint(driver, cfg))
            self.cur_joint_angle.append(-9999)

    def disable_all_joints(self):
        self.broadcast.disable_motor()

    def stop_all_joints(self):
        self.broadcast.stop_now()

    def run_scheduled_task(self):
        self.broadcast.broadcast_run()

    def check_all_arrived(self):
        pass


    def get_joints(self):
        joints = []
        
        for j in self.joints:
            if not j.enable:
                joints.append(0)
                continue
            
            # if self.cur_joint_angle[int(j.id) - 1] != angles[int(j.id) - 1]:
            angle = j.get_joint_angle()
            joints.append(angle)

        return joints        

    def homing(self, wait_arrived=True):
        self.broadcast.trigger_sensor_zeroing()

        if not wait_arrived:
            return

        while 1:
            arrived = True
            for j in self.joints:
                if not arrived:
                    continue
                if not j.enable:
                    continue
                status = j.driver.read_zeroing_status()
                print("j", j.name, status)
                if status is None:
                    continue
                if status["ZEROING_FAILED"]:
                    raise Exception(j.name + "zeroing failed")
                if status["ZEROING_WORKING"] == True:
                    arrived = False
            if arrived:
                print("Zeroing success.")
                break

    def move_joints_joint_move_motion(self, angles, wait_arrived=True):
        for j in self.joints:
            if not j.enable:
                continue
            
            # if self.cur_joint_angle[int(j.id) - 1] != angles[int(j.id) - 1]:
            j.move_joint_schedule(angles[int(j.id) - 1])

        self.run_scheduled_task()

        if not wait_arrived:
            self.cur_joint_angle = angles
            return

        while True:
            arrived = True
            for j in self.joints:
                if not arrived or not j.enable:
                    continue
                status = j.driver.read_motor_status_flags()
                if status is None:
                    continue
                if not status["MOTOR_ARRIVED_TARGET"]:
                    arrived = False

            if arrived:
                break

        self.cur_joint_angle = angles
        print("move joints to: ", self.cur_joint_angle, " success")


    def move_joints_joint_move_linear_motion(self, angles, wait_arrived=True):
        curr_joints = self.get_joints()
        joints_curr = np.array(curr_joints)
        new_target = np.array(angles)
        
        # Pad new_target to the same size as joints_curr
        if len(new_target) < len(joints_curr):
            new_target = np.pad(new_target, (0, len(joints_curr) - len(new_target)), 'constant', constant_values=0)
        
        joints_curr = joints_curr

        # print("current, ", joints_curr)
        # print("target, ", new_target)
        
        
        
        joint_delta_abs = abs(new_target - joints_curr)
        print("move delta: ", joint_delta_abs)
        # min_delta = min(joint_delta_abs)
        
        # i = 0
        # for j in angles:
        
        
        # min_delta_2 = (j.steps_per_degree * min_delta) / j.steps_per_rev


        # accel_delay_us_hop = (256 - self.kAcc) * 50 # takes 8800us time delta to increment and decrement rpm by 1 
        
        min_arrive = 999 
        
        
        # longest joint angle / joint angle ratio, apply to all joints 
        
        
        longest_joint_angle =  max(joint_delta_abs)
        max_rev_needed_to_arrive = 0 
        
        # for i in range(0, len(joint_delta_abs)):
        #     if not self.joints[i].enable:
        #         continue
        #     if joint_delta_abs[i] == longest_joint_angle:
                
        # for j in self.joints:
            # if not j.enable:
                # continue
            
            # rev_needed_to_arrive = (j.steps_per_degree * joint_delta_abs[]) / j.steps_per_rev

    
        for j in self.joints:
            if not j.enable:
                continue
            a = joint_delta_abs[int(j.id)-1]
            rev_needed_to_arrive = (j.steps_per_degree * a)
            if rev_needed_to_arrive > max_rev_needed_to_arrive:
                max_rev_needed_to_arrive = rev_needed_to_arrive
                print("newest is ", max_rev_needed_to_arrive, "joint", j.id)


        # max_rev_needed_to_arrive
        
        # print(longest_joint_angle)
        for j in self.joints:
            if not j.enable:
                continue
            a = joint_delta_abs[int(j.id)-1]
            
            
                        
            rev_needed_to_arrive = (j.steps_per_degree * a)
            ratio = rev_needed_to_arrive/ max_rev_needed_to_arrive
            # if ratio != 1:
                # ratio = ratio * 1.1
            
            print(int(j.id), "revs needed to arrive", rev_needed_to_arrive, "ratio to longest:", ratio, " adjusted rpm:", int(self.kMaxRpm * ratio))
            
            
            
            # rps = self.curve_rps[self.kMaxRpm]
            print(j.id , "rpm:", int(self.kMaxRpm * ratio))
        
            j.move_joint_linear_schedule(angles[int(j.id)-1], int(self.kMaxRpm * ratio), self.kAcc)
            

        

        self.run_scheduled_task()

        if not wait_arrived:
            self.cur_joint_angle = angles
            return

        while True:
            arrived = True
            for j in self.joints:
                if not arrived or not j.enable:
                    continue
                status = j.driver.read_motor_status_flags()
                if status is None:
                    continue
                if not status["MOTOR_ARRIVED_TARGET"]:
                    arrived = False

            if arrived:
                break

        self.cur_joint_angle = angles
        print("move joints to: ", self.cur_joint_angle, " success")
